#include "DebugConfiguration.h"
#include "configuration.h"

#if HAS_TELEMETRY && !MESHTASTIC_EXCLUDE_AIR_QUALITY_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "AirQualityTelemetry.h"
#include "Default.h"
#include "GPSStatus.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "PowerFSM.h"
#include "RTC.h"
#include "Router.h"
#include "TransmitHistory.h"
#include "UnitConversions.h"
#include "graphics/ScreenFonts.h"
#include "graphics/SharedUIDisplay.h"
#include "graphics/images.h"
#include "main.h"
#include "sleep.h"
#include <Throttle.h>
#include <cmath>
#include <algorithm>

// ----------------- Anomaly detection / dynamic monitoring parameters -----------------
// Tunable parameters (easy to change)
#ifndef MOVING_WINDOW_SAMPLE_COUNT
static constexpr size_t MOVING_WINDOW_SAMPLE_COUNT = 10; // number of samples in moving window
#endif
#ifndef FREQUENT_MONITORING_CYCLES
static constexpr int FREQUENT_MONITORING_CYCLES = 10; // number of frequent cycles to stay in frequent mode
#endif
#ifndef FREQUENT_MONITORING_CYCLE_SEC
static constexpr uint32_t FREQUENT_MONITORING_CYCLE_SEC = 60; // seconds between frequent measurements
#endif
#ifndef ANOMALY_STDEV_FACTOR
static constexpr float ANOMALY_STDEV_FACTOR = 3.0f; // k factor for stdev
#endif
#ifndef MIN_ANOMALY_DELTA
#define MIN_ANOMALY_DELTA_PM_TEMPERATURE 0.5f
#define MIN_ANOMALY_DELTA_PM_HUMIDITY 2.0f
#define MIN_ANOMALY_DELTA_PM10 5.0f
#define MIN_ANOMALY_DELTA_PM_VOC_INDEX 5.0f
#endif
#ifndef ANOMALY_PERCENT_DELTA
static constexpr float ANOMALY_PERCENT_DELTA_PM_TEMPERATURE = 10.0f;
static constexpr float ANOMALY_PERCENT_DELTA_PM_HUMIDITY = 10.0f;
static constexpr float ANOMALY_PERCENT_DELTA_PM10 = 15.0f;
static constexpr float ANOMALY_PERCENT_DELTA_PM_VOC_INDEX = 15.0f;
#endif

// Persistence macro for RTC memory where supported (ESP32)
// On ESP32, RTC_DATA_ATTR places variables in RTC slow memory and they are retained across deep sleep.
// On non-ESP32 targets, RTC_PERSIST expands to nothing and the variables are not preserved.
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32_S3)
#define RTC_PERSIST RTC_DATA_ATTR
#else
#define RTC_PERSIST
#endif

// Ring buffers stored in RTC memory where available so deep sleep doesn't lose history
RTC_PERSIST static float pm_temperature_buffer[MOVING_WINDOW_SAMPLE_COUNT];
RTC_PERSIST static size_t pm_temperature_index = 0;
RTC_PERSIST static size_t pm_temperature_count = 0;

RTC_PERSIST static float pm_humidity_buffer[MOVING_WINDOW_SAMPLE_COUNT];
RTC_PERSIST static size_t pm_humidity_index = 0;
RTC_PERSIST static size_t pm_humidity_count = 0;

RTC_PERSIST static float pm10_buffer[MOVING_WINDOW_SAMPLE_COUNT];
RTC_PERSIST static size_t pm10_index = 0;
RTC_PERSIST static size_t pm10_count = 0;

RTC_PERSIST static float voc_buffer[MOVING_WINDOW_SAMPLE_COUNT];
RTC_PERSIST static size_t voc_index = 0;
RTC_PERSIST static size_t voc_count = 0;

// Persistence test helper stored in RTC memory when supported
RTC_PERSIST static bool rtcPersistenceFlag = false;

// Helper: update ring buffer (FIFO/circular)
static inline void updateRingBuffer(float *buf, size_t &index, size_t &count, float newVal)
{
    buf[index] = newVal;
    index = (index + 1) % MOVING_WINDOW_SAMPLE_COUNT;
    if (count < MOVING_WINDOW_SAMPLE_COUNT)
        ++count;
}

// Helper: calculate mean of `count` samples in buf (assumes count>0)
static float calculateMean(const float *buf, size_t count)
{
    if (count == 0)
        return 0.0f;
    double s = 0.0;
    for (size_t i = 0; i < count; ++i)
        s += buf[i];
    return (float)(s / (double)count);
}

// Helper: sample standard deviation (n-1) denominator for sample stdev
static float calculateStDev(const float *buf, size_t count)
{
    if (count <= 1)
        return 0.0f;
    double mean = calculateMean(buf, count);
    double ss = 0.0;
    for (size_t i = 0; i < count; ++i) {
        double d = buf[i] - mean;
        ss += d * d;
    }
    return (float)std::sqrt(ss / (double)(count - 1));
}

// Helper: anomaly check using combined stdev + absolute delta + percentage delta thresholds.
// direction: true => detect increase (new > mean+threshold), false => detect decrease (new < mean-threshold)
static bool isAnomaly(float newVal, const float *buf, size_t count, bool directionIncrease, float minDelta,
                      float percentDelta)
{
    if (count == 0)
        return false;
    float mean = calculateMean(buf, count);
    float stdev = calculateStDev(buf, count);
    float percentThreshold = fabsf(mean) * (percentDelta / 100.0f);
    float threshold = fmaxf(fmaxf(ANOMALY_STDEV_FACTOR * stdev, minDelta), percentThreshold);
    if (directionIncrease) {
        return newVal > (mean + threshold);
    } else {
        return newVal < (mean - threshold);
    }
}


static constexpr uint16_t TX_HISTORY_KEY_AIR_QUALITY_TELEMETRY = 0x8004;

// Sensors
#include "Sensor/AddI2CSensorTemplate.h"
#include "Sensor/PMSA003ISensor.h"
#include "Sensor/SEN5XSensor.h"
#if __has_include(<SensirionI2cScd4x.h>)
#include "Sensor/SCD4XSensor.h"
#endif
#if __has_include(<SensirionI2cSfa3x.h>)
#include "Sensor/SFA30Sensor.h"
#endif
#if __has_include(<SensirionI2cScd30.h>)
#include "Sensor/SCD30Sensor.h"
#endif

void AirQualityTelemetryModule::i2cScanFinished(ScanI2C *i2cScanner)
{
    if (!moduleConfig.telemetry.air_quality_enabled && !AIR_QUALITY_TELEMETRY_MODULE_ENABLE) {
        return;
    }
    LOG_INFO("Air Quality Telemetry adding I2C devices...");

    /*
        Uncomment the preferences below if you want to use the module
        without having to configure it from the PythonAPI or WebUI.
        Note: this was previously on runOnce, which didnt take effect
        as other modules already had already been initialized (screen)
    */

    // moduleConfig.telemetry.air_quality_enabled = 1;
    // moduleConfig.telemetry.air_quality_screen_enabled = 1;
    // moduleConfig.telemetry.air_quality_interval = 15;

    // order by priority of metrics/values (low top, high bottom)
    addSensor<PMSA003ISensor>(i2cScanner, ScanI2C::DeviceType::PMSA003I);
    addSensor<SEN5XSensor>(i2cScanner, ScanI2C::DeviceType::SEN5X);
#if __has_include(<SensirionI2cScd4x.h>)
    addSensor<SCD4XSensor>(i2cScanner, ScanI2C::DeviceType::SCD4X);
#endif
#if __has_include(<SensirionI2cSfa3x.h>)
    addSensor<SFA30Sensor>(i2cScanner, ScanI2C::DeviceType::SFA30);
#endif
#if __has_include(<SensirionI2cScd30.h>)
    addSensor<SCD30Sensor>(i2cScanner, ScanI2C::DeviceType::SCD30);
#endif
}

int32_t AirQualityTelemetryModule::runOnce()
{
    // İstediğimiz ölçümü henüz alamadıysak sensörleri uykuya yatırmayacağız. Zaten SEN55 VOC stabilizasyonu için uzun bekliyoruz. Uykuya yatırmak bunu iyice sabote edebilir.
    bool allowSleep = false;

    moduleConfig.telemetry.air_quality_enabled = 1;
    moduleConfig.telemetry.air_quality_screen_enabled = 1;
    moduleConfig.telemetry.air_quality_interval = 830; // Kullanıcı ayarından bağımsız olarak kodda her 15 dakikada bir ölçüm yapacak şekilde ayarladık.
    //moduleConfig.telemetry.air_quality_interval = 300; // Kullanıcı ayarından bağımsız olarak kodda her 15 dakikada bir ölçüm yapacak şekilde ayarladık.

    if (sleepOnNextExecution == false) {
        LOG_INFO("RTC persistence flag=%s, voc_count=%u", rtcPersistenceFlag ? "true" : "false", voc_count);
    } else {
        LOG_INFO("Will sleep on this run. Setting RTC persistence flag to true for test.");
        rtcPersistenceFlag = true;
    }

    if (sleepOnNextExecution == true) {
        sleepOnNextExecution = false;
        uint32_t nightyNightMs = Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.air_quality_interval,
                                                                   default_telemetry_broadcast_interval_secs);
        LOG_DEBUG("Sleeping for %ims, then awaking to send metrics again.", nightyNightMs);
        doDeepSleep(nightyNightMs, true, false);
    }

    uint32_t result = UINT32_MAX;

    if (!(moduleConfig.telemetry.air_quality_enabled || moduleConfig.telemetry.air_quality_screen_enabled ||
          AIR_QUALITY_TELEMETRY_MODULE_ENABLE)) {
        // If this module is not enabled, and the user doesn't want the display screen don't waste any OSThread time on it
        return disable();
    }

    if (firstTime) {
        // This is the first time the OSThread library has called this function, so
        // do some setup
        firstTime = false;

        LOG_INFO("Air quality Telemetry: First call of runOnce, doing setup");

        if (moduleConfig.telemetry.air_quality_enabled) {
            LOG_INFO("Air quality Telemetry: init");

            // check if we have at least one sensor
            if (!sensors.empty()) {
                result = DEFAULT_SENSOR_MINIMUM_WAIT_TIME_BETWEEN_READS;
            }
        }

        // it's possible to have this module enabled, only for displaying values on the screen.
        // therefore, we should only enable the sensor loop if measurement is also enabled
        // return result == UINT32_MAX ? disable() : setStartDelay();

        // Firmware standart olarak ilk başta 75sn bekliyordu. AirQuality sensörlerini uyandurmak için bile bu süreyi bekliyorduk. Bu vakti kaybetmemek için kısa bekleyip yeniden runOnce çağıracağız.
        return result == UINT32_MAX ? disable() : 500; // 500ms sonra wakeup ve measurement mode geçiş deneyelim bakalım ne oluyor?
        // return result == UINT32_MAX ? disable() : 40000;
    } else {
        LOG_INFO("Air quality Telemetry: Subsequent call of runOnce");
        // if we somehow got to a second run of this module with measurement disabled, then just wait forever
        if (!moduleConfig.telemetry.air_quality_enabled && !AIR_QUALITY_TELEMETRY_MODULE_ENABLE) {
            return disable();
        }

        // Wake up the sensors that need it
        LOG_INFO("Waking up sensors...");
        uint32_t lastTelemetry =
            transmitHistory ? transmitHistory->getLastSentToMeshMillis(TX_HISTORY_KEY_AIR_QUALITY_TELEMETRY) : 0;
        for (TelemetrySensor *sensor : sensors) {
            if (!sensor->canSleep()) {
                LOG_DEBUG("%s sensor doesn't have sleep feature. Skipping", sensor->sensorName);
            } else {
                // Detailed debugging for the else if condition
                bool lastTelemetryZero = (lastTelemetry == 0);
                uint32_t intervalMs = Default::getConfiguredOrDefaultMsScaled(
                    moduleConfig.telemetry.air_quality_interval,
                    default_telemetry_broadcast_interval_secs, numOnlineNodes);
                int32_t wakeUpTimeMs = sensor->wakeUpTimeMs();
                uint32_t timeSinceLastTelemetry = lastTelemetry > wakeUpTimeMs ? lastTelemetry - wakeUpTimeMs : 0;
                bool isWithinTimespan = Throttle::isWithinTimespanMs(timeSinceLastTelemetry, intervalMs);
                bool timeCondition = lastTelemetryZero || !isWithinTimespan || config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR;
                bool channelUtilOk = airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR);
                bool airUtilOk = airTime->isTxAllowedAirUtil();
                
                LOG_DEBUG("%s condition check: lastTelemetry=%u, wakeUpTime=%ld, interval=%u, timeSince=%u, isWithin=%s, timeOk=%s, channelUtil=%s, airUtil=%s",
                    sensor->sensorName, lastTelemetry, wakeUpTimeMs, intervalMs, timeSinceLastTelemetry,
                    isWithinTimespan ? "true" : "false", timeCondition ? "true" : "false",
                    channelUtilOk ? "true" : "false", airUtilOk ? "true" : "false");
                
                if (timeCondition && channelUtilOk && airUtilOk) {
                    if (!sensor->isActive()) {
                        LOG_DEBUG("Waking up: %s", sensor->sensorName);
                        return sensor->wakeUp();
                    } else {
                        int32_t pendingForReadyMs = sensor->pendingForReadyMs();
                        LOG_DEBUG("%s. Pending for ready %ums", sensor->sensorName, pendingForReadyMs);
                        if (pendingForReadyMs) {
                            return pendingForReadyMs;
                        }
                    }
                } else {
                    LOG_DEBUG("Not waking up %s yet. Time condition: %s, Channel Util: %s, Air Util: %s",
                        sensor->sensorName, timeCondition ? "true" : "false",
                        channelUtilOk ? "true" : "false", airUtilOk ? "true" : "false");
                }
            }
        }

        if ((config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR ||(lastTelemetry == 0) ||
             !Throttle::isWithinTimespanMs(lastTelemetry, Default::getConfiguredOrDefaultMsScaled(
                                                              moduleConfig.telemetry.air_quality_interval,
                                                              default_telemetry_broadcast_interval_secs, numOnlineNodes))) &&
            airTime->isTxAllowedChannelUtil(config.device.role != meshtastic_Config_DeviceConfig_Role_SENSOR) &&
            airTime->isTxAllowedAirUtil()) {

            //sendTelemetry();

            // Transmithistory'yi sadece gerçekten veri gönderimi yapılırsa güncelle.
            if (sendTelemetry() && transmitHistory) {
                allowSleep = true;
                transmitHistory->setLastSentToMesh(TX_HISTORY_KEY_AIR_QUALITY_TELEMETRY);
            }
        } else if (((lastSentToPhone == 0) || !Throttle::isWithinTimespanMs(lastSentToPhone, sendToPhoneIntervalMs)) &&
                   (service->isToPhoneQueueEmpty())) {
            // Just send to phone when it's not our time to send to mesh yet
            // Only send while queue is empty (phone assumed connected)
            sendTelemetry(NODENUM_BROADCAST, true);
            lastSentToPhone = millis();
        }

        // Send to sleep sensors that consume power
        if (allowSleep) {
            LOG_DEBUG("Sending sensors to sleep");
            for (TelemetrySensor *sensor : sensors) {
                if (sensor->isActive() && sensor->canSleep()) {
                    if (sensor->wakeUpTimeMs() <
                        (int32_t)Default::getConfiguredOrDefaultMsScaled(moduleConfig.telemetry.air_quality_interval,
                                                                        default_telemetry_broadcast_interval_secs, numOnlineNodes)) {
                        LOG_DEBUG("Disabling %s until next period", sensor->sensorName);
                        sensor->sleep();
                    } else {
                        LOG_DEBUG("Sensor stays enabled due to warm up period");
                    }
                }
            }
        } else {
            LOG_DEBUG("Not sending sensors to sleep because no telemetry was sent");
        }

        // Bu aşamada cihazı hızla uyutmak istiyoruz. Bu yüzden bir sonraki runOnce çalışmasını 1sn sonra yaptıracağız ve uykuya dalması gerekiyor.
        // 1sn sonra uyutunca henüz LoRa iletimi tamamlanamıyor sanırım. Cihazı power saving = enabled durumda işlettiğmizde karşıya veri iletimi olmuyor çoğu zaman veya hiç. O yüzden daha uzun gecikme koyuyoruz.
        return 5000;
    }
    return min(sendToPhoneIntervalMs, result);
}

bool AirQualityTelemetryModule::wantUIFrame()
{
    return moduleConfig.telemetry.air_quality_screen_enabled;
}

#if HAS_SCREEN
void AirQualityTelemetryModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
    // === Setup display ===
    display->clear();
    display->setFont(FONT_SMALL);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    int line = 1;

    // === Set Title
    const char *titleStr = (graphics::currentResolution == graphics::ScreenResolution::High) ? "Air Quality" : "AQ.";

    // === Header ===
    graphics::drawCommonHeader(display, x, y, titleStr);

    // === Row spacing setup ===
    const int rowHeight = FONT_HEIGHT_SMALL - 4;
    int currentY = graphics::getTextPositions(display)[line++];

    // === Show "No Telemetry" if no data available ===
    if (!lastMeasurementPacket) {
        display->drawString(x, currentY, "No Telemetry");
        return;
    }

    // Decode the telemetry message from the latest received packet
    const meshtastic_Data &p = lastMeasurementPacket->decoded;
    meshtastic_Telemetry telemetry;
    if (!pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &telemetry)) {
        display->drawString(x, currentY, "No Telemetry");
        return;
    }

    const auto &m = telemetry.variant.air_quality_metrics;

    // Check if any telemetry field has valid data
    bool hasAny = m.has_pm10_standard || m.has_pm25_standard || m.has_pm100_standard || m.has_co2;

    if (!hasAny) {
        display->drawString(x, currentY, "No Telemetry");
        return;
    }

    // === First line: Show sender name + time since received (left), and first metric (right) ===
    const char *sender = getSenderShortName(*lastMeasurementPacket);
    uint32_t agoSecs = service->GetTimeSinceMeshPacket(lastMeasurementPacket);
    String agoStr = (agoSecs > 864000) ? "?"
                    : (agoSecs > 3600) ? String(agoSecs / 3600) + "h"
                    : (agoSecs > 60)   ? String(agoSecs / 60) + "m"
                                       : String(agoSecs) + "s";

    String leftStr = String(sender) + " (" + agoStr + ")";
    display->drawString(x, currentY, leftStr); // Left side: who and when

    // === Collect sensor readings as label strings (no icons) ===
    std::vector<String> entries;

    if (m.has_pm10_standard)
        entries.push_back("PM1: " + String(m.pm10_standard) + "ug/m3");
    if (m.has_pm25_standard)
        entries.push_back("PM2.5: " + String(m.pm25_standard) + "ug/m3");
    if (m.has_pm100_standard)
        entries.push_back("PM10: " + String(m.pm100_standard) + "ug/m3");
    if (m.has_co2)
        entries.push_back("CO2: " + String(m.co2) + "ppm");
    if (m.has_form_formaldehyde)
        entries.push_back("HCHO: " + String(m.form_formaldehyde) + "ppb");

    // === Show first available metric on top-right of first line ===
    if (!entries.empty()) {
        String valueStr = entries.front();
        int rightX = SCREEN_WIDTH - display->getStringWidth(valueStr);
        display->drawString(rightX, currentY, valueStr);
        entries.erase(entries.begin()); // Remove from queue
    }

    // === Advance to next line for remaining telemetry entries ===
    currentY += rowHeight;

    // === Draw remaining entries in 2-column format (left and right) ===
    for (size_t i = 0; i < entries.size(); i += 2) {
        // Left column
        display->drawString(x, currentY, entries[i]);

        // Right column if it exists
        if (i + 1 < entries.size()) {
            int rightX = SCREEN_WIDTH / 2;
            display->drawString(rightX, currentY, entries[i + 1]);
        }

        currentY += rowHeight;
    }
    graphics::drawCommonFooter(display, x, y);
}
#endif

bool AirQualityTelemetryModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *t)
{
    if (t->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
#if defined(DEBUG_PORT) && !defined(DEBUG_MUTE)
        const char *sender = getSenderShortName(mp);

        if (t->variant.air_quality_metrics.has_pm10_standard)
            LOG_INFO("(Received from %s): pm10_standard=%i, pm25_standard=%i, "
                     "pm100_standard=%i",
                     sender, t->variant.air_quality_metrics.pm10_standard, t->variant.air_quality_metrics.pm25_standard,
                     t->variant.air_quality_metrics.pm100_standard);

        if (t->variant.air_quality_metrics.has_co2)
            LOG_INFO("CO2=%i, CO2_T=%.2f, CO2_H=%.2f", t->variant.air_quality_metrics.co2,
                     t->variant.air_quality_metrics.co2_temperature, t->variant.air_quality_metrics.co2_humidity);

        if (t->variant.air_quality_metrics.has_form_formaldehyde)
            LOG_INFO("HCHO=%.2f, HCHO_T=%.2f, HCHO_H=%.2f", t->variant.air_quality_metrics.form_formaldehyde,
                     t->variant.air_quality_metrics.form_temperature, t->variant.air_quality_metrics.form_humidity);
#endif
        // release previous packet before occupying a new spot
        if (lastMeasurementPacket != nullptr)
            packetPool.release(lastMeasurementPacket);

        lastMeasurementPacket = packetPool.allocCopy(mp);
    }

    return false; // Let others look at this message also if they want
}

bool AirQualityTelemetryModule::getAirQualityTelemetry(meshtastic_Telemetry *m)
{
    // Note: this is different to the case in EnvironmentTelemetryModule
    // There, if any sensor fails to read - valid = false.
    bool valid = false;
    bool hasSensor = false;
    m->time = getTime();
    m->which_variant = meshtastic_Telemetry_air_quality_metrics_tag;
    m->variant.air_quality_metrics = meshtastic_AirQualityMetrics_init_zero;

    bool sensor_get = false;
    for (TelemetrySensor *sensor : sensors) {
        LOG_DEBUG("Reading %s", sensor->sensorName);
        // Note - this function doesn't get properly called if within a conditional
        sensor_get = sensor->getMetrics(m);
        valid = valid || sensor_get;
        hasSensor = true;
    }

    if (powerStatus) {
        m->variant.air_quality_metrics.has_battery_level = true;
        m->variant.air_quality_metrics.battery_level =
            (!powerStatus->getHasBattery() || powerStatus->getIsCharging()) ? 101u : powerStatus->getBatteryChargePercent();
        m->variant.air_quality_metrics.has_voltage = true;
        m->variant.air_quality_metrics.voltage = powerStatus->getBatteryVoltageMv() / 1000.0;
    }

    // Add GPS or configured position information
    if (gpsStatus) {
        int32_t lat = gpsStatus->getLatitude();
        int32_t lon = gpsStatus->getLongitude();
        int32_t alt = gpsStatus->getAltitude();

        LOG_INFO("Air quality telemetry position: gpsStatus available, lat=%ld, lon=%ld, alt=%ld", (long)lat, (long)lon,
                 (long)alt);

        if (lat != 0 || lon != 0) {
            m->variant.air_quality_metrics.has_latitude_i = true;
            m->variant.air_quality_metrics.latitude_i = lat;
            m->variant.air_quality_metrics.has_longitude_i = true;
            m->variant.air_quality_metrics.longitude_i = lon;
            m->variant.air_quality_metrics.has_altitude = true;
            m->variant.air_quality_metrics.altitude = alt;
            LOG_INFO("Air quality telemetry position fields added to packet");
        } else {
            LOG_WARN("Air quality telemetry position fields not added: lat/lon are zero");
        }
    } else {
        LOG_WARN("Air quality telemetry position: gpsStatus not available, position fields not added");
    }

    return valid && hasSensor;
}

meshtastic_MeshPacket *AirQualityTelemetryModule::allocReply()
{
    if (currentRequest) {
        if (isMultiHopBroadcastRequest() && !isSensorOrRouterRole()) {
            ignoreRequest = true;
            return NULL;
        }
        auto req = *currentRequest;
        const auto &p = req.decoded;
        meshtastic_Telemetry scratch;
        meshtastic_Telemetry *decoded = NULL;
        memset(&scratch, 0, sizeof(scratch));
        if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Telemetry_msg, &scratch)) {
            decoded = &scratch;
        } else {
            LOG_ERROR("Error decoding AirQualityTelemetry module!");
            return NULL;
        }
        // Check for a request for air quality metrics
        if (decoded->which_variant == meshtastic_Telemetry_air_quality_metrics_tag) {
            meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
            if (getAirQualityTelemetry(&m)) {
                LOG_INFO("Air quality telemetry reply to request");
                return allocDataProtobuf(m);
            } else {
                return NULL;
            }
        }
    }
    return NULL;
}

bool AirQualityTelemetryModule::sendTelemetry(NodeNum dest, bool phoneOnly)
{
    meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
    m.which_variant = meshtastic_Telemetry_air_quality_metrics_tag;
    m.time = getTime();

    if (getAirQualityTelemetry(&m)) {
        // Update anomaly detection buffers and state based on this measurement
        processAnomalyDetection(m);
        if (m.variant.air_quality_metrics.has_pm_voc_idx && m.variant.air_quality_metrics.pm_voc_idx > 0) {
            bool hasAnyPM =
                m.variant.air_quality_metrics.has_pm10_standard || m.variant.air_quality_metrics.has_pm25_standard ||
                m.variant.air_quality_metrics.has_pm100_standard || m.variant.air_quality_metrics.has_pm10_environmental ||
                m.variant.air_quality_metrics.has_pm25_environmental || m.variant.air_quality_metrics.has_pm100_environmental;

            if (hasAnyPM) {
                LOG_INFO("Send: pm10_standard=%u, pm25_standard=%u, pm100_standard=%u", m.variant.air_quality_metrics.pm10_standard,
                        m.variant.air_quality_metrics.pm25_standard, m.variant.air_quality_metrics.pm100_standard);
                if (m.variant.air_quality_metrics.has_pm10_environmental)
                    LOG_INFO("pm10_environmental=%u, pm25_environmental=%u, pm100_environmental=%u",
                            m.variant.air_quality_metrics.pm10_environmental, m.variant.air_quality_metrics.pm25_environmental,
                            m.variant.air_quality_metrics.pm100_environmental);
            }

            bool hasAnyCO2 = m.variant.air_quality_metrics.has_co2 || m.variant.air_quality_metrics.has_co2_temperature ||
                            m.variant.air_quality_metrics.has_co2_humidity;

            if (hasAnyCO2) {
                LOG_INFO("Send: co2=%i, co2_t=%.2f, co2_rh=%.2f", m.variant.air_quality_metrics.co2,
                        m.variant.air_quality_metrics.co2_temperature, m.variant.air_quality_metrics.co2_humidity);
            }

            bool hasAnyHCHO = m.variant.air_quality_metrics.has_form_formaldehyde ||
                            m.variant.air_quality_metrics.has_form_temperature || m.variant.air_quality_metrics.has_form_humidity;

            if (hasAnyHCHO) {
                LOG_INFO("Send: hcho=%.2f, hcho_t=%.2f, hcho_rh=%.2f", m.variant.air_quality_metrics.form_formaldehyde,
                        m.variant.air_quality_metrics.form_temperature, m.variant.air_quality_metrics.form_humidity);
            }

            meshtastic_MeshPacket *p = allocDataProtobuf(m);
            p->to = dest;
            p->decoded.want_response = false;
            if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR)
                p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
            else
                p->priority = meshtastic_MeshPacket_Priority_BACKGROUND;

            // release previous packet before occupying a new spot
            if (lastMeasurementPacket != nullptr)
                packetPool.release(lastMeasurementPacket);

            lastMeasurementPacket = packetPool.allocCopy(*p);
            if (phoneOnly) {
                LOG_INFO("Sending packet to phone");
                service->sendToPhone(p);
            } else {
                LOG_INFO("Sending packet to mesh");
                service->sendToMesh(p, RX_SRC_LOCAL, true);

                // If we're in dynamic frequent-monitoring mode, schedule next cycle
                if (monitoringMode == FREQUENT_MONITORING_MODE) {
                    // Reset sleep flag so device won't deep-sleep in the next run
                    sleepOnNextExecution = false;
                    // Decrement remaining cycles (unless -1 meaning infinite)
                    if (frequentCyclesRemaining > 0) {
                        --frequentCyclesRemaining;
                    }
                    LOG_INFO("In FREQUENT_MONITORING mode, cycles remaining=%d", frequentCyclesRemaining);
                    // If cycles exhausted, go back to normal
                    if (frequentCyclesRemaining <= 0) {
                        monitoringMode = NORMAL_MODE;
                        LOG_INFO("FREQUENT_MONITORING expired, returning to NORMAL_MODE");
                        // After finishing frequent monitoring, allow normal sleep scheduling
                        if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR && config.power.is_power_saving) {
                            meshtastic_ClientNotification *notification = clientNotificationPool.allocZeroed();
                            notification->level = meshtastic_LogRecord_Level_INFO;
                            notification->time = getValidTime(RTCQualityFromNet);
                            sprintf(notification->message, "Ending frequent monitoring, will sleep for %us interval",
                                    Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.air_quality_interval,
                                                                    default_telemetry_broadcast_interval_secs) /
                                        1000U);
                            service->sendClientNotification(notification);
                            sleepOnNextExecution = true;
                            setIntervalFromNow(FIVE_SECONDS_MS);
                        }
                    } else {
                        // Schedule next frequent measurement non-blocking
                        setIntervalFromNow(FREQUENT_MONITORING_CYCLE_SEC * 1000U);
                    }
                } else if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR && config.power.is_power_saving) {
                    // Default behaviour: after sending telemetry in sensor+power_saving, schedule sleep
                    meshtastic_ClientNotification *notification = clientNotificationPool.allocZeroed();
                    notification->level = meshtastic_LogRecord_Level_INFO;
                    notification->time = getValidTime(RTCQualityFromNet);
                    sprintf(notification->message, "Sending telemetry and sleeping for %us interval in a moment",
                            Default::getConfiguredOrDefaultMs(moduleConfig.telemetry.air_quality_interval,
                                                            default_telemetry_broadcast_interval_secs) /
                                1000U);
                    service->sendClientNotification(notification);
                    sleepOnNextExecution = true;
                    LOG_DEBUG("Start next execution in 5s, then sleep");
                    setIntervalFromNow(FIVE_SECONDS_MS);
                }
            }
            return true;
        } else {
            LOG_WARN("Invalid VOC index, not sending telemetry");
            return false;
        }
    }
    return false;
}

// Process anomaly detection using moving window buffers for selected metrics.
// Checks are only performed when the buffer is full; buffers are updated every measurement.
void AirQualityTelemetryModule::processAnomalyDetection(const meshtastic_Telemetry &m)
{
    bool detected = false;
    // Track maximum remaining samples needed to fill buffers (for initial collection)
    size_t maxRemainingToFill = 0;

    // pm_temperature (detect increase)
    if (m.variant.air_quality_metrics.has_pm_temperature) {
        float newVal = m.variant.air_quality_metrics.pm_temperature;
        if (pm_temperature_count == MOVING_WINDOW_SAMPLE_COUNT) {
            if (isAnomaly(newVal, pm_temperature_buffer, pm_temperature_count, true, MIN_ANOMALY_DELTA_PM_TEMPERATURE,
                          ANOMALY_PERCENT_DELTA_PM_TEMPERATURE)) {
                LOG_INFO("Anomaly detected on pm_temperature: new=%.2f", newVal);
                detected = true;
            }
        }
        updateRingBuffer(pm_temperature_buffer, pm_temperature_index, pm_temperature_count, newVal);
        maxRemainingToFill = std::max(maxRemainingToFill, MOVING_WINDOW_SAMPLE_COUNT - pm_temperature_count);
    }

    // pm_humidity (detect decrease)
    if (m.variant.air_quality_metrics.has_pm_humidity) {
        float newVal = m.variant.air_quality_metrics.pm_humidity;
        if (pm_humidity_count == MOVING_WINDOW_SAMPLE_COUNT) {
            if (isAnomaly(newVal, pm_humidity_buffer, pm_humidity_count, false, MIN_ANOMALY_DELTA_PM_HUMIDITY,
                          ANOMALY_PERCENT_DELTA_PM_HUMIDITY)) {
                LOG_INFO("Anomaly detected on pm_humidity (decrease): new=%.2f", newVal);
                detected = true;
            }
        }
        updateRingBuffer(pm_humidity_buffer, pm_humidity_index, pm_humidity_count, newVal);
        maxRemainingToFill = std::max(maxRemainingToFill, MOVING_WINDOW_SAMPLE_COUNT - pm_humidity_count);
    }

    // pm10 (detect increase)
    if (m.variant.air_quality_metrics.has_pm10_standard) {
        float newVal = (float)m.variant.air_quality_metrics.pm10_standard;
        if (pm10_count == MOVING_WINDOW_SAMPLE_COUNT) {
            if (isAnomaly(newVal, pm10_buffer, pm10_count, true, MIN_ANOMALY_DELTA_PM10, ANOMALY_PERCENT_DELTA_PM10)) {
                LOG_INFO("Anomaly detected on pm10: new=%.2f", newVal);
                detected = true;
            }
        }
        updateRingBuffer(pm10_buffer, pm10_index, pm10_count, newVal);
        maxRemainingToFill = std::max(maxRemainingToFill, MOVING_WINDOW_SAMPLE_COUNT - pm10_count);
    }

    // VOC index (detect increase)
    if (m.variant.air_quality_metrics.has_pm_voc_idx) {
        float newVal = m.variant.air_quality_metrics.pm_voc_idx;
        if (voc_count == MOVING_WINDOW_SAMPLE_COUNT) {
            if (isAnomaly(newVal, voc_buffer, voc_count, true, MIN_ANOMALY_DELTA_PM_VOC_INDEX,
                          ANOMALY_PERCENT_DELTA_PM_VOC_INDEX)) {
                LOG_INFO("Anomaly detected on VOC index: new=%.2f", newVal);
                detected = true;
            }
        }
        updateRingBuffer(voc_buffer, voc_index, voc_count, newVal);
        maxRemainingToFill = std::max(maxRemainingToFill, MOVING_WINDOW_SAMPLE_COUNT - voc_count);
    }

    // If any buffer isn't full yet, ensure we stay in frequent-monitoring until filled
    if (maxRemainingToFill > 0 && monitoringMode != FREQUENT_MONITORING_MODE) {
        monitoringMode = FREQUENT_MONITORING_MODE;
        frequentCyclesRemaining = (int)maxRemainingToFill; // do enough cycles to fill the largest gap
        LOG_INFO("Beginning initial fill: staying in FREQUENT_MONITORING for %d cycles to populate buffers", frequentCyclesRemaining);
        sleepOnNextExecution = false;
        setIntervalFromNow(FREQUENT_MONITORING_CYCLE_SEC * 1000U);
        return;
    }

    if (detected) {
        // Enter or extend frequent monitoring
        if (monitoringMode == FREQUENT_MONITORING_MODE) {
            frequentCyclesRemaining = FREQUENT_MONITORING_CYCLES; // extend
            LOG_INFO("Extending FREQUENT_MONITORING cycles to %d", frequentCyclesRemaining);
        } else {
            monitoringMode = FREQUENT_MONITORING_MODE;
            frequentCyclesRemaining = FREQUENT_MONITORING_CYCLES;
            LOG_INFO("Entering FREQUENT_MONITORING mode for %d cycles", frequentCyclesRemaining);
        }
        // Ensure we stay awake and schedule next frequent sample
        sleepOnNextExecution = false;
        setIntervalFromNow(FREQUENT_MONITORING_CYCLE_SEC * 1000U);
    }
}

AdminMessageHandleResult AirQualityTelemetryModule::handleAdminMessageForModule(const meshtastic_MeshPacket &mp,
                                                                                meshtastic_AdminMessage *request,
                                                                                meshtastic_AdminMessage *response)
{
    AdminMessageHandleResult result = AdminMessageHandleResult::NOT_HANDLED;

    for (TelemetrySensor *sensor : sensors) {
        result = sensor->handleAdminMessage(mp, request, response);
        if (result != AdminMessageHandleResult::NOT_HANDLED)
            return result;
    }

    return result;
}

#endif
