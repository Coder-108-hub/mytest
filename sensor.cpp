//sensor.cpp
#include "sensor.h"

// Global variable definitions
volatile int AQI = 0;
volatile float temperature = 0.0;
volatile float humidity = 0.0;

SensirionI2CSen5x sen5x;
Sensor_t sensor_data;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String deviceName = "";

// MQTT Server Details
const char *mqtt_server = "";
const char *username = "";
const char *password = "";

// Chart series definitions
lv_chart_series_t *ui_PM1chart_series_1 = {0};
lv_coord_t ui_PM1chart_series_1_array[CHART_DATA_LENGTH] = {0};

lv_chart_series_t *ui_PM25chart_series_1 = {0};
lv_coord_t ui_PM25chart_series_1_array[CHART_DATA_LENGTH] = {0};

lv_chart_series_t *ui_PM4chart_series_1 = {0};
lv_coord_t ui_PM4chart_series_1_array[CHART_DATA_LENGTH] = {0};

lv_chart_series_t *ui_PM10chart_series_1 = {0};
lv_coord_t ui_PM10chart_series_1_array[CHART_DATA_LENGTH] = {0};

lv_chart_series_t *ui_TVOCchart_series_1 = {0};
lv_coord_t ui_TVOCchart_series_1_array[CHART_DATA_LENGTH] = {0};

// AQI Breakpoints definitions
AQIBreakpoint pm1Bps[] = {{0.0, 8.0, 0, 50},      {8.1, 25.4, 51, 100},
                          {25.5, 35.4, 101, 150}, {35.5, 50.4, 151, 200},
                          {50.5, 75.4, 201, 300}, {75.5, 500.4, 301, 500}};

AQIBreakpoint pm4Bps[] = {{0.0, 35.0, 0, 50},       {35.1, 75.4, 51, 100},
                          {75.5, 125.4, 101, 150},  {125.5, 175.4, 151, 200},
                          {175.5, 250.4, 201, 300}, {250.5, 500.4, 301, 500}};

AQIBreakpoint pm25Bps[] = {{0.0, 12.0, 0, 50},       {12.1, 35.4, 51, 100},
                           {35.5, 55.4, 101, 150},   {55.5, 150.4, 151, 200},
                           {150.5, 250.4, 201, 300}, {250.5, 500.4, 301, 500}};

AQIBreakpoint pm10Bps[] = {{0, 54, 0, 50},       {55, 154, 51, 100},
                           {155, 254, 101, 150}, {255, 354, 151, 200},
                           {355, 424, 201, 300}, {425, 604, 301, 500}};

AQIBreakpoint tvocBps[] = {{0.0, 300, 0, 50},      {300, 500, 51, 100},
                           {500, 1000, 101, 150},  {1000, 3000, 151, 200},
                           {4000, 5000, 201, 300}, {5000, 10000, 301, 500}};

// Function implementations
int calculateSubIndex(float Cp, AQIBreakpoint bp) {
    float Ip = ((bp.Ip_Hi - bp.Ip_Lo) / (bp.Cp_Hi - bp.Cp_Lo)) * (Cp - bp.Cp_Lo) + bp.Ip_Lo;
    return (int)round(Ip);
}

AQIBreakpoint getBreakpoint(float Cp, AQIBreakpoint bps[], int numBps) {
    for (int i = 0; i < numBps; i++) {
        if (Cp >= bps[i].Cp_Lo && Cp <= bps[i].Cp_Hi) {
            return bps[i];
        }
    }
    // Return the highest breakpoint if Cp exceeds the range
    return bps[numBps - 1];
}

String getAQICategory(int aqi) {
    if (aqi >= 0 && aqi <= 50)
        return "Good";
    else if (aqi <= 100)
        return "Satisfactory";
    else if (aqi <= 150)
        return "Moderate";
    else if (aqi <= 200)
        return "Unhealthy";
    else if (aqi <= 300)
        return "Very Unhealthy";
    else
        return "Hazardous";
}

uint32_t getAQIColor(int aqi) {
    if (aqi >= 0 && aqi <= 50)
        return 0x00E400; // Good
    else if (aqi <= 100)
        return 0x9CFF9C; // Satisfactory
    else if (aqi <= 150)
        return 0xFFFF00; // Moderate
    else if (aqi <= 200)
        return 0xFF7E00; // Unhealthy
    else if (aqi <= 300)
        return 0xFF0000; // Very Unhealthy
    else
        return 0x8F3F97; // Hazardous
}

void callback(char *topic, byte *payload, unsigned int length) {
    // handle message arrived
}

boolean reconnect() {
    if (!mqttClient.connected()) {
        String clientId = "AIROWL";
        clientId += String(random(0xffffff), HEX);
        if (mqttClient.connect(clientId.c_str(), username, password)) {
            // M5.Log.println("MQTT Connected");
        }
    }
    return mqttClient.connected();
}

void setupMQTT() {
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(callback);
}


SensorManager::SensorManager() : 
    temperature(0.0), humidity(0.0), pm1(0.0), pm25(0.0), pm4(0.0), pm10(0.0),
    tvoc(0), aqi(0), sensor_initialized(false), last_read_time(0),
    buffer_index(0), buffer_filled(false) {
    
    // Initialize moving average buffers
    for(int i = 0; i < MOVING_AVERAGE_WINDOW; i++) {
        temp_buffer[i] = 0.0;
        humidity_buffer[i] = 0.0;
    }
}

SensorManager::~SensorManager() {
    // Cleanup if needed
}

bool SensorManager::initialize() {
    Wire.begin(2, 1, 100000L);
    
    uint16_t error;
    char errorMessage[256];
    
    // Initialize SEN5x sensor
    sen5x.begin(Wire);
    
    error = sen5x.deviceReset();
    if (error) {
        errorToString(error, errorMessage, 256);
        return false;
    }
    
    // Set temperature offset
    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        errorToString(error, errorMessage, 256);
        return false;
    }
    
    // Start measurement
    error = sen5x.startMeasurement();
    if (error) {
        errorToString(error, errorMessage, 256);
        return false;
    }
    
    sensor_initialized = true;
    last_read_time = millis();
    return true;
}

void SensorManager::reset() {
    sensor_initialized = false;
    last_read_time = 0;
    buffer_index = 0;
    buffer_filled = false;
    
    // Reset all values
    temperature = humidity = pm1 = pm25 = pm4 = pm10 = 0.0;
    tvoc = aqi = 0;
    
    // Clear buffers
    for(int i = 0; i < MOVING_AVERAGE_WINDOW; i++) {
        temp_buffer[i] = 0.0;
        humidity_buffer[i] = 0.0;
    }
}

bool SensorManager::readSensors() {
    if (!sensor_initialized) {
        return false;
    }
    
    uint16_t error;
    char errorMessage[256];
    float t_pm1, t_pm25, t_pm4, t_pm10, t_hum, t_temp, vocIndex, noxIndex;
    
    error = sen5x.readMeasuredValues(t_pm1, t_pm25, t_pm4, t_pm10, 
                                     t_hum, t_temp, vocIndex, noxIndex);
    
    if (error || isnan(t_pm1) || isnan(t_pm25) || isnan(t_pm4) || 
        isnan(t_pm10) || isnan(vocIndex)) {
        return false;
    }
    
    // Validate readings
    if (!validateSensorReading(t_temp, -40.0, 85.0) ||
        !validateSensorReading(t_hum, 0.0, 100.0) ||
        !validateSensorReading(t_pm25, 0.0, 1000.0)) {
        return false;
    }
    
    // Update values with moving average
    temperature = calculateMovingAverage(temp_buffer, t_temp);
    humidity = calculateMovingAverage(humidity_buffer, t_hum);
    
    pm1 = t_pm1;
    pm25 = t_pm25;
    pm4 = t_pm4;
    pm10 = t_pm10;
    tvoc = (uint16_t)vocIndex;
    
    // Calculate AQI
    aqi = calculateAQI();
    
    last_read_time = millis();
    updateMovingAverageBuffers();
    
    return true;
}

bool SensorManager::readTemperatureHumidity() {
    // This is handled in readSensors() for SEN5x
    return readSensors();
}

bool SensorManager::readPM25PM10() {
    // This is handled in readSensors() for SEN5x
    return readSensors();
}

bool SensorManager::readTVOCCO2() {
    // This is handled in readSensors() for SEN5x
    return readSensors();
}

bool SensorManager::readBatteryLevel() {
    // Implement battery reading logic here if you have battery monitoring
    // For now, return true as placeholder
    return true;
}

SensorData SensorManager::getCurrentData() {
    SensorData data;
    data.temperature = temperature;
    data.humidity = humidity;
    data.pm1 = pm1;
    data.pm25 = pm25;
    data.pm4 = pm4;
    data.pm10 = pm10;
    data.tvoc = tvoc;
    // data.co2 = 0; // Not available in SEN5x
    // data.battery_level = 0.0; // Implement if needed
    data.timestamp = last_read_time;
    return data;
}

bool SensorManager::isDataValid() const {
    uint32_t current_time = millis();
    // Consider data valid if it's less than 10 seconds old
    return sensor_initialized && 
           (current_time - last_read_time) < 10000 &&
           !isnan(temperature) && !isnan(humidity);
}

void SensorManager::calibrateTemperature(float offset) {
    uint16_t error = sen5x.setTemperatureOffsetSimple(offset);
    if (!error) {
        // Calibration successful
    }
}

// void SensorManager::calibrateHumidity(float offset) {
//     // SEN5x doesn't have separate humidity offset
//     // You could implement software calibration here
//     // For now, this is a placeholder
// }

void SensorManager::calibratePM(float factor) {
    // Software calibration - multiply PM readings by factor
    pm1 *= factor;
    pm25 *= factor;
    pm4 *= factor;
    pm10 *= factor;
}

uint8_t SensorManager::calculateAQI() {
    // Calculate AQI for each pollutant
    AQIBreakpoint pm25Bp = getBreakpoint(pm25, pm25Bps, sizeof(pm25Bps) / sizeof(pm25Bps[0]));
    AQIBreakpoint pm10Bp = getBreakpoint(pm10, pm10Bps, sizeof(pm10Bps) / sizeof(pm10Bps[0]));
    AQIBreakpoint tvocBp = getBreakpoint(tvoc, tvocBps, sizeof(tvocBps) / sizeof(tvocBps[0]));
    AQIBreakpoint pm1Bp = getBreakpoint(pm1, pm1Bps, sizeof(pm1Bps) / sizeof(pm1Bps[0]));
    AQIBreakpoint pm4Bp = getBreakpoint(pm4, pm4Bps, sizeof(pm4Bps) / sizeof(pm4Bps[0]));
    
    // Calculate sub-indices
    int pm25Index = calculateSubIndex(pm25, pm25Bp);
    int pm10Index = calculateSubIndex(pm10, pm10Bp);
    int tvocIndex = calculateSubIndex(tvoc, tvocBp);
    int pm1Index = calculateSubIndex(pm1, pm1Bp);
    int pm4Index = calculateSubIndex(pm4, pm4Bp);
    
    // Return maximum sub-index as overall AQI
    int maxAQI = max(max(pm25Index, pm10Index), max(tvocIndex, max(pm1Index, pm4Index)));
    return (uint8_t)maxAQI;
}

void SensorManager::printSensorData() {
    // Implement serial printing for debugging
    Serial.println("=== Sensor Data ===");
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
    Serial.print("PM1.0: "); Serial.print(pm1); Serial.println(" µg/m³");
    Serial.print("PM2.5: "); Serial.print(pm25); Serial.println(" µg/m³");
    Serial.print("PM4.0: "); Serial.print(pm4); Serial.println(" µg/m³");
    Serial.print("PM10: "); Serial.print(pm10); Serial.println(" µg/m³");
    Serial.print("TVOC: "); Serial.print(tvoc); Serial.println(" index");
    Serial.print("AQI: "); Serial.println(aqi);
    Serial.println("==================");
}

String SensorManager::getSensorDataJSON() {
    String json = "{";
    json += "\"temperature\":" + String(temperature, 2) + ",";
    json += "\"humidity\":" + String(humidity, 2) + ",";
    json += "\"pm1\":" + String(pm1, 2) + ",";
    json += "\"pm25\":" + String(pm25, 2) + ",";
    json += "\"pm4\":" + String(pm4, 2) + ",";
    json += "\"pm10\":" + String(pm10, 2) + ",";
    json += "\"tvoc\":" + String(tvoc) + ",";
    json += "\"aqi\":" + String(aqi) + ",";
    json += "\"timestamp\":" + String(last_read_time);
    json += "}";
    return json;
}

// Private helper functions
float SensorManager::calculateMovingAverage(float* buffer, float new_value) {
    buffer[buffer_index] = new_value;
    
    float sum = 0.0;
    int count = buffer_filled ? MOVING_AVERAGE_WINDOW : (buffer_index + 1);
    
    for(int i = 0; i < count; i++) {
        sum += buffer[i];
    }
    
    return sum / count;
}

void SensorManager::updateMovingAverageBuffers() {
    buffer_index = (buffer_index + 1) % MOVING_AVERAGE_WINDOW;
    if (buffer_index == 0) {
        buffer_filled = true;
    }
}

bool SensorManager::validateSensorReading(float value, float min_val, float max_val) {
    return !isnan(value) && !isinf(value) && value >= min_val && value <= max_val;
}

uint8_t SensorManager::calculatePMAQI(float pm25_value) {
    AQIBreakpoint bp = getBreakpoint(pm25_value, pm25Bps, sizeof(pm25Bps) / sizeof(pm25Bps[0]));
    return (uint8_t)calculateSubIndex(pm25_value, bp);
}

// Hardware-specific sensor reading functions (placeholders)
bool SensorManager::initializeI2CSensors() {
    // This is handled in initialize() for SEN5x
    return initialize();
}

bool SensorManager::readSHT30() {
    // SEN5x has integrated temperature/humidity sensor
    return readSensors();
}

bool SensorManager::readPMS5003() {
    // SEN5x has integrated PM sensor
    return readSensors();
}

bool SensorManager::readSGP30() {
    // SEN5x has integrated VOC sensor
    return readSensors();
}

// bool SensorManager::readAXP2101() {
//     // Implement battery level reading if you have AXP2101 power management IC
//     // For now, return true as placeholder
//     return true;
// }


void setupCharts() {
    ui_PM1chart_series_1 = lv_chart_add_series(
        ui_PM1chart, lv_color_hex(0x41b4d1), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(ui_PM1chart, ui_PM1chart_series_1,
                             ui_PM1chart_series_1_array);
    lv_chart_set_range(ui_PM1chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);

    ui_PM25chart_series_1 = lv_chart_add_series(
        ui_PM25chart, lv_color_hex(0x41b4d1), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(ui_PM25chart, ui_PM25chart_series_1,
                             ui_PM25chart_series_1_array);
    lv_chart_set_range(ui_PM25chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);

    ui_PM10chart_series_1 = lv_chart_add_series(
        ui_PM10chart, lv_color_hex(0x41b4d1), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(ui_PM10chart, ui_PM10chart_series_1,
                             ui_PM10chart_series_1_array);
    lv_chart_set_range(ui_PM10chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);

    ui_PM4chart_series_1 = lv_chart_add_series(
        ui_PM4chart, lv_color_hex(0x41b4d1), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(ui_PM4chart, ui_PM4chart_series_1,
                             ui_PM4chart_series_1_array);
    lv_chart_set_range(ui_PM4chart, LV_CHART_AXIS_PRIMARY_Y, 0, 50);

    ui_TVOCchart_series_1 = lv_chart_add_series(
        ui_TVOCchart, lv_color_hex(0x41b4d1), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_ext_y_array(ui_TVOCchart, ui_TVOCchart_series_1,
                             ui_TVOCchart_series_1_array);
    lv_chart_set_range(ui_TVOCchart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
}

void sensorData(void *params) {
    Wire.begin(2, 1, 100000L);
    sen5x.begin(Wire);

    uint16_t error;
    char errorMessage[256];
    error = sen5x.deviceReset();
    if (error) {
        // M5.Log.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        // M5.Log.println(errorMessage);
    }

    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        // M5.Log.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        // M5.Log.println(errorMessage);
    } else {
        // M5.Log.print("Temperature Offset set to ");
        // M5.Log.println(" deg. Celsius (SEN54/SEN55 only");
    }

    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        // M5.Log.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        // M5.Log.println(errorMessage);
    }

    setupCharts();
    setupMQTT();
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    deviceName = "AIROWL_" + mac.substring(6);

    while (1) {
        uint16_t error;
        char errorMessage[256];

        // Read Measurement
        float t_pm1;
        float t_pm25;
        float t_pm4;
        float t_pm10; 
        float t_hum;
        float t_temp;
        float vocIndex;
        float noxIndex;

        error = sen5x.readMeasuredValues(t_pm1, t_pm25, t_pm4, t_pm10, t_hum,
                                         t_temp, vocIndex, noxIndex);

        if (error || isnan(t_pm1) || isnan(t_pm25) || isnan(t_pm4) ||
            isnan(t_pm10) || isnan(vocIndex)) {
            // M5.Log.print("Error trying to execute readMeasuredValues(): ");
            errorToString(error, errorMessage, 256);
            // M5.Log.println(errorMessage);
        } else {
            sensor_data.pm1 += t_pm1;
            sensor_data.pm25 += t_pm25;
            sensor_data.pm10 += t_pm10;
            sensor_data.pm4 += t_pm4;
            sensor_data.count++;

            char pm1buffer[6] = {0};
            dtostrf(t_pm1, 6, 1, pm1buffer);
            lv_label_set_text(ui_pm1label, pm1buffer);

            char pm25buffer[6] = {0};
            dtostrf(t_pm25, 6, 1, pm25buffer);
            lv_label_set_text(ui_pm25label, pm25buffer);

            char pm4buffer[6] = {0};
            dtostrf(t_pm4, 6, 1, pm4buffer);
            lv_label_set_text(ui_pm4label, pm4buffer);

            char pm10buffer[6] = {0};
            dtostrf(t_pm10, 6, 1, pm10buffer);
            lv_label_set_text(ui_pm10label, pm10buffer);

            char tvocbuffer[6] = {0};
            if (isnan(vocIndex)) {
                // M5.Log.println("n/a");
            } else {
                sensor_data.tvoc += vocIndex;
                dtostrf(vocIndex, 6, 1, tvocbuffer);
                lv_label_set_text(ui_tvoclabel, tvocbuffer);
            }

            char humbuffer[4] = {0};
            if (isnan(t_hum)) {
                // Humidity n/a handling (unchanged)
            } else {
                dtostrf(t_hum, 4, 1, humbuffer);
                lv_label_set_text(ui_RHlabel, humbuffer);
                humidity = t_hum; // Update global humidity variable
            }

            char tempbuffer[4] = {0};
            if (isnan(t_temp)) {
                // Temperature n/a handling (unchanged)
            } else {
                dtostrf(t_temp, 4, 1, tempbuffer);
                lv_label_set_text(ui_templabel, tempbuffer);
                temperature = t_temp; // Update global temperature variable
            }

            if (sensor_data.count == DATA_FREQ) {
                float avgPM1 = (sensor_data.pm1 / sensor_data.count);
                float avgPM25 = (sensor_data.pm25 / sensor_data.count);
                float avgPM10 = (sensor_data.pm10 / sensor_data.count);
                float avgPM4 = (sensor_data.pm4 / sensor_data.count);
                float avgTVOC = (sensor_data.tvoc / sensor_data.count);

                AQIBreakpoint pm25Bp = getBreakpoint(
                    avgPM25, pm25Bps, sizeof(pm25Bps) / sizeof(pm25Bps[0]));
                AQIBreakpoint pm10Bp = getBreakpoint(
                    avgPM10, pm10Bps, sizeof(pm10Bps) / sizeof(pm10Bps[0]));
                AQIBreakpoint tvocBp = getBreakpoint(
                    avgTVOC, tvocBps, sizeof(tvocBps) / sizeof(tvocBps[0]));
                AQIBreakpoint pm1Bp = getBreakpoint(
                    avgPM1, pm1Bps, sizeof(pm1Bps) / sizeof(pm1Bps[0]));
                AQIBreakpoint pm4Bp = getBreakpoint(
                    avgPM4, pm4Bps, sizeof(pm4Bps) / sizeof(pm4Bps[0]));

                // Calculate sub-indices
                int pm25Index = calculateSubIndex(avgPM25, pm25Bp);
                int pm10Index = calculateSubIndex(avgPM10, pm10Bp);
                int tvocIndex = calculateSubIndex(avgTVOC, tvocBp);
                int pm1Index = calculateSubIndex(avgPM1, pm1Bp);
                int pm4Index = calculateSubIndex(avgPM4, pm4Bp);

                uint32_t pm25_color = getAQIColor(pm25Index);
                lv_obj_set_style_text_color(ui_pm25label,
                                            lv_color_hex(pm25_color),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);

                uint32_t pm10_color = getAQIColor(pm10Index);
                lv_obj_set_style_text_color(ui_pm10label,
                                            lv_color_hex(pm10_color),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);

                uint32_t tvoc_color = getAQIColor(tvocIndex);
                lv_obj_set_style_text_color(ui_tvoclabel,
                                            lv_color_hex(tvoc_color),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);

                uint32_t pm1_color = getAQIColor(pm1Index);
                lv_obj_set_style_text_color(ui_pm1label,
                                            lv_color_hex(pm1_color),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);

                uint32_t pm4_color = getAQIColor(pm4Index);
                lv_obj_set_style_text_color(ui_pm4label,
                                            lv_color_hex(pm4_color),
                                            LV_PART_MAIN | LV_STATE_DEFAULT);

                // Combine sub-indices (choose one method)
                // Method 1: Maximum Sub-Index
                int aqi = max(max(pm25Index, pm10Index), tvocIndex);
                AQI = aqi;
                // Method 2: Weighted Average
                // float aqi = (pm25Index * 0.5) + (pm10Index * 0.3) +
                // (tvocIndex * 0.2); aqi = round(aqi);

                // Get AQI category
                String airQualityCategory = getAQICategory(aqi);
                uint32_t eye_color = getAQIColor(aqi);
                lv_obj_set_style_bg_color(ui_lefteye, lv_color_hex(eye_color),
                                          LV_PART_MAIN | LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(ui_righteye, lv_color_hex(eye_color),
                                          LV_PART_MAIN | LV_STATE_DEFAULT);

                // Set PM1 Chart Screen
                char pm1avg[6] = {0};
                char pm1max[6] = {0};
                dtostrf(avgPM1, 6, 1, pm1avg);
                lv_label_set_text(ui_pm1avg, pm1avg);
                // shift the other values to the left
                memcpy(ui_PM1chart_series_1_array,
                       ui_PM1chart_series_1_array + 1,
                       (CHART_DATA_LENGTH - 1) * sizeof(lv_coord_t));
                if (sensor_data.pm1_max < avgPM1) {
                    sensor_data.pm1_max = int(avgPM1);
                    if (sensor_data.pm1_max > 50)
                        lv_chart_set_range(ui_PM1chart, LV_CHART_AXIS_PRIMARY_Y,
                                           0, sensor_data.pm1_max + 20);
                }
                dtostrf(sensor_data.pm1_max, 6, 1, pm1max);
                lv_label_set_text(ui_pm1max, pm1max);
                // Insert new value
                ui_PM1chart_series_1_array[CHART_DATA_LENGTH - 1] =
                    (uint16_t)(avgPM1);
                lv_chart_set_ext_y_array(ui_PM1chart, ui_PM1chart_series_1,
                                         ui_PM1chart_series_1_array);

                // Set PM2.5 Chart Screens
                char pm25avg[6] = {0};
                char pm25max[6] = {0};
                dtostrf(avgPM25, 6, 1, pm25avg);
                lv_label_set_text(ui_pm25avg, pm25avg);
                // shift the other values to the left
                memcpy(ui_PM25chart_series_1_array,
                       ui_PM25chart_series_1_array + 1,
                       (CHART_DATA_LENGTH - 1) * sizeof(lv_coord_t));
                if (sensor_data.pm25_max < (avgPM25)) {
                    sensor_data.pm25_max = int(avgPM25);
                    if (sensor_data.pm25_max > 50)
                        lv_chart_set_range(ui_PM25chart,
                                           LV_CHART_AXIS_PRIMARY_Y, 0,
                                           sensor_data.pm25_max + 20);
                }
                dtostrf(sensor_data.pm25_max, 6, 1, pm25max);
                lv_label_set_text(ui_pm25max, pm25max);
                // Insert new value
                ui_PM25chart_series_1_array[CHART_DATA_LENGTH - 1] =
                    (uint16_t)(avgPM25);
                lv_chart_set_ext_y_array(ui_PM25chart, ui_PM25chart_series_1,
                                         ui_PM25chart_series_1_array);

                // Set PM10 Chart Screens
                char pm10avg[6] = {0};
                char pm10max[6] = {0};
                dtostrf(avgPM10, 6, 1, pm10avg);
                lv_label_set_text(ui_pm10avg, pm10avg);
                // shift the other values to the left
                memcpy(ui_PM10chart_series_1_array,
                       ui_PM10chart_series_1_array + 1,
                       (CHART_DATA_LENGTH - 1) * sizeof(lv_coord_t));
                if (sensor_data.pm10_max < (avgPM10)) {
                    sensor_data.pm10_max = int(avgPM10);
                    if (sensor_data.pm10_max > 50)
                        lv_chart_set_range(ui_PM10chart,
                                           LV_CHART_AXIS_PRIMARY_Y, 0,
                                           sensor_data.pm10_max + 20);
                }
                dtostrf(sensor_data.pm10_max, 6, 1, pm10max);
                lv_label_set_text(ui_pm10max, pm10max);
                // Insert new value
                ui_PM10chart_series_1_array[CHART_DATA_LENGTH - 1] =
                    (uint16_t)(avgPM10);
                lv_chart_set_ext_y_array(ui_PM10chart, ui_PM10chart_series_1,
                                         ui_PM10chart_series_1_array);

                // Set PM4 Chart Screens
                char pm4avg[6] = {0};
                char pm4max[6] = {0};
                dtostrf(avgPM4, 6, 1, pm4avg);
                lv_label_set_text(ui_pm4max, pm4max);
                // shift the other values to the left
                memcpy(ui_PM4chart_series_1_array,
                       ui_PM4chart_series_1_array + 1,
                       (CHART_DATA_LENGTH - 1) * sizeof(lv_coord_t));
                if (sensor_data.pm4_max < (avgPM4)) {
                    sensor_data.pm4_max = int(avgPM4);
                    if (sensor_data.pm4_max > 50)
                        lv_chart_set_range(ui_PM4chart, LV_CHART_AXIS_PRIMARY_Y,
                                           0, sensor_data.pm4_max + 20);
                }
                lv_label_set_text(ui_pm4avg, pm4avg);
                dtostrf(sensor_data.pm4_max, 6, 1, pm4max);
                // Insert new value
                ui_PM4chart_series_1_array[CHART_DATA_LENGTH - 1] =
                    (uint16_t)(avgPM4);
                lv_chart_set_ext_y_array(ui_PM4chart, ui_PM4chart_series_1,
                                         ui_PM4chart_series_1_array);

                // Set TVOC Chart Screens
                char tvocavg[6] = {0};
                char tvocmax[6] = {0};
                dtostrf(avgTVOC, 6, 1, tvocavg);
                lv_label_set_text(ui_tvocavg, tvocavg);
                // shift the other values to the left
                memcpy(ui_TVOCchart_series_1_array,
                       ui_TVOCchart_series_1_array + 1,
                       (CHART_DATA_LENGTH - 1) * sizeof(lv_coord_t));
                if (sensor_data.tvoc_max < (avgTVOC)) {
                    sensor_data.tvoc_max = int(avgTVOC);
                    if (sensor_data.tvoc_max > 100)
                        lv_chart_set_range(ui_TVOCchart,
                                           LV_CHART_AXIS_PRIMARY_Y, 0,
                                           sensor_data.tvoc_max + 20);
                }
                dtostrf(sensor_data.tvoc_max, 6, 1, tvocmax);
                lv_label_set_text(ui_tvocmax, tvocmax);
                // Insert new value
                ui_TVOCchart_series_1_array[CHART_DATA_LENGTH - 1] =
                    (uint16_t)(avgTVOC);
                lv_chart_set_ext_y_array(ui_TVOCchart, ui_TVOCchart_series_1,
                                         ui_TVOCchart_series_1_array);

                if (WiFi.status() == WL_CONNECTED) {
                    lv_img_set_src(ui_nose, &ui_img_airowl_2_png);
                    if (!mqttClient.connected()) {
                        reconnect();
                    }
                    // Construct the JSON string
                    String jsonString = "{";
                    jsonString += "\"deviceId\":\"";
                    jsonString += deviceName;
                    jsonString += "\",";
                    jsonString += "\"p3\":";
                    jsonString += String(avgPM1, 2);
                    jsonString += ",";
                    jsonString += "\"p1\":";
                    jsonString += String(avgPM25, 2);
                    jsonString += ",";
                    jsonString += "\"p2\":";
                    jsonString += String(avgPM10, 2);
                    jsonString += ",";
                    jsonString += "\"p5\":";
                    jsonString += String(avgPM4, 2);
                    jsonString += ",";
                    jsonString += "\"v2\":";
                    jsonString += String(avgTVOC, 2);
                    jsonString += "}";
                    mqttClient.publish("airowl", jsonString.c_str());
                    mqttClient.loop();
                } else {
                    lv_img_set_src(ui_nose, &ui_img_airowl_1_png);
                }
                sensor_data.pm1 = 0;
                sensor_data.pm10 = 0;
                sensor_data.pm25 = 0;
                sensor_data.pm4 = 0;
                sensor_data.tvoc = 0;
                sensor_data.count = 0;
            }
        }
        esp_task_wdt_reset(); // Reset watchdog for this task
        delay(2000);
    }
}