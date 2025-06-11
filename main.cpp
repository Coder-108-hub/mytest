//main.cpp
#include <Arduino.h>
#if defined(ARDUINO_M5STACK_Core2)
#include <M5Core2.h>
#endif
#if defined(ARDUINO_M5STACK_CORES3)
#include <M5Unified.h>
#endif
#include <esp_task_wdt.h>
#include <WiFiManager.h>
#include <WiFiManagerTz.h>
#include "Sensor.h"
#include "ui/lv_setup.h"
#include "ui/ui.h"
#include "config.h"
#include "time_func.h"
#include <FS.h>
#include <Matter.h>
#include <MatterEndpoints/MatterAirQualitySensor.h>
#include <MatterEndpoints/MatterTemperatureSensor.h>
#include <MatterEndpoints/MatterHumiditySensor.h>
#include <Preferences.h>
#include <esp_now.h>
// #include <WiFi.h>
#include "cluster_manager.h"
#include "cluster_types.h"
#include "esp_now_manager.h"

// Constants
#define WDT_TIMEOUT 60
#define FIRMWARE_VERSION "version - 1.1"

// Global objects
ClusterManager cluster_manager;
ESPNowManager* espnow_manager;
SensorManager sensor_manager;
WiFiManager wm;
Preferences preferences;
MatterAirQualitySensor air_quality_sensor;
MatterTemperatureSensor temperature_sensor;
MatterHumiditySensor humidity_sensor;

// Task handles
TaskHandle_t cluster_task_handle;
TaskHandle_t sensor_task_handle;
TaskHandle_t wifi_task_handle;
TaskHandle_t ui_task_handle;

// Global state variables
static bool matter_initialized = false;
static bool was_commissioned = false;

// Function prototypes
void clusterTask(void* parameter);
void sensorTask(void* parameter);
void wifiTask(void* parameter);
void uiTask(void* parameter);
void setupDisplay();
void updateDisplay();
void handleESPNowMessage(const uint8_t* mac_addr, const uint8_t* data, int len);

// NTP callback
void on_time_available(struct timeval *t) {
    Serial.println("Received time adjustment from NTP");
    struct tm timeInfo;
    getLocalTime(&timeInfo, 1000);
    Serial.println(&timeInfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
    M5.Rtc.setDateTime(&timeInfo);
}

void setup() {
    Serial.begin(115200);
    Serial.println("===== SETUP STARTED =====");

    M5.begin();
    M5.Display.setRotation(3);
    
    // Initialize watchdog timer
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    // Initialize preferences and determine device role
    preferences.begin("MatterPrefs", false);
    preferences.end();

    // Setup display
    // setupDisplay();

    // Initialize managers
    if (!sensor_manager.initialize()) {
        Serial.println("Failed to initialize sensors");
        return;
    }

    // Initialize ESP-NOW and cluster manager
    espnow_manager = ESPNowManager::getInstance();
    if (!espnow_manager->initialize()) {
        Serial.println("Failed to initialize ESP-NOW");
        return;
    }
    espnow_manager->setMessageCallback(handleESPNowMessage);

    if (!cluster_manager.initialize()) {
        Serial.println("Failed to initialize cluster manager");
        return;
    }

    // WiFi and network setup
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    
    String mac = WiFi.macAddress();
    mac.replace(":", "");
    String apName = "AIROWL_" + mac.substring(6);

    if (cluster_manager.isMaster()) {
        // Master WiFi setup
        WiFiManagerNS::NTP::onTimeAvailable(&on_time_available);
        WiFiManagerNS::init(&wm, nullptr);
        std::vector<const char *> menu = {"wifi", "info", "custom", "param", "sep", "restart", "exit"};
        wm.setMenu(menu);
        wm.setConfigPortalBlocking(false);
        wm.setTitle("AIROWL Master Configuration");
        wm.autoConnect(apName.c_str(), "");
    } else {
        // Slave WiFi setup (optional)
        WiFiManagerNS::init(&wm, nullptr);
        wm.setConfigPortalBlocking(false);
        wm.setTitle("AIROWL Slave Configuration");
        wm.autoConnect(apName.c_str(), "");
    }

    // Initialize UI
    lv_begin();
    ui_init();
    lv_label_set_text(ui_devicename, apName.c_str());
    lv_label_set_text(ui_qrcodename, apName.c_str());
    lv_label_set_text(ui_firmwareversion, FIRMWARE_VERSION);

    String qrcodeurl = (WiFi.status() == WL_CONNECTED) ? 
        "https://opendata.oizom.com/device/" + apName : 
        "WIFI:T:WPA;S:" + apName + ";P:12345678;;";
    ui_qrcodedata = qrcodeurl.c_str();
    lv_qrcode_update(ui_qrcode_obj, ui_qrcodedata, strlen(ui_qrcodedata));
    lv_obj_center(ui_qrcode_obj);

    time_init();

    // Create tasks
    xTaskCreatePinnedToCore(clusterTask, "ClusterTask", 8192, NULL, 2, &cluster_task_handle, 1);
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, &sensor_task_handle, 0);
    xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 8192, NULL, 1, &wifi_task_handle, 0);
    xTaskCreatePinnedToCore(uiTask, "UITask", 4096, NULL, 1, &ui_task_handle, 1);

    esp_task_wdt_add(cluster_task_handle);
    esp_task_wdt_add(sensor_task_handle);
    esp_task_wdt_add(wifi_task_handle);
    esp_task_wdt_add(ui_task_handle);

    Serial.println("===== SETUP COMPLETED =====");
}

void loop() {
    M5.update();
      // Check for long press on Button A to trigger master candidacy
    if (M5.BtnPWR.pressedFor(MASTER_BUTTON_PRESS_TIME)) {
        cluster_manager.handleButtonPress();
    }
    wm.process();
    esp_task_wdt_reset();
    delay(5);
}

// Cluster management task
void clusterTask(void* parameter) {
    while (true) {
        cluster_manager.updateState();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Sensor reading and processing task
void sensorTask(void* parameter) {
    while (true) {
        if (sensor_manager.readSensors()) {
            // Update Matter sensors if initialized
            if (matter_initialized) {
                air_quality_sensor.setAQI(sensor_manager.getAQI());
                temperature_sensor.setTemperature(sensor_manager.getTemperature());
                humidity_sensor.setHumidity(sensor_manager.getHumidity());
            }
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// WiFi, MQTT and Matter management task
void wifiTask(void* parameter) {
    TickType_t lastMqttPublish = 0;
    
    while (true) {
        // Process WiFi Manager
        wm.process();

        // Initialize Matter if WiFi is connected
        if (!matter_initialized && WiFi.status() == WL_CONNECTED) {
            Serial.println("Starting Matter Setup...");
            air_quality_sensor.begin(sensor_manager.getAQI());
            temperature_sensor.begin(sensor_manager.getTemperature());
            humidity_sensor.begin(sensor_manager.getHumidity());
            ArduinoMatter::begin();
            matter_initialized = true;
            Serial.println("Matter initialized");
        }

        // Handle Matter commissioning
        if (matter_initialized) {
            if (!was_commissioned && ArduinoMatter::isDeviceCommissioned()) {
                Serial.println("Matter Node commissioned");
                was_commissioned = true;
            }
        }

        // MQTT Publishing (Master only)
        if (cluster_manager.isMaster() && 
            (xTaskGetTickCount() - lastMqttPublish) >= pdMS_TO_TICKS(MQTT_PUBLISH_INTERVAL)) {
            cluster_manager.publishMQTTData();
            lastMqttPublish = xTaskGetTickCount();
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// UI update task
void uiTask(void* parameter) {
    while (true) {
        lv_handler();
        updateDisplay();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// // Display setup
// void setupDisplay() {
//     M5.Display.clear();
//     M5.Display.setCursor(0, 0);
//     M5.Display.println("AirOwl Cluster");
//     M5.Display.println("Network System");
//     M5.Display.println(FIRMWARE_VERSION);
// }

// Display update function
void updateDisplay() {
    static uint32_t last_update = 0;
    if (millis() - last_update < 2000) return;

    // Update display with current state and sensor data
    // This will be updated based on the LVGL UI requirements

    last_update = millis();
}

// ESP-NOW message handler
void handleESPNowMessage(const uint8_t* mac_addr, const uint8_t* data, int len) {
    cluster_manager.handleESPNowMessage(mac_addr, data, len);
}
