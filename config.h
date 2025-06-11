#ifndef __CONFIG_H
#define __CONFIG_H

#define DEMO_VERSION "0.6"

#define GC0308_ADDR  0x21
#define LTR553_ADDR  0x23
#define AXP2101_ADDR 0x34
#define AW88298_ADDR 0x36
#define FT6336_ADDR  0x38
#define ES7210_ADDR  0x40
#define BM8563_ADDR  0x51
#define AW9523_ADDR  0x58
#define BMI270_ADDR  0x69
#define BMM150_ADDR  0x10

// Device Configuration
#define DEVICE_ID_FLASH_ADDR 0x1000
#define ZONE_ID_FLASH_ADDR 0x1004
#define WIFI_CAPABILITY_ADDR 0x1008

// Cluster Configuration
#define DEFAULT_ZONE_ID 1
#define MASTER_BUTTON_PRESS_TIME 3000  // 3 seconds
#define CLUSTER_DISCOVERY_TIMEOUT 300000  // 5 minutes
#define SLAVE_DATA_TIMEOUT 30000  // 30 seconds
#define HEARTBEAT_INTERVAL 5000  // 5 seconds

#define MAX_DIRECT_SLAVES 5      // Reduced from 10 to allow hop masters
#define MAX_HOP_SLAVES 8         // Slaves per hop master
#define MAX_HOP_MASTERS 3        // Number of hop masters per main master
#define MAX_HOP_SLAVES_PER_MASTER 8
#define MAX_NETWORK_LEVELS 3
#define ESP_NOW_MAX_DATA_LEN 250 // ESP-NOW message limit

// Network Configuration
#define MAX_SLAVES_PER_MASTER 10
#define ESP_NOW_CHANNEL 1
#define DISCOVERY_INTERVAL_MS 60000
#define DATA_REPORT_INTERVAL_MS 30000
#define MQTT_PUBLISH_INTERVAL 30000
// #define DATA_AGGREGATION_INTERVAL_MS 60000
#define MASTER_TIMEOUT_MS 30000

// ESP-NOW Configuration
#define ESP_NOW_MAX_PEERS 20
#define ESP_NOW_BROADCAST_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// MQTT Configuration (update with your values)
#define MQTT_BROKER ""
#define MQTT_PORT 1883
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_TOPIC_PREFIX ""

// Button Configuration
#define POWER_BUTTON_PIN 41  // M5Stack Core S3 SE power button
#define MASTER_SELECTION_HOLD_TIME 3000

// Device States
enum DeviceState {
    STATE_INITIALIZING,
    STATE_DISCOVERY,
    STATE_MASTER_CANDIDATE,
    STATE_MASTER_ACTIVE,
    STATE_SLAVE_SEARCHING,
    STATE_SLAVE_ACTIVE,
    STATE_ERROR
};

// Message Types
enum MessageType {
    MSG_DISCOVERY_ANNOUNCE,
    MSG_MASTER_CANDIDACY,
    MSG_SLAVE_REGISTRATION_REQUEST,
    MSG_SLAVE_REGISTRATION_RESPONSE,
    MSG_SENSOR_DATA,
    MSG_HOP_SENSOR_DATA,
    MSG_HOP_REGISTRATION_REQUEST,
    MSG_HEALTH_CHECK,
    MSG_CLUSTER_INFO,
    MSG_HEARTBEAT // Heartbeat/keepalive from master
};

// Data Reporting Intervals
#define SENSOR_READ_INTERVAL 5000  // 5 seconds
#define DATA_REPORT_INTERVAL 30000  // 30 seconds
// #define DATA_AGGREGATION_INTERVAL 60000  // 60 seconds

// ESP-NOW Configuration
#define ESPNOW_DATA_RATE WIFI_PHY_RATE_18M
#define ESPNOW_POWER WIFI_POWER_19_5dBm
#define ESPNOW_MAX_RETRY 3

// Sensor Configuration
#define TEMP_OFFSET 0.0f
#define HUMIDITY_OFFSET 0.0f
#define PM_CALIBRATION_FACTOR 1.0f
#define TVOC_BASELINE 0
#define MOVING_AVERAGE_WINDOW 5

// Battery Configuration
#define BATTERY_CHECK_INTERVAL 60000  // 1 minute
#define LOW_BATTERY_THRESHOLD 20  // 20%
#define CRITICAL_BATTERY_THRESHOLD 10  // 10%

#define SYS_I2C_PORT 0
#define SYS_I2C_SDA  12
#define SYS_I2C_SCL  11

#define EXT_I2C_PORT 0

#define PORTA_PIN_0  1
#define PORTA_PIN_1  2
#define PORTB_PIN_0  8
#define PORTB_PIN_1  9
#define PORTC_PIN_0  18
#define PORTC_PIN_1  17

#define POWER_MODE_USB_IN_BUS_IN 0
#define POWER_MODE_USB_IN_BUS_OUT 1
#define POWER_MODE_USB_OUT_BUS_IN 2
#define POWER_MODE_USB_OUT_BUS_OUT 3

#define MIC_BUF_SIZE 256

#define MONKEY_TEST_ENABLE 0

#endif  // __CONFIG_H
