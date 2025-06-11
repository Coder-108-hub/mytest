#ifndef CLUSTER_TYPES_H
#define CLUSTER_TYPES_H

#include "config.h"
#include <Arduino.h>
// #include <WiFi.h>
#include <ArduinoJson.h>

// ESP-NOW Message Structure
struct ESPNowMessage {
    MessageType msg_type;
    uint32_t sender_id;
    uint8_t zone_id;
    uint8_t hop_level;      // NEW: 0=WiFi Master, 1=Direct/Hop Master, 2=Hop Slave
    uint8_t hop_master_id;  // NEW: ID of hop master (0 for direct slaves)
    uint16_t sequence;
    uint8_t payload[195];   // REDUCED: Account for new fields
};

// Device Information Structure
struct DeviceInfo {
    uint32_t device_id;
    uint8_t zone_id;
    uint8_t device_mac[6];
    uint32_t last_seen;
    bool is_online;
    device_role_t role;           // NEW: Device role in hierarchy
    device_level_t level;         // NEW: Device level (0-2)
    uint8_t hop_master_id;        // NEW: Which hop master this device belongs to
    bool can_be_hop_master;       // NEW: Device capability flag
    // uint8_t battery_level;        // NEW: Battery status
};

// Master Info Structure
struct MasterInfo {
    uint8_t master_mac[6];
    uint8_t zone_id;
    device_level_t level;         // NEW: Master level in hierarchy
    uint8_t hop_master_id;        // NEW: Hop master ID (0 for WiFi master)
    uint8_t available_slots;      // NEW: Available slave slots
};

// Sensor Data Structure
struct SensorData {
    uint8_t zone_id;
    uint8_t slave_id;
    uint8_t hop_master_id;        // NEW: For routing through hop masters
    device_level_t source_level;  // NEW: Source device level
    float temperature;
    float humidity;
    float pm1;
    float pm25;
    float pm4;
    float pm10;
    float tvoc;
    uint16_t air_quality_index;
    uint32_t timestamp;
    uint8_t hop_count;            // NEW: Number of hops to reach WiFi master
    // uint8_t battery_level;        // NEW: Source device battery

    String toJson() const {
        StaticJsonDocument<384> doc;
        doc["zone_id"] = zone_id;
        doc["slave_id"] = slave_id;
        doc["hop_master_id"] = hop_master_id;
        doc["source_level"] = source_level;
        doc["temperature"] = temperature;
        doc["humidity"] = humidity;
        doc["pm1"] = pm1;
        doc["pm25"] = pm25;
        doc["pm4"] = pm4;
        doc["pm10"] = pm10;
        doc["tvoc"] = tvoc;
        doc["aqi"] = air_quality_index;
        doc["timestamp"] = timestamp;
        doc["hop_count"] = hop_count;
        // doc["battery"] = battery_level;
        String output;
        serializeJson(doc, output);
        return output;
    }
};

// NEW: Hop Master Registration Structure
struct HopMasterRegistration {
    uint32_t device_id;
    uint8_t zone_id;
    uint8_t device_mac[6];
    uint8_t available_hop_slots;
    bool wifi_capability;
    uint8_t signal_strength;
};

// NEW: Hop Slave Registration Structure
struct HopSlaveRegistration {
    uint32_t device_id;
    uint8_t zone_id;
    uint8_t device_mac[6];
    uint8_t preferred_hop_master;
    uint8_t signal_strength;
};

// Enhanced Registration Response
struct RegistrationResponse {
    bool accepted;
    uint8_t slave_id;
    uint8_t hop_master_id;        // NEW: Assigned hop master
    device_role_t assigned_role;  // NEW: Assigned role
    device_level_t assigned_level;// NEW: Assigned level
    uint32_t zone_id;
    uint16_t report_interval;
    uint8_t max_hop_slaves;       // NEW: For hop masters
};

// NEW: Network Topology Message
struct NetworkTopology {
    uint8_t zone_id;
    uint8_t wifi_master_id;
    uint8_t direct_slave_count;
    uint8_t hop_master_count;
    uint8_t total_hop_slaves;
    struct {
        uint8_t hop_master_id;
        uint8_t hop_slave_count;
    } hop_masters[MAX_HOP_MASTERS];
};

// NEW: Route Information for Data Forwarding
struct RouteInfo {
    uint8_t destination_mac[6];
    uint8_t next_hop_mac[6];
    uint8_t hop_count;
    uint32_t last_used;
};

// Aggregated Data Structure
// Deprecated: No longer used for per-device reporting. Retained for reference only.
// struct AggregatedData {
//     uint8_t zone_id;
//     float avg_temperature;
//     float avg_humidity;
//     float avg_pm1;
//     float avg_pm25;
//     float avg_pm4;
//     float avg_pm10;
//     float avg_tvoc;
//     uint16_t avg_air_quality;
//     uint8_t devices_online;
//     uint32_t timestamp;
//     uint8_t total_devices;
// };
    
#endif