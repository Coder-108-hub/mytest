#include "cluster_manager.h"
#include "esp_now_manager.h"
#include "Sensor.h"
#include <WiFi.h>

ClusterManager::ClusterManager() 
    : current_state(STATE_INITIALIZING),
      cluster_id(0),
      is_master(false),
      my_role(ROLE_DIRECT_SLAVE),
      my_level(LEVEL_DIRECT_SLAVE),
      discovery_start_time(0),
      last_report_time(0),
      last_hop_heartbeat(0),
      last_topology_broadcast(0),
      my_hop_master_id(0),
      my_hop_slave_id(0) {
    
    memset(&local_device, 0, sizeof(local_device));
    memset(&master_device, 0, sizeof(master_device));
    memset(&my_wifi_master, 0, sizeof(my_wifi_master));
    memset(&my_hop_master, 0, sizeof(my_hop_master));
}

ClusterManager::~ClusterManager() {
    // Cleanup resources
}

bool ClusterManager::initialize() {
    // Initialize device info
    WiFi.macAddress(local_device.device_mac);
    local_device.device_id = ESP.getEfuseMac();
    local_device.zone_id = DEFAULT_ZONE_ID;
    local_device.is_online = true;
    local_device.role = ROLE_DIRECT_SLAVE;
    local_device.level = LEVEL_DIRECT_SLAVE;
    local_device.hop_master_id = 0;
    local_device.can_be_hop_master = true; // Default capability
    
    current_state = STATE_DISCOVERY;
    discovery_start_time = millis();
    
    Serial.printf("[ClusterManager] Initialized as Device ID: %08X, Zone: %d\n", 
                  local_device.device_id, local_device.zone_id);
    
    return true;
}

void ClusterManager::setDeviceInfo(const DeviceInfo& info) {
    local_device = info;
}

void ClusterManager::updateState() {
    switch (current_state) {
        case STATE_DISCOVERY:
            handleDiscoveryPhase();
            break;
            
        case STATE_MASTER_ACTIVE:
            if (my_role == ROLE_WIFI_MASTER) {
                handleWiFiMasterOperations();
            } else if (my_role == ROLE_HOP_MASTER) {
                handleHopMasterOperations();
            }
            break;
            
        case STATE_SLAVE_ACTIVE:
            if (my_role == ROLE_DIRECT_SLAVE) {
                handleSlaveOperations();
            } else if (my_role == ROLE_HOP_SLAVE) {
                handleHopSlaveOperations();
            }
            break;
            
        default:
            break;
    }
}

void ClusterManager::handleButtonPress() {
    if (current_state == STATE_DISCOVERY) {
        current_state = STATE_MASTER_CANDIDATE;
        
        // Determine role based on WiFi capability
        if (local_device.can_be_hop_master) {
            // Check if we can be WiFi master (first choice)
            bool wifi_master_exists = false;
            for (const auto& device : discovered_devices) {
                if (device.role == ROLE_WIFI_MASTER) {
                    wifi_master_exists = true;
                    break;
                }
            }
            
            if (!wifi_master_exists) {
                initializeAsWiFiMaster();
            } else {
                initializeAsHopMaster();
            }
        }
    }
}

// NEW: WiFi Master initialization
void ClusterManager::initializeAsWiFiMaster() {
    my_role = ROLE_WIFI_MASTER;
    my_level = LEVEL_WIFI_MASTER;
    is_master = true;
    
    // Generate cluster ID
    cluster_id = (local_device.device_id & 0xFFFF) | (local_device.zone_id << 16);
    
    // Initialize WiFi and MQTT
    // WiFi connection code here
    
    current_state = STATE_MASTER_ACTIVE;
    Serial.printf("[ClusterManager] Became WiFi Master - ID: %08X\n", cluster_id);
}

// NEW: Hop Master initialization
void ClusterManager::initializeAsHopMaster() {
    my_role = ROLE_HOP_MASTER;
    my_level = LEVEL_HOP_MASTER;
    is_master = true;
    
    // Find WiFi master to register with
    for (const auto& device : discovered_devices) {
        if (device.role == ROLE_WIFI_MASTER && device.zone_id == local_device.zone_id) {
            my_wifi_master = device;
            break;
        }
    }
    
    current_state = STATE_MASTER_ACTIVE;
    Serial.printf("[ClusterManager] Became Hop Master\n");
}

// NEW: WiFi Master operations
void ClusterManager::handleWiFiMasterOperations() {
    static uint32_t last_heartbeat = 0;
    static uint32_t last_topology_check = 0;
    
    uint32_t current_time = millis();
    
    // Send heartbeat to all registered devices
    if (current_time - last_heartbeat > HEARTBEAT_INTERVAL) {
        ESPNowMessage heartbeat_msg;
        heartbeat_msg.msg_type = MSG_HEARTBEAT;
        heartbeat_msg.sender_id = local_device.device_id;
        heartbeat_msg.zone_id = local_device.zone_id;
        heartbeat_msg.hop_level = LEVEL_WIFI_MASTER;
        heartbeat_msg.hop_master_id = 0;
        
        // Send to direct slaves
        for (const auto& slave : direct_slave_devices) {
            ESPNowManager::getInstance()->sendMessage(heartbeat_msg, slave.device_mac);
        }
        
        // Send to hop masters
        for (const auto& hop_master : hop_master_devices) {
            ESPNowManager::getInstance()->sendMessage(heartbeat_msg, hop_master.mac);
            }
        
        last_heartbeat = current_time;
    }
    
    // Perform health checks
    if (current_time - last_topology_check > 30000) { // Every 30 seconds
        performHopMasterHealthCheck();
        broadcastTopologyUpdate();
        last_topology_check = current_time;
    }
    
    // Publish MQTT data
    publishMQTTData();
}

// NEW: Hop Master operations
void ClusterManager::handleHopMasterOperations() {
    static uint32_t last_wifi_master_report = 0;
    uint32_t current_time = millis();
    
    // Send heartbeat to hop slaves
    if (current_time - last_hop_heartbeat > HEARTBEAT_INTERVAL) {
        sendHopHeartbeat();
        last_hop_heartbeat = current_time;
    }
    
    // Report to WiFi master periodically
    if (current_time - last_wifi_master_report > DATA_REPORT_INTERVAL) {
        // Aggregate hop slave data and send to WiFi master
        if (my_hop_slaves.size() > 0) {
            // Create aggregated sensor data
            SensorData aggregated_data;
            aggregated_data.zone_id = local_device.zone_id;
            aggregated_data.slave_id = my_hop_master_id;
            aggregated_data.hop_master_id = my_hop_master_id;
            aggregated_data.source_level = LEVEL_HOP_MASTER;
            aggregated_data.hop_count = 1;
            aggregated_data.timestamp = current_time;
            
            // Calculate averages from hop slaves
            float temp_sum = 0, hum_sum = 0, pm25_sum = 0;
            int active_slaves = 0;
            
            for (const auto& slave : my_hop_slaves) {
                if (slave.isOnline) {
                    temp_sum += slave.temperature;
                    hum_sum += slave.humidity;
                    pm25_sum += slave.pm25;
                    active_slaves++;
                }
            }
            
            if (active_slaves > 0) {
                aggregated_data.temperature = temp_sum / active_slaves;
                aggregated_data.humidity = hum_sum / active_slaves;
                aggregated_data.pm25 = pm25_sum / active_slaves;
            }
            
            forwardDataToWiFiMaster(aggregated_data);
        }
        
        last_wifi_master_report = current_time;
    }
    
    // Perform hop slave health checks
    performHopSlaveHealthCheck();
}

// NEW: Hop Slave operations
void ClusterManager::handleHopSlaveOperations() {
    static uint32_t last_data_send = 0;
    uint32_t current_time = millis();
    
    // Send sensor data to hop master
    if (current_time - last_data_send > DATA_REPORT_INTERVAL) {
        sendDataToHopMaster();
        last_data_send = current_time;
    }
    
    // Check hop master connectivity
    if (current_time - my_hop_master.last_seen > MASTER_TIMEOUT_MS) {
        Serial.println("[ClusterManager] Hop master lost, returning to discovery");
        current_state = STATE_DISCOVERY;
        my_role = ROLE_DIRECT_SLAVE;
        my_level = LEVEL_DIRECT_SLAVE;
    }
}

void ClusterManager::handleESPNowMessage(const uint8_t* mac_addr, const uint8_t* data, int len) {
    if (len < sizeof(ESPNowMessage)) return;
    
    ESPNowMessage* msg = (ESPNowMessage*)data;
    
    // Zone filtering
    if (msg->zone_id != local_device.zone_id) return;
    
    switch (msg->msg_type) {
        case MSG_DISCOVERY_ANNOUNCE:
            processDiscoveryMessage(*msg);
            break;
            
        case MSG_SLAVE_REGISTRATION_REQUEST:
            if (my_role == ROLE_WIFI_MASTER) {
                handleSlaveRegistration(*msg);
            } else if (my_role == ROLE_HOP_MASTER) {
                processHopSlaveRegistration(*msg);
            }
            break;
            
        case MSG_SENSOR_DATA:
            if (my_role == ROLE_WIFI_MASTER || my_role == ROLE_HOP_MASTER) {
                processSensorData(*msg);
            }
            break;
            
        case MSG_HEARTBEAT:
            // Update last seen time for sender
            if (my_role == ROLE_DIRECT_SLAVE && msg->sender_id == master_device.device_id) {
                master_device.last_seen = millis();
            } else if (my_role == ROLE_HOP_SLAVE && msg->sender_id == my_hop_master.device_id) {
                my_hop_master.last_seen = millis();
            }
            break;
            
        default:
            break;
    }
}

// NEW: Send data to hop master
void ClusterManager::sendDataToHopMaster() {
    if (my_role != ROLE_HOP_SLAVE) return;
    
    // Get current sensor data
    SensorData sensor_data;
    sensor_data.zone_id = local_device.zone_id;
    sensor_data.slave_id = my_hop_slave_id;
    sensor_data.hop_master_id = my_hop_master_id;
    sensor_data.source_level = LEVEL_HOP_SLAVE;
    sensor_data.hop_count = 2; // Hop slave -> Hop master -> WiFi master
    sensor_data.timestamp = millis();
    
    // Get sensor readings from SensorManager
    // sensor_data.temperature = SensorManager::getTemperature();
    // sensor_data.humidity = SensorManager::getHumidity();
    // ... other sensor readings
    
    ESPNowMessage msg;
    msg.msg_type = MSG_SENSOR_DATA;
    msg.sender_id = local_device.device_id;
    msg.zone_id = local_device.zone_id;
    msg.hop_level = LEVEL_HOP_SLAVE;
    msg.hop_master_id = my_hop_master_id;
    
    memcpy(msg.payload, &sensor_data, sizeof(sensor_data));
 
    ESPNowManager::getInstance()->sendMessage(msg, my_hop_master.device_mac);
}

// NEW: Forward data to WiFi master
void ClusterManager::forwardDataToWiFiMaster(const SensorData& data) {
    if (my_role != ROLE_HOP_MASTER) return;
    
    ESPNowMessage msg;
    msg.msg_type = MSG_SENSOR_DATA;
    msg.sender_id = local_device.device_id;
    msg.zone_id = local_device.zone_id;
    msg.hop_level = LEVEL_HOP_MASTER;
    msg.hop_master_id = my_hop_master_id;
    
    memcpy(msg.payload, &data, sizeof(data));
    
    ESPNowManager::getInstance()->sendMessage(msg, my_wifi_master.device_mac);
}

// NEW: Send hop heartbeat
void ClusterManager::sendHopHeartbeat() {
    if (my_role != ROLE_HOP_MASTER) return;

    ESPNowMessage msg;
    ESPNowMessage heartbeat_msg;
    heartbeat_msg.msg_type = MSG_HEARTBEAT;
    heartbeat_msg.sender_id = local_device.device_id;
    heartbeat_msg.zone_id = local_device.zone_id;
    heartbeat_msg.hop_level = LEVEL_HOP_MASTER;
    heartbeat_msg.hop_master_id = my_hop_master_id;
    
    for (const auto& slave : my_hop_slaves) {
        if (slave.isOnline) {
            ESPNowManager::getInstance()->sendMessage(msg, slave.mac);
        }
    }
}

// NEW: Process hop slave registration
void ClusterManager::processHopSlaveRegistration(const ESPNowMessage& msg) {
    if (my_role != ROLE_HOP_MASTER) return;
    if (my_hop_slaves.size() >= MAX_HOP_SLAVES) return;
    
    HopSlaveRegistration* reg_data = (HopSlaveRegistration*)msg.payload;
    
    // Create new hop slave entry
    HopSlaveDevice new_slave;
    memcpy(new_slave.mac, reg_data->device_mac, 6);
    new_slave.device_id = reg_data->device_id;
    new_slave.hopslaveId = my_hop_slaves.size() + 1;
    new_slave.lastSeen = millis();
    new_slave.isOnline = true;
    
    my_hop_slaves.push_back(new_slave);
    
    // Send registration response
    RegistrationResponse response;
    response.accepted = true;
    response.slave_id = new_slave.hopslaveId;
    response.hop_master_id = my_hop_master_id;
    response.assigned_role = ROLE_HOP_SLAVE;
    response.assigned_level = LEVEL_HOP_SLAVE;
    response.zone_id = local_device.zone_id;
    response.report_interval = DATA_REPORT_INTERVAL;
    
    ESPNowMessage response_msg;
    response_msg.msg_type = MSG_SLAVE_REGISTRATION_RESPONSE;
    response_msg.sender_id = local_device.device_id;
    response_msg.zone_id = local_device.zone_id;
    response_msg.hop_level = LEVEL_HOP_MASTER;
    response_msg.hop_master_id = my_hop_master_id;
    
    memcpy(response_msg.payload, &response, sizeof(response));
    
    ESPNowManager::getInstance()->sendMessage(msg, new_slave.mac);

    Serial.printf("[ClusterManager] Registered hop slave %08X as ID %d\n", 
                  reg_data->device_id, new_slave.hopslaveId);
}

// NEW: Health check for hop masters
void ClusterManager::performHopMasterHealthCheck() {
    if (my_role != ROLE_WIFI_MASTER) return;
    
    uint32_t current_time = millis();
    
    for (auto it = hop_master_devices.begin(); it != hop_master_devices.end();) {
        if (current_time - it->lastSeen > MASTER_TIMEOUT_MS) {
            Serial.printf("[ClusterManager] Hop master %d offline, removing\n", it->hopMasterId);
            it = hop_master_devices.erase(it);
        } else {
            ++it;
        }
    }
}

// NEW: Health check for hop slaves
void ClusterManager::performHopSlaveHealthCheck() {
    if (my_role != ROLE_HOP_MASTER) return;
    
    uint32_t current_time = millis();
    
    for (auto& slave : my_hop_slaves) {
        if (current_time - slave.lastSeen > SLAVE_DATA_TIMEOUT) {
            if (slave.isOnline) {
                Serial.printf("[ClusterManager] Hop slave %d offline\n", slave.hopslaveId);
                slave.isOnline = false;
            }
        }
    }
}

// NEW: Broadcast topology update
void ClusterManager::broadcastTopologyUpdate() {
    if (my_role != ROLE_WIFI_MASTER) return;
    
    NetworkTopology topology;
    topology.zone_id = local_device.zone_id;
    topology.wifi_master_id = local_device.device_id & 0xFF;
    topology.direct_slave_count = direct_slave_devices.size();
    topology.hop_master_count = hop_master_devices.size();
    
    uint8_t total_hop_slaves = 0;
    for (int i = 0; i < hop_master_devices.size() && i < MAX_HOP_MASTERS; i++) {
        topology.hop_masters[i].hop_master_id = hop_master_devices[i].hopMasterId;
        topology.hop_masters[i].hop_slave_count = hop_master_devices[i].hopSlaveCount;
        total_hop_slaves += hop_master_devices[i].hopSlaveCount;
    }
    topology.total_hop_slaves = total_hop_slaves;
    
    ESPNowMessage msg;
    ESPNowMessage topo_msg;
    topo_msg.msg_type = MSG_CLUSTER_INFO;
    topo_msg.sender_id = local_device.device_id;
    topo_msg.zone_id = local_device.zone_id;
    topo_msg.hop_level = LEVEL_WIFI_MASTER;
    topo_msg.hop_master_id = 0;
    
    memcpy(topo_msg.payload, &topology, sizeof(topology));
    
    // Broadcast to all devices
    uint8_t broadcast_mac[6] = ESP_NOW_BROADCAST_ADDR;
    ESPNowManager::getInstance()->sendMessage(msg, broadcast_mac);
}

// NEW: Get total network devices
uint8_t ClusterManager::getTotalNetworkDevices() const {
    uint8_t total = direct_slave_devices.size() + hop_master_devices.size() + 1; // +1 for wifi master
    
    for (const auto& hop_master : hop_master_devices) {
        total += hop_master.hopSlaveCount;
    }
    
    return total;
}

// NEW: Get total hop slave count
uint8_t ClusterManager::getTotalHopSlaveCount() const {
    uint8_t total = 0;
    for (const auto& hop_master : hop_master_devices) {
        total += hop_master.hopSlaveCount;
    }
    return total;
}

// NEW: Print network topology
void ClusterManager::printNetworkTopology() {
    if (my_role != ROLE_WIFI_MASTER) return;
    
    Serial.println("\n=== Network Topology ===");
    Serial.printf("WiFi Master: %08X (Zone %d)\n", local_device.device_id, local_device.zone_id);
    Serial.printf("├── Direct Slaves: %d/%d\n", direct_slave_devices.size(), MAX_DIRECT_SLAVES);
    
    for (const auto& slave : direct_slave_devices) {
        Serial.printf("│   ├── Slave %08X\n", slave.device_id);
    }
    
    Serial.printf("├── Hop Masters: %d/%d\n", hop_master_devices.size(), MAX_HOP_MASTERS);
    for (const auto& hop_master : hop_master_devices) {
        Serial.printf("│   ├── Hop Master %d (%08X)\n", hop_master.hopMasterId, hop_master.device_id);
        Serial.printf("│   │   └── Hop Slaves: %d/%d\n", hop_master.hopSlaveCount, MAX_HOP_SLAVES);
    }
    
    Serial.printf("Total Devices: %d\n", getTotalNetworkDevices());
    Serial.println("========================\n");
}

// Existing methods (update sensor data, publish MQTT, etc.)
void ClusterManager::updateSensorData(const SensorData& data) {
    latest_sensor_data[data.slave_id] = data;
}

void ClusterManager::publishMQTTData() {
    if (my_role != ROLE_WIFI_MASTER) return;
    
    static uint32_t last_publish = 0;
    uint32_t current_time = millis();
    
    if (current_time - last_publish < MQTT_PUBLISH_INTERVAL) return;
    
    // Publish individual device data
    for (const auto& data_pair : latest_sensor_data) {
        String topic = String(MQTT_TOPIC_PREFIX) + "/zone" + String(data_pair.second.zone_id) + 
                       "/device" + String(data_pair.first);
        String payload = data_pair.second.toJson();
        
        // MQTTManager::publish(topic.c_str(), payload.c_str());
    }
    
    last_publish = current_time;
}

// Implementation continues with remaining methods...
// (handleDiscoveryPhase, handleSlaveOperations, processSensorData, etc.)

void ClusterManager::handleDiscoveryPhase() {
    // Existing discovery logic + new hop master discovery
    static uint32_t last_announce = 0;
    uint32_t current_time = millis();
    
    if (current_time - last_announce > DISCOVERY_INTERVAL_MS) {
        ESPNowMessage msg;
        ESPNowMessage announce_msg;
        announce_msg.msg_type = MSG_DISCOVERY_ANNOUNCE;
        announce_msg.sender_id = local_device.device_id;
        announce_msg.zone_id = local_device.zone_id;
        announce_msg.hop_level = my_level;
        announce_msg.hop_master_id = 0;
        
        // Add device capabilities to payload
        DeviceInfo device_info = local_device;
        memcpy(announce_msg.payload, &device_info, sizeof(device_info));
        
        uint8_t broadcast_mac[6] = ESP_NOW_BROADCAST_ADDR;
        ESPNowManager::getInstance()->sendMessage(msg, broadcast_mac);
        last_announce = current_time;
    }
    
    // Auto-promotion logic after discovery timeout
    if (current_time - discovery_start_time > CLUSTER_DISCOVERY_TIMEOUT) {
        if (discovered_devices.empty()) {
            // No other devices found, become WiFi master
            initializeAsWiFiMaster();
        }
    }
}

void ClusterManager::processDiscoveryMessage(const ESPNowMessage& msg) {
    DeviceInfo* device_info = (DeviceInfo*)msg.payload;
    
    // Update discovered devices list
    bool found = false;
    for (auto& device : discovered_devices) {
        if (device.device_id == device_info->device_id) {
            device.last_seen = millis();
            found = true;
            break;
        }
    }
    
    if (!found) {
        device_info->last_seen = millis();
        discovered_devices.push_back(*device_info);
        Serial.printf("[ClusterManager] Discovered device %08X, role: %d, level: %d\n", 
                      device_info->device_id, device_info->role, device_info->level);
    }
}

// Continue with remaining method implementations...
// (This is a partial implementation showing the key new mesh functionality)