#ifndef CLUSTER_MANAGER_H
#define CLUSTER_MANAGER_H

#include "cluster_types.h"
#include "esp_now_manager.h"
#include "config.h" // Include config.h for DeviceState
#include "Sensor.h" // Include Sensor.h for SensorManager
#include <vector>
#include <map>

// Device hierarchy levels
typedef enum {
    LEVEL_WIFI_MASTER = 0,    // WiFi + MQTT master
    LEVEL_DIRECT_SLAVE = 1,   // Direct slave to WiFi master
    LEVEL_HOP_MASTER = 1,     // Hop master (slave to WiFi master, master to hop slaves)
    LEVEL_HOP_SLAVE = 2       // Slave to hop master
} device_level_t;

// Device roles
typedef enum {
    ROLE_WIFI_MASTER,
    ROLE_DIRECT_SLAVE,
    ROLE_HOP_MASTER,
    ROLE_HOP_SLAVE
} device_role_t;

struct HopSlaveDevice {
    uint8_t mac[6];
    uint32_t device_id;
    uint8_t hopslaveId;
    unsigned long lastSeen;
    bool isOnline;
    float temperature;
    float humidity;
    float pm1;
    float pm25;
    float pm4;
    float pm10;
    float tvoc;
    uint16_t aqi;
    // uint8_t battery_level;
    // uint8_t signal_strength;
};

// NEW: Enhanced Hop Master Device Structure
struct HopMasterDevice {
    uint8_t mac[6];
    uint32_t device_id;
    uint8_t hopMasterId;
    unsigned long lastSeen;
    bool isOnline;
    std::vector<HopSlaveDevice> hopSlaves;
    uint8_t hopSlaveCount;
    uint8_t maxHopSlaves;
    bool canAcceptSlaves;
    // uint8_t signal_strength;
};

class ClusterManager {
private:
    DeviceState current_state;
    DeviceInfo local_device;
    DeviceInfo master_device;
    uint32_t cluster_id;
    bool is_master;
    device_role_t my_role;           // NEW: Current device role
    device_level_t my_level;         // NEW: Current device level
    
    // Discovery management
    std::vector<DeviceInfo> discovered_devices;
    uint32_t discovery_start_time;
    
    // WiFi Master management (Level 0)
    std::vector<DeviceInfo> direct_slave_devices;           // Direct slaves
    std::vector<HopMasterDevice> hop_master_devices;        // Hop masters
    std::map<uint8_t, SensorData> latest_sensor_data;       // All sensor data
    
    // Hop Master management (Level 1)
    DeviceInfo my_wifi_master;                              // Reference to WiFi master
    std::vector<HopSlaveDevice> my_hop_slaves;              // My hop slaves
    uint8_t my_hop_master_id;                               // My ID as hop master
    
    // Hop Slave management (Level 2)
    DeviceInfo my_hop_master;                               // Reference to hop master
    uint8_t my_hop_slave_id;                                // My ID as hop slave
    
    // Timing management
    uint32_t last_report_time;
    uint32_t last_hop_heartbeat;
    uint32_t last_topology_broadcast;
    
    // NEW: Mesh-specific methods
    bool canBecomeHopMaster() const;
    void initializeAsWiFiMaster();
    void initializeAsDirectSlave();
    void initializeAsHopMaster();
    void initializeAsHopSlave();
    
    // WiFi Master operations
    void handleWiFiMasterOperations();
    bool registerDirectSlave(const DeviceInfo& slave_info);
    bool registerHopMaster(const HopMasterRegistration& hop_master_info);
    void forwardDataToMQTT(const SensorData& data);
    void broadcastTopologyUpdate();
    
    // Hop Master operations
    void handleHopMasterOperations();
    bool registerHopSlave(const HopSlaveRegistration& slave_info);
    void forwardDataToWiFiMaster(const SensorData& data);
    void sendHopHeartbeat();
    
    // Hop Slave operations
    void handleHopSlaveOperations();
    void sendDataToHopMaster();
    void findAndRegisterWithHopMaster();
    
    // Enhanced message handling
    void processHopMasterRegistration(const ESPNowMessage& msg);
    void processHopSlaveRegistration(const ESPNowMessage& msg);
    void processHopSensorData(const ESPNowMessage& msg);
    void processTopologyUpdate(const ESPNowMessage& msg);
    void processHopHeartbeat(const ESPNowMessage& msg);
    
    // Routing and forwarding
    void routeMessage(const ESPNowMessage& msg);
    bool forwardToNextHop(const ESPNowMessage& msg);
    void updateRoutingTable(uint32_t device_id, const uint8_t* next_hop_mac);
    
    // Health monitoring
    void performHopMasterHealthCheck();
    void performHopSlaveHealthCheck();
    void handleHopMasterFailure(uint8_t hop_master_id);
    
    // Internal utility methods
    bool isInSameZone(uint8_t zone_id) const { return zone_id == local_device.zone_id; }
    void processDiscoveryMessage(const ESPNowMessage& msg);
    void handleSlaveRegistration(const ESPNowMessage& msg);
    bool registerSlave(const DeviceInfo& slave_info);
    void sendRegistrationResponse(const DeviceInfo& slave_info, bool accepted, uint8_t slave_id);
    void processSensorData(const ESPNowMessage& msg);
    void sendSensorDataToMaster();
    void performSlaveHealthCheck();
    void findAndRegisterWithMaster();
    
public:
    ClusterManager();
    ~ClusterManager();
    
    // Initialization
    bool initialize();
    void setDeviceInfo(const DeviceInfo& info);
    
    // State management
    DeviceState getCurrentState() const { return current_state; }
    device_role_t getMyRole() const { return my_role; }           // NEW
    device_level_t getMyLevel() const { return my_level; }       // NEW
    void updateState();
    void handleButtonPress();
    
    // Network operations
    void handleESPNowMessage(const uint8_t* mac_addr, const uint8_t* data, int len);
    void handleDiscoveryPhase();
    void handleMasterOperations();
    void handleSlaveOperations();
    
    // NEW: Hop-specific operations
    void handleHopMasterMode();
    void handleHopSlaveMode();
    bool promoteToHopMaster();
    void demoteFromHopMaster();
    
    // Data management
    void updateSensorData(const SensorData& data);
    void publishMQTTData();
    
    // NEW: Network topology methods
    uint8_t getTotalNetworkDevices() const;
    uint8_t getDirectSlaveCount() const { return direct_slave_devices.size(); }
    uint8_t getHopMasterCount() const { return hop_master_devices.size(); }
    uint8_t getTotalHopSlaveCount() const;
    void printNetworkTopology();
    
    // Utility methods
    bool isMaster() const { return is_master; }
    bool isWiFiMaster() const { return my_role == ROLE_WIFI_MASTER; }    // NEW
    bool isHopMaster() const { return my_role == ROLE_HOP_MASTER; }      // NEW
    uint8_t getZoneId() const { return local_device.zone_id; }
    uint32_t getClusterId() const { return cluster_id; }
    size_t getSlaveCount() const { return direct_slave_devices.size(); } // Updated
    void printClusterStatus();
    
    // Configuration methods
    void setZoneId(uint8_t zone) { local_device.zone_id = zone; }
    void setHopMasterCapability(bool can_hop) { local_device.can_be_hop_master = can_hop; }

    // Helper methods
    uint8_t getLocalSlaveId() const;
    uint8_t getMyHopMasterId() const { return my_hop_master_id; }        // NEW
};

#endif // CLUSTER_MANAGER_H