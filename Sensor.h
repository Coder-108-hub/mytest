//Sensor.h
#pragma once

#include "ui/ui.h"
#include <PubSubClient.h>
#include <SensirionI2CSen5x.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "cluster_types.h"

#define MAXBUF_REQUIREMENT 48
#define CHART_DATA_LENGTH 15
#define DATA_FREQ 5


// Device Role Definitions
enum DeviceRole {
    ROLE_UNDEFINED = 0,
    ROLE_WIFI_MASTER = 1,      // Level 0 - WiFi + MQTT access
    ROLE_DIRECT_SLAVE = 2,     // Level 1 - Direct to WiFi Master
    ROLE_HOP_MASTER = 3,       // Level 1 - Bridge between WiFi Master and Hop Slaves
    ROLE_HOP_SLAVE = 4         // Level 2 - Connected to Hop Masters
};

// Network Level Definitions
enum NetworkLevel {
    LEVEL_0 = 0,  // WiFi Master
    LEVEL_1 = 1,  // Direct Slaves + Hop Masters
    LEVEL_2 = 2   // Hop Slaves
};

// Message Types for Mesh Communication
enum MeshMessageType {
    MSG_SENSOR_DATA = 0x01,
    MSG_HEARTBEAT = 0x02,
    MSG_NETWORK_DISCOVERY = 0x03,
    MSG_ROLE_ASSIGNMENT = 0x04,
    MSG_DATA_AGGREGATION = 0x05,
    MSG_NETWORK_STATUS = 0x06,
    MSG_HOP_REGISTRATION = 0x07,
    MSG_HOP_DATA_RELAY = 0x08
};

// Mesh Node Information Structure
typedef struct {
    uint8_t mac_address[6];
    DeviceRole role;
    NetworkLevel level;
    uint8_t node_id;
    uint8_t parent_id;  // For hop slaves, this is their hop master
    uint32_t last_seen;
    float battery_level;
    bool is_active;
    uint8_t hop_count;  // Distance from WiFi Master
    uint8_t signal_strength;
} MeshNodeInfo;

// Hop Master Management Structure
typedef struct {
    uint8_t hop_master_id;
    uint8_t mac_address[6];
    uint8_t slave_count;
    uint8_t slave_ids[MAX_HOP_SLAVES_PER_MASTER];
    uint32_t last_heartbeat;
    bool is_active;
    float aggregated_data[8];  // temp, humidity, pm1, pm25, pm4, pm10, tvoc, aqi
} HopMasterInfo;

// Enhanced Sensor Data Structure for Mesh
typedef struct {
    uint8_t source_node_id;
    DeviceRole source_role;
    NetworkLevel source_level;
    uint8_t hop_count;
    uint32_t timestamp;
    float temperature;
    float humidity;
    float pm1;
    float pm25;
    float pm4;
    float pm10;
    uint16_t tvoc;
    uint8_t aqi;
    float battery_level;
    uint8_t signal_strength;
} MeshSensorData;

// Network Topology Structure
typedef struct {
    uint8_t total_nodes;
    uint8_t active_nodes;
    uint8_t wifi_master_count;
    uint8_t direct_slave_count;
    uint8_t hop_master_count;
    uint8_t hop_slave_count;
    uint32_t network_uptime;
    uint8_t max_hop_count;
    float network_efficiency;
} NetworkTopology;

#if (defined(I2C_BUFFER_LENGTH) &&                                             \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) ||                             \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#endif

// Forward declarations
class SensorManager {
private:
    float temperature;
    float humidity;
    float pm1;
    float pm25;
    float pm4;
    float pm10;
    uint16_t tvoc;
    uint8_t aqi;
    bool sensor_initialized;
    uint32_t last_read_time;
    
    // Moving average buffers
    float temp_buffer[MOVING_AVERAGE_WINDOW];
    float humidity_buffer[MOVING_AVERAGE_WINDOW];
    uint8_t buffer_index;
    bool buffer_filled;
    
    // Mesh Network Management
    DeviceRole current_role;
    NetworkLevel current_level;
    uint8_t node_id;
    uint8_t parent_node_id;
    uint8_t hop_count;
    
    // Network State Management
    MeshNodeInfo node_registry[50];  // Registry of known nodes
    HopMasterInfo hop_masters[MAX_HOP_MASTERS];
    NetworkTopology network_topology;
    
    // Data Aggregation Buffers
    MeshSensorData aggregation_buffer[MAX_DIRECT_SLAVES + MAX_HOP_MASTERS];
    uint8_t buffer_count;
    uint32_t last_aggregation_time;
    
    // Mesh Communication Timers
    uint32_t last_heartbeat_time;
    uint32_t last_discovery_time;
    uint32_t last_topology_update;

public:
    SensorManager();
    ~SensorManager();
    
    // Initialization
    bool initialize();
    void reset();
    
    // Sensor reading functions
    bool readSensors();
    bool readTemperatureHumidity();
    bool readPM25PM10();
    bool readTVOCCO2();
    bool readBatteryLevel();
    
    // Data access functions
    SensorData getCurrentData();
    MeshSensorData getCurrentMeshData();
    float getTemperature() const { return temperature; }
    float getHumidity() const { return humidity; }
    float getPM1() const { return pm1; }
    float getPM2_5() const { return pm25; }
    float getPM4() const { return pm4; }
    float getPM10() const { return pm10; }
    uint16_t getTVOC() const { return tvoc; }
    uint8_t getAQI() const { return aqi; }
    
    // Mesh Network Functions
    bool initializeMeshNetwork();
    void setDeviceRole(DeviceRole role);
    DeviceRole getDeviceRole() const { return current_role; }
    NetworkLevel getNetworkLevel() const { return current_level; }
    uint8_t getNodeId() const { return node_id; }
    uint8_t getHopCount() const { return hop_count; }
    
    // Network Discovery and Registration
    bool discoverNetworkNodes();
    bool registerWithParent();
    bool registerHopSlave(uint8_t slave_id, uint8_t* slave_mac);
    void updateNodeRegistry(MeshNodeInfo* node_info);
    
    // Data Aggregation and Forwarding
    void aggregateSensorData();
    void forwardDataToParent();
    void relayDataFromHopSlaves();
    bool publishAggregatedData();
    
    // Hop Master Specific Functions
    void manageHopSlaves();
    bool acceptHopSlaveRegistration(uint8_t slave_id);
    void aggregateHopSlaveData();
    void sendHeartbeatToHopSlaves();
    
    // WiFi Master Specific Functions
    void manageDirectSlaves();
    void manageHopMasters();
    void processNetworkTopology();
    void publishNetworkStatus();
    
    // Network Health and Monitoring
    void sendHeartbeat();
    void processHeartbeats();
    void checkNodeTimeouts();
    void updateNetworkTopology();
    float calculateNetworkEfficiency();
    
    // Message Processing
    void processMeshMessage(uint8_t* data, size_t len, uint8_t* sender_mac);
    void handleSensorDataMessage(MeshSensorData* data);
    void handleHeartbeatMessage(uint8_t* sender_mac);
    void handleNetworkDiscovery(uint8_t* sender_mac);
    void handleRoleAssignment(uint8_t role, uint8_t level);
    
    // Status functions
    bool isSensorReady() const { return sensor_initialized; }
    bool isDataValid() const;
    bool isNetworkConnected() const;
    uint32_t getLastReadTime() const { return last_read_time; }
    NetworkTopology getNetworkTopology() const { return network_topology; }
    
    // Calibration functions
    void calibrateTemperature(float offset);
    void calibrateHumidity(float offset);
    void calibratePM(float factor);
    
    // AQI calculation
    uint8_t calculateAQI();
    
    // Utility functions
    void printSensorData();
    void printNetworkStatus();
    String getSensorDataJSON();
    String getNetworkStatusJSON();
    String getMeshDataJSON();
    
    // Network Optimization Functions
    void optimizeNetworkRouting();
    void balanceHopMasterLoad();
    void handleNodeFailure(uint8_t failed_node_id);
    void promoteHopSlaveToMaster(uint8_t slave_id);
    
private:
    // Internal helper functions
    float calculateMovingAverage(float* buffer, float new_value);
    void updateMovingAverageBuffers();
    bool validateSensorReading(float value, float min_val, float max_val);
    uint8_t calculatePMAQI(float pm25_value);
    
    // Mesh Network Helper Functions
    uint8_t generateNodeId();
    void assignNodeRole();
    bool isNodeInRegistry(uint8_t* mac_address);
    MeshNodeInfo* findNodeInRegistry(uint8_t node_id);
    void removeNodeFromRegistry(uint8_t node_id);
    
    // Data Processing Helpers
    void packMeshSensorData(MeshSensorData* data);
    void unpackMeshSensorData(uint8_t* raw_data, MeshSensorData* data);
    bool validateMeshData(MeshSensorData* data);
    
    // Network Health Helpers
    void updateNodeLastSeen(uint8_t node_id);
    bool isNodeTimedOut(uint8_t node_id);
    void markNodeInactive(uint8_t node_id);
    
    // Hardware-specific functions
    bool initializeI2CSensors();
    bool readSHT30();  // Temperature & Humidity
    bool readPMS5003(); // PM2.5 & PM10
    bool readSGP30();   // TVOC & CO2
    bool readAXP2101(); // Battery level
};

// Struct definitions
typedef struct {
    float pm1 = 0;
    float pm25 = 0;
    float pm10 = 0;
    float pm4 = 0;
    float tvoc = 0;
    int pm1_max = 0;
    int pm25_max = 0;
    int pm10_max = 0;
    int pm4_max = 0;
    int tvoc_max = 0;
    int count = 0;
} Sensor_t;

typedef struct {
    float Cp_Lo; // Low concentration breakpoint
    float Cp_Hi; // High concentration breakpoint
    int Ip_Lo;   // Low index breakpoint
    int Ip_Hi;   // High index breakpoint
} AQIBreakpoint;

// External variable declarations (not definitions)
extern volatile int AQI;
extern volatile float temperature;
extern volatile float humidity;
extern SensorManager sensorManager;
extern SensirionI2CSen5x sen5x;
extern Sensor_t sensor_data;
extern WiFiClient espClient;
extern PubSubClient mqttClient;
extern String deviceName;

// MQTT Server Details (extern declarations)
extern const char *mqtt_server;
extern const char *username;
extern const char *password;

// Chart series declarations
extern lv_chart_series_t *ui_PM1chart_series_1;
extern lv_coord_t ui_PM1chart_series_1_array[CHART_DATA_LENGTH];
extern lv_chart_series_t *ui_PM25chart_series_1;
extern lv_coord_t ui_PM25chart_series_1_array[CHART_DATA_LENGTH];
extern lv_chart_series_t *ui_PM4chart_series_1;
extern lv_coord_t ui_PM4chart_series_1_array[CHART_DATA_LENGTH];
extern lv_chart_series_t *ui_PM10chart_series_1;
extern lv_coord_t ui_PM10chart_series_1_array[CHART_DATA_LENGTH];
extern lv_chart_series_t *ui_TVOCchart_series_1;
extern lv_coord_t ui_TVOCchart_series_1_array[CHART_DATA_LENGTH];

// AQI Breakpoints (extern declarations)
extern AQIBreakpoint pm1Bps[];
extern AQIBreakpoint pm4Bps[];
extern AQIBreakpoint pm25Bps[];
extern AQIBreakpoint pm10Bps[];
extern AQIBreakpoint tvocBps[];

// Function declarations
int calculateSubIndex(float Cp, AQIBreakpoint bp);
AQIBreakpoint getBreakpoint(float Cp, AQIBreakpoint bps[], int numBps);
String getAQICategory(int aqi);
uint32_t getAQIColor(int aqi);
void callback(char *topic, byte *payload, unsigned int length);
boolean reconnect();
void setupMQTT();
void setupCharts();
void sensorData(void *params);

// Mesh Network Function Declarations
void initializeMeshNetworking();
void processMeshNetworkLoop();
void handleMeshDataReceived(uint8_t* data, size_t len, uint8_t* sender_mac);
void sendMeshData(MeshSensorData* data, uint8_t* target_mac);
void broadcastNetworkDiscovery();
void processNetworkHealthCheck();
String generateMeshStatusReport();