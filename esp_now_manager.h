#ifndef ESP_NOW_MANAGER_H
#define ESP_NOW_MANAGER_H

#include "cluster_types.h"
#include <esp_now.h>
#include <functional>
#include <map>
#include <vector>

class ESPNowManager {
private:
    static ESPNowManager* instance;
    
    // Message routing table for multi-hop communication
    std::map<uint32_t, uint8_t[6]> routing_table;  // device_id -> next_hop_mac
    
    // Message sequence tracking to prevent loops
    std::map<uint32_t, uint32_t> last_sequence_numbers;  // sender_id -> last_seq
    
    // Retry mechanism for failed messages
    struct PendingMessage {
        ESPNowMessage msg;
        uint8_t target_mac[6];
        uint32_t timestamp;
        uint8_t retry_count;
    };
    std::vector<PendingMessage> pending_messages;
    
    // Internal methods
    bool shouldProcessMessage(uint32_t sender_id, uint32_t sequence);
    void cleanupOldSequenceNumbers();
    void retryFailedMessages();
    
public:
    ESPNowManager();
    ~ESPNowManager();
    static ESPNowManager* getInstance();
    
    // Initialization
    bool initialize();
    void setMessageCallback(std::function<void(const uint8_t*, const uint8_t*, int)> callback);
    
    // Message Sending - Enhanced for mesh
    bool sendMessage(const ESPNowMessage& msg, const uint8_t* mac_addr = nullptr);
    bool broadcastMessage(const ESPNowMessage& msg);
    bool sendMessageWithRouting(const ESPNowMessage& msg, uint32_t target_device_id);
    bool forwardMessage(const ESPNowMessage& msg, const uint8_t* next_hop_mac);
    
    // Routing Management
    void updateRoutingTable(uint32_t device_id, const uint8_t* next_hop_mac);
    void removeFromRoutingTable(uint32_t device_id);
    bool getNextHop(uint32_t device_id, uint8_t* next_hop_mac);
    void printRoutingTable();
    
    // Peer Management - Enhanced
    bool addPeer(const uint8_t* mac_addr, uint8_t channel = 0);
    bool removePeer(const uint8_t* mac_addr);
    bool isPeerRegistered(const uint8_t* mac_addr);
    void cleanupInactivePeers();
    
    // Message Quality and Statistics
    void printStatistics();
    uint32_t getMessagesSent() const { return messages_sent; }
    uint32_t getMessagesReceived() const { return messages_received; }
    uint32_t getSendFailures() const { return send_failures; }
    float getSuccessRate() const;
    
    // Mesh-specific methods
    void performMeshMaintenance();  // Call periodically to maintain mesh health
    void broadcastTopologyUpdate();
    
    // Callbacks
    static void onDataReceived(const uint8_t* mac_addr, const uint8_t* data, int len);
    static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);

protected:
    friend void OnDataSent(const uint8_t*, esp_now_send_status_t);
    friend void OnDataRecv(const uint8_t*, const uint8_t*, int);

    // Message received callback
    std::function<void(const uint8_t*, const uint8_t*, int)> message_callback;
    
    // Enhanced Statistics
    uint32_t messages_sent;
    uint32_t messages_received;
    uint32_t send_failures;
    uint32_t messages_forwarded;    // NEW: Messages forwarded in mesh
    uint32_t routing_failures;      // NEW: Failed routing attempts
    uint32_t duplicate_messages;    // NEW: Duplicate messages filtered
    
    // Peer management with timestamps
    struct PeerInfo {
        esp_now_peer_info_t peer_info;
        uint32_t last_seen;
        uint8_t signal_strength;
        bool is_active;
    };
    std::map<uint8_t, PeerInfo> peers_enhanced;  // Using last byte of MAC as key
    
    // Mesh maintenance
    uint32_t last_maintenance_time;
    uint32_t last_topology_broadcast;
};

#endif