#include "esp_now_manager.h"
#include "config.h"
#include <esp_wifi.h>
#include <algorithm>

#ifndef ESPNOW_TIMEOUT_MS
#define ESPNOW_TIMEOUT_MS 5000
#endif

#ifndef MAX_RETRY_COUNT
#define MAX_RETRY_COUNT 3
#endif

#ifndef SEQUENCE_CLEANUP_INTERVAL
#define SEQUENCE_CLEANUP_INTERVAL 300000  // 5 minutes
#endif

ESPNowManager* ESPNowManager::instance = nullptr;

// ESP-NOW callback functions
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    ESPNowManager* manager = ESPNowManager::getInstance();
    if (status == ESP_NOW_SEND_SUCCESS) {
        manager->messages_sent++;
    } else {
        manager->send_failures++;
    }
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int len) {
    ESPNowManager* manager = ESPNowManager::getInstance();
    
    if (len >= sizeof(ESPNowMessage)) {
        ESPNowMessage* msg = (ESPNowMessage*)data;
        
        // Check for duplicate messages using sequence numbers
        if (!manager->shouldProcessMessage(msg->sender_id, msg->sequence)) {
            manager->duplicate_messages++;
            return;  // Skip duplicate message
        }
        
        // Update peer activity
        uint8_t peer_key = mac_addr[5];
        auto peer_it = manager->peers_enhanced.find(peer_key);
        if (peer_it != manager->peers_enhanced.end()) {
            peer_it->second.last_seen = millis();
            peer_it->second.is_active = true;
        }
    }
    
    manager->messages_received++;
    if (manager->message_callback) {
        manager->message_callback(mac_addr, data, len);
    }
}

ESPNowManager::ESPNowManager() :
    messages_sent(0),
    messages_received(0),
    send_failures(0),
    messages_forwarded(0),
    routing_failures(0),
    duplicate_messages(0),
    last_maintenance_time(0),
    last_topology_broadcast(0) {
}

ESPNowManager::~ESPNowManager() {
    // Clean up peers
    for (auto& p : peers_enhanced) {
        esp_now_del_peer(p.second.peer_info.peer_addr);
    }
    peers_enhanced.clear();
    routing_table.clear();
    pending_messages.clear();
}

ESPNowManager* ESPNowManager::getInstance() {
    if (!instance) {
        instance = new ESPNowManager();
    }
    return instance;
}

bool ESPNowManager::initialize() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return false;
    }
    
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb([](const esp_now_recv_info_t* info, const uint8_t* data, int len) {
        OnDataRecv(info->src_addr, data, len);
    });
    
    Serial.println("ESP-NOW initialized successfully");
    return true;
}

void ESPNowManager::setMessageCallback(std::function<void(const uint8_t*, const uint8_t*, int)> callback) {
    message_callback = callback;
}

bool ESPNowManager::sendMessage(const ESPNowMessage& msg, const uint8_t* mac_addr) {
    if (!mac_addr) {
        Serial.println("Invalid MAC address for sendMessage");
        return false;
    }
    
    if (!addPeer(mac_addr)) {
        Serial.println("Failed to add peer for sendMessage");
        return false;
    }
    
    esp_err_t result = esp_now_send(mac_addr, reinterpret_cast<const uint8_t*>(&msg), sizeof(ESPNowMessage));
    
    if (result != ESP_OK) {
        // Add to retry queue if it's an important message
        if (msg.msg_type == MSG_SENSOR_DATA || msg.msg_type == MSG_HOP_SENSOR_DATA) {
            PendingMessage pending;
            pending.msg = msg;
            memcpy(pending.target_mac, mac_addr, 6);
            pending.timestamp = millis();
            pending.retry_count = 0;
            pending_messages.push_back(pending);
        }
        return false;
    }
    
    return true;
}

bool ESPNowManager::broadcastMessage(const ESPNowMessage& msg) {
    uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    return sendMessage(msg, broadcast_addr);
}

bool ESPNowManager::sendMessageWithRouting(const ESPNowMessage& msg, uint32_t target_device_id) {
    uint8_t next_hop_mac[6];
    
    if (getNextHop(target_device_id, next_hop_mac)) {
        return sendMessage(msg, next_hop_mac);
    } else {
        // No route found, try broadcast as fallback
        Serial.printf("No route to device %08X, attempting broadcast\n", target_device_id);
        routing_failures++;
        return broadcastMessage(msg);
    }
}

bool ESPNowManager::forwardMessage(const ESPNowMessage& msg, const uint8_t* next_hop_mac) {
    // Create a copy of the message for forwarding
    ESPNowMessage forward_msg = msg;
    
    // Update hop count if this is a hop message
    if (msg.msg_type == MSG_HOP_SENSOR_DATA || msg.msg_type == MSG_HOP_REGISTRATION_REQUEST) {
        // Could add hop count field to message structure if needed
    }
    
    bool success = sendMessage(forward_msg, next_hop_mac);
    if (success) {
        messages_forwarded++;
        Serial.printf("Forwarded message from %08X via %02X:%02X:%02X:%02X:%02X:%02X\n", 
                     msg.sender_id, next_hop_mac[0], next_hop_mac[1], next_hop_mac[2], 
                     next_hop_mac[3], next_hop_mac[4], next_hop_mac[5]);
    }
    
    return success;
}

void ESPNowManager::updateRoutingTable(uint32_t device_id, const uint8_t* next_hop_mac) {
    memcpy(routing_table[device_id], next_hop_mac, 6);
    Serial.printf("Updated route: Device %08X -> %02X:%02X:%02X:%02X:%02X:%02X\n", 
                 device_id, next_hop_mac[0], next_hop_mac[1], next_hop_mac[2], 
                 next_hop_mac[3], next_hop_mac[4], next_hop_mac[5]);
}

void ESPNowManager::removeFromRoutingTable(uint32_t device_id) {
    auto it = routing_table.find(device_id);
    if (it != routing_table.end()) {
        routing_table.erase(it);
        Serial.printf("Removed route for device %08X\n", device_id);
    }
}

bool ESPNowManager::getNextHop(uint32_t device_id, uint8_t* next_hop_mac) {
    auto it = routing_table.find(device_id);
    if (it != routing_table.end()) {
        memcpy(next_hop_mac, it->second, 6);
        return true;
    }
    return false;
}

void ESPNowManager::printRoutingTable() {
    Serial.println("=== Routing Table ===");
    for (const auto& route : routing_table) {
        const uint8_t* mac = route.second;
        Serial.printf("Device %08X -> %02X:%02X:%02X:%02X:%02X:%02X\n", 
                     route.first, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    Serial.println("====================");
}

bool ESPNowManager::addPeer(const uint8_t* mac_addr, uint8_t channel) {
    uint8_t key = mac_addr[5];
    
    auto it = peers_enhanced.find(key);
    if (it == peers_enhanced.end()) {
        PeerInfo peer_info;
        memcpy(peer_info.peer_info.peer_addr, mac_addr, 6);
        peer_info.peer_info.channel = (channel == 0) ? ESP_NOW_CHANNEL : channel;
        peer_info.peer_info.encrypt = false;
        peer_info.last_seen = millis();
        peer_info.signal_strength = 0;
        peer_info.is_active = true;
        
        if (esp_now_add_peer(&peer_info.peer_info) != ESP_OK) {
            Serial.printf("Failed to add peer: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
            return false;
        }
        
        peers_enhanced[key] = peer_info;
        Serial.printf("Added peer: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                     mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        // Update existing peer
        it->second.last_seen = millis();
        it->second.is_active = true;
    }
    
    return true;
}

bool ESPNowManager::removePeer(const uint8_t* mac_addr) {
    uint8_t key = mac_addr[5];
    auto it = peers_enhanced.find(key);
    
    if (it != peers_enhanced.end()) {
        esp_now_del_peer(mac_addr);
        peers_enhanced.erase(it);
        Serial.printf("Removed peer: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                     mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        return true;
    }
    
    return false;
}

bool ESPNowManager::isPeerRegistered(const uint8_t* mac_addr) {
    uint8_t key = mac_addr[5];
    return peers_enhanced.find(key) != peers_enhanced.end();
}

void ESPNowManager::cleanupInactivePeers() {
    uint32_t current_time = millis();
    std::vector<uint8_t> keys_to_remove;
    
    for (auto& peer : peers_enhanced) {
        if (current_time - peer.second.last_seen > ESPNOW_TIMEOUT_MS * 3) {  // 3x timeout
            peer.second.is_active = false;
            keys_to_remove.push_back(peer.first);
        }
    }
    
    for (uint8_t key : keys_to_remove) {
        esp_now_del_peer(peers_enhanced[key].peer_info.peer_addr);
        peers_enhanced.erase(key);
    }
    
    if (!keys_to_remove.empty()) {
        Serial.printf("Cleaned up %d inactive peers\n", keys_to_remove.size());
    }
}

bool ESPNowManager::shouldProcessMessage(uint32_t sender_id, uint32_t sequence) {
    auto it = last_sequence_numbers.find(sender_id);
    
    if (it == last_sequence_numbers.end()) {
        // First message from this sender
        last_sequence_numbers[sender_id] = sequence;
        return true;
    }
    
    // Check if this is a newer message
    if (sequence > it->second) {
        it->second = sequence;
        return true;
    }
    
    // This is a duplicate or old message
    return false;
}

void ESPNowManager::cleanupOldSequenceNumbers() {
    uint32_t current_time = millis();
    
    if (current_time - last_maintenance_time > SEQUENCE_CLEANUP_INTERVAL) {
        // Remove very old sequence numbers to prevent memory buildup
        // In a real implementation, you might want to be more selective
        if (last_sequence_numbers.size() > 100) {
            last_sequence_numbers.clear();
            Serial.println("Cleaned up old sequence numbers");
        }
        last_maintenance_time = current_time;
    }
}

void ESPNowManager::retryFailedMessages() {
    uint32_t current_time = millis();
    std::vector<size_t> indices_to_remove;
    
    for (size_t i = 0; i < pending_messages.size(); i++) {
        PendingMessage& pending = pending_messages[i];
        
        if (current_time - pending.timestamp > 1000) {  // Retry after 1 second
            if (pending.retry_count < MAX_RETRY_COUNT) {
                if (sendMessage(pending.msg, pending.target_mac)) {
                    indices_to_remove.push_back(i);  // Success, remove from queue
                } else {
                    pending.retry_count++;
                    pending.timestamp = current_time;
                }
            } else {
                indices_to_remove.push_back(i);  // Max retries reached, give up
                Serial.printf("Message to %02X:%02X:%02X:%02X:%02X:%02X failed after %d retries\n", 
                             pending.target_mac[0], pending.target_mac[1], pending.target_mac[2], 
                             pending.target_mac[3], pending.target_mac[4], pending.target_mac[5], 
                             MAX_RETRY_COUNT);
            }
        }
    }
    
    // Remove completed/failed messages (reverse order to maintain indices)
    for (int i = indices_to_remove.size() - 1; i >= 0; i--) {
        pending_messages.erase(pending_messages.begin() + indices_to_remove[i]);
    }
}

void ESPNowManager::performMeshMaintenance() {
    cleanupInactivePeers();
    cleanupOldSequenceNumbers();
    retryFailedMessages();
    
    // Periodic topology broadcast
    uint32_t current_time = millis();
    if (current_time - last_topology_broadcast > 60000) {  // Every minute
        broadcastTopologyUpdate();
        last_topology_broadcast = current_time;
    }
}

void ESPNowManager::broadcastTopologyUpdate() {
    // This would be called by the cluster manager with topology information
    // Implementation depends on how you want to handle topology updates
    Serial.println("Broadcasting topology update");
}

float ESPNowManager::getSuccessRate() const {
    uint32_t total_attempts = messages_sent + send_failures;
    if (total_attempts == 0) return 100.0f;
    return (float)messages_sent / total_attempts * 100.0f;
}

void ESPNowManager::printStatistics() {
    Serial.println("=== ESP-NOW Statistics ===");
    Serial.printf("Messages Sent: %lu\n", messages_sent);
    Serial.printf("Messages Received: %lu\n", messages_received);
    Serial.printf("Send Failures: %lu\n", send_failures);
    Serial.printf("Messages Forwarded: %lu\n", messages_forwarded);
    Serial.printf("Routing Failures: %lu\n", routing_failures);
    Serial.printf("Duplicate Messages: %lu\n", duplicate_messages);
    Serial.printf("Success Rate: %.2f%%\n", getSuccessRate());
    Serial.printf("Active Peers: %d\n", peers_enhanced.size());
    Serial.printf("Routing Table Entries: %d\n", routing_table.size());
    Serial.printf("Pending Messages: %d\n", pending_messages.size());
    Serial.println("==========================");
}

void ESPNowManager::onDataReceived(const uint8_t* mac_addr, const uint8_t* data, int len) {
    OnDataRecv(mac_addr, data, len);
}

void ESPNowManager::onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    OnDataSent(mac_addr, status);
}