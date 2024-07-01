#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x4C, 0x75, 0x25, 0xC4, 0x76, 0xB4};
//uint8_t broadcastAddress[] = {0x4C,0x75,0x25,0xC6,0x5C,0xE0};

esp_now_peer_info_t peerInfo;

typedef struct struct_message {
  uint16_t buttons;
  unsigned long timeSent;
} struct_message;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  static int16_t errors=0;
  Serial.printf("Received %i bytes from ", len);
  for (int ct = 0; ct < 5; ++ct)
    Serial.printf("%02x:", *(recvInfo->src_addr+ct));
  Serial.printf("%02x\n", *(recvInfo->src_addr+5));
  struct_message msg;
  if (len == sizeof(msg)) {
    memcpy((void *)&msg, (void *)incomingData, len);
//    esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg)); // send back to determine latency
    Serial.printf("received %04x errors %d in %i ms round trip\n",
                  msg.buttons, errors, millis() - msg.timeSent);
  } else
    Serial.printf("ERROR! packet size expected %d receive %d errors %d\n", sizeof(struct_message), len, ++errors);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial.println("ESP-NOW receive a structure");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Send data structure via ESP-NOW
  static struct_message msg;
  msg.buttons++;
  msg.timeSent = millis();
  esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
//  Serial.print("ESP32 Board MAC Address:  ");  Serial.println(WiFi.macAddress());
  delay(1000);
}
