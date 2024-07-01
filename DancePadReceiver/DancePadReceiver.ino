#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR TRANSMITTER MAC Address
uint8_t broadcastAddress[] = {0x0C,0x8B,0x95,0x94,0xE5,0x2C}; 

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
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  esp_now_send(mac, incomingData, len); // send back to determine latency
  static int16_t errors=0;
  Serial.printf("Received %i bytes from ", len);
  for (int ct = 0; ct < 5; ++ct)
    Serial.printf("%02x:",*(mac+ct));
  Serial.printf("%02x\n",*(mac+5));
  struct_message msg;
  if (len == sizeof(msg)) {
    memcpy((void *)&msg, (void *)incomingData, len);
    Serial.printf("received %04x errors %d\n",
                  msg.buttons, errors);
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
}
