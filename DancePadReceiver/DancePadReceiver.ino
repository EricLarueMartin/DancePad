//#include "USB.h"
//#include "USBHIDKeyboard.h"
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR TRANSMITTER MAC Address
uint8_t broadcastAddress[] = {0xDC,0x54,0x75,0xC3,0x07,0x74}; 

esp_now_peer_info_t peerInfo;

typedef struct struct_message {
  bool buttonState;
  int buttonNum;
  unsigned long timeSent;
} struct_message;


const char buttonKey[] = {'q','w','e','a','d','z','x','c','7','8','9','4','6','1','2','3'};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  esp_now_send(recvInfo->src_addr, incomingData, len); // send back to determine latency
  static struct_message msg;
  if (len != sizeof(msg))  {
    Serial.println("Received bad packet.");
    return;
  }
  
  memcpy((void *)&msg, (void *)incomingData, len);
  Serial.printf("%s is %i\n",buttonKey[msg.buttonNum],msg.buttonState);
}

void espNowSetup() {
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

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);  
  delay(5000);  
  Serial.println("ESP-NOW receive a structure");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
//  Serial.print("ESP32 Board MAC Address:  ");  Serial.println(WiFi.macAddress());

  espNowSetup();
}

void serialUpdateMAC() {
  static unsigned long lastMACUpdate = 0;
  if (millis() - lastMACUpdate < 10000) return;
  lastMACUpdate = millis();
  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  serialUpdateMAC();
}
