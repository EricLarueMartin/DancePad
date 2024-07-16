/* DancePadReceiver
 * Receivers button state changes from transmitter.
 * Sends packet back to transmitter for confirmation.
 * Sends keyboard button press and release signals through USB. 
 */

//#include "USB.h"
//#include "USBHIDKeyboard.h"
#include <esp_now.h>
#include <WiFi.h>

#define ESPNOW_WIFI_CHANNEL 6

// REPLACE WITH YOUR TRANSMITTER MAC Address
uint8_t broadcastAddress[] = {0xDC,0x54,0x75,0xC3,0x07,0x74};
//uint8_t broadcastAddress[] = {0xDC,0x54,0x75,0xC2,0x60,0xF0};
//uint8_t broadcastAddress[] = {0x0C,0x8B,0x95,0x94,0xE5,0x2C};

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
  struct_message msg;
  if (len != sizeof(msg))  {
    Serial.println("Received bad packet.");
    return;
  }
  memcpy((void *)&msg, (void *)incomingData, len);
  Serial.printf("%c is %s\n",buttonKey[msg.buttonNum],(msg.buttonState?"released":"pressed"));
  esp_now_send(recvInfo->src_addr, incomingData, len); // send back to determine latency
}

void espNowSetup() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

//  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

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
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }
  Serial.print("ESP32 Board MAC Address:  "); Serial.println(WiFi.macAddress());

  espNowSetup();
}

void serialUpdateMAC() {
  static unsigned long lastMACUpdate = 0;
  if (millis() - lastMACUpdate < 10000) return;
  lastMACUpdate = millis();
  Serial.print("ESP32 Board MAC Address:  "); Serial.println(WiFi.macAddress());
}

void loop() {
//  serialUpdateMAC();
}
