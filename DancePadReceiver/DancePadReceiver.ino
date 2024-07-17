/* DancePadReceiver
 * Receivers button state changes from transmitter.
 * Sends packet back to transmitter for confirmation.
 * Sends keyboard button press and release signals through USB. 
 */
#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#warning This sketch should be used when USB is in OTG mode
void setup() {}
void loop() {}
#else

#include "USB.h"
#include "USBHIDKeyboard.h"
#include <esp_now.h>
#include <WiFi.h>

#define ESPNOW_WIFI_CHANNEL 6

USBHIDKeyboard Keyboard;

//#define COMPILE_SERIAL

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

const uint8_t buttonKey[] = {'q' ,'w' ,'e' ,'a' ,'d' ,'z' ,'x' ,'c' ,'7' ,'8' ,'9' ,'4' ,'6' ,'1' ,'2' ,'3'};
//const uint8_t buttonKey[] =   {0x14,0x1A,0x08,0x04,0x07,0x1D,0x1B,0x06,0x5f,0x60,0x61,0x5C,0x5E,0x59,0x5A,0x5B};

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef COMPILE_SERIAL
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  struct_message msg;
  if (len != sizeof(msg))  {
//    Serial.println("Received bad packet.");
    return;
  }
  memcpy((void *)&msg, (void *)incomingData, len);
#ifdef COMPILE_SERIAL
  Serial.printf("%c is %s\n",buttonKey[msg.buttonNum],(msg.buttonState?"released":"pressed"));
#endif
  if (!msg.buttonState) Keyboard.press(buttonKey[msg.buttonNum]);
  else Keyboard.release(buttonKey[msg.buttonNum]);
  esp_now_send(recvInfo->src_addr, incomingData, len); // send back to determine latency
}

void espNowSetup() {
  if (esp_now_init() != ESP_OK) {
#ifdef COMPILE_SERIAL
  Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }

//  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
//    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}

void setup() {
  // Initialize Serial Monitor
#ifdef COMPILE_SERIAL
  Serial.begin(115200);  
  delay(5000);  
  Serial.println("ESP-NOW receive a structure");
#endif
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }
#ifdef COMPILE_SERIAL
  Serial.print("ESP32 Board MAC Address:  "); Serial.println(WiFi.macAddress());
#endif

  espNowSetup();
  Keyboard.begin();
  USB.begin();
}

void serialUpdateMAC() {
  static unsigned long lastMACUpdate = 0;
  if (millis() - lastMACUpdate < 10000) return;
  lastMACUpdate = millis();
#ifdef COMPILE_SERIAL
  Serial.print("ESP32 Board MAC Address:  "); Serial.println(WiFi.macAddress());
#endif
}

void loop() {
//  serialUpdateMAC();
}

#endif
