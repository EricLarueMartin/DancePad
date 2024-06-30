// ESP-NOW transmit a structure

/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x4C,0x75,0x25,0xC6,0x5C,0xE0}; //{ 0x84, 0xCC, 0xA8, 0x7A, 0x56, 0x6C };  //{0x24,0x62,0xAB,0xE0,0x64,0x54};

esp_now_peer_info_t peerInfo;

// test structure
struct __attribute__((packed)) Data {
  int16_t seq;  // sequence number
  int32_t distance;
  float voltage;
  char text[50];
} data = { 0, 56, 3.14159, "hello test" };  // sample data

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println("ESP-NOW transmitting a structure");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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
}

void loop() {
  // Send data structure via ESP-NOW
  Serial.printf("seq %d distaance %ld voltage %f text '%s'\n",
                (int)data.seq, (long)data.distance, data.voltage, data.text);
  esp_now_send(broadcastAddress, (uint8_t *)&data, sizeof(data));
  delay(1000);
  data.seq++;  // update data ready for next transmission
  data.distance += 10;
  data.voltage += 2.5;
  data.text[9]++;
}
