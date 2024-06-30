// ESP-NOW receive a structure

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

// test structure
struct __attribute__((packed)) Data {
  int16_t seq;  // sequence number
  int32_t distance;
  float voltage;
  char text[50];
} data;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  static int16_t seqExpected = 0, errors=0;
  Serial.printf("Bytes received: %d ", len);
  if (len == sizeof(data)) {
    memcpy((void *)&data, (void *)incomingData, len);
    Serial.printf("seq %d distaance %ld voltage %f text '%s' errors %d\n",
                  (int)data.seq, (long)data.distance, data.voltage, data.text, errors);
    if (data.seq != seqExpected)  // check sequence number received
      Serial.printf("Error! seq expected %d received %d errors %d\n", seqExpected, data.seq, ++errors);
    seqExpected = data.seq;  // set sequence number ready for next data
    seqExpected++;
  } else
    Serial.printf("ERROR! packek size expected %d receive %d errors %d\n", sizeof(data), len, ++errors);
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

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
}
