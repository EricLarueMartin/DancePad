/* DancePadTransmitter
 * Reads pins on an MCP32017
 * If measured state doesn't match stored state then the changed button status is sent to receiver.
 * Receiver will send the packet back, and stored button state is updated on receipt of packet.
 * Will only send packets at a minimum period of DEBOUNCE_DELAY.
 */

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MCP23X17.h>

#define ESPNOW_WIFI_CHANNEL 6
#define NUM_BUTTONS 16

#define DEBOUNCE_DELAY 30 // will only send button status updates this often for a single button

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xDC,0x54,0x75,0xC2,0x60,0xF0};
//uint8_t broadcastAddress[] = {0xDC,0x54,0x75,0xC2,0xC1,0x88};
//uint8_t broadcastAddress[] = {0x0C,0x8B,0x95,0x96,0x62,0x14};
//uint8_t broadcastAddress[] = {0x48,0x27,0xE2,0x44,0x6A,0xC8};

typedef struct struct_message {
  bool buttonState;
  uint8_t buttonNum;
  unsigned long timeSent;
} struct_message;

esp_now_peer_info_t peerInfo;

Adafruit_MCP23X17 mcp;

const uint8_t buttonKey[] = {'q' ,'w' ,'e' ,'a' ,'d' ,'z' ,'x' ,'c' ,'7' ,'8' ,'9' ,'4' ,'6' ,'1' ,'2' ,'3'};

bool buttonState[256];
unsigned long lastChanged[256];

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  static struct_message msg;
  if (len == sizeof(msg)) {
    memcpy((void *)&msg, (void *)incomingData, len);
    buttonState[msg.buttonNum] = msg.buttonState;
    Serial.printf("received button %i state %i in %i ms round trip\n",
                  msg.buttonNum, msg.buttonState, millis() - msg.timeSent);
  } else
    Serial.printf("ERROR! packet size expected %d receive %d\n", sizeof(struct_message), len);
}

void mcpSetup() {
  if (!mcp.begin_I2C()) {
    Serial.println("MCP23017 setup error.");
    return;
  }

  // configure pin for input with pull up
  for (int ctPin = 0; ctPin < 16; ++ctPin) {
    mcp.pinMode(ctPin, INPUT_PULLUP);
    lastChanged[ctPin] = 0;
    buttonState[ctPin] = mcp.digitalRead(ctPin);
  }
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
  Serial.println("ESP-NOW transmit a structure");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }
  Serial.print("ESP32 Board MAC Address:  "); Serial.println(WiFi.macAddress());

  espNowSetup();
  mcpSetup();
}

void serialUpdateMAC() {
  static long lastMACUpdate = 0;
  if (millis() - lastMACUpdate < 10000) return;
  lastMACUpdate = millis();
  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
}

void mcpLoop() {
  for(int ctPin = 0; ctPin < NUM_BUTTONS; ++ctPin) {
    bool readState = mcp.digitalRead(ctPin);
    uint8_t buttonNum = buttonKey[ctPin];
    if (readState != buttonState[buttonNum] && millis() - lastChanged[buttonNum] > DEBOUNCE_DELAY)  {
      lastChanged[buttonNum] = millis();
      struct_message msg;
      msg.timeSent = millis();
      msg.buttonNum = buttonNum;
      msg.buttonState = readState;
      esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
//      buttonState[msg.buttonNum] = msg.buttonState;
      Serial.printf("Button %i changed to %i.\n",ctPin,readState);
    }
  }  
}

void mcpTest() {
  static long lastUpdate = 0;
  if (millis() - lastUpdate < 10000) return;
  lastUpdate = millis();
  for(int ctPin = 0; ctPin < NUM_BUTTONS; ++ctPin) {
    bool readState = mcp.digitalRead(ctPin);
    Serial.printf("Button %i state %i.\n",ctPin,readState);
  }  
}


void loop() {
//  serialUpdateMAC();
  mcpLoop();
//  mcpTest();
}
