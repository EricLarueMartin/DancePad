#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_MCP23X17.h>

#define NUM_BUTTONS 16
#define DEBOUNCE_DELAY 100

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xDC, 0x54, 0x75, 0xC2, 0xC1, 0x88};

typedef struct struct_message {
  bool buttonState;
  int buttonNum;
  unsigned long timeSent;
} struct_message;

esp_now_peer_info_t peerInfo;

Adafruit_MCP23X17 mcp;

bool buttonState[NUM_BUTTONS];
unsigned long lastChanged[NUM_BUTTONS];

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
  Serial.println("ESP-NOW transmit a structure");
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

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
    if (readState != buttonState[ctPin] && millis() - lastChanged[ctPin] > DEBOUNCE_DELAY)  {
      buttonState[ctPin] = readState;
      lastChanged[ctPin] = millis();
      struct_message msg;
      msg.timeSent = millis();
      msg.buttonNum = ctPin;
      msg.buttonState = readState;
      esp_now_send(broadcastAddress, (uint8_t *)&msg, sizeof(msg));
      Serial.printf("Button %i changed to %i.\n",ctPin,readState);
    }
  }  
}

void loop() {
  serialUpdateMAC();
  mcpLoop();
}
