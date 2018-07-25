/**
   /*
   Robo Tank controller
   Based on Slave example of ESP_now

   Andreas Spiess (2018)


   Attention: The telemetry channel from the controller to the remote does not work yet.

*/
#include <ArduinoJson.h>
#include <esp_now.h>
#include <WiFi.h>
#include <GlobalDefinitions.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CHANNEL 1

unsigned long entry;
byte masterMAC[6];

Adafruit_SSD1306 display = Adafruit_SSD1306();

unsigned long lastDisplay = 0;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  char* SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void sendData(byte *peer_addr) {
  uint8_t commandJSON[COMMANDLENGTH];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["battery"] = telemetry.batteryVoltage;
  char jsonChar[COMMANDLENGTH];
  root.printTo(jsonChar, COMMANDLENGTH);
  root.printTo(Serial);
  Serial.println();
  memcpy(commandJSON, jsonChar, sizeof(jsonChar));
  int JSONlen = 0;
  while (commandJSON[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 1) JSONlen++; // find end of JSON string
  Serial.println(JSONlen);
  esp_err_t result = esp_now_send(peer_addr, commandJSON, JSONlen + 1);
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
  delay(50);
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *json, int data_len) {
  char macStr[18];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(json);
  for (int i = 0; i < 6; i++) masterMAC[i] = mac_addr[i];

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           masterMAC[0], masterMAC[1], masterMAC[2], masterMAC[3], masterMAC[4], masterMAC[5]);
 // Serial.print("Last Packet Recv from: ");
 // Serial.println(macStr);

  if (!root.success()) {
    Serial.println("parseObject() failed");
  } else {
    int roboSpeed = root["speed"];
    int roboAngle = root["angle"];

    roboSpeed -= SPEED_CORRECT;
    roboAngle -= ANGLE_CORRECT;

    roboSpeed = map(roboSpeed,0,511,-255,255);
    roboAngle = map(roboAngle,0,511,-180,180);
    
    roboSpeed < 2 && roboSpeed > -2 ? roboSpeed = 0 : roboSpeed;
    roboAngle < 2 && roboAngle > -2 ? roboAngle = 0 : roboAngle;
    
    float roboBattery = root["battery"];
    
    Serial.print(roboSpeed);
    Serial.print(",");
    Serial.println(roboAngle);

    if(millis() - lastDisplay > 1000){
      display.setCursor(42,0);
      display.print(roboSpeed);
      display.print("   ");
      display.setCursor(42,8);
      display.print(roboAngle);
      display.print("   ");
      display.display();
      lastDisplay = millis();
    }
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr1, esp_now_send_status_t status) {
  char macStr[18];
 // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
 //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
 // Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);

  display.begin(SSD1306_SWITCHCAPVCC,0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(0,0);
  display.clearDisplay();
  display.println("Speed:");
  display.println("Angle:");
  display.display();
}

void loop() {
/* if (millis() > entry + 1000) {
    Serial.print("Sending ");
    telemetry.batteryVoltage = 10.6;
    for (int i = 0; i < 6; i++) Serial.println(masterMAC[i], HEX);

   sendData(masterMAC);
    entry = millis();
  }
*/
}
