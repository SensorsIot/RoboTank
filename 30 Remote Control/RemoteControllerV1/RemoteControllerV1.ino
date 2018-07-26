/*
   Robo Tank Remote controller
   Based on Master example of ESP_now

   Analog Pins VP and VN

   Andreas Spiess (2018)

   Attention: The telemetry channel from the controller to the remote does not work yet.

   !!!!!!!!!!! Please use ArduinoJson Library 5.13, not 6.xx beta

*/

#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <GlobalDefinitions.h>

#include <Wire.h>
#include <SSD1306.h>

SSD1306 display(0x3c, 21,22);

#define CHANNEL 3
#define PRINTSCANRESULTS 0

int xmax = 0;
int xmin = 511;
int xzero;
int ymax = 0;
int ymin = 511;
int yzero;

int roboSpeed;
int roboAngle;

unsigned long lastDisplay = 0;

float roboBattery;
uint8_t peer_addr[6];

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    InitESPNow();
    esp_now_register_recv_cb(OnDataRecv);
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x%c",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slaves[SlaveCnt].channel = CHANNEL; // pick a channel
        slaves[SlaveCnt].encrypt = 0; // no encryption
        SlaveCnt++;
      }
    }
    //   *peer_addr = slaves[0].peer_addr;
  }

  if (SlaveCnt > 0) {
    Serial.print(SlaveCnt); Serial.println(" Slave(s) found, processing..");
  } else {
    Serial.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      const esp_now_peer_info_t *peer = &slaves[i];
      const uint8_t *peer_addr = slaves[i].peer_addr;
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(peer_addr);
      if (exists) {
        // Slave already paired.
        Serial.println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(peer);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
  }
}

void readJoyStick() {
  int rawSpeed = analogRead(36);
  int rawAngle = analogRead(39);

  if(rawSpeed < yzero - 5){
    roboSpeed = map(rawSpeed,ymin,yzero,-255,0);
  } else if(rawSpeed > yzero - 5 && rawSpeed < yzero + 5){
    roboSpeed = 0;
  } else if(rawSpeed > yzero + 5){
    roboSpeed = map(rawSpeed,yzero,ymax,0,255);
  }

  if(rawAngle < xzero - 5){
    roboAngle = map(rawAngle,xmin,xzero,-180,0);
  } else if(rawAngle > xzero - 5 && rawAngle < xzero + 5){
    roboAngle = 0;
  } else if(rawAngle > xzero + 5){
    roboAngle = map(rawAngle,xzero,xmax,0,180);
  }

  command.speed = roboSpeed;
  command.angle = roboAngle;
  /* Serial.print(command.speed);
    Serial.print(",");
    Serial.println(command.angle);
  */
}

void calibrate(){
  display.drawString(0,0,"Starting \nCalibration!");
  display.display();
  delay(500);
  display.clear();
  display.drawString(0,0,"Move Joystick\nthrough outermost\ncircle!");
  display.display();
  unsigned long start = millis();
  int y;
  int x;
  while(millis() - start < 5000){
    y = analogRead(36);
    x = analogRead(39);

    y > ymax ? ymax = y : ymax;
    y < ymin ? ymin = y : ymin;

    x > xmax ? xmax = x : xmax;
    x < xmin ? xmin = x : xmin;
  }
  display.clear();
  display.drawString(0,0,"Release Joystick\nand WAIT!");
  display.display();
  delay(4000);
  xzero = analogRead(39);
  yzero = analogRead(36);
  display.clear();
  display.drawString(0,0,"Calibration\nDONE!");
  display.display();

  Serial.print("XMAX:");Serial.println(xmax);
  Serial.print("XMIN:");Serial.println(xmin);
  Serial.print("YMAX:");Serial.println(ymax);
  Serial.print("YMIN:");Serial.println(ymin);
  Serial.print("XZERO:");Serial.println(xzero);
  Serial.print("YZERO:");Serial.println(yzero);
  
  delay(1000);
}

void displaySpeedAngle(){
  display.clear();
  display.drawString(0,0,"Speed:");
  display.drawString(0,16,"Angle:");
  display.drawString(55,0, String(roboSpeed));
  display.drawString(55,16, String(roboAngle));
  display.display();
}

void sendData(byte *peer_addr) {
  uint8_t commandJSON[COMMANDLENGTH];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["speed"] = command.speed;
  root["angle"] = command.angle;
  char jsonChar[COMMANDLENGTH];
  root.printTo(jsonChar, COMMANDLENGTH);
  //  root.printTo(Serial);
  memcpy(commandJSON, jsonChar, sizeof(jsonChar));
  int JSONlen = 0;
  while (commandJSON[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 1) JSONlen++; // find end of JSON string
  for (int i = 0; i < SlaveCnt; i++) {
    //    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) { // print only for first slave
      // Serial.print("Sending: ");
    }
    esp_err_t result = esp_now_send(peer_addr, commandJSON, JSONlen + 1);
    // Serial.print("Send Status: ");
    if (result == ESP_OK) {
      //   Serial.println("Success");
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
  }
  delay(50);
}

// callback when data is recv from Slave
void OnDataRecv(const uint8_t *slave_peer_addr, const uint8_t *json, int data_len) {
  char macStr[18];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(json);
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print("Last Packet Recv from: ");
  // Serial.println(macStr);

  if (!root.success()) {
    Serial.println("parseObject() failed");
  } else {
    roboBattery = root["battery"];
    Serial.println("Received ");
    Serial.print(roboBattery);
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  //  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  Serial.begin(115200);
  analogReadResolution(9);  // 9 bit resolution for the joystick is enough

  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically();
  display.display();
  calibrate();

  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow/Multi-Slave/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());

  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (SlaveCnt > 0) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    manageSlave();
    // pair success or already paired
    // Send data to device
  } else {
    // No slave found to process
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  readJoyStick();
  sendData(slaves[0].peer_addr);

  if(millis() - lastDisplay > 50){
    displaySpeedAngle();
    lastDisplay = millis();
  }
}
