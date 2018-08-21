/**
   /*
   Robo Tank controller
   Based on Slave example of ESP_now

   Andreas Spiess (2018)


   Attention: The telemetry channel from the controller to the remote does not work yet.

   !!!!!!!!!!! Please use ArduinoJson Library 5.13, not 6.xx beta

*/
#include <ArduinoJson.h>
#include <esp_now.h>
#include <WiFi.h>
#include <GlobalDefinitions.h>

#include <Wire.h>
#include <SSD1306.h>


int roboSpeed;
int roboAngle;
int motor1speed = 0;
int motor2speed = 0;
int lastSpeed1, lastSpeed2;

unsigned long entry;
byte masterMAC[6];

//IO
#define pwm_motor1 12 //Has to be a PWM output (max 20KHZ)
#define pwm_motor2 14 //Has to be a PWM output (max 20KHZ)
#define A1 27
#define A2  13
#define B1  26
#define B2 25

uint8_t masterDeviceMac[] = {0x24, 0x0A, 0xC4, 0x0D, 0x4B, 0xD8}; // Remote Control

#define WIFI_CHANNEL 1
esp_now_peer_info_t master;
const esp_now_peer_info_t *masterNode = &master;
const byte maxDataFrameSize = 200;
uint8_t dataToSend[maxDataFrameSize];
byte cnt = 0;
esp_err_t sendResult;



SSD1306 display(0x3c, 21, 22);

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


/*

  void sendData(byte *peer_addr) {


  uint8_t commandJSON[COMMANDLENGTH];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["battery"] = telemetry.batteryVoltage;
  char jsonChar[COMMANDLENGTH];
  root.printTo(jsonChar, COMMANDLENGTH);
  root.printTo(Serial);

  for (int i = 0; i < 6; i++) {
    Serial.print(peer_addr[i], HEX);
    Serial.print(":");
  }

  Serial.println();
  memcpy(commandJSON, jsonChar, sizeof(jsonChar));
  int JSONlen = 0;
  while (commandJSON[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 1) JSONlen++; // find end of JSON string
  Serial.println(JSONlen);



  esp_err_t result = esp_now_send(master.peer_addr, commandJSON, JSONlen + 1);
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

*/

byte charToByte(char in) {
  if (in > '@' )return in - 'A' + 10;
  else return in - '0';
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  char jsonChar[COMMANDLENGTH];
  //  const char hi[30] ={'Test'};
  String hh;
  Serial.printf("\r\nReceived\t%d Bytes\t%d", data_len, data[0]);

  uint8_t commandJSON[COMMANDLENGTH];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(data);
  for (int i = 0; i < 6; i++) masterMAC[i] = mac_addr[i];


  if (!root.success()) {
    Serial.println("parseObject() failed");
  } else {
    roboSpeed = root["speed"];
    roboAngle = root["angle"];
    String hi = root["Master"];
    hh = hi;
  }
  Serial.println("");
  Serial.println("MAC ");
  for (int L = 0; L < 6; L ++) {
    masterMAC[L] = 16 * charToByte(hh[L * 3]) + charToByte(hh[(L * 3 + 1)]);
    Serial.print(masterMAC[L], HEX);
    Serial.print("*");
  }
  Serial.println();
  Serial.print(roboSpeed);
  Serial.print(",");
  Serial.println(roboAngle);


  // Transmitting
  root["battery"] = telemetry.batteryVoltage;
  root.printTo(jsonChar, COMMANDLENGTH);
  root.printTo(Serial);
  memcpy( dataToSend, jsonChar, data_len );

  Serial.println();
  memcpy(commandJSON, jsonChar, sizeof(jsonChar));
  int JSONlen = 0;
  while (commandJSON[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 1) JSONlen++; // find end of JSON string
  Serial.println(JSONlen);
  //Add the master node to this slave node
  memcpy( &master.peer_addr, &masterDeviceMac, 6 );

  Serial.println("");
  Serial.print("NEXT MAC ");
  for (int L = 0; L < 6; L ++) {
    Serial.print(master.peer_addr[L], HEX);
    Serial.print("*");
  }
  Serial.println();
  

  sendResult = esp_now_send(master.peer_addr, commandJSON, JSONlen + 1);
  if (sendResult == ESP_OK) {
    Serial.println("Success");
  } else if (sendResult == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (sendResult == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (sendResult == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (sendResult == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (sendResult == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  }
  else if (sendResult == ESP_ERR_ESPNOW_IF) {
    Serial.println("Interface Error.");
  }   else {
    Serial.printf("\r\nNot sure what happened\t%d", sendResult);
  }
}


/*
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
    roboSpeed = root["speed"];
    roboAngle = root["angle"];
    String hi = root["Master"];

    for (int L = 0; L < 6; L ++) {
      masterMAC[L] = 16 * charToByte(hi[L * 3]) + charToByte(hi[(L * 3 + 1)]);
    }
    Serial.print(hi);
    Serial.print(",");
    Serial.print(roboSpeed);
    Serial.print(",");
    Serial.println(roboAngle);
  }
  }
*/

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr1, esp_now_send_status_t status) {
  char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void displaySpeedAngle() {
  display.clear();
  display.drawString(0, 0, "Speed:");
  display.drawString(0, 16, "Angle:");
  display.drawString(55, 0, String(roboSpeed));
  display.drawString(55, 16, String(roboAngle));
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");

  pinMode(pwm_motor1, OUTPUT);
  pinMode(pwm_motor2, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);

  ledcSetup(0, 1000, 8);
  ledcAttachPin(pwm_motor1, 0);

  ledcSetup(1, 1000, 8);
  ledcAttachPin(pwm_motor2, 1);

  Serial.print("\r\n\r\n");
  WiFi.mode(WIFI_AP_STA);
  Serial.println( WiFi.softAPmacAddress() );
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success!");
  }
  else
  {
    Serial.println("ESPNow Init Failed....");
  }
  master.channel = WIFI_CHANNEL;
  master.encrypt = 0;
  master.ifidx = ESP_IF_WIFI_AP;
  //Add the remote master node to this slave node
  if ( esp_now_add_peer(masterNode) == ESP_OK)
  {
    Serial.println("Added Master Node!");
  }
  else
  {
    Serial.println("Master Node could not be added...");
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);


  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically();
  display.display();
}

void loop() {
  telemetry.batteryVoltage = 10.6;
  //controll forward or backwardspeed speed
  motor1speed = roboSpeed;
  motor2speed = roboSpeed;

  //ajusting to the angle
  if (roboSpeed != 0) {
    motor1speed = motor1speed + roboAngle;
    motor2speed = motor2speed - roboAngle;
  } else {
    motor1speed = 0;
    motor2speed = 0;
  }

  /*
    Serial.print("lastSpeed1 ");
    Serial.print(lastSpeed1 );
    Serial.print(" lastSpeed2 ");
    Serial.print(lastSpeed2 );
    Serial.print(" motorSpeed1 ");
    Serial.print( motor1speed);
    Serial.print(" motorSpeed2 ");
    Serial.println(motor2speed);
  */
  if (abs(lastSpeed1 - motor1speed) > 5 || abs(lastSpeed2 - motor2speed) > 5 ) setMotorSpeed(motor1speed, motor2speed);
  lastSpeed1 = motor1speed;
  lastSpeed2 = motor2speed;

  if (millis() - lastDisplay > 50) {
    displaySpeedAngle();
    lastDisplay = millis();
  }

}

void setMotorSpeed(int speed1, int speed2) {
  //check if signal has ecceeded the maximum speed and correct it if necessary
  if (speed1 > 255) {
    speed1 = 255;
  }
  else if (speed1 < -255) {
    speed1 = -255;
  }

  if (speed2 > 255) {
    speed2 = 255;
  }
  else if (speed2 < -255) {
    speed2 = -255;
  }

  //create conrollsignals
  if (speed1 == 0) {
    digitalWrite(A1, LOW);
    digitalWrite(B1, LOW);
  }
  else if (speed1 >= 0) {
    digitalWrite(A1, HIGH);
    digitalWrite(B1, LOW);
  }
  else {
    digitalWrite(A1, LOW);
    digitalWrite(B1, HIGH);
  }

  if (speed2 == 0) {
    digitalWrite(A2, LOW);
    digitalWrite(B2, LOW);
  }
  else if (speed2 >= 0) {
    digitalWrite(A2, HIGH);
    digitalWrite(B2, LOW);
  }
  else {
    digitalWrite(A2, LOW);
    digitalWrite(B2, HIGH);
  }
  ledcWrite(0, abs(speed1));
  ledcWrite(1, abs(speed2));

  Serial.print(" motorSpeed1 ");
  Serial.print( motor1speed);
  Serial.print(" motorSpeed2 ");
  Serial.println(motor2speed);

  Serial.print(" Speed1 ");
  Serial.print( speed1);
  Serial.print(" Speed2 ");
  Serial.println(speed2);
}

