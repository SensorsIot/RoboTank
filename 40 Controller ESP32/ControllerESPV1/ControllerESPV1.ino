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
#include <Wire.h>
#include <SSD1306.h>

#include "GlobalDefinitions.h"
#include "Tank_IO.h"
#define HasDisplay false

#define WIFI_CHANNEL 1
esp_now_peer_info_t master;
const esp_now_peer_info_t *masterNode = &master;
uint8_t masterDeviceMac[] = REMOTE_MAC; // Remote Control
const byte maxDataFrameSize = 200;
byte cnt = 0;

uint8_t dataToSend[maxDataFrameSize];

#if HasDisplay
SSD1306 display(0x3c, OLED_SDA, OLED_SCL);
unsigned long  displayUpdate_ms = 50;
unsigned long lastDisplay = 0;
#endif 

//Motor Stuff
#define motor1Chan 0
#define motor2Chan 1

int roboSpeed;
int roboAngle;
int motor1speed = 0;
int motor2speed = 0;
int lastSpeed1, lastSpeed2;

void setup()
{
  Serial.begin(115200);
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

  //Add the master node to this slave node
  memcpy( &master.peer_addr, &masterDeviceMac, 6 );
  master.channel = WIFI_CHANNEL;
  master.encrypt = 0;
  master.ifidx = ESP_IF_WIFI_AP;    //!TODO: Investigate
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
}

void loop()
{
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

#if HasDisplay
  if (millis() - lastDisplay > displayUpdate_ms) {
    displayUpdate();
    lastDisplay = millis();
  }
#endif
  
  //yield();  //!TODO: Is this needed?
  
}

void setMotorSpeed(int speed1, int speed2) {
  //check if signal has exceeded the maximum speed and correct it if necessary
  //!TODO: Simplify using Max and Min
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

  //create control signals
  //!TODO: Simplify
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
  ledcWrite(motor1Chan, abs(speed1));
  ledcWrite(motor2Chan, abs(speed2));

  Serial.print(" motorSpeed1 ");
  Serial.print( motor1speed);
  Serial.print(" motorSpeed2 ");
  Serial.println(motor2speed);

  Serial.print(" Speed1 ");
  Serial.print( speed1);
  Serial.print(" Speed2 ");
  Serial.println(speed2);
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  char jsonChar[COMMANDLENGTH];
  int len = std::min(COMMANDLENGTH, data_len);
  memcpy( jsonChar, data, len );
  jsonChar[len]=0;
  Serial.print("Received ");
  Serial.print(len);
  Serial.print(" bytes:");
  Serial.println(jsonChar);
  
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(jsonChar);
  if (!root.success()) {
    Serial.println("parseObject() failed");
  } else {
    Serial.println("parsed");
    roboSpeed = root["speed"];
    roboAngle = root["angle"];
    Serial.println();
    Serial.print(roboSpeed);
    Serial.print(",");
    Serial.println(roboAngle);
  }

  root["battery"] = telemetry.batteryVoltage;
  root.printTo(jsonChar, COMMANDLENGTH);

  // find length of JSON string being sent
  int JSONlen = 0;
  while (jsonChar[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 2) JSONlen++; 
  JSONlen++;  //Account for starting at 0
  jsonChar[JSONlen] = 0;

  Serial.print("Sending ");
  Serial.print(JSONlen);
  Serial.print(" bytes:");
  Serial.print(jsonChar);
  Serial.print("\t");

  uint8_t commandJSON[COMMANDLENGTH];
  memcpy(commandJSON, jsonChar, JSONlen + 1);
  esp_err_t sendResult = esp_now_send(master.peer_addr, commandJSON, JSONlen + 1);
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
  } else if (sendResult == ESP_ERR_ESPNOW_IF) {
    Serial.println("Interface Error.");
  } else {
    Serial.printf("Unknown Error: \t%d", sendResult);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " Delivery Success" : " Delivery Fail");
}

#if HasDisplay
void displayUpdate() {
  display.clear();
  display.drawString(0, 0, "Speed:");
  display.drawString(0, 16, "Angle:");
  display.drawString(55, 0, String(roboSpeed));
  display.drawString(55, 16, String(roboAngle));
  display.display();
}
#endif
