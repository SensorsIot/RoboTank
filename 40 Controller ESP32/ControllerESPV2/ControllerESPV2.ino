/**
   /*
   Robo Tank controller
   Based on Slave example of ESP_now

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

#define WIFI_CHANNEL 1

//uint8_t masterDeviceMac[] = {0x24, 0x0A, 0xC4, 0x0D, 0x17, 0xE8};
//uint8_t masterDeviceMac[] = {0x24, 0x0A, 0xC4, 0x0D, 0x17, 0xE9};

//uint8_t masterDeviceMac[] = {0x24, 0x0A, 0xC4, 0x0D, 0x81, 0xE0}; // worked
//uint8_t masterDeviceMac[] ={0x24, 0x0A, 0xC4, 0x0D, 0x81, 0xE1};
//uint8_t masterDeviceMac[] = {0x30, 0xAE, 0xA4, 0x1A, 0xCD, 0x88}; // Lolin32
uint8_t masterDeviceMac[] = {0x24, 0x0A, 0xC4, 0x0D, 0x4B, 0xD8}; // Remote Control


esp_now_peer_info_t master;
const esp_now_peer_info_t *masterNode = &master;
const byte maxDataFrameSize = 200;
uint8_t dataToSend[maxDataFrameSize];
byte cnt = 0;
esp_err_t sendResult;

//IO
#define pwm_motor1 12 //Has to be a PWM output (max 20KHZ)
#define pwm_motor2 14 //Has to be a PWM output (max 20KHZ)
#define A1 27
#define A2  13
#define B1  26
#define B2 25

int roboSpeed;
int roboAngle;
int motor1speed = 0;
int motor2speed = 0;
int lastSpeed1, lastSpeed2;

unsigned long lastDisplay = 0;

SSD1306 display(0x3c, 21, 22);

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

  if (millis() - lastDisplay > 50) {
 //   displaySpeedAngle();
    lastDisplay = millis();
  }
  yield();
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

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  uint8_t commandJSON[COMMANDLENGTH];
  char jsonChar[COMMANDLENGTH];

  Serial.printf("\r\nReceived\t%d Bytes\t%d", data_len, data[0]);
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(data);

  if (!root.success()) {
    Serial.println("parseObject() failed");
  } else {
    roboSpeed = root["speed"];
    roboAngle = root["angle"];
    Serial.println();
    Serial.print(roboSpeed);
    Serial.print(",");
    Serial.println(roboAngle);
  }

  root["battery"] = telemetry.batteryVoltage;
  root.printTo(jsonChar, COMMANDLENGTH);
  root.printTo(Serial);
  memcpy(commandJSON, jsonChar, sizeof(jsonChar));

  int JSONlen = 0;
  while (commandJSON[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 1) JSONlen++; // find end of JSON string
  Serial.println(JSONlen);

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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void displaySpeedAngle() {
  display.clear();
  display.drawString(0, 0, "Speed:");
  display.drawString(0, 16, "Angle:");
  display.drawString(55, 0, String(roboSpeed));
  display.drawString(55, 16, String(roboAngle));
  display.display();
}
