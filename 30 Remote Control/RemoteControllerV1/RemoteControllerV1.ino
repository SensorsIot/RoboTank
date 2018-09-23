/*
   Robo Tank Remote controller
   Based on Master example of ESP_now

   Analog Pins VP and VN

   Andreas Spiess (2018)

   Attention: The telemetry channel from the controller to the remote does not work yet.

   !!!!!!!!!!! Please use ArduinoJson Library 5.13, not 6.xx beta

Further info is in the GitHub documentation.

*/

#include <ArduinoJson.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SSD1306.h>

#include "GlobalDefinitions.h"
#include "RC_IO.h"
#include "JoystickClass.cpp"

#include <EEPROM.h>

#define WIFI_CHANNEL 1
esp_now_peer_info_t slave;
const esp_now_peer_info_t *peer = &slave;
uint8_t remoteMac[] = MASTER_MAC;
unsigned long  sendUpdate_ms = 100;
const uint8_t maxDataFrameSize = 200;
byte cnt = 0;

unsigned long teleEntry;

SSD1306 display(0x3c, OLED_SDA, OLED_SCL);
unsigned long  displayUpdate_ms = 50;

#define EEPROM_SIZE 64
#define MAGIC_NUMBER 12315

Joystick drive_stick(JS1_VX, JS1_VY);

struct calibrationStruct {
  int magicNumber = MAGIC_NUMBER;//'Magic Number - To check to see if the calibration has been performed'
  StickCalibration drive_stick_cal;
};
calibrationStruct calibration;

int roboSpeed;
int roboAngle;

unsigned long lastDisplay = 0;
unsigned long lastSend = 0;
unsigned long pressStart = 0;

float roboBattery;

void setup()
{
  Serial.begin(115200);
  Serial.print("\r\n\r\n");

  WiFi.mode(WIFI_STA);
  Serial.println( WiFi.macAddress() );
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESP NOW INIT!");
  }
  else
  {
    Serial.println("ESP NOW INIT FAILED....");
  }

  memcpy( &slave.peer_addr, &remoteMac, 6 );
  slave.channel = WIFI_CHANNEL;
  slave.encrypt = 0;
  if ( esp_now_add_peer(peer) == ESP_OK)
  {
    Serial.println("Added Peer!");
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  analogReadResolution(9);  // 9 bit resolution for the joystick is enough

  pinMode(RECALIBRATION_PIN, INPUT_PULLUP);

  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically();   //!TODO: Make Configurable
  display.display();

  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialise EEPROM");
    Serial.println("Restarting...");
    delay(1000);
    ESP.restart();
  }

  calibration.magicNumber = MAGIC_NUMBER;
  calibrationStruct calibrationFromEEPROM;
  EEPROM.get(0, calibrationFromEEPROM);

  if (calibrationFromEEPROM.magicNumber != calibration.magicNumber) {
    calibrateJoystick();
  } else {
    calibration = calibrationFromEEPROM;

    display.clear();
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 0, "Previous\nCalibration found!\n...Booting...");
    display.display();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
  }

  drive_stick.setCalibration( calibration.drive_stick_cal );
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  ReceiveMessage(data, data_len);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? " Delivery Success" : " Delivery Fail");
}

void loop()
{
  drive_stick.update();

  roboSpeed = constrain ( sqrt( sq((float)drive_stick.getXConvertedValue()) + sq((float)drive_stick.getYConvertedValue()) ) * ( drive_stick.getYConvertedValue()>0 ? 1 : -1 ), -510, 510) / 2;
  roboAngle = map(drive_stick.getXConvertedValue(),drive_stick.minPositionValue, drive_stick.maxPositionValue, -90, 90);  //Arduino doesn't have inverse trig

  if (!digitalRead(RECALIBRATION_PIN)) {    //! can you change pin polarity to make this clearer?
    bool pressed = true;
    pressStart = millis();
    display.clear();
    display.drawString(0, 0, "Hold to\nrecalibrate!");
    display.display();
    int i = 0;
    while (millis() - pressStart < 3000) {    //! This is blocking the main loop, fix
      if (digitalRead(RECALIBRATION_PIN)) {
        pressed = false;
        break;
      }
    }
    if (pressed = true && !digitalRead(RECALIBRATION_PIN)) {
      calibrateJoystick();
    }
  }
  
  if (millis() - lastDisplay > displayUpdate_ms) {
    displayUpdate();
    lastDisplay = millis();
  }
  if (millis() - lastSend > sendUpdate_ms) {
    sendData(slave.peer_addr);
    lastSend = millis();
  }
}

void calibrateJoystick() {
  calibrationStruct recalibration; //Create new struct variable, otherwise input validation won't work.
  display.clear();
  display.drawString(0, 0, "Starting \nCalibration!");
  display.display();
  delay(500);
  display.clear();
  display.drawString(0, 0, "Move Joystick\nthrough outermost\ncircle!");
  display.display();

  drive_stick.calibrateJoystickEdges();
  
  display.clear();
  display.drawString(0, 0, "Release Joystick\nand WAIT!");
  display.display();
  delay(4000);
  
  drive_stick.calibrateJoystickNeutral();

  display.clear();
  display.drawString(0, 0, "Calibration\nDONE!");
  display.display();

  recalibration.drive_stick_cal = drive_stick.getCalibration();

  Serial.print("XMAX:"); Serial.println(recalibration.drive_stick_cal.xMax);
  Serial.print("XMIN:"); Serial.println(recalibration.drive_stick_cal.xMin);
  Serial.print("YMAX:"); Serial.println(recalibration.drive_stick_cal.yMax);
  Serial.print("YMIN:"); Serial.println(recalibration.drive_stick_cal.yMin);
  Serial.print("XZERO:"); Serial.println(recalibration.drive_stick_cal.xZero);
  Serial.print("YZERO:"); Serial.println(recalibration.drive_stick_cal.yZero);

  EEPROM.put(0, recalibration);

  calibration = recalibration;
  EEPROM.commit();
}

void displayUpdate() {
  display.clear();
  display.drawString(0, 0, "Speed:");
  display.drawString(0, 16, "Angle:");
  display.drawString(0, 32, "Battery:");
  
  display.drawString(55, 0, String(roboSpeed));
  display.drawString(55, 16, String(roboAngle));
  display.drawString(55, 32, String(roboBattery));
  display.display();
}

void sendData(byte *peer_addr) {
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["speed"] = roboSpeed;
  root["angle"] = roboAngle;
  char jsonChar[COMMANDLENGTH];
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
  esp_err_t result = esp_now_send(peer_addr, commandJSON, JSONlen + 1);
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
    Serial.printf("Unknown Error: \t%d", result);
  }
  delay(50);
}

void ReceiveMessage(const uint8_t *data, int data_len)
{
  Serial.print("Received ");
  Serial.print(data_len);
  Serial.print(" bytes:");
  char jsonChar[COMMANDLENGTH];
  jsonChar[COMMANDLENGTH-1]=0;
  strncpy( jsonChar, (char*)data, COMMANDLENGTH-1 );
  Serial.print(jsonChar);
  Serial.print("\t");
  
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  Serial.print("parseObject ");
  JsonObject& root = jsonBuffer.parseObject(jsonChar);
  if (!root.success()) {
    Serial.println("failed!!!");
  } else {
    Serial.println("parsed");
    roboBattery = root["battery"];
    Serial.print("Battery= ");
    Serial.println(roboBattery);
  }
}
