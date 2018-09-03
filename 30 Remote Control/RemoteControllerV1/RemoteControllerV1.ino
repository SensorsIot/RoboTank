/*
   Robo Tank Remote controller
   Based on Master example of ESP_now

   Analog Pins VP and VN

   Andreas Spiess (2018)

   Attention: The telemetry channel from the controller to the remote does not work yet.

   !!!!!!!!!!! Please use ArduinoJson Library 5.13, not 6.xx beta


  Connections:
  OLED
  GND : GND
  VCC : 3.3V
  SCL : 22
  SDA : 21

  Joystick
  GND : GND
  VCC : 3.3V
  Speed: VP (39)
  Angle: VN (36)

  To recalibrate: connect 27 to ground!

  Further info is in the GitHub documentation.

*/

#include <esp_now.h>
#include <WiFi.h>

#include <ArduinoJson.h>
#include <GlobalDefinitions.h>

#include <Wire.h>
#include <SSD1306.h>
#include <EEPROM.h>

#define WIFI_CHANNEL 1
esp_now_peer_info_t slave;
uint8_t remoteMac[] = {0x24, 0x0A, 0xC4, 0x0D, 0x81, 0xE1};

SSD1306 display(0x3c, 21, 22);

#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define EEPROM_SIZE 64
#define RECALIBRATION_PIN 27
#define MAGIC_NUMBER 12315

const uint8_t maxDataFrameSize = 200;
const esp_now_peer_info_t *peer = &slave;
uint8_t dataToSend[maxDataFrameSize];
byte cnt = 0;
unsigned long teleEntry;


struct calibrationStruct {
  int magicNumber = MAGIC_NUMBER;//'Magic Number - To check to see if the calibration has been performed'
  int xmax = 0;
  int xmin = 511;
  int xzero;
  int ymax = 0;
  int ymin = 511;
  int yzero;
};
calibrationStruct calibration;

int roboSpeed;
int roboAngle;

unsigned long lastDisplay = 0;
unsigned long pressStart = 0;

float roboBattery;
uint8_t peer_addr[6];

void readJoyStick() {
  int rawSpeed = analogRead(36);
  int rawAngle = analogRead(39);

  if (rawSpeed < calibration.yzero - 5) {
    roboSpeed = map(rawSpeed, calibration.ymin, calibration.yzero, -255, 0);
  } else if (rawSpeed > calibration.yzero - 5 && rawSpeed < calibration.yzero + 5) {
    roboSpeed = 0;
  } else if (rawSpeed > calibration.yzero + 5) {
    roboSpeed = map(rawSpeed, calibration.yzero, calibration.ymax, 0, 255);
  }

  if (rawAngle < calibration.xzero - 5) {
    roboAngle = map(rawAngle, calibration.xmin, calibration.xzero, -180, 0);
  } else if (rawAngle > calibration.xzero - 5 && rawAngle < calibration.xzero + 5) {
    roboAngle = 0;
  } else if (rawAngle > calibration.xzero + 5) {
    roboAngle = map(rawAngle, calibration.xzero, calibration.xmax, 0, 180);
  }

  command.speed = roboSpeed;
  command.angle = roboAngle;
  /* Serial.print(command.speed);
    Serial.print(",");
    Serial.println(command.angle);
  */
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
  unsigned long start = millis();
  int y;
  int x;

  //Values for calibration sucess check
  int xInit = analogRead(39);
  int yInit = analogRead(36);

  while (millis() - start < 5000) {
    y = analogRead(36);
    x = analogRead(39);

    y > recalibration.ymax ? recalibration.ymax = y : recalibration.ymax;
    y < recalibration.ymin ? recalibration.ymin = y : recalibration.ymin;

    x > recalibration.xmax ? recalibration.xmax = x : recalibration.xmax;
    x < recalibration.xmin ? recalibration.xmin = x : recalibration.xmin;
  }


  //Check for success
  if (recalibration.xmax - xInit > 110 && xInit - recalibration.xmin > 110 && recalibration.ymax - yInit > 110 && yInit - recalibration.ymin > 110) {
    display.clear();
    display.drawString(0, 0, "Release Joystick\nand WAIT!");
    display.display();
    delay(4000);
    recalibration.xzero = analogRead(39);
    recalibration.yzero = analogRead(36);
    display.clear();
    display.drawString(0, 0, "Calibration\nDONE!");
    display.display();

    Serial.print("XMAX:"); Serial.println(recalibration.xmax);
    Serial.print("XMIN:"); Serial.println(recalibration.xmin);
    Serial.print("YMAX:"); Serial.println(recalibration.ymax);
    Serial.print("YMIN:"); Serial.println(recalibration.ymin);
    Serial.print("XZERO:"); Serial.println(recalibration.xzero);
    Serial.print("YZERO:"); Serial.println(recalibration.yzero);

    EEPROM.put(0, recalibration);

    delay(1000);
  } else {
    display.clear();
    display.drawString(0, 0, "Calibration\nfailed. Trying\nagain.");
    display.display();
    delay(1000);
    calibrateJoystick();
  }
  calibration = recalibration;
  EEPROM.commit();
}

void displaySpeedAngle() {
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
  uint8_t commandJSON[COMMANDLENGTH];
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["speed"] = command.speed;
  root["angle"] = command.angle;
  char jsonChar[COMMANDLENGTH];
  root.printTo(jsonChar, COMMANDLENGTH);

  //  root.printTo(Serial);
  memcpy(commandJSON, jsonChar, sizeof(jsonChar));
  Serial.println(jsonChar);

  int JSONlen = 0;
  while (commandJSON[JSONlen] != '}' && JSONlen < COMMANDLENGTH - 1) JSONlen++; // find end of JSON string
  Serial.println(jsonChar);
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
  delay(50);
}


void setup()
{
  Serial.begin(115200);
  Serial.print("\r\n\r\n");

  analogReadResolution(9);  // 9 bit resolution for the joystick is enough

  pinMode(RECALIBRATION_PIN, INPUT_PULLUP);

  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically();
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
}


void loop()
{
  readJoyStick();
  if (!digitalRead(RECALIBRATION_PIN)) {
    bool pressed = true;
    pressStart = millis();
    display.clear();
    display.drawString(0, 0, "Hold to\nrecalibrate!");
    display.display();
    int i = 0;
    while (millis() - pressStart < 3000) {
      if (digitalRead(RECALIBRATION_PIN)) {
        pressed = false;
        break;
      }
    }
    if (pressed = true && !digitalRead(RECALIBRATION_PIN)) {
      calibrateJoystick();
    }
  }
  if (millis() - lastDisplay > 50) {
    displaySpeedAngle();
    lastDisplay = millis();
  }
  sendData(slave.peer_addr);

  for (cnt = 0; cnt < maxDataFrameSize; cnt++)
  {
    dataToSend[cnt]++;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *json, int data_len)
{
  StaticJsonBuffer<COMMANDLENGTH> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(json);
  if (!root.success()) {
    Serial.println("parseObject() failed");
  } else {
    roboBattery = root["battery"];
    Serial.println();
    Serial.print("Received ");
    Serial.print(roboBattery);
  }
}
