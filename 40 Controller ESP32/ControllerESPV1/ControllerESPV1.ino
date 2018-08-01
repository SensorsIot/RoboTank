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

#define CHANNEL 1

int roboSpeed;
int roboAngle;
int motor1speed = 0;
int motor2speed = 0;
int lastSpeed1, lastSpeed2;

unsigned long entry;
byte masterMAC[6];

//IO
#define pwm_motor1 27 //Has to be a PWM output (max 20KHZ)
#define pwm_motor2 25 //Has to be a PWM output (max 20KHZ)
#define cw_motor1 4
#define cw_motor2  19
#define ccw_motor1  5
#define ccw_motor2 16



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
    roboSpeed = root["speed"];
    roboAngle = root["angle"];

    float roboBattery = root["battery"];

    /*   Serial.print(roboSpeed);
       Serial.print(",");
       Serial.println(roboAngle);
    */
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
  pinMode(cw_motor1, OUTPUT);
  pinMode(cw_motor2, OUTPUT);
  pinMode(ccw_motor1, OUTPUT);
  pinMode(ccw_motor2, OUTPUT);

  ledcSetup(0, 1000, 8);
  ledcAttachPin(pwm_motor1, 0);

  ledcSetup(1, 1000, 8);
  ledcAttachPin(pwm_motor2, 1);

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

  display.init();
  display.setFont(ArialMT_Plain_16);
  display.flipScreenVertically();
  display.display();
}
int hi = 10;
void loop() {
  /* if (millis() > entry + 1000) {
     Serial.print("Sending ");
     telemetry.batteryVoltage = 10.6;
     for (int i = 0; i < 6; i++) Serial.println(masterMAC[i], HEX);

    sendData(masterMAC);
     entry = millis();
    }
  */

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
    digitalWrite(cw_motor1, LOW);
    digitalWrite(ccw_motor1, LOW);
  }
  else if (speed1 >= 0) {
    digitalWrite(cw_motor1, HIGH);
    digitalWrite(ccw_motor1, LOW);
  }
  else {
    digitalWrite(cw_motor1, LOW);
    digitalWrite(ccw_motor1, HIGH);
  }

  if (speed2 == 0) {
    digitalWrite(cw_motor2, LOW);
    digitalWrite(ccw_motor2, LOW);
  }
  else if (speed2 >= 0) {
    digitalWrite(cw_motor2, HIGH);
    digitalWrite(ccw_motor2, LOW);
  }
  else {
    digitalWrite(cw_motor2, LOW);
    digitalWrite(ccw_motor2, HIGH);
  }
  ledcWrite(0, speed1);
  ledcWrite(1, speed2);

  Serial.print(" motorSpeed1 ");
  Serial.print( motor1speed);
  Serial.print(" motorSpeed2 ");
  Serial.println(motor2speed);

  Serial.print(" Speed1 ");
  Serial.print( speed1);
  Serial.print(" Speed2 ");
  Serial.println(speed2);
}
