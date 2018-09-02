/*
   Robo Tank Remote controller
   Connection Definitions
*/

/*
OLED 
GND : GND
VCC : 3.3V
SCL : 22
SDA : 21
*/

#define OLED_SCL 22
#define OLED_SDA 21

//IO
#define pwm_motor1 13
#define A2 14 //cw_motor1
#define B2 12 //ccw_motor1
#define pwm_motor2 25
#define A1 27 //cw_motor2
#define B1 26 //ccw_motor2

//MAC for the Remote Controller
#define REMOTE_MAC {0x30, 0xAE, 0xA4, 0x1A, 0x19, 0x08}




