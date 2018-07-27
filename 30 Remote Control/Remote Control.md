# Remote Control ESP

## Connections
**OLED *( : ESP)*** <br />
GND : GND <br />
VCC : 3.3v <br />
SCL : 22 <br />
SDA : 21

**Joystick *( :ESP)***<br />
GND : GND <br />
VCC : 3.3v <br />
Speed : VP (39) <br />
Angle : VN (36) <br />

---

## JSON Structure
``` json
{
    "speed": "-255 -> 255",
    "angle": "-180 -> 180"
}
```
---
## Implemented Features
* Read Speed and Angle from Joystick
* Joystick Calibration and Mapping
* JSON Packing (See above *'Json Structure'*)
* ESP-NOW master, connects to slave (Controller ESP) and sends the json formatted data. (See above *'Json Structure'*)
* OLED Output of speed and angle data, also instructions for joystick calibration.

## Upcoming Features (Short Term / V1)
* Calibration data stored in EEPROM
* Calibration verification
* Display robot and remote controller battery status.

## Future Additions
* Second Joystick:
    * Gun Control
    * Menu Control
* Buttons
* Menu system.
