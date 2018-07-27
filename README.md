# RoboTank

This is the Repo of our Summer Project, introducted [in this video](https://youtu.be/tjFopEwtp18
)


Source and documentation for Remote control and the ESP32 controller are in the respective directories.

A "globalDefinitions.h file is in the Misc directory. It has to be copied to the Arduino library folder

---
## V1 Specification
* Controller ESP receives JSON messages via ESP-NOW from the remote (**completed**. see remote controller documentation for more info)
* The controller will make PWM signals based of the values received by the remote controller.
* The drive will create a signal proportianal to the speed of the two motors:
    * This will allow the ESP to make sure that both motors run at the same speed when angle = 0

* The first version of the remote controller is an ESP32 with a joystick connected, which, as mentioned above will send the values of the joystick to the Controller over ESP-NOW, formatted as JSON (**completed**)
    * Moving forward, the remote controller will have another joystick, and have a nice 3D printed case.
* A small OLED will be integrated into the remote controller:
    * Display Speed / Angle values (**completed**: displays values selected by joystick)
    * Display RoboTank and remote controller battery voltages
    * Display calibration prompts (**completed**)
    * *Display a menu system for different features of the tank. (**long term**)*

---
## Getting Involved
* Joining [the Discord server](https://discord.gg/qsf9wgq) is a must for anyone to take part in this project!
* The [Google Drive](https://drive.google.com/drive/folders/1kyE0lTHJltUxwlJrEisewf1MkYsreueS) has lots of information about what has already been discussed. The [ideas page](https://docs.google.com/document/d/1vkkHuSMScHjFKJGnA5bpHOsTGzSksKyNr09aGIKpglI/edit) is a good read to see ideas that have already been mentioned.
* Watch Andreas' videos if you haven't already:
    * [Summer Project: Build an autonomous Robo Tank together with my viewers](https://www.youtube.com/watch?v=tjFopEwtp18)
    * [Robo Tank Update (Summer Project)](https://www.youtube.com/watch?v=AIBTRxL6ICc)
  