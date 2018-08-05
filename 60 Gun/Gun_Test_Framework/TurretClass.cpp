#include <Arduino.h>
#include <Servo.h>


/****************************************************************************************************
 * CLASS: Turret Object
 * 
 * The software model to the turret hardware.
 * Turret lifecycle is:
 *     Instantiated               Exists, but not functional
 *     Initialized                Functional
 *     Change Commited            Firing commits are immediate by calling the fire function. 
 *                                Position commits can move two axis at the same time, and are completed by update() the change.
 *             
 * Michael Green (2018)
 *             
 * TODO add in error conditions, like run out of ammo and diplay it in Serial, communicate back to main, and maybe a red led.
 *      Add in Turret lifecycle states
 *      Porportional change of sweep to match calibration.
 ****************************************************************************************************/
 class Turret
{
  /****************************************************************************************************
  * Enum
  * Provides the state of the weapons
  * 
  ****************************************************************************************************/
  public:
  enum weaponState {cold, prepared, firing};
  
  private:
  int pan;                                                                        // Value of pan 0 to 180
  int panHome;                                                                    // A default postion used as a center of pan's sweep
  int panMin = 0;                                                                     // Calibrated minimum value of pan's sweep
  int panMax = 180;                                                                     // Calibrated maximum value of pan's sweep
  

  int tilt;                                                                       // Value of tilt 0 to 180
  int tiltHome;                                                                   // A default postion used as a center of tilts's sweep
  int tiltMin = 0;                                                                    // Calibrated minimum value of tilt's sweep
  int tiltMax = 180;                                                                    // Calibrated maximum value of tilt's sweep
  
  weaponState weaponCurrentState;                                                 // Current state of the weapon

  int panPin;                                                                     // Pin to set pan
  int tiltPin;                                                                    // Pin to set tilt
  int weaponPin;                                                                  // Pin to set fire

  Servo panServo;                                                                 // Pan Servo
  Servo tiltServo;                                                                // Tilt Servo
 
  /**************************************************
  * Constructor
  * 
  * INPUT:       int pan               Current Pan Value                          TODO: Currently 0-180 will need to change
  *              int tilt              Current Tilt Value                         TODO: Currently 0-180 will need to change
  *              int panHome           What the default neutral postion is for pan
  *              int tiltHome          What the default neutral postion is for tilt
  *              int panPin            Needs to be an analog pin
  *              int tiltPin           Needs to be an analog pin 
  *  
  **************************************************/
  public:
  Turret(int tempPan, int tempTilt, int tempPanHome, int tempTiltHome, int tempPanPin, int tempTiltPin, int tempWeaponPin) {
    panPin = tempPanPin;                                                          // set the hardware values
    pan = tempPan;
    panHome = tempPanHome; 
     
    tiltPin = tempTiltPin;   
    tilt = tempTilt; 
    tiltHome = tempTiltHome;
     
    weaponPin = tempWeaponPin;
    pinMode(weaponPin, OUTPUT);
    weaponCurrentState = prepared;
  }
  
  /**************************************************
  * Member Function: Getters and Setters 
  **************************************************/
  int getPan(){return pan;}
  void setPan(int tempPan) { 
    if (tempPan < panMin) {pan = panMin;}
    else if (tempPan > panMax) {pan = panMax;}
    else {pan = tempPan;}
  }
  
  int getTilt(){return tilt;}
  void setTilt(int tempTilt) { 
    if (tempTilt < tiltMin) {tilt = tiltMin;}
    else if (tempTilt > tiltMax) {tilt = tiltMax;}
    else {tilt = tempTilt;}
  }
  
  int getPanHome(){return panHome;}
  void setPanHome(int tempPanHome) { panHome = tempPanHome; }
  int getPanMin(){return panMin;}
  void setPanMin(int tempPanMin) { panMin = tempPanMin; }
  int getPanMax(){return panMax;}
  void setPanMax(int tempPanMax) { panMax = tempPanMax; }
  
  int getTiltHome(){return tiltHome;}
  void setTiltHome(int tempTiltHome) { tiltHome = tempTiltHome; }
  int getTiltMin(){return tiltMin;}
  void setTiltMin(int tempTiltMin) { tiltMin = tempTiltMin; }
  int getTiltMax(){return tiltMax;}
  void setTiltMax(int tempTiltMax) { tiltMax = tempTiltMax; }
    
  int getWeaponCurrentState(){return weaponCurrentState;}
  void setWeaponCurrentState(int tempWeaponCurrentState) { weaponCurrentState = tempWeaponCurrentState; }

  /**************************************************
  * Member Function: initiateTurret 
  * 
  * Allows the Turret to be a global variable and initiated seperately
  * (Servos REALLY don't like to be initiated before the main programm is)
  **************************************************/
  void initiateTurret() {
     panServo.attach(panPin);
     panServo.write(pan);
     tiltServo.attach(tiltPin);
     tiltServo.write(tilt);
  }
  
  /**************************************************
  * Member Function: update 
  * 
  * Provides the means to set the turret to the latest values
  **************************************************/
  void update() {    
     panServo.write(pan);
     tiltServo.write(tilt);
  }

  /**************************************************
  * Member Function: Fire 
  * 
  * Fires the main weapon
  * (Useful if we need to move a weapon into prepared state
  * or if ammo runs out)
  **************************************************/
  void fire() {
    if (weaponCurrentState == prepared) {
      digitalWrite(weaponPin, HIGH);   // turn the LED on (HIGH is the voltage level)
      weaponCurrentState = firing;
    } else if (weaponCurrentState == firing) {
      digitalWrite(weaponPin, LOW);   // turn the LED on (LOW is the voltage level)
      weaponCurrentState = prepared;
    }
  }

  /**************************************************
  * Member Function: toString 
  * 
  * Prints out the current state of the object 
  * 
  **************************************************/
  void toString() {    
    if(Serial) {
      Serial.println("==============================");
      Serial.println("<Turret>");
      Serial.print("pan: ");
      Serial.println(pan);
      Serial.print("panHome: ");
      Serial.println(panHome);
      Serial.print("panMin: ");
      Serial.println(panMin);
      Serial.print("panMax: ");
      Serial.println(panMax);
      Serial.print("tilt: ");
      Serial.println(tilt);
      Serial.print("tiltMin: ");
      Serial.println(tiltMin);
      Serial.print("tiltMax: ");
      Serial.println(tiltMax);
      Serial.print("tiltHome: ");
      Serial.println(tiltHome);
      Serial.print("weaponCurrentState: ");
      Serial.println(weaponCurrentState);
      Serial.print("panPin: ");
      Serial.println(panPin);
      Serial.print("tiltPin: ");
      Serial.println(tiltPin);
      Serial.print("weaponPin: ");
      Serial.println(weaponPin);   
      Serial.println("==============================");  
    }
  }
};
