/*************************************************************************************************
* Gun Test Framework
* 
* Provides the turret and the means of controlling the turret
* 
* Michael Green (2018)
* 
**************************************************************************************************/
#include "ButtonClass.cpp"
#include "TurretClass.cpp"
#include "JoystickClass.cpp"

/**************************************************
* Global Variables
**************************************************/
Button button(9);
int currentButtonValue = 0;
int previousButtonValue = 0;
Turret weapon(100, 100, 90, 97, 10, 11, 12);
Joystick joystick(14, 15);

/*************************************************************************************************
* Setup
* 
**************************************************************************************************/
void setup() {
  Serial.begin(115200);   
    if(Serial){
      Serial.println("<Ready>");
    }
  previousButtonValue = button.getButtonValue();
  weapon.initiateTurret();
  weapon.setTiltMin(100);
  weapon.setTiltMax(170);

  //Calibrate Joystick     
  if(Serial){
    Serial.println("Calibrating joystick - please leave in neutral postion");
    delay(1000);
    joystick.calibrateJoystickNeutral();
    Serial.println("Neutral calicration complete");
    Serial.println("Calibrating sweep, please move joystick around perimeter");
    delay(1000);
    joystick.calibrateJoystickEdges();
    Serial.println("Sweep calicration complete");
  }
  
}

/*************************************************************************************************
* Loop
* 
**************************************************************************************************/
void loop() {
  updateFiringButton();
  joystick.update();
  serialReader();
  int xRead = joystick.getXConvertedValue();
  int yRead = joystick.getYConvertedValue();

  int xConvertedValue = map(xRead,-512,512,5,180);
  weapon.setPan(xConvertedValue);
  int yConvertedValue = map(yRead,-512,512,0,170);
  weapon.setTilt(yConvertedValue);
  weapon.update();

//  
//  Serial.print("yRead: ");
//  Serial.println(yRead);
//  
//  Serial.print("yConvertedValue: ");
//  Serial.println(yConvertedValue);
}

void updateFiringButton() {
  button.update();
  currentButtonValue = button.getButtonValue();
  if (currentButtonValue != previousButtonValue) {
    button.toString();
    previousButtonValue = currentButtonValue;
    
    weapon.fire();    
    if(Serial){
      Serial.print("WeaponCurrentState: ");
      Serial.println(weapon.getWeaponCurrentState()); 
    } 
  }  
}

/****************************************************************************************************
 * Function: serialReader 
 * 
 * Finite state machine controller to read serial input from keyboard and put it to the turret
 * Allows to test using serial inputs * 
 * 
 * Input: 
 *          w - Up
 *          s - Down
 *          a - Left
 *          d - Right
 *          q - Return Tilt Home
 *          e - Return Pan Home
 *         ' '- Fire
 *          z - Set current tilt to home
 *          c - set current pan to home
 *          ? = pring value of turret to Serial
 * 
 * Output: Nothing
 * 
 ****************************************************************************************************/
void serialReader() { 
  char readCharecter;
  while (Serial.available()) {
    readCharecter = Serial.read();
    switch (readCharecter) {
      case 'w':
// Tilt up
        weapon.setTilt(weapon.getTilt()+1);
        weapon.update();
        Serial.print("Tilt: ");
        Serial.println(weapon.getTilt());
      break;      
      case 's':
// Tilt down      
        weapon.setTilt(weapon.getTilt()-1);
        weapon.update();
        Serial.print("Tilt: ");
        Serial.println(weapon.getTilt());
      break;      
      case 'd':
// Pan left      
        weapon.setPan(weapon.getPan()+1);
        weapon.update();
        Serial.print("Pan: ");
        Serial.println(weapon.getPan());
      break;      
      case 'a':
// Pan right
        weapon.setPan(weapon.getPan()-1);
        weapon.update();
        Serial.print("Pan: ");
        Serial.println(weapon.getPan());
      break;      
      case 'q':
// Reset Tilt to Home
        weapon.setTilt(weapon.getTiltHome());
         weapon.update();
        Serial.print("Tilt: ");
        Serial.println(weapon.getTilt());
      break;      
      case 'e':
// Reset Pan to Home
        weapon.setPan(weapon.getPanHome());
        weapon.update();
        Serial.print("Pan: ");
        Serial.println(weapon.getPan());
      break;    
      case 'z':
// Set tiltHome to currentvalue
        weapon.setTiltHome(weapon.getTilt());
        weapon.update();
        Serial.print("TiltHome: ");
        Serial.println(weapon.getTiltHome());
      break;      
      case 'c':
// Set panHome to currentvalue
        weapon.setPanHome(weapon.getPan());
        weapon.update();
        Serial.print("PanHome: ");
        Serial.println(weapon.getPanHome());
      break;            case 'o':
// Tilt Max
        weapon.setTiltMax(weapon.getTilt());
        weapon.update();
        Serial.print("TiltMax: ");
        Serial.println(weapon.getTiltMax());
      break;      
      case 'l':
// Tilt Min      
        weapon.setTiltMin(weapon.getTilt());
        weapon.update();
        Serial.print("TiltMin: ");
        Serial.println(weapon.getTiltMin());
      break;      
      case 'k':
// Pan Min      
        weapon.setPanMin(weapon.getPan());
        weapon.update();
        Serial.print("PanMin: ");
        Serial.println(weapon.getPanMin());
      break;      
      case ';':
// Pan Max
        weapon.setPanMax(weapon.getPan());
        weapon.update();
        Serial.print("PanMax: ");
        Serial.println(weapon.getPanMin());
      break;      
      case ' ':
// FIRE!!!
        weapon.fire();
        Serial.print("WeaponCurrentState: ");
        Serial.println(weapon.getWeaponCurrentState());        
      break;       
      case '?':
// Print weapon toString
        weapon.toString();
      break; 
      case '/':
// Print Joysatick toString
        joystick.toString();
      break;            
      default:
// Ignore input
      break;
    }
  }
}

