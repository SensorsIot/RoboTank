#include <Arduino.h>

class StickCalibration {
  public: int xMax = 0;
  public: int xMin = 511;
  public: int xZero;
  public: int yMax = 0;
  public: int yMin = 511;
  public: int yZero;

  public: StickCalibration(){}

  public: StickCalibration( int xmax, int xmin, int xzero, int ymax, int ymin, int yzero) {
    xMax = xmax;
    xMin = xmin;
    xZero = xzero;
    yMax = ymax;
    yMin = ymin;
    yZero = yzero;
  }
};

/****************************************************************************************************
*    Class: Joystick
*    
*    Joystick has the following elements
*       xValue                The current value of the X axis potentiometer
*       xMax                  The maximum value of the X axis potentiometer
*       xMin                  The minimum value of the X axis potentiometer
*       xZero                 Offset of X neutral position of controller
*       yValue                The current value of theY axis potentiometer
*       yMax                  The maximum value of the Y axis potentiometer
*       yMin                  The minimum value of the Y axis potentiometer
*       yZero                 Offset of Y neutral position of controller    
*    
*       xPin                  Pin of the x axis
*       yPin                  Pin of the y axis
*       
*       minPositionValue      Ideal value of the the min postition potientiometer
*       maxPositionValue      Ideal value of the the max postition potientiometer
*       neutralPositionValue  Ideal value of the the neutral postition potientiometer
****************************************************************************************************/
class Joystick {
  /**************************************************
  *    Class Member Variables 
  **************************************************/
  int xValue = 0; 
  int xConvertedValue = 0;  
  int xMax = 1020;
  int xMin = 0;
  int xZero = 512;
  
  int yValue = 0; 
  int yConvertedValue = 0;  
  int yMax = 1020;
  int yMin = 0;
  int yZero = 512; 

  int xPin; 
  int yPin; 

  StickCalibration calibration;

  public: const int minPositionValue = -512;                 // Note We want full pres
  public: const int maxPositionValue = 512;
  const int neutralPositionValue = 0;
  
  /****************************************************************************************************
  *    Constructor: 
  *    
  *    Takes in:
  *      X Axis Pin               Must be an analog pin
  *      Y Axis Pin               Must be an analog pin
  *    
  ****************************************************************************************************/
  public:
  Joystick(int tempXPin, int tempYPin)   {
    xPin = tempXPin;
    yPin = tempYPin;
  }
  
  /**************************************************
  * Member Function: Getters and Setters 
  **************************************************/
  int getXConvertedValue(){return xConvertedValue;}
  void setXConvertedValue(int tempXConvertedValue) { xConvertedValue = tempXConvertedValue; }

  int getYConvertedValue(){return yConvertedValue;}
  void setYConvertedValue(int tempYConvertedValue) { yConvertedValue = tempYConvertedValue; }

  StickCalibration getCalibration(){return calibration;}
  void setCalibration(StickCalibration cal){
    xMax = cal.xMax;
    xMin = cal.xMin;
    xZero = cal.xZero;
    yMax = cal.yMax;
    yMin = cal.yMin;
    yZero = cal.yZero;
  }

  /****************************************************************************************************
  *    Member Function: Update
  *    
  *    Reviews the state of the controller and updates the variables
  *    This is used during normal operation
  *    
  ****************************************************************************************************/
  void update() {
    readJoyStick();
  }
  
  /****************************************************************************************************
  *   Function: readJoyStick
  *   
  *   Read the postition of the joysticks via pins, and add the calibration values to them
  *   to accurately reflect values, and transpose the range down to refect that the joysticks 
  *   operate in the "middle" of the range - so neutral is the middle of the values.
  *   
  *   The drives and the turret will convert these over to the values they need - we don't want to lose
  *   precision or try to describe them here.
  *   
  *   NOTE: Since the joystick limiter is circular, this means a 45deg angle is max X/Y but a fraction of both
  * 
  *   TODO: Probably should seperate value conversion from reading.
  *         Probably should allow range to be passed in during construction.
  ****************************************************************************************************/
  void readJoyStick() {
    int xRead = analogRead(xPin);
    int yRead = analogRead(yPin);
    const int marginOfError = 15;

    if(xRead < xZero - marginOfError){                                               // If negative
      xConvertedValue = map(xRead,xMin,xZero,minPositionValue,neutralPositionValue);
    } else if(xRead > xZero - marginOfError && xRead < xZero + marginOfError){       // If neutral
      xConvertedValue = neutralPositionValue;
    } else if(xRead > xZero + marginOfError){                                        // If positive
      xConvertedValue = map(xRead,xZero,xMax,neutralPositionValue,maxPositionValue);
    }

    if(yRead < yZero - marginOfError){                                               // If negative
      yConvertedValue = map(yRead,yMin,yZero,minPositionValue,neutralPositionValue);
    } else if(yRead > yZero - marginOfError && yRead < yZero + marginOfError){       // If neutral
      yConvertedValue = neutralPositionValue;
    } else if(yRead > yZero + marginOfError){                                        // If positive
      yConvertedValue = map(yRead,yZero,yMax,neutralPositionValue,maxPositionValue);
    }

  }
  /****************************************************************************************************
  *   Function: calibrateJoystickEdges
  *   
  *   Read the postition of the joysticks via pins, assuming moving to extreme postions of the joystick,
  *   set the result as the offset so read values correspond with a joystick position
  * 
  ****************************************************************************************************/
  void calibrateJoystickEdges(){
    unsigned long start = millis();
    int xRead;
    int yRead;

    while(millis() - start < 5000){
      xRead = analogRead(xPin);
      yRead = analogRead(yPin);
      
      if(xRead > xMax){xMax = xRead;}
      if(xRead < xMin){xMin = xRead;}

      if(yRead > yMax){yMax = yRead;}
      if(yRead < yMin){yMin = yRead;}
    }
  }

  /****************************************************************************************************
  *   Function: calibrateJoystickNeutral
  *   
  *   Read the postition of the joysticks via pins, assuming moving to neutral postion of the joystick,
  *   set the result as the offset so read values correspond with a neutral joystick
  * 
  ****************************************************************************************************/
  void calibrateJoystickNeutral(){
    xZero = analogRead(xPin);
    yZero = analogRead(yPin);
  }
  
  /****************************************************************************************************
  *   Function: toString
  *   
  *   Prints out the values of the controller object, if Serial is available.
  * 
  ****************************************************************************************************/
  void toString() {
    if(Serial) {
      Serial.println("==============================");
      Serial.println("<Joystick>");
      Serial.print("xValue: ");
      Serial.println(xValue);
      
      Serial.print("xMax: ");
      Serial.println(xMax);
      
      Serial.print("xMin: ");
      Serial.println(xMin);
      
      Serial.print("xZero: ");
      Serial.println(xZero);
      
      Serial.print("xConvertedValue: ");
      Serial.println(xConvertedValue);
      
      Serial.print("yValue: ");
      Serial.println(yValue);
      
      Serial.print("yMax: ");
      Serial.println(yMax);
      
      Serial.print("yMin: ");
      Serial.println(yMin);
      
      Serial.print("xMin: ");
      Serial.println(xMin);
      
      Serial.print("yZero: ");
      Serial.println(yZero);
      
      Serial.print("yConvertedValue: ");
      Serial.println(yConvertedValue);
      
      Serial.print("xPin: ");
      Serial.println(xPin);
      
      Serial.print("yPin: ");
      Serial.println(yPin);
      Serial.println("==============================");
    }
  }
};

