#include <Arduino.h>

/****************************************************************************************************
* CLASS: Button
* 
*    Has updates and debounced built in.
*    
*    Button has the following elements
*         buttonPin;                             Number of the LED pin
*         currentButtonState;                    Current reading from the input pin
*         previousButtonState = LOW;             Previous reading from the input pin
*         debounceTimer = 0;                     Last time the output pin was toggled
*         DEBOUNCE_DELAY = 50;                   Debounce time; increase if the output flickers   
*    
*         pin                                    Pin of the button
*         
* Michael Green (2018)
*        
****************************************************************************************************/
class Button
{
  /**************************************************
  *    Class Member Variables 
  **************************************************/
  int buttonPin;      
  int currentButtonState;       
  int previousButtonState = LOW; 
  unsigned long debounceTimer = 0; 
  unsigned long DEBOUNCE_DELAY = 50; 

  /*************************************************************************************************
  * Constructor for Button Class
  **************************************************************************************************/
  public:
  Button(int tempPin)
  {
    buttonPin = tempPin;
    pinMode(buttonPin, INPUT);
    currentButtonState = 0;
    previousButtonState = 1;
  }
  
  /**************************************************
  * Member Function: Getters and Setters 
  **************************************************/
  int getButtonValue(){
    return currentButtonState;
  }
  /**************************************************
  * Member Function: update 
  * 
  * Updates the model with the information from the hardware
  **************************************************/
  void update(){  
    int readingButtonState = digitalRead(buttonPin);                        // read the state of the switch into a local variable:  
  
    boolean bUnstableState = readingButtonState != previousButtonState;     // Allow the count to continue only if the state is stable between loops
    if (bUnstableState) {      
      debounceTimer = millis();// reset the debouncing timer
    }
    
    boolean bStableState = (millis() - debounceTimer) > DEBOUNCE_DELAY;     // only check the read if the button's state hasn't changed in the debounce delay time
    if (bStableState) { 
      if (readingButtonState != currentButtonState) {                     // if the button state has changed:
        currentButtonState = readingButtonState;
      }
    }
    previousButtonState = readingButtonState;   
  }

  /****************************************************************************************************
  * Member Function: toString
  *   
  * Prints out the values of the controller object, if Serial is available.
  * 
  ****************************************************************************************************/
  void toString() {
    if(Serial) {
      Serial.println("==============================");
      Serial.println("<Button>");
      Serial.print("buttonPin: ");
      Serial.println(buttonPin);
      
      Serial.print("currentButtonState: ");
      Serial.println(currentButtonState);
      
      Serial.print("previousButtonState: ");
      Serial.println(previousButtonState);
      
      Serial.print("debounceTimer: ");
      Serial.println(debounceTimer);
      
      Serial.print("DEBOUNCE_DELAY: ");
      Serial.println(DEBOUNCE_DELAY);
      Serial.println("==============================");
   }
  }
};
