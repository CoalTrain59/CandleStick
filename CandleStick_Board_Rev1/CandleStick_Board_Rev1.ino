//CandleStick
//March 27, 2022
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <avr/pgmspace.h>
#include <pgmStrToRAM.h>
#include <SPI.h>
#include "CL_Defines.h"
#include "screenColors.h"
#include "TextDic.h"

// Rotary Pin Actions
volatile byte rotaryActionState = notMoved;  // 0 nothing, 1 is cwRotated, 2 is ccwRotated
volatile byte buttonAction = noChange; // 0 not pressed, 1 pressed
//buttonState
int prevButtonState = lifted;
int currentButtonState = lifted;

Adafruit_SSD1331 display = Adafruit_SSD1331(&SPI, cs, dc, rst);

//HMI Variables
int lineSelected = 0;
int screenSelected = homeScreen;
bool updateNumberMode = false;
int _setpointLine = setpointLine;
int _runLine = runLine;

//Phase Shift Globals
volatile unsigned long pulseTime = 0;
volatile unsigned long phaseShiftOutputOnTime = 0;
volatile byte phaseShiftOutput = 0; //byte
int phaseShiftTimeDelay; //uS

//Temperature Variables
int storedSetpoint = 1000; //currently used as uS for testing... 
int setpointStepSize = 250; //needs to be interval of stored setpoint above this line. 

//Time Delays
//rotary
bool rotaryTimerExpired = true;
unsigned long rotaryTimeStartValue = 0;
unsigned long rotaryTimeLength = 100; //mS
unsigned long rotaryTimerCurrentValue = 0; //mS
//pushbutton
bool buttonTimerExpired = true;
unsigned long buttonTimeStartValue = 0;
unsigned long buttonTimeLength = 100; //mS
unsigned long buttonTimerCurrentValue = 0; //mS

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  phaseShiftOutput = DISABLED;
  phaseShiftTimeDelay = getPhaseShift(storedSetpoint); //init the phast shift value. currently stored setpoint for testing.
  // OLED Setup
  Serial.println(F("Start OLED Display"));
  display.begin();
  display.setTextWrap(wrapText);
  display.setAddrWindow(0, 0, mw, mh);
  display.clear();
  display.setTextColor(LED_BLUE_HIGH);
  display.setTextSize(1);
  /*display.print("This is a test");
  display.setCursor(0, screenRowHeight);
  display.print("This is a test");*/

  pinMode(cwPin, INPUT);
  pinMode(ccwPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(triacDriverPin, OUTPUT);
//  pinMode(7, OUTPUT);

  digitalWrite(triacDriverPin, LOW);

  attachInterrupt(digitalPinToInterrupt(cwPin), decideRotaryAction, FALLING);
  attachInterrupt(digitalPinToInterrupt(optoSig), optoSigDetectedInterrupt, RISING);

  delay(3000);
  screenManager(home_stringTable, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  monitorRotaryEncoder();
  decideButtonPress();
  phaseShiftOutputControl();
}

int getPhaseShift(int shift_uS){
  return shift_uS;
}

void phaseShiftOutputControl(void){
  noInterrupts();
  unsigned long currentTime = micros();
  if((currentTime - pulseTime)>=phaseShiftTimeDelay && phaseShiftOutput == ARMED){
    //fire the output, set state to phaseShift = ON, as to not send the pulse again;
    digitalWrite(triacDriverPin, HIGH);
    phaseShiftOutputOnTime = micros();
    phaseShiftOutput = ON;
  } else if((currentTime - phaseShiftOutputOnTime) >= 1 && phaseShiftOutput == ON){
    //if phase shift output has been on for more than one millisecond, turn it off. 
    digitalWrite(triacDriverPin,LOW);
    phaseShiftOutput = OFF;
  }
  interrupts();
}

void optoSigDetectedInterrupt(void){
  //each interrupt is the "start" of a half wave (pos or neg, doesn't matter).
  pulseTime = micros();
  //re-arm the phase shifter
  if(phaseShiftOutput != DISABLED) phaseShiftOutput = ARMED;
}

void increaseDecreaseLineSelected(int direction){
  if(!updateNumberMode){
    if(direction == up){lineSelected = lineSelected - 1;};
    if(direction == down){lineSelected = lineSelected + 1;};
    if(lineSelected > getNumberOfOptions()-1){lineSelected = 0;};
    if(lineSelected < 0){lineSelected = getNumberOfOptions()-1;};
    screenManager(home_stringTable, 0);
  }
}

void increaseDecreaseTempSetpoint(int direction){
  if(direction == up){storedSetpoint = storedSetpoint + setpointStepSize;}
  if(direction == down){storedSetpoint = storedSetpoint - setpointStepSize;}
  if(storedSetpoint < 750) storedSetpoint = 750;
  if(storedSetpoint > 8000) storedSetpoint = 8000;
  phaseShiftTimeDelay = storedSetpoint;
  screenManager(home_stringTable, 0);
}


//------------------------------------------------------------------------------
//Timer functions to control the time delay between Rotary actions
//------------------------------------------------------------------------------
void setDelay(int timeDelay){
  rotaryTimeStartValue = millis();
  rotaryTimerExpired = false;
  rotaryTimeLength = timeDelay;
}

bool isRotaryTimerExpired(void){
  rotaryTimerCurrentValue = millis();
  if((rotaryTimerCurrentValue - rotaryTimeStartValue) > rotaryTimeLength){
      rotaryTimerExpired = true;
    }
  return rotaryTimerExpired;
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//Timer functions to control the time delay between pushbutton actions
//------------------------------------------------------------------------------
void setPbDelay(int timeDelay){
  buttonTimeStartValue = millis();
  buttonTimerExpired = false;
  buttonTimeLength = timeDelay;
}

bool isPushButtonTimerExpired(void){
  buttonTimerCurrentValue = millis();
  if((buttonTimerCurrentValue - buttonTimeStartValue) > buttonTimeLength){
      buttonTimerExpired = true;
    }
  return buttonTimerExpired;
}
//------------------------------------------------------------------------------

void monitorRotaryEncoder(void)
{
  //no movement (specifically don't want screen updates) while phaseShiftOutput is on.
  //only allow rotary cw or ccw when disabled. Allow button press to exit run state.
  if(phaseShiftOutput == DISABLED){
    if(isRotaryTimerExpired()){
      if (rotaryActionState == cwRotated)
      {
        if(!updateNumberMode){
          increaseDecreaseLineSelected(down);
        } else {
          increaseDecreaseTempSetpoint(up);
        }
        setDelay(rotaryTimeLength);
        rotaryActionState = notMoved;
      }
      if (rotaryActionState == ccwRotated)
      {
        if(!updateNumberMode){
          increaseDecreaseLineSelected(up);
        } else {
          increaseDecreaseTempSetpoint(down);
        }
        setDelay(rotaryTimeLength);
        rotaryActionState = notMoved;
      }
    }
  }
  if (buttonAction == pressed)
  {
    if(lineSelected == _setpointLine){
      updateNumberMode = !updateNumberMode;
    } else if(lineSelected == _runLine){
      if(phaseShiftOutput != DISABLED){
        phaseShiftOutput = DISABLED;
      } else {
        phaseShiftOutput = OFF;
      }
    }
    screenManager(home_stringTable, 0);
    buttonAction = noChange;
    prevButtonState = pressed;
  }
  else if (buttonAction == lifted)
  {
    buttonAction = noChange;
    prevButtonState = lifted;
  }
}


void decideRotaryAction(void)
{
  //only queue up the state if the phaseshiftoutput is disabled.
  if(phaseShiftOutput == DISABLED){
    if (digitalRead(ccwPin) == HIGH)
    {
      rotaryActionState = ccwRotated;
    }
    else
    {
      rotaryActionState = cwRotated;
    }
  }
}

//------------------------------------------------------------------------
//Decide if button is pressed. Monitor, and if pressed carry out actions
//------------------------------------------------------------------------
void decideButtonPress(void)
{
  int isPressed = !digitalRead(buttonPin);
  if (isPressed && prevButtonState == lifted && isPushButtonTimerExpired() == true){
    currentButtonState = pressed;
    buttonAction = pressed;
    setPbDelay(buttonTimeLength);
  }
  if (!isPressed && prevButtonState == pressed && isPushButtonTimerExpired() == true){
    currentButtonState = lifted;
    buttonAction = lifted;
    setPbDelay(buttonTimeLength);
  }
}
//------------------------------------------------------------------------

int getNumberOfOptions(void)
{
  int unitlength = sizeof(const char *const);

  switch (screenSelected)
  {
  case homeScreen:
    return sizeof(home_stringTable) / unitlength;
    break;
  default:
    Serial.println(F("Could not determine screen selected in getNumberOfOptions function."));
    break;
  }
}


void screenManager(const char *const *screenTable, int shift)
{
  // rule of thumb is if you are going to stay on a screen, but re-render it, send the shiftRows for int shift
  // if you want to change screens, send 0 so it resets

  display.clear();
  // get each line of options
  int optLength = getNumberOfOptions();
  display.setCursor(0, rowOne);

  char buffer[20]; // max character on screen is 14, add one for terminating char ( i think ). 20 is just extra space.
  int maxRowsOnScreen_ = maxRowsOnScreen;
  for (int i = 0; i < maxRowsOnScreen_; i++){
    display.setCursor(0, (rowHeight * (i)));
    if(i < optLength){
      strcpy_P(buffer, (char *)pgm_read_word(&(screenTable[i])));
      if(i == _runLine){
        if(phaseShiftOutput != DISABLED){
          display.setTextColor(LED_RED_HIGH);
          strcpy(buffer, "Running!");
        }
        display.print(buffer);
        display.setTextColor(LED_BLUE_HIGH);
      }
      if(i == _setpointLine){
        //populate the value into a char array to be printed
        char valueCharArray[10];
        int value = storedSetpoint;
        itoa(value, valueCharArray, 10);
        strcpy(buffer, valueCharArray);
        if(updateNumberMode){
          display.setTextColor(LED_RED_HIGH);
        }
        display.print(buffer);
      } 
      if(i != _setpointLine && i != _runLine){
        display.setTextColor(LED_BLUE_HIGH);
        display.print(buffer);
      } 
      if (lineSelected == i){
          display.print('<');
      }
      display.setTextColor(LED_BLUE_HIGH);
    }
  }

}