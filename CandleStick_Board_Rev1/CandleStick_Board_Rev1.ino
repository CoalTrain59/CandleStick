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

//Phase Shift Globals
volatile unsigned long pulseTime = 0;
volatile unsigned long phaseShiftOutputOnTime = 0;
volatile byte phaseShiftOutput = 0; //byte
const int timeDelay = 6; // start with a start delay of 4ms


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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
  monitorButtonPress();
  phaseShiftOutputControl();
}

void phaseShiftOutputControl(void){
  noInterrupts();
  unsigned long currentTime = millis();
  if((currentTime - pulseTime)>=timeDelay && phaseShiftOutput == ARMED){
    //fire the output, set state to phaseShift = ON, as to not send the pulse again;
    digitalWrite(triacDriverPin, HIGH);
    phaseShiftOutputOnTime = millis();
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
  pulseTime = millis();
  //re-arm the phase shifter
  phaseShiftOutput = ARMED;
}

void increaseDecreaseLineSelected(int direction){
  if(direction == up){lineSelected = lineSelected - 1;};
  if(direction == down){lineSelected = lineSelected + 1;};
  if(lineSelected > 6){lineSelected = 0;};
  if(lineSelected < 0){lineSelected = 6;};
  screenManager(home_stringTable, 0);
}

void monitorRotaryEncoder(void)
{
  if (rotaryActionState == cwRotated)
  {
    increaseDecreaseLineSelected(down);
    delay(100);
    rotaryActionState = notMoved;
  }
  if (rotaryActionState == ccwRotated)
  {
    increaseDecreaseLineSelected(up);
    delay(100);
    rotaryActionState = notMoved;
  }
  if (buttonAction == pressed)
  {
    display.setTextColor(LED_RED_HIGH);
    screenManager(home_stringTable, 0);
    buttonAction = noChange;
    prevButtonState = pressed;
  }
  else if (buttonAction == lifted)
  {
    display.setTextColor(LED_BLUE_HIGH);
    screenManager(home_stringTable, 0);
    buttonAction = noChange;
    prevButtonState = lifted;
  }
}


void decideRotaryAction(void)
{
  if (digitalRead(ccwPin) == HIGH)
  {
    rotaryActionState = ccwRotated;
  }
  else
  {
    rotaryActionState = cwRotated;
  }
}

void monitorButtonPress(void)
{
  int isPressed = !digitalRead(buttonPin);
  if (isPressed && prevButtonState == lifted){
    currentButtonState = pressed;
    buttonAction = pressed;
  }
  if (!isPressed && prevButtonState == pressed){
    currentButtonState = lifted;
    buttonAction = lifted;
  }
}

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
      //display.setTextColor(LED_GREEN_HIGH);
      display.print(buffer);
      if (lineSelected == i){
          display.print('<');
      }
    }
  }

}