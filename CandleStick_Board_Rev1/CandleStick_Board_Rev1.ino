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

//debug ticker (how often it prints)
int tickerTimeInterval = 250;

//HMI Variables
int lineSelected = 0;
int screenSelected = homeScreen;
bool updateNumberMode = false;
int _setpointLine = setpointLine;
int _runLine = runLine;
int _percentLine = percentLine;

//Phase Shift Globals
volatile unsigned long pulseTime = 0;
volatile unsigned long phaseShiftOutputOnTime = 0;
volatile byte phaseShiftOutput = 0; //byte
int phaseShiftPercent; //uS
long globalTimeDelay; //for debugging (access to value outside of function);

//Temperature Variables
int storedSetpoint = 50; //currently used as percent for testing... 
int setpointStepSize = 5; //needs to be interval of stored setpoint above this line. 
double knownResistorValue = 10930;
int heaterOutputPin = 5;
//Therm calculation variables
double K = 273.15;
double T0 = 295.3; //Kelvin Temp read in room
double R0 = 11830;   //Resistance of therm read in room at temp T0
double B = 3950;     //B value from manufacturer

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
  phaseShiftPercent = 50; //init the phase shift value. currently stored setpoint for testing.
  // OLED Setup
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
  pinMode(therm_InputPin, INPUT_PULLUP);
//  pinMode(7, OUTPUT);

  digitalWrite(triacDriverPin, LOW);

  attachInterrupt(digitalPinToInterrupt(cwPin), decideRotaryAction, FALLING);
  attachInterrupt(digitalPinToInterrupt(optoSig), optoSigDetectedInterrupt, RISING);

  delay(3000);
  screenManager(home_stringTable, 0);

}

void loop() {
  // put your main code here, to run repeatedly:
  double temp = getTemperature();
  double filteredTemp = filterInputTemp(temp);
  if(debugPrintTicker() == true && false){
    Serial.println(filteredTemp);
  }
  monitorRotaryEncoder();
  decideButtonPress();
  phaseShiftOutputControl();
}

void newLine(void)
{
  Serial.print('\n');
}

double filterInputTemp(double newTemp){
  static double avgArray [50];
  int arraySize = sizeof(avgArray) / sizeof(avgArray[0]);
  int lastIndex = arraySize-1;
  for(int i=lastIndex; i>0; i--){
    avgArray[i] = avgArray[i-1];
  }
  avgArray[0] = newTemp;
  double sum = 0;
  for(int i=0; i<arraySize; i++){
    sum = avgArray[i] + sum;
  }
  double avg = sum / arraySize;
  return avg;
}

//------------------------------------------------------------------------------------------------
//Pass in 100 for 100%, get full power (that we are willing to apply);
//Pass in 0 for 0%, get very little / no power
//------------------------------------------------------------------------------------------------
long getPhaseShiftTimeFromPercent(int percent){
  long uS_min = 8000; //8000uS delay equates to 0 percent power
  long uS_max = 4000; //4000uS delay equates to 100 percent power (that we are willing to apply);
  long phaseShift_uS = 0;
  phaseShift_uS = (uS_min - uS_max) * percent;
  phaseShift_uS = phaseShift_uS / 100;
  phaseShift_uS = uS_min - phaseShift_uS;
  return phaseShift_uS;
}
//------------------------------------------------------------------------------------------------

void phaseShiftOutputControl(void){
  noInterrupts();
  //convert percentage phase shift into phastshifttimedelay
  long phaseShiftTimeDelay = getPhaseShiftTimeFromPercent(storedSetpoint);
  globalTimeDelay = phaseShiftTimeDelay;
  unsigned long currentTime = micros();
  if((currentTime - pulseTime)>=phaseShiftTimeDelay && phaseShiftOutput == ARMED){
    //fire the output, set state to phaseShift = ON, as to not send the pulse again;
    digitalWrite(triacDriverPin, HIGH);
    phaseShiftOutputOnTime = micros();
    phaseShiftOutput = ON;
  } else if((currentTime - phaseShiftOutputOnTime) >= 500 && phaseShiftOutput == ON){
    //if phase shift output has been on for more than half millisecond, turn it off. 
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
  if(storedSetpoint < 0) storedSetpoint = 0;
  if(storedSetpoint > 100) storedSetpoint = 100;
  phaseShiftPercent = storedSetpoint;
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

//------------------------------------------------------------------------------
//Timer function to control the debugger print
//------------------------------------------------------------------------------
bool debugPrintTicker(void){
  bool tickerTimerExpired = false;
  static unsigned long prevTickerTimeStarted = millis();
  unsigned long currentTickerTime = millis();
  if((currentTickerTime - prevTickerTimeStarted) > tickerTimeInterval){
    tickerTimerExpired = true;
    prevTickerTimeStarted = currentTickerTime + 1;
  }
  return tickerTimerExpired;
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
      if(i == _percentLine){
        //populate the value into a char array to be printed
        char valueCharArray[10];
        long value = globalTimeDelay;
        itoa(value, valueCharArray, 10);
        strcpy(buffer, valueCharArray);
        if(updateNumberMode){
          display.setTextColor(LED_GREEN_HIGH);
        }
        display.print(buffer);
      } 
      if(i != _setpointLine && i != _runLine && i != _percentLine){
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

double getTemperature(void){

//Reset to 5 for testing. Currently not using the measured input voltage value.
  double VccInput = 5;
  //read thermister input voltage ADC value
  double thermInputADC = analogRead(therm_InputPin);
  //0 - 1023 equates to 0 - 5V
  //convert ADC value to voltage
  double thermInputVolts = thermInputADC * ((0.0048875855327468));
  /* Serial.print(F("Thermister Volts = "));
  Serial.print(thermInputVolts);
  Serial.print('\n'); */

  //convert thermInput voltage to thermister resistance using voltage divider
  //note that known resistor is 10.95kOhms
  double thermResistance = thermInputVolts * knownResistorValue / (5 - thermInputVolts);
  /* Serial.print(F("Therm Resistance: "));
  Serial.print(thermResistance);
  newLine(); */
  //use thermister resistance in equation below to determine Temperature
  // T0 = 295.3 Kelvin (measured temperature in room)
  // R0 = 11.83kOhms (measured resistance in room at T0 Temp)
  // B = B value provided by manufacturer
  //Equation to solve for Resistance: R0 * e^(B * (1/T - 1/T0))
  //Substituting Values: 11.83k * e^(3950 * (1/T - 1/295.3 Kelvin))
  //Solve for T
  //T = 1 / ((1/T0) + (ln(R/R0)/B))
  //double thermTemperature = 1 / ((1/T0) + (log(thermResistance / R0) / B));
  double thermTemperature = 1 / (0.0033898305 + (log(thermResistance / R0) / B));
  //Print the temperature to terminal every second
/*   Serial.print(F("Temp: "));
  Serial.print(thermTemperature);
  newLine(); */
  //Loop Delay
  thermTemperature = thermTemperature - K;

/*   Serial.print(F("Volts = "));
  Serial.print(thermInputVolts);
  Serial.print(F(", Resistance: "));
  Serial.print(thermResistance);
  Serial.print(F(", Temperature: ")); */
  
  //Serial.println(thermTemperature);
  
  return thermTemperature;
  }