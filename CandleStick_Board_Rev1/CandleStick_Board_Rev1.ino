//CandleStick
//March 27, 2022
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <avr/pgmspace.h>
/* #include <pgmStrToRAM.h> */
#include <SPI.h>
#include "CL_Defines.h"
#include "screenColors.h"
#include "TextDic.h"
#include <PID_v1.h>

// Rotary Pin Actions
volatile byte rotaryActionState = notMoved;  // 0 nothing, 1 is cwRotated, 2 is ccwRotated
volatile byte buttonAction = noChange; // 0 not pressed, 1 pressed
//buttonState
int prevButtonState = lifted;
int currentButtonState = lifted;

Adafruit_SSD1331 display = Adafruit_SSD1331(&SPI, cs, dc, rst);

//debug ticker (how often it prints)
int tickerTimeInterval = 250;
bool plotData = true; //turn on and off plotting.

//HMI Variables
int lineSelected = 0;
int screenSelected = homeScreen;
bool updateNumberMode = false;
int _setpointLine = setpointLine;
int _runLine = runLine;
int _currentTempLine = currentTempLine;

//Phase Shift Globals
volatile unsigned long pulseTime = 0;
volatile unsigned long phaseShiftOutputOnTime = 0;
volatile byte phaseShiftOutput = 3; //byte
int phaseShiftOutputDisplayOverride;
int phaseShiftPercent; //uS
long globalTimeDelay; //for debugging (access to value outside of function);

//Temperature Variables
double storedSetpoint = 250; //temperature setpoint
int upperAllowableTemperature = 350; //don't allow user to set above this value
int upperBelieveableTemperature = 300; //used for thermister "believable" upper limit.
int roomTemperatureValue = 50; //used for "believeable" temperature check
int setpointStepSize = 10; //needs to be interval of stored setpoint above this line. 
double knownResistorValue = 10930;
int heaterOutputPin = 5;
//Therm calculation variables
double K = 273.15;
double T0 = 298.55; //Kelvin Temp read in room 76F
double R0 = 10160;   //Resistance of therm read in room at temp T0 10.16k
double B = 3950;     //B value from manufacturer

//errors
byte errorMode = NOERROR; //0 is no error

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
//Temperature Timer
unsigned long tempTimerDuration = 10; //mS
unsigned long setpointTimerDuration = 10000; //mS
int setpointTimerExpiredFlag = COUNTING; //FLAG!! must be reset after calling it's respective timer.

//PID Control
double input_pid, output_pid;
double Kp=1.65,Ki=0.02,Kd=0.0;
PID pid(&input_pid, &output_pid, &storedSetpoint,Kp,Ki,Kd,DIRECT);

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
  //pinMode(therm_InputPin, INPUT_PULLUP); Dont use a pull up for an analog input!!!
//  pinMode(7, OUTPUT);

  digitalWrite(triacDriverPin, LOW);

  attachInterrupt(digitalPinToInterrupt(cwPin), decideRotaryAction, FALLING);
  attachInterrupt(digitalPinToInterrupt(optoSig), optoSigDetectedInterrupt, RISING);
  pid.SetOutputLimits(0,100); //set PID to feed 0 to 100 percent output.
  pid.SetMode(AUTOMATIC);

  //screenManager(home_stringTable, 0);
}

void loop() {
  //**************************************************************************************************
  //REMEMBER! -- Any code in here will effect program speed and drastically effects
  //the pulse time for the TRIAC!
  //REMEMBER! -- For some reason swapping the oscilloscope leads shows a different signal on TRIAC OUTPUT!
  //CONSIDER: consider using "nonEssentialCode" which is called at the optimal time. 8mS of time is available.
  //before phase shift outputs begin to fail timing.
//**************************************************************************************************
  // put your main code here, to run repeatedly:

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
  if(percent > 100) percent = 100; //sanitize the input
  long uS_min = 8000; //8000uS delay equates to 0 percent power
  long uS_max = 5000; //5000uS delay equates to 100 percent power (that we are willing to apply);
  long phaseShift_uS = 0;
  phaseShift_uS = (uS_min - uS_max) * percent;
  phaseShift_uS = phaseShift_uS / 100;
  phaseShift_uS = uS_min - phaseShift_uS;
  //return -1 so that we don't fire after a zero cross, if close to setpoint
  if(phaseShift_uS > 7800) phaseShift_uS = 7800;
  return phaseShift_uS;
}
//------------------------------------------------------------------------------------------------

void nonEssentialCode(void){
  if(tempTimerExpired()){
      double temp = getTemperature();
      double filteredTemp = filterInputTemp(temp);
      input_pid = filteredTemp;
      if(filteredTemp > upperBelieveableTemperature || filteredTemp < roomTemperatureValue){
        //go into error mode. disable the phase shift output
        errorMode = errorMode | THERMERROR;
        phaseShiftOutput = DISABLED;
      } else {
        errorMode = errorMode & B11111110; //clears the thermerror bit.
      }
      if(phaseShiftOutput != DISABLED){
         pid.Compute();
      } else {
        output_pid = 0;
      }
      int isSetpointTimerExpired = setpointTimerExpired();
      if(isSetpointTimerExpired == EXPIRED || isSetpointTimerExpired == HOLD){
        //if phaseShiftOutput is ON , this is not safe to do. Wait untill OFF or DISABLED!
        //but if ARMED, then set it back to "OFF" after work done.
        //executes every 10 seconds!
        if(phaseShiftOutput != ON){
          noInterrupts();
          byte prevPhaseShiftOutput = phaseShiftOutput;
          if (prevPhaseShiftOutput == ARMED) prevPhaseShiftOutput = OFF;
          phaseShiftOutput = DISABLED;
          //create logic to update the screen appropriately. Need to disable output for safety,
          //but then the screen logic doesn't work. display override fixes this.
          if(prevPhaseShiftOutput != DISABLED){
            phaseShiftOutputDisplayOverride = UPDATINGRUNNINGSCREEN;
          } else {
            phaseShiftOutputDisplayOverride = NOOVERRIDE;
          }
          screenManager(home_stringTable, 0);
          phaseShiftOutput = prevPhaseShiftOutput;
          setpointTimerExpiredFlag = COUNTING;
          interrupts();
        } else {
          //put timer on HOLD, untill conditions above are met and code can execute.
          setpointTimerExpiredFlag = HOLD;
        }
      }
      if(debugPrintTicker() == true && plotData){
        Serial.print(filteredTemp);
        Serial.print(",");
        Serial.print(output_pid);
        Serial.print(",");
        Serial.println(storedSetpoint);
      }
    }
}

void phaseShiftOutputControl(void){
  bool okToRunSlowSoftware = false;
  noInterrupts();
  //convert percentage phase shift into phastshifttimedelay
  //long phaseShiftTimeDelay = getPhaseShiftTimeFromPercent(storedSetpoint); //used for manual setting/debug
  long phaseShiftTimeDelay = getPhaseShiftTimeFromPercent(output_pid);
  globalTimeDelay = phaseShiftTimeDelay;
  unsigned long currentTime = micros();
  if(phaseShiftOutput == DISABLED){
    digitalWrite(triacDriverPin,LOW);
  } else if((currentTime - pulseTime)>=phaseShiftTimeDelay && phaseShiftOutput == ARMED && phaseShiftTimeDelay != -1){
    //fire the output, set state to phaseShift = ON, as to not send the pulse again;
    digitalWrite(triacDriverPin, HIGH);
    phaseShiftOutputOnTime = micros();
    phaseShiftOutput = ON;
    okToRunSlowSoftware = false;
  } else if((currentTime - phaseShiftOutputOnTime) >= 10 && phaseShiftOutput == ON){
    //if phase shift output has been on for more than 10 microSeconds, turn it off. 
    digitalWrite(triacDriverPin,LOW);
    phaseShiftOutput = OFF;
    okToRunSlowSoftware = true;
  }
  interrupts();
  //Because ultimate speed is needed, 8mS MAX is available immediatetly afer phaseShiftOutput is turned ON
  //Code here must complete in 8mS or less!!!
  if(okToRunSlowSoftware || phaseShiftOutput == DISABLED) nonEssentialCode();
  okToRunSlowSoftware = false;
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
  if(storedSetpoint > upperAllowableTemperature) storedSetpoint = upperAllowableTemperature;
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
//Timer function to control when to calculate temperature
//------------------------------------------------------------------------------

bool tempTimerExpired(void){
  bool tempTimerExpired = false;
  static unsigned long prevTempTimerStarted = millis();
  unsigned long currentTime = millis();
  if((currentTime - prevTempTimerStarted) > tempTimerDuration){
    tempTimerExpired = true;
    prevTempTimerStarted = currentTime + 1;
  }
  return tempTimerExpired;
}

//------------------------------------------------------------------------------
//Timer function to control when to check if we are in the setpoint tolerance.
//MUST RESET THE FLAG in function calling this!! This allows program to continue, 
//if the code relying on this can't execute. 
//------------------------------------------------------------------------------

int setpointTimerExpired(void){
  static unsigned long prevSetPointTimerStarted = millis();
  unsigned long currentTime = millis();
  if((currentTime - prevSetPointTimerStarted) > setpointTimerDuration && setpointTimerExpiredFlag != HOLD){
    setpointTimerExpiredFlag = EXPIRED;
    prevSetPointTimerStarted = currentTime + 1;
  }
  return setpointTimerExpiredFlag;
}

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
        phaseShiftOutputDisplayOverride = NOOVERRIDE; 
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

void printNewLine()
{
  Serial.print('\n');
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
    buffer[0] = '\0'; //this adds null character to "reinitialize" the char array.
    display.setCursor(0, (rowHeight * (i)));
    if(i < optLength){
      strcpy_P(buffer, (char *)pgm_read_word(&(screenTable[i])));
      if(i == _runLine){
        if((phaseShiftOutput != DISABLED || phaseShiftOutputDisplayOverride == UPDATINGRUNNINGSCREEN) && errorMode == 0){
          display.setTextColor(LED_RED_HIGH);
          strcpy(buffer, "Running!");
        }
        display.print(buffer);
        display.setTextColor(LED_BLUE_HIGH);
      }
      if(i == _setpointLine){
        //populate the value into a char array to be printed
        char valueCharArray[10];
        //int value = int(storedSetpoint);
        int value = storedSetpoint;
        itoa(value, valueCharArray, 10);
        int len = strlen(buffer); //get length of buffer
        int valLength = strlen(valueCharArray);
        valueCharArray[valLength] = '\0'; //at null at end of char array. for some reason, this is needed.
        for(int j = 0; j <= valLength; j++){
          buffer[len+j] = valueCharArray[j];
        }
        if(updateNumberMode){
          display.setTextColor(LED_RED_HIGH);
        }
        display.print(buffer);
      } 
      if(i == _currentTempLine){
        //populate the value into a char array to be printed
        char valueCharArray[10];
        int value = int(input_pid);
        itoa(value, valueCharArray, 10);
        int len = strlen(buffer); //get length of buffer
        int valLength = strlen(valueCharArray);
        valueCharArray[valLength] = '\0'; //at null at end of char array. for some reason, this is needed.
        for(int j = 0; j <= valLength; j++){
          buffer[len+j] = valueCharArray[j];
        }
        if(abs(storedSetpoint - input_pid) < storedSetpoint * 0.05){
          display.setTextColor(LED_GREEN_HIGH);
        } else {
          display.setTextColor(LED_BLUE_HIGH);
        }
        if(errorMode & THERMERROR){
          //therm error is present. hijack the buffer
          strcpy(buffer, "THERM ERROR");
          display.setTextColor(LED_RED_HIGH);
        }
        display.print(buffer);
      }
      if(i != _setpointLine && i != _runLine && i != _currentTempLine){
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
  double thermInputVolts = thermInputADC * (VccInput/1024);
  //convert thermInput voltage to thermister resistance using voltage divider
  //note that known resistor is 10.93kOhms
  double thermResistance = thermInputVolts * knownResistorValue / (VccInput - thermInputVolts);
  //use thermister resistance in equation below to determine Temperature
  // T0 = 298.35 Kelvin (measured temperature in room)
  // R0 = 10.16kOhms (measured resistance in room at T0 Temp)
  // B = B value provided by manufacturer
  //Equation to solve for Resistance: R0 * e^(B * (1/T - 1/T0))
  //Substituting Values: 10.16k * e^(3950 * (1/T - 1/298.65 Kelvin))
  //Solve for T
  //T = 1 / ((1/T0) + (ln(R/R0)/B))
  //double thermTemperature = 1 / ((1/T0) + (log(thermResistance / R0) / B));
  double thermTemperature = 1 / (1/T0 + (log(thermResistance / R0) / B));
  //Print the temperature to terminal every second
  //Loop Delay
  thermTemperature = thermTemperature - K;
  double thermTemperatureF = thermTemperature * 9/5 +32;
  
  return thermTemperatureF;
  }
