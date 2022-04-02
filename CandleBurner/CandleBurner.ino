//Candle Burner
#include <avr/pgmspace.h>

//pin declaration
int thermInputPin = A0;
double knownResistorValue = 10930;
int heaterOutputPin = 5;

//Therm calculation variables
double K = 273.15;
double T0 = 295.3; //Kelvin Temp read in room
double R0 = 11830;   //Resistance of therm read in room at temp T0
double B = 3950;     //B value from manufacturer


void setup()
{
  // Start Serial
  Serial.begin(115200);
  pinMode(thermInputPin, INPUT_PULLUP);
  pinMode(heaterOutputPin, OUTPUT);
  digitalWrite(heaterOutputPin, LOW);
  delay(1000);
  newLine();
  delay(3000);
  // Set up Digital Output Pin
}

void loop()
{
  static int heaterOn = 0; //0 = unknown, 1 = on, 2 = off
  long currentTime = millis();
  static long lastTime = 0;

  double temp = getTemperature();
  
  //double setPoint = 26.667; //celcius equive to 80 farhenheit
  double setPoint = 65; 
  double upperBound = setPoint + 0.5;
  double lowerBound = setPoint - 0.5;

  if(temp < lowerBound){
    if(heaterOn == 0 || heaterOn == 2){
      digitalWrite(heaterOutputPin, LOW);
      heaterOn = 1;
      //Serial.println(F("Turn Heater On"));
    }
  } else if(temp > upperBound){
    if(heaterOn == 0 || heaterOn == 1){
      digitalWrite(heaterOutputPin, HIGH);
      heaterOn = 2;
      //Serial.println(F("Turn Heater Off"));
    }
  }


  if(currentTime - lastTime > 3000){
    /* Serial.print("Celcius: ");
    Serial.print(temp);
    newLine(); */
    lastTime = currentTime;
  }

  delay(1000);
}

void newLine(void)
{
  Serial.print('\n');
}


double getTemperature(void){

//Reset to 5 for testing. Currently not using the measured input voltage value.
  double VccInput = 5;
  //read thermister input voltage ADC value
  double thermInputADC = analogRead(thermInputPin);
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
  Serial.println(thermTemperature);
  
  return thermTemperature;
  }

//Program loop
//read input voltage, determine if it is 1023 as expected. Do nothing with this for now
//read thermister input voltage ADC value
//0 - 1023 equates to 0 - 5V
//convert ADC value to voltage
//convert thermInput voltage to thermister resistance using voltage divider
//note that known resistor is 10.95kOhms
//use thermister resistance in equation below to determine Temperature
// T0 = 295.3 Kelvin (measured temperature in room)
// R0 = 11.83kOhms (measured resistance in room at T0 Temp)
// B = B value provided by manufacturer
//Equation to solve for Resistance: R0 * e^(B * (1/T - 1/T0))
//Substituting Values: 11.83k * e^(3950 * (1/T - 1/295.3 Kelvin))
//Solve for T
//T = 1 / ((1/T0) + (ln(R/R0)/B))
//Print the temperature to terminal every second
