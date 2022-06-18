#ifndef TEXT_DIC_H
#define TEXT_DIC_H_

//Home Screen
const char string_homeName  []    PROGMEM = "CandleStick";
const char string_Date      []    PROGMEM = "03/28/2022";
const char string_setTemp   []    PROGMEM = "Setpoint:"; 
const char string_ActTemp   []    PROGMEM = "Temp:";
const char string_Run       []    PROGMEM = "Run!";

const char *const home_stringTable[] PROGMEM = {string_homeName, string_Date, string_setTemp, string_ActTemp, string_Run};

//Text Dic *lines start at 0
#define setpointLine 2;
#define currentTempLine 3;
#define runLine 4;

#endif