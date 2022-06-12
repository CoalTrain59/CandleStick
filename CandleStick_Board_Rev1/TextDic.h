#ifndef TEXT_DIC_H
#define TEXT_DIC_H_

//Home Screen
const char string_homeName  []    PROGMEM = "CandleStick";
const char string_Date      []    PROGMEM = "03/28/2022";
const char string_blank     []    PROGMEM = "blank"; //a holder for a variable value printed on screen
const char string_Run       []    PROGMEM = "Run!";

const char *const home_stringTable[] PROGMEM = {string_homeName, string_Date, string_blank, string_Run, string_blank};

//Text Dic
#define setpointLine 2;
#define runLine 3;
#define percentLine 4;

#endif