#ifndef CL_DEFINES_H
#define CL_DEFINES_H_

// Pin Setups
// Hardware SPI - OLED SSD1331 Display Defines
//#define sclk 14 //this value is not used, due to hardware spi being enabled it's defaulted
//#define mosi 13 //this value is not used, due to hardware spi being enabled it's defaulted
#define cs A5 // Analog Pin 5
// DC & RST swapped for use with Arduino, switch back to match Chatterbox Rev 1
#define dc A4  // Analog Pin 4
#define rst A3 // Analog Pin 3

#define therm_InputPin A2

#define clear() fillScreen(0)

//uno only has 2 interrupts. one is needed for rotary encoder interrupt, other is needed for optocoupler signal.
//cwPin for rotary encoder is interrupt. the other is not. 
// uno:
#define cwPin 2
#define optoSig 3
#define ccwPin 4
#define buttonPin 5

#define triacDriverPin 6

// Screens
#define clear() fillScreen(0)
#define show endWrite
#define mw 96  
#define mh 64
#define rowHeight 10
#define rowOne 0
/*#define rowTwo 10
#define rowThree 20
#define rowFour 30
#define rowFive 40*/
#define rowSix 50
#define wrapText false
#define trimLength 14
#define maxRowsOnScreen 6

//Cursor movement
#define up 1
#define down 0

//Screens
#define homeScreen 0

#define back 0
#define blankIndex 0

// Rotary States
#define notMoved 0
#define cwRotated 1
#define ccwRotated 2

// Button States
#define lifted 0
#define pressed 1
#define noChange 3

// Running States
#define running 1
#define stopped 0

#define OFF 0
#define ON 1
#define ARMED 2
#define DISABLED 3

//states for timers that get reset after code that needs it executes, not necessarily next loop.
#define EXPIRED 0
#define HOLD 1
#define COUNTING 2

//errors *SEE NOTE
#define NOERROR 0
#define THERMERROR 1
//errors are bit anded, so that multiples can exist.

#endif
