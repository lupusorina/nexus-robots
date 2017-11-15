#ifndef IR_H
#define IR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#if defined (__AVR_ATmega8__) || defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__) 
#define ANALOGMAX 5
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
#define ANALOGMAX 15
#endif

#ifndef PIN_UNDEFINED
#define PIN_UNDEFINED 255
#endif

class IR {
public:
	IR(unsigned char analog);
	unsigned int getDist();
	unsigned int getLastDist() const;

	unsigned char setPin(unsigned char analog);
	unsigned char getPin() const;

private:
	IR();
	unsigned char _analogPin;	// analog pin only
	unsigned int _lastDist;		// cm
};



#endif



