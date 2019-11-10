//.CPP LIBRARY CODE

/*
* 
* Texas Instruments HDQ implementation for the Arduino API 
* (cleft) Matthieu Lalonde 2008 (matth@mlalonde.net) 
* Creative Commons BY-SA-NC 
* 
* http://trac.mlalonde.net/cral/browser/HDQ/ 
* 
* Revision 1 
* 
*
*/

/* *********
*	USAGE!
* The only other thing needed is to have a string pull up (4.7K did the job, 
* I saw 10K used in the app notes).
************
*/

extern "C" { 
#include <pins_arduino.h> 
}
#include <Arduino.h>
#include "BQ2754X_HDQ.h"

#define _BQ2754X_HDQ_readPin() (*inputReg & bitmask)>>pin // Change me to inline!*/

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef BCD8_TO_BYTE
#define BCD8_TO_BYTE(bcd) ((10*(bcd&0xF0)>>4) + (bcd&0x0F))
#endif

#ifndef BCD16_TO_WORD
#define BCD16_TO_WORD(bcd) (1000*((bcd&0xF000)>>12) + 100*((bcd&0x0F00)>>8) + 10*((bcd&0x00F0)>>4) + (bcd&0x000F))
#endif

uint16_t BCD16bitToWord(uint16_t bcd) {
	return (uint16_t) BCD16_TO_WORD(bcd);
}

/*
* 
* Constructor 
* @param pinArg: pin number to attach to 
*
*/ 
BQ2754X_HDQ::BQ2754X_HDQ(uint8_t pinArg) { 
	pin = pinArg; 
	port = digitalPinToPort(pin); 
	bitmask = digitalPinToBitMask(pin); 
	outputReg = portOutputRegister(port); 
	inputReg = portInputRegister(port); 
	modeReg = portModeRegister(port); 
}

/*
* 
* sendBreak: writes a break to the HDQ line 
*/ 
void BQ2754X_HDQ::doBreak(void) { 
	sbi(*modeReg, pin); // Set pin as output

	// Singal a break on the line
	cbi(*outputReg, pin);   // Bring pin low
	delayMicroseconds(HDQ_DELAY_TB);

	// Make sure we leave enough time for the slave to recover
	cbi(*modeReg, pin);     // Release pin
	delayMicroseconds(HDQ_DELAY_TBR);

}

/*
 writeByte: write a raw byte of data to the bus 
 @param payload: the byte to send 
*/ 
void BQ2754X_HDQ::writeByte(uint8_t payload) {
sbi(*modeReg, pin); // Set pin as output

for (uint8_t ii = 0; ii < 8; ii++) {

    // Start bit
    cbi(*outputReg, pin);   // Bring pin low
    delayMicroseconds(HDQ_DELAY_THW1);
	
    // Toggle the pin for this bit
    if (payload>>ii & 0x01) {	// LSB First
        sbi(*outputReg, pin); // High
    }
    else {
        cbi(*outputReg, pin); // Low
    }

    // Bit time
    delayMicroseconds(HDQ_DELAY_THW0 - HDQ_DELAY_THW1 + 5);

	// Stop bit
    sbi(*outputReg, pin);   // Bring the pin high
	delayMicroseconds(HDQ_DELAY_TCYCH - HDQ_DELAY_THW0);
		
}

// Make sure we leave enough time for the slave to recover
cbi(*modeReg, pin);     // Release pin
//delayMicroseconds(HDQ_DELAY_TBR);
delayMicroseconds(HDQ_DELAY_TCYCH - HDQ_DELAY_THW0);

return;

}



/*
* 
* write: send a payload to the device 
* @param reg: the address of the register to write to 
* @param payload: data to be sent 
* @return: false, unless if verif is set, then 
* it will read back the register and 
* return true if it matches the payload 
*
*/ 
bool BQ2754X_HDQ::write(uint8_t reg, uint8_t payload) { 
// Singal a break 
BQ2754X_HDQ::doBreak();

// Write the register to write
BQ2754X_HDQ::writeByte((reg |= HDQ_ADDR_MASK_WRITE));

// Wait for the slave to finish reading the register
delayMicroseconds((HDQ_DELAY_TRSPS_MAX - HDQ_DELAY_BIT_TOTAL) / 2);

// Write the payload
BQ2754X_HDQ::writeByte(payload);

// Wait for the slave to finish writing the payload
delayMicroseconds((HDQ_DELAY_TRSPS_MAX - HDQ_DELAY_BIT_TOTAL) / 2);

cbi(*modeReg, pin);     // Release pin

return true;

}

/*
* 
* Write with verification 
*
*/ 
bool BQ2754X_HDQ::write(uint8_t reg, uint8_t payload, bool verif) { // Write the payload 
BQ2754X_HDQ::write(reg, payload);

// Verify the write
if (payload == BQ2754X_HDQ::read(reg)) return true;

return false;

}

/*
* 
* read: read from the device 
* @param register: address of the register to read 
* @return a uint8_t integer 
*
*/ 
uint8_t BQ2754X_HDQ::read(uint8_t reg) {
uint8_t result = 0; 
uint8_t maxTries = HDQ_DELAY_FAIL_TRIES; // ~128uS at 8Mhz with 8 instructions per loop 

// Singal a break
BQ2754X_HDQ::doBreak();

// Write the register to read
BQ2754X_HDQ::writeByte((reg |= HDQ_ADDR_MASK_READ));

for (uint8_t ii = 0; ii < 8; ii++) {
    // Wait for the slave to toggle a low, or fail
    maxTries = HDQ_DELAY_FAIL_TRIES;
	while (_BQ2754X_HDQ_readPin() != 0 && maxTries-- > 0)
		if (maxTries == 1) return 0xFF;

    // Wait until Tdsub and half or one bit has passed
	delayMicroseconds(((HDQ_DELAY_TDW0 - HDQ_DELAY_TDW1) / 2) + HDQ_DELAY_TDW1);
    // Read the bit
    result |= _BQ2754X_HDQ_readPin()<<ii;

    // Wait until Tssub has passed
	delayMicroseconds(HDQ_DELAY_TCYCD - HDQ_DELAY_TDW0);
}

delayMicroseconds(HDQ_DELAY_TB);

return result;

}

uint16_t BQ2754X_HDQ::commandRead(uint16_t command) {
	BQ2754X_HDQ::write(0x0, 0x01);
	BQ2754X_HDQ::write(0x1, 0x00);
	//why?

	/*
	uint8_t tmp = BQ2754X_HDQ::read(lowByte(command));
	return word(BQ2754X_HDQ::read(highByte(command)), tmp);
	*/
	return word(BQ2754X_HDQ::read(highByte(command)), BQ2754X_HDQ::read(lowByte(command)));
}

uint16_t BQ2754X_HDQ::commandControl(uint16_t subcommand) {
	BQ2754X_HDQ::write(lowByte(BQ2754X_COMMAND_CNTL), lowByte(subcommand));
	BQ2754X_HDQ::write(highByte(BQ2754X_COMMAND_CNTL), highByte(subcommand));
	/*
	return (uint16_t) word(BQ2754X_HDQ::read(highByte(BQ2754X_COMMAND_CNTL)),
							BQ2754X_HDQ::read(lowByte(BQ2754X_COMMAND_CNTL)));
	*/
	uint8_t tmp = BQ2754X_HDQ::read(lowByte(BQ2754X_COMMAND_CNTL));
	return word(BQ2754X_HDQ::read(highByte(BQ2754X_COMMAND_CNTL)), tmp);
}

int16_t BQ2754X_HDQ::deviceType() {
	BQ2754X_HDQ::write(lowByte(BQ2754X_COMMAND_CNTL), lowByte(CONTROL_DEVICE_TYPE));
	BQ2754X_HDQ::write(highByte(BQ2754X_COMMAND_CNTL), highByte(CONTROL_DEVICE_TYPE));
	uint8_t low_byte = BQ2754X_HDQ::read(lowByte(BQ2754X_COMMAND_CNTL));
	uint8_t high_byte = BQ2754X_HDQ::read(highByte(BQ2754X_COMMAND_CNTL));
	if (low_byte != 0xFF && high_byte != 0xFF) {
		// convert 16-bit BCD to int.
		return (int16_t) 100*BCD8_TO_BYTE(high_byte) + BCD8_TO_BYTE(low_byte);
	}
	else {
		return -1;
	}
	
}


