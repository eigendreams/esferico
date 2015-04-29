#ifndef _SABERSERIAL_H_INCLUDED
#define _SABERSERIAL_H_INCLUDED

#include "HardwareSerial.h"

#define START_SABER_ADDRESS			128
#define BASELINE_BAUD				9600
#define OPTIMAL_BAUD				19600
#define OPTIMAL_BAUD_CODE			3

// Command list:
#define DRIVE_FORWARD_MOTOR_1 		0x00
#define DRIVE_BACKWARDS_MOTOR_1 	0x01
#define MIN_VOLTAGE 				0x02
#define MAX_VOLTAGE 				0x03
#define DRIVE_FORWARD_MOTOR_2 		0x04
#define DRIVE_BACKWARDS_MOTOR_2 	0x05
#define DRIVE_MOTOR_1_7_BIT			0x06
#define DRIVE_MOTOR_2_7_BIT			0x07
#define DRIVE_FORWARD_MIXED			0x08
#define DRIVE_BACKWARDS_MIXED		0x09
#define TURN_RIGHT_MIXED			0x0a
#define TURN_LEFT_MIXED				0x0b
#define DRIVE_F_B_7_BIT				0x0c
#define TURN_7_BIT					0x0d
#define SERIAL_TIMEOUT 				0X0e
#define BAUD_RATE					0x0f
#define RAMPING						0x10
#define DEADBAND					0x11

class SaberSerialClass {
public:
	uint8_t addressSaber;
	SaberSerialClass(HardwareSerial* io);								// Hardware mode, use START_SABER_ADDRESS
	SaberSerialClass(HardwareSerial* io, uint8_t address);				// Hardware mode, calls setAddress
	SaberSerialClass(uint8_t txPin);									// Software mode, for ARDUINO ONLY!
	SaberSerialClass(uint8_t txPin, uint8_t address);					// Software mode, calls setAddress, for ARDUINO ONLY!
	void setAddress(uint8_t address);									// Sets address, sums to STAR_SABER_ADDRESS if less than it
	void begin();														// Start Serial, default address = 128
	void begin(uint8_t address);										// Start Serial and calls setAddress
	void send(uint8_t command, uint8_t data);							// Send at address
	void motor1(int speed);												// Argument from -127 to +127
	void motor2(int speed);												//
	void drive(int speed1, int speed2);									// Set motors speeds at address, from -127 to +127
	void init();														// Init Saber at address
	void init(uint8_t address);											// Init Saber at address
private:
	bool isHardware : 1;
	HardwareSerial* iostream;
	uint8_t txpin;
	unsigned int baudValue;

	uint8_t checksum(uint8_t address, uint8_t command, uint8_t data);	// Calculate checksum
	void atomWrite(uint8_t data);										// Encapsulate the write operation, hardware abstraction

	uint8_t _transmitBitMask;											// SOFTWARE SERIAL STUFF
	volatile uint8_t *_transmitPortRegister;
	uint16_t _tx_delay;
	void setTX(uint8_t tx);
	void tx_pin_write(uint8_t pin_state);
	static inline void tunedDelay(uint16_t delay);
};
#endif
