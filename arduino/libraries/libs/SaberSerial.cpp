#include <Arduino.h>
#include <HardwareSerial.h>
#include <SaberSerial.h>

const int XMIT_START_ADJUSTMENT = 5;

SaberSerialClass::SaberSerialClass(HardwareSerial* io) {
	isHardware = true;
	iostream = io;

	this->setAddress(START_SABER_ADDRESS);
	baudValue = OPTIMAL_BAUD;
}

SaberSerialClass::SaberSerialClass(HardwareSerial* io, uint8_t address) {
	isHardware = true;
	iostream = io;

	this->setAddress(address);
	baudValue = OPTIMAL_BAUD;
}

SaberSerialClass::SaberSerialClass(uint8_t txPin) {
	isHardware = false;
	txpin = txPin;
	this->setTX(txPin);

	this->setAddress(START_SABER_ADDRESS);
	baudValue = OPTIMAL_BAUD;
}

SaberSerialClass::SaberSerialClass(uint8_t txPin, uint8_t address) {
	isHardware = false;
	txpin = txPin;
	this->setTX(txPin);

	this->setAddress(address);
	baudValue = OPTIMAL_BAUD;
}

void SaberSerialClass::setAddress(uint8_t address) {
	address < START_SABER_ADDRESS ? addressSaber = address + START_SABER_ADDRESS : addressSaber = address;
}

void SaberSerialClass::begin() {
	this->begin(addressSaber);
}

void SaberSerialClass::begin(uint8_t address) {
	this->setAddress(address);

	if (isHardware)
		iostream->begin(baudValue);
	else {
		_tx_delay = 0;
		if (baudValue == 2400)
			_tx_delay = 947;
		if (baudValue == 9600)
			_tx_delay = 233;
		if (baudValue == 19200)
			_tx_delay = 114;
		if (baudValue == 38400)
			_tx_delay = 54;
	}
}

void SaberSerialClass::send(uint8_t command, uint8_t data) {
	this->atomWrite(addressSaber);
	this->atomWrite(command);
	this->atomWrite(data);
	this->atomWrite(this->checksum(addressSaber, command, data));
}

void SaberSerialClass::motor1(int speed) {
	speed = constrain(speed, -127, 127);

	if (speed < 0)
		this->send(DRIVE_BACKWARDS_MOTOR_1, abs(speed));
	else
		this->send(DRIVE_FORWARD_MOTOR_1, speed);
}

void SaberSerialClass::motor2(int speed) {
	speed = constrain(speed, -127, 127);

	if (speed < 0)
		this->send(DRIVE_BACKWARDS_MOTOR_2, abs(speed));
	else
		this->send(DRIVE_FORWARD_MOTOR_2, speed);
}

void SaberSerialClass::drive(int speed1, int speed2) {
	this->motor1(speed1);
	this->motor2(speed2);
}

void SaberSerialClass::init() {
	this->init(addressSaber);
}

void SaberSerialClass::init(uint8_t address) {
	this->setAddress(address);

	baudValue = BASELINE_BAUD;
	// Ensure baud rate is optimal (if set before, commands at low speed will be ignored,
	// but since we go back to high speed it's ok)
	if (isHardware) {
		iostream->end();
		iostream->begin(baudValue);
		this->send(BAUD_RATE, OPTIMAL_BAUD_CODE);
		baudValue = OPTIMAL_BAUD;
		iostream->end();
		iostream->begin(baudValue);
	} else {
		this->begin();
		this->send(BAUD_RATE, OPTIMAL_BAUD_CODE);
		baudValue = OPTIMAL_BAUD;
		this->begin();
	}

	// 5s timeOut, 2s full swing delay, +-2 deadband
	this->send(SERIAL_TIMEOUT, 50);
	this->send(RAMPING, 18);
	this->send(DEADBAND, 2);
}

uint8_t SaberSerialClass::checksum(uint8_t address, uint8_t command, uint8_t data) {
	uint8_t checksum_val = (address + command + data) & B01111111;
	return checksum_val;
}

void SaberSerialClass::atomWrite(uint8_t data) {
	if (isHardware)
		iostream->write(data);
	else {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
		if (_tx_delay == 0)
			return;

		uint8_t oldSREG = SREG;
		cli();
		// turn off interrupts for a clean txmit

		// Write the start bit
		tx_pin_write(LOW);
		tunedDelay(_tx_delay + XMIT_START_ADJUSTMENT);

		for (byte mask = 0x01; mask; mask <<= 1) {
			if (data & mask) // choose bit
				tx_pin_write(HIGH); // send 1
			else
				tx_pin_write(LOW); // send 0

			tunedDelay(_tx_delay);
		}

		tx_pin_write(HIGH); // restore pin to natural state

		SREG = oldSREG; // turn interrupts back on
		tunedDelay(_tx_delay);
#endif
		return;
	}
}

void SaberSerialClass::setTX(uint8_t tx) {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
	_transmitBitMask = digitalPinToBitMask(tx);
	uint8_t port = digitalPinToPort(tx);
	_transmitPortRegister = portOutputRegister(port);
#endif
	digitalWrite(tx, HIGH);
	pinMode(tx, OUTPUT);
	return;
}

void SaberSerialClass::tx_pin_write(uint8_t pin_state) {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
	if (pin_state == LOW)
		*_transmitPortRegister &= ~_transmitBitMask;
	else
		*_transmitPortRegister |= _transmitBitMask;
#endif
	return;
}

/* static */
inline void SaberSerialClass::tunedDelay(uint16_t delay) {
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
	uint8_t tmp = 0;

	asm volatile("sbiw    %0, 0x01 \n\t"
			"ldi %1, 0xFF \n\t"
			"cpi %A0, 0xFF \n\t"
			"cpc %B0, %1 \n\t"
			"brne .-10 \n\t"
			: "+r" (delay), "+a" (tmp)
			: "0" (delay)
	);
#endif
	return;
}
