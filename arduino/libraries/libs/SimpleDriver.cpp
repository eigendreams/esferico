#include "Arduino.h"

#include "SimpleDriver.h"

SimpleDriverClass::SimpleDriverClass(int pwmPin, int umbral, int range) {
	isEnable = false;
	isDualDir = false;
	enablepin = pwmPin;
	this->umbral = umbral;
	this->range = range;

	// ENSURE MOTOR STARTS OFF!!!
	analogWrite(enablepin, 127);
}

SimpleDriverClass::SimpleDriverClass(int enablePin, int dirPin, int umbral, int range) {
	isEnable = true;
	isDualDir = false;
	enablepin = enablePin;
	dir1pin = dirPin;
	this->umbral = umbral;
	this->range = range;
}

SimpleDriverClass::SimpleDriverClass(int enablePin, int dir1Pin, int dir2Pin, int umbral, int range) {
	isEnable = true;
	isDualDir = true;
	enablepin = enablePin;
	dir1pin = dir1Pin;
	dir2pin = dir2Pin;
	this->umbral = umbral;
	this->range = range;
}

void SimpleDriverClass::begin() {
	// ENSURE MOTOR STARTS OFF AND FORWARDS!!!
	analogWrite(enablepin, 0);
	if (isEnable) { 
		pinMode(dir1pin, OUTPUT);
		digitalWrite(dir1pin, LOW);
		if (isDualDir) {
			pinMode(dir2pin, OUTPUT);
			digitalWrite(dir2pin, HIGH);
		}
	}
}

void SimpleDriverClass::write(int value) {
	value = constrain(value, -abs(range), abs(range));

	if (abs(value) <= umbral) value = 0;
	this->value = value;

	if (range < 0) value = -value;
	
	value = map(value, -100, 100, -127, 127);

	if (isEnable) {
		digitalWrite(dir1pin, value >= 0 ? LOW : HIGH);
		if (isDualDir)
			digitalWrite(dir2pin, value >= 0 ? HIGH : LOW);
		analogWrite(enablepin, 2 * abs(value));
	} else {
		analogWrite(enablepin, 127 + value);
	}
}

int SimpleDriverClass::read() {
	return value;
}

void SimpleDriverClass::stop() {
	this->write(0);
}
