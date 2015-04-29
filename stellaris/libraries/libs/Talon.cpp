#include "Arduino.h"

#include "Talon.h"

TalonClass::TalonClass(int min, int max, int center, uint8_t umbral, int8_t sense) : servo() {
	this->min = min;
	this->max = max;
	this->center = center;
	this->umbral = umbral;
	this->sense = sense;
}

TalonClass::TalonClass(int min, int max, uint8_t umbral, int8_t sense) : servo() {
	this->min = min;
	this->max = max;
	this->center = (min + max) / 2;
	this->umbral = umbral;
	this->sense = sense;
}

TalonClass::TalonClass(uint8_t umbral, int8_t sense) : servo() {
	min = MIN_TALON_DEFAULT;
	max = MAX_TALON_DEFAULT;
	center = CENTER_TALON_DEFAULT;
	this->umbral = umbral;
	this->sense = sense;
}

void TalonClass::write(int value) {
	if (value < MIN_PULSE_WIDTH) {
		value = constrain(value, MIN_PWM_ANGLE, MAX_PWM_ANGLE);
		value = constrain(value, -abs(sense), abs(sense));
		if (abs(value) <= umbral) value = 0;
		if (sense < 0) value = -value;
		// Added mapping for center stuff
		if (value <= 0)
			value = map(value, MIN_PWM_ANGLE, 0, min, center);
		else
			value = map(value, 0, MAX_PWM_ANGLE, center, max);
	}
	this->writeMicroseconds(value);
}

void TalonClass::writeMicroseconds(int value) {
	servo.writeMicroseconds(value);
}

int TalonClass::read() {
	int value = this->readMicroseconds() + 1;

	if (value <= center)
		value = map(value, min, center, MIN_PWM_ANGLE, 0);
	else
		value = map(value, center, max, 0, MAX_PWM_ANGLE);

	if (sense < 0) value = -value;
	return value;
}

int TalonClass::readMicroseconds() {
	return servo.readMicroseconds();
}

void TalonClass::calibrate(int value) {
	center += value;
}

void TalonClass::stop() {
	this->writeMicroseconds(center);
}

void TalonClass::attach(uint8_t servoPin) {
	servo.attach(servoPin, min, max);
}
