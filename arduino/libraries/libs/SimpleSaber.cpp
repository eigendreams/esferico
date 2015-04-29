#include <Arduino.h>

#include <SimpleSaber.h>

SimpleSaberClass::SimpleSaberClass(int pwmPin, int umbral, int range) {
	isAnalog = true;
	isAddress = false;
	pwmpin = pwmPin;

	this->umbral = umbral;
	this->range = range;
}

SimpleSaberClass::SimpleSaberClass(SaberSerialClass* saberSerial, int index, int umbral, int range) {
	isAnalog = false;
	isAddress = false;
	saberserial = saberSerial;

	this->umbral = umbral;
	this->range = range;

	this->index = index;
	address = START_SABER_ADDRESS;
}

SimpleSaberClass::SimpleSaberClass(SaberSerialClass* saberSerial, int index, int address, int umbral, int range) {
	isAnalog = false;
	isAddress = true;
	saberserial = saberSerial;

	this->umbral = umbral;
	this->range = range;

	this->index = index;
	this->address = address;
}

void SimpleSaberClass::write(int value) {
	value = constrain(value, -abs(range), abs(range));

	if (abs(value) <= umbral) value = 0;
	this->value = value;

	if (range < 0) value = -value;

	value = map(value, -100, 100, -127, 127);

	if (isAnalog)
		analogWrite(pwmpin, 127 + value);
	else {
		// Make local copy of original address
		int orgAddress = saberserial->addressSaber;

		// If needed, set the address in this
		if (isAddress)
			saberserial->setAddress(address);

		// Send the motor data depending on index and value, if index is wrong do nothing
		if (index == 0)
			saberserial->motor1(value);
		if (index == 1)
			saberserial->motor2(value);

		// if needed, restore original address
		if (isAddress)
			saberserial->setAddress(orgAddress);
	}
}

int SimpleSaberClass::read() {
	return value;
}

void SimpleSaberClass::stop() {
	this->write(0);
}

void SimpleSaberClass::begin() {
	if (!isAnalog) {
		// Make local copy of original address
		int orgAddress = saberserial->addressSaber;

		// if needed, set the address in this
		if (isAddress)
			saberserial->setAddress(address);

		// init the SaberTooth (also performs begin on saberserial)
		saberserial->init();

		// if needed, restore original address
		if (isAddress)
			saberserial->setAddress(orgAddress);
	}
	else 
		// ENSURE MOTOR STARTS OFF!!!
		analogWrite(pwmpin, 127);
	// Ensure all starts off!!!
	this->stop();
}
