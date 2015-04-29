#include "Arduino.h"

#include "SimplePID.h"

SimplePID::SimplePID(float Kp, float Ki, float Kd, float Km, int umbral, int Range) {
	sampleTime = 50;
	this->umbral = umbral;
	this->setTunings(Kp, Ki, Kd, Km);
	this->setRange(MAX_PID_OUT);
	range = Range; 
}

int SimplePID::compute(int input, int setPoint) {
	int i_error = setPoint - input;
	float error;

	if (abs(i_error) <= umbral) error = 0;

	else error = float(i_error);

	// Only use the integral term if the error is small, else make
	// the integral term contribution null. 32 is an eight of a full turn
	// that goes from 0 to 256. Outside of this error range use the Kp part
	if (abs(i_error) < 15) {
		// If the abs_error is LESS than umbral (wich may be 0), kill ITerm. This is done to
		// ensure that the Talon controllers enter brake mode gracefully. This can be done since
		// we are controlling position, once the error is null, the motors should receive no PWM
		// input anymore assuming that they can hold the position themselves, as some drivers
		// enter brake mode when feed with null input
		if (abs(i_error) < int(umbral))
			ITerm = 0;
		else {
			ITerm += (ki * error);
			// Output ITerm limitation, not with constrain as Iterm is float
			if (ITerm > 15) ITerm = 15;
			if (ITerm < -15) ITerm = -15;
		}
	} else {
		ITerm = 0;
	}

	float dInput = input - lastInput;
	float output = kp * error + ITerm - kd * dInput + km * setPoint;

	lastInput = input;
	nowOutput = int(output);
	nowOutput = constrain(nowOutput, -abs(range), abs(range));

	return nowOutput;
}

int SimplePID::get() {
	return nowOutput;
}

void SimplePID::setTunings(float Kp, float Ki, float Kd, float Km) {
	//if (Kp < 0 || Ki < 0 || Kd < 0 || Km < 0) return;
	float SampleTimeInSec = float(sampleTime) / 1000.;
	kp = Kp;
	ki = Ki * SampleTimeInSec;
	kd = Kd / SampleTimeInSec;
	km = Km;
}

void SimplePID::setSampleTime(unsigned int SampleTime) {
	if (SampleTime < 0) return;
	float ratio = float(SampleTime) / float(sampleTime);
	ki *= ratio;
	kd /= ratio;
	sampleTime = SampleTime;
}

void SimplePID::setRange(int Range) {
	range = Range;
}
