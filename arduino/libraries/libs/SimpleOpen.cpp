#include "Arduino.h"
#include "SimpleOpen.h"

SimpleOpen::SimpleOpen(float KScale, int StartValue, int OutputFixed) {
	k = KScale;
	estimatedvalue = StartValue;
	estimatedvalueint = StartValue;
	desiredvalue = StartValue;
	lastdesiredvalue = StartValue;
	outputfixed = OutputFixed;

	direction = 0;
	timelast = millis();

	done = true;
}

void SimpleOpen::setGoal(int DesiredValue) {
	desiredvalue = DesiredValue;
	if (desiredvalue != lastdesiredvalue)
		done = false;
	lastdesiredvalue = desiredvalue;
}

int SimpleOpen::read() {
	return estimatedvalue;
}

int SimpleOpen::check() {
	unsigned long timenow = millis();
	unsigned long elapsedtime = timenow - timelast;

	estimatedvalue += ((direction * elapsedtime) * k) / 1000.;
	estimatedvalueint = int(estimatedvalue);

	if (!done) {
		if (((estimatedvalueint >= desiredvalue) && (direction == 1))
				|| ((estimatedvalueint <= desiredvalue) && (direction == -1))) {
			direction = 0;
			done = true;
			return 0;
		}

		if ((estimatedvalueint <= desiredvalue) && (direction <= 0))
			direction = 1;
		if ((estimatedvalueint >= desiredvalue) && (direction >= 0))
			direction = -1;
	}

	return outputfixed * direction;
}
