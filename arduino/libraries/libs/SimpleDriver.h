#ifndef _SIMPLEDRIVER_H_INCLUDED
#define _SIMPLEDRIVER_H_INCLUDED

class SimpleDriverClass {
public:
	SimpleDriverClass(int pwmPin, int umbral, int range);										// Simple driver analog output, like with the SaberTooth in analog mode -127 -> 0 and 127 -> 254 PWM output
	SimpleDriverClass(int enablePin, int dirPin, int umbral, int range);						// Dual driver output, with an Enable Pin and a Direction Pin, forwards is dirPin = LOW, -127 or +127 -> 254 PWM output
	SimpleDriverClass(int enablePin, int dir1Pin, int dir2Pin, int umbral, int range);  	// Tri driver output, like with and L293D, forwards is dir1Pin = LOW, dir2Pin = HIGH
	void begin();																					// Angles less or equal than umbral are sent as 0s
	void write(int value);																					// Write a PWM angle from -127 to +127
	int read(); 																							// Returns current PWM as an angle between -127 and 127 degrees
	void stop();																							// Sets center (sugar for setting a PWM of 0)
private:
	bool isEnable : 1;																						// Is single PWM pin or has enable and dir(s)?
	bool isDualDir : 1;																						// Is a dual direction pin?
	int value;																							// PWM angle from -127 to +127
	int enablepin;
	int dir1pin;
	int dir2pin;
	int umbral;
	int range;
};
#endif
