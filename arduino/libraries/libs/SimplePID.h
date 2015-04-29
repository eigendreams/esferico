#ifndef _SIMPLEPID_H_INCLUDED
#define _SIMPLEPID_H_INCLUDED

/*
 * This PID class uses code from the PID_V1 class. However, there are some important differences. Since we want to agree on the
 * whole -127 to 127 output thing, and a lecture of the encoders in the SAME terms, the input and output variables can be ints.
 * Also, to minimize the use of "spooky" action at distance with pointers, the compute method needs the lecture and the desired
 * value at each call. This also makes the code "timing unaware", in order to synchronize the encoder lecture taking and PID update,
 * the timing of the compute method IS LEFT to the outside methods. This is somewhat inconvenient, but I want to guarantee  deterministic
 * behavior and minimize the use of pointers and extra instructions in the loop.
 *
 * A get method can be used to retrieve the last calculated value. A setSampleTime method still exists AND MUST be set properly when
 * using the Ki or Kd parts.
 *
 * Output is limited to -127 to +127 by default and this is fixed in the code.
 *
 * Since this class tries to be bare-bones, I would recommend to set the desired value to the lecture value at the start of the sketch,
 * so the controller does not jumps or jerks. There is no manual mode either, and the controller direction is understood to be
 * ALWAYS positive. The encoder class can deal with sign fixing, and the talon, driver and servo class can be either changed in sign
 * with the constructor or swapping the connections to the motors.
 *
 */

#define MAX_PID_OUT		255

class SimplePID {
public:
	SimplePID(float Kp, float Ki, float Kd, float Km, int umbral, int range);	// Set the PID constants in the constructor
	int compute(int lecture, int desired);								// Calculates and returns an output from -127 to +127
	int get();															// Returns the calculated output
	void setTunings(float Kp, float Ki, float Kd, float Km);			// Set all the tunings of the controller
	void setSampleTime(unsigned int sampleTime);						// Set sample time
	void setRange(int Range);

private:
	float kp;                  						// (P)roportional Tuning Parameter
	float ki;                  						// (I)ntegral Tuning Parameter
	float kd;                  						// (D)erivative Tuning Parameter
	float km;										// Plant constant, helps in speed control

	float ITerm;									// Holds the Integral term output contribution
	int lastInput;									// The lastInput is used to determine the derivate of the error
	int nowOutput;

	unsigned int sampleTime;						// Serves to adjust the parameters when changing the sample time
	int umbral;
	int range;
};
#endif
