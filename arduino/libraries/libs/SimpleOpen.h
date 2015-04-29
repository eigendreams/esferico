#ifndef _SIMPLEOPEN_H_INCLUDED
#define _SIMPLEOPEN_H_INCLUDED

// Made without timeOut interrupts so servo operation is not affected and desired values can be dispatched dynamically

class SimpleOpen {
public:
	SimpleOpen(float KScale, int StartValue, int OutputFixed);
	void setGoal(int DesiredValue);
	int read();
	int check();

private:
	float k;
	int outputfixed;

	int desiredvalue;
	int lastdesiredvalue;

	float estimatedvalue;
	int estimatedvalueint;

	unsigned long timelast;

	int direction;

	bool done;
};
#endif
