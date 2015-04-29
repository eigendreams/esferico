#include "structs.h"
#include <math.h>
#include "../../HLayer/HL_Time.h"

#ifndef PIDSPACE_CORE
#define PIDSPACE_CORE

namespace PIDSpace
{
class Core
{
public:

	Core(PID_init_Data *DataInit, float *Error, float *Action)
{
		dataInit = DataInit;
		action = Action;
		error = Error;

		ITerm = 0;
		DTerm = 0;
		lastError = 0;

		lastTimeStamp = (HLayerSpace::Time_micros_HL());
}

	float compute()
	{
		unsigned long now = HLayerSpace::Time_micros_HL();
		float delta_t;

		// Check for time overflow
		if (now < lastTimeStamp)
			delta_t = (float)((0xFFFFFFFF - lastTimeStamp) + now) / 1000000.;
		else
			delta_t = (float)(now - lastTimeStamp) / 1000000.;

		// Anti-windup effect
		if (fabs(*error) < (*dataInit).errWindup)
		{
			ITerm += ((*dataInit).ki * *error * delta_t);
			if (ITerm > (*dataInit).maxITerm) ITerm = (*dataInit).maxITerm;
			else if (ITerm < -(*dataInit).maxITerm) ITerm = -(*dataInit).maxITerm;
		}
		else
			ITerm = 0;

		DTerm = (*error - lastError) / delta_t;

		*action = (*dataInit).kp * *error + ITerm + (*dataInit).kd * DTerm;

		// Remember some variables for next time
		lastError = *error;
		lastTimeStamp = now;

		if (!(*dataInit).mode) *action = -*action;

		return *action;
	}

	float fetch()
	{	return *action;	}

private:

	PID_init_Data *dataInit;
	float *action;
	float *error;

	float ITerm;
	float DTerm;
	float lastError;

	unsigned long lastTimeStamp;
};
}

#endif
