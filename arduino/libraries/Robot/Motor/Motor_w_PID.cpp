#include "PID/PID.cpp"
#include "Encoder/Encoder.cpp"
#include "PID/structs.h"

#ifndef MOTORSPACE_MOTOR_W_PID
#define MOTORSPACE_MOTOR_W_PID

namespace MotorSpace
{
class Motor_w_PID
{
public :

	float w_set_point;
	bool *force_off;
	bool *modePassControl;
	bool *modePassEncoder;

	EncoderSpace::Encoder encoder;
	PIDSpace::PID pid;

	Motor_w_PID(bool *ForceOff, bool *ModePassControl, bool *ModePassEncoder):
		encoder(),
		pid(&(encoder.encoderOutput.w_rads_kalman), &w_set_point)
	{
		force_off = ForceOff;
		modePassControl = ModePassControl;
		modePassEncoder = ModePassEncoder;
		w_set_point = 0;
	}

	void compute()
	{
		if (*modePassEncoder)
			encoder.pass();
		else
			encoder.get();

		if (*force_off)
			pid.off();
		else
		{
			if (*modePassControl)
				pid.pass();
			else
				pid.push();
		};
	}
};
}

#endif
