#include "Core.cpp"
#include "../../HLayer/LinkPWM.cpp"

#ifndef PIDSPACE_PID
#define PIDSPACE_PID

namespace PIDSpace
{
class PID
{
public:

	PID(float *W_real, float *Set_point) :
		pid_core(&data_init, &error, &action_control)
{
		w_real = W_real;
		set_point = Set_point;
}

	// El modo de dirección especifica el signo que tiene la salida de control
	// deseada, si se quiere revertir. La contribución de la constante del motor debe
	// sumarse también con el signo adecuada para funcionar correctamente, y las
	// salidas limitarse al rango especificado
	float push()
	{
		error = *set_point - *w_real;
		pid_core.compute();

		if (!data_init.mode) action_control -= data_init.km * *set_point;
		else action_control += data_init.km * *set_point;

		if (action_control > data_init.maxOut) action_control = data_init.maxOut;
		else if (action_control < -data_init.maxOut) action_control = -data_init.maxOut;

		action_control += data_init.zeroRef;
		HLayerSpace::OutPWM(data_init.pinHardware, action_control);

		return action_control;
	}

	float fetch()
	{	return action_control;	}

	// Special methods
	void off()
	{	HLayerSpace::OutPWM(data_init.pinHardware, data_init.zeroRef);	}
	void pass()
	{	HLayerSpace::OutPWM(data_init.pinHardware, *set_point);	}

	float action_control;
	float error;

	float *w_real;
	float *set_point;

	PID_init_Data data_init;
	Core pid_core;
};
}

#endif
