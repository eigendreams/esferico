#include "Core.cpp"
#include "../Motor/Motor_w_PID.cpp"
#include "../structs.h"

#ifndef ODOMSPACE_ODOM
#define ODOMSPACE_ODOM

namespace OdomSpace
{
class Odom
{
public :

	// Los datos del robot deben ser inicializados, deberían serlo si se usan los encoders, aunque por el
	// funcionamiento de los encoders, no es estrictamente necesario. Esta clase de odometría es ingenua,
	// pero debería ser rápida aunque propensa a ruido
	Odom(RobotSpace::robot_data *RobotData, class MotorSpace::Motor_w_PID *MotorR, class MotorSpace::Motor_w_PID *MotorL) :
		core(	&(*RobotData),
				&((*MotorR).encoder.encoderOutput.w_rads_kalman),
				&((*MotorL).encoder.encoderOutput.w_rads_kalman),
				&average_delta,
				&actualPos)
{
		robotData = RobotData;
		motorR = MotorR;
		motorL = MotorL;
}

	// Se calcula el tiempo promedio de ambos tickets, y en core se actualiza la posición actual
	// con base en los datos de los encoders EN RAD/S. No se hace chequeo de tickets, precedencia
	// o demás, por lo que la clase superior debe manejar los posibles conflictos
	odom_output get()
	{
		average_delta = ((*motorR).encoder.encoderOutput.delta_t + (*motorL).encoder.encoderOutput.delta_t) / 2.;
		core.compute();
		return actualPos;
	}

	odom_output fetch()
	{	return actualPos;	}

	// Podemos actualizar el valor actual de posición, para hacerle concordar con el determinado en alguna
	// isntancia superior, suponiendo que esta sea más precisa, como sería el caso de usar SLAM, por ejemplo.
	// Además de ello, debe considerarse que si ests clase se usa en conjunto con la de planeación,
	// los valores de posición deseada de esa también deben de resetearse por consistencia
	void push_pos(float new_x, float new_y, float new_theta)
	{
		actualPos.x = new_x;
		actualPos.y = new_y;
		actualPos.theta = new_theta;
	}

	Core core;

	RobotSpace::robot_data *robotData;
	odom_output actualPos;

private :

	class MotorSpace::Motor_w_PID *motorR;
	class MotorSpace::Motor_w_PID *motorL;

	float average_delta;
};
}

#endif
