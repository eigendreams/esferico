#include "Core.cpp"
#include "structs.h"
#include "../structs.h"

#ifndef PLANNINGSPACE_PLANNING
#define PLANNINGSPACE_PLANNING

namespace PlanningSpace
{
class Planning
{
public:

	// EL struct constants DEBE ser inicializado en el sketch para poder hacer
	// uso de la clase, y los bindings deben establecerse en algún nivel superior
	// para asociar esta clase con la odometría en tiempo real
	Planning(OdomSpace::odom_output *ActualPos, RobotSpace::robot_data *RobotData) :
		core(	&constants,
				&(*ActualPos),
				&endPos,
				&vw_Law)
{
		actualPos = ActualPos;
		robotData  = RobotData;
}

	// Devuelve un par de velocidades deseadas para las ruedas en términos de una velocidad
	// linear y angular para el robot dadas por la ley de control conbtenida en core
	RobotSpace::w_w_form get()
	{
		core.compute();
		ww_Law.w_r = (vw_Law.v + (vw_Law.w * (*robotData).wheel_separation) / 2) / (*robotData).wheel_radii;
		ww_Law.w_l = (vw_Law.v - (vw_Law.w * (*robotData).wheel_separation) / 2) / (*robotData).wheel_radii;
		return ww_Law;
	}

	RobotSpace::w_w_form fetch()
	{	return ww_Law;	}

	// Permite hacer el push de un "cambio" deseado de coordenadas para el robot, que la clase
	// intentará ejercer por medio de la ley de control en get(). EL arreglo endPos es
	// público, por lo que puede "borrarse" desde una clase superior sin problemas, si al
	// mismo tiempo TAMBIÉN se resetea la posición en Odom, por esto no se incluye
	// una función de reset en la clase
	void push_change(float changeX, float changeY, float changeTheta)
	{
		endPos.x = (*actualPos).x + changeX;
		endPos.y = (*actualPos).y + changeY;
		endPos.theta = (*actualPos).theta + changeTheta;
	}

	Core core;

	control_law_k constants;
	RobotSpace::w_w_form ww_Law;
	OdomSpace::odom_output endPos;

private:

	OdomSpace::odom_output *actualPos;
	RobotSpace::robot_data *robotData;
	RobotSpace::v_w_form vw_Law;
};
}
#endif
