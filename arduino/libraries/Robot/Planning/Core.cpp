#include "structs.h"
#include "../structs.h"
#include "../Odom/structs.h"
#include "math.h"

#ifndef PLANNINGSPACE_CORE
#define PLANNINGSPACE_CORE

namespace PlanningSpace
{
class Core
{
	/* I have x, y, theta, either the instantaneous and imprecise value or the precise value
		 pushed from the computer, and I am issued dx, dy, dtheta.

		 The possible scenarios are:

		 1.- Not in position and not in angle
		 2.- In position but not in angle (end position is within tolerance)
		 3.- In position and in angle

		 In angle but not in position is not generally possible, because the robot has first to
		 travel to the position, as in case 1. The terminal case, of course, is case 3, and thus
		 equates doing nothing.

		 The algorithm needs to know two things, the actual position and the final position. This
		 last one is determined from the increments and ocassionally pushed from the computer, as to
		 ensure precission and consistency. Applied constantly, the pseudocode is:

		 (if dr > tol)
		 	 calculate theta_aim
		 	 (if e_theta_aim > tol)
		 	 	 aim to final position via differential speeds
		     (if e_theta_aim < tol)
		 	 	 advance via equal wheel speeds
		 (if dr < tol)
		 	 calculate theta_aim
		 	 (if e_theta_aim > tol)
		 	 	 aim to final position via differential speeds
		 	 (if e_theta_aim < tol)
		 	 	 do nothing

		 This is one of the finitely many optimal trayectories, but it's one with various problems,
		 since it could happen that the robot exits the position tolerance and performs a very costly
		 full turn to try to get back. For such reasons, an implementation of optimal trayectories
		 should be done on the computer. A simpler but asymptoticall control law is then:

		 g = atan2(dy,dx) - theta + Pi
		 d = g + theta
		 v = k1 * r * cos(g)
		 w = k2 * g + k1 * (g + k3 * d) * sen(g) * cos(g) / g

		 This should be used only for testing porpuses, however, since it's not an optimal solution*/

public:

	Core(	control_law_k *Constants,
			OdomSpace::odom_output *Start_pos,
			OdomSpace::odom_output *End_pos,
			RobotSpace::v_w_form *Control_law)
{
		constants = Constants;
		startPos = Start_pos;
		endPos = End_pos;
		controlLaw = Control_law;
}

	RobotSpace::v_w_form compute()
	{
		float dx = (*endPos).x - (*startPos).x;
		float dy = (*endPos).y - (*startPos).y;

		float r = sqrt(dx * dx + dy * dy);
		float g = atan2(dy, dx) - (*startPos).theta + 3.1415926539;
		float d = g + (*startPos).theta;

		(*controlLaw).v = (*constants).k1 * r * cos(g);
		(*controlLaw).w = (*constants).k2 * g + (*constants).k1 * (g + (*constants).k3 * d) * sin(g) * cos(g) / g;

		return *controlLaw;
	}

	RobotSpace::v_w_form fetch()
	{	return *controlLaw;	}

	OdomSpace::odom_output *startPos;
	OdomSpace::odom_output *endPos;
	RobotSpace::v_w_form *controlLaw;
	control_law_k *constants;
};
}
#endif
