#include "structs.h"
#include "../structs.h"
#include "math.h"

#ifndef ODOMSPACE_CORE
#define ODOMSPACE_CORE

namespace OdomSpace
{
class Core
{
public:

	Core(RobotSpace::robot_data *RobotData, float *W_r, float *W_l, float *Delta_t, odom_output *Output)
{
		robotData = RobotData;
		w_r = W_r;
		w_l = W_l;
		delta_t = Delta_t;

		output = Output;
}

	odom_output compute()
	{
		float vkTs = *delta_t * (*robotData).wheel_radii * 0.5 * (*w_r + *w_l);
		float wkTs = *delta_t * (*robotData).wheel_radii * (*w_r - *w_l) / (*robotData).wheel_separation;

		(*output).x += vkTs * cos((*output).theta + 0.5 * wkTs);
		(*output).y += vkTs * sin((*output).theta + 0.5 * wkTs);
		(*output).theta += wkTs;

		return *output;
	}

	odom_output fetch()
	{	return *output;	}

	RobotSpace::robot_data *robotData;
	float *w_r;
	float *w_l;
	float *delta_t;

	odom_output *output;
};
}

#endif
