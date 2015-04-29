#ifndef ROBOTSPACE_STRUCTS_H
#define ROBOTSPACE_STRUCTS_H

namespace RobotSpace
{

//struct K_all
//{
//	float km;
//	float kp;
//	float ki;
//	float kd;
//};
//
//struct modes_IO
//{
//	bool input_w;
//	bool output_w;
//};

struct v_w_form
{
	float v;
	float w;
};

struct w_w_form
{
	float w_r;
	float w_l;
};

struct robot_data
{
	float wheel_separation;
	float wheel_radii;
	float position_tol;
	float theta_tol;
};
}

#endif
