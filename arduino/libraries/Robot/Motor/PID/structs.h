#ifndef PIDSPACE_STRUCTS_H
#define PIDSPACE_STRUCTS_H

namespace PIDSpace
{
struct PID_init_Data
{
	float kp;
	float ki;
	float kd;
	float km;

	float zeroRef;
	float mode;

	float maxITerm;
	float errWindup;
	float maxOut;

	int pinHardware;
};
}

#endif
