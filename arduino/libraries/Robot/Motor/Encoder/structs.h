#ifndef ENCODERSPACE_STRUCTS_H
#define ENCODERSPACE_STRUCTS_H

namespace EncoderSpace
{
struct encoder_init_data
{
	int max_steep;
	int range_enc;
	int pinHardware;
	bool mode;
};

struct encoder_output
{
	float w_rads;
	float w_rads_kalman;
	float delta_phi;

	float delta_t;
	unsigned long timeStamp;

	bool ticket;
};

struct kalman_vars
{
	float A;
	float B;
	float C;
	float Q;
	float R;

	float P_0;
};
}

#endif
