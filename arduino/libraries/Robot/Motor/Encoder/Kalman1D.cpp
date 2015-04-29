#include "structs.h"

#ifndef ENCODERSPACE_KALMAN
#define ENCODERSPACE_KALMAN

namespace EncoderSpace
{
class Kalman1D
{
	/* IMPORTANT FILTER VARIABLES:
	 * P_0_0 = 1; IF 0 -> INFINITE CONFIDENCE INITIAL STATE
	 * X_0_0 = 0; ROBOTS DEPARTS FROM REST
	 *
	 * A = 2
	 * B = -1
	 * C = 1
	 * Q = d_t ^ 2 * covar_Q = ro ^ 2
	 * R = covar_R = ro ^ 2
	 *
	 * SYSTEM:
	 *
	 * x_k1 = 1 * x_k_k + (x_k_k - x_km1_km1) = actual speed + estimated acceleration
	 * if acceleration = const + noise, this works perfectly!!!
	 *
	 * TU:
	 *
	 * x_k1_k = A * x_k_k + B * x_km1_km1
	 * P_k1_k = A * P_k_k * At + Q
	 *
	 * MU:
	 *
	 * K_k1 = C * P_k1_k * Ct + R
	 * L_k1 = P_k1_k * Ct * (1 / K_k1)
	 * x_k1_k1 = x_k1_k + L_k1 * (measure - C * x_k1_k)
	 * P_k1_k1 = P_k1_k * L_k1 * K_k1 * L_K1t
	 */

public:

	float *w_measure;
	float *w_kalman;

	Kalman1D(float *W_measure, float *W_kalman)
	{
		w_measure = W_measure;
		w_kalman = W_kalman;

		x_km1_km1 = 0;
		x_k1_k1 = 0;
		x_k1_k = 0;
		x_k_k = 0;
		P_k1_k = 0;
		P_k1_k1 = 0;
		P_k_k = 0;
		L_k1 = 0;
		K_k1 = 0;

		iteration = 0;
	}

	float compute()
	{
		if (iteration == 0)
			P_k_k = data_kalman.P_0 ;	//data_kalman.P_0;

		// TU:

		/* x_k1_k = A * x_k_k + B * x_km1_km1
		 * P_k1_k = A * P_k_k * At + Q
		 */

		x_k1_k = data_kalman.A * x_k_k + data_kalman.B * (x_k_k - x_km1_km1);
		P_k1_k = data_kalman.A * P_k_k * transpose(data_kalman.A) + data_kalman.Q;

		// MU:

		/* K_k1 = C * P_k1_k * Ct + R
		 * L_k1 = P_k1_k * Ct * (1 / K_k1)
		 * x_k1_k1 = x_k1_k + L_k1 * (measure - C * x_k1_k)
		 * P_k1_k1 = P_k1_k * L_k1 * K_k1 * L_K1t
		 */

		K_k1 = data_kalman.C * P_k1_k * transpose(data_kalman.C) + data_kalman.R;
		L_k1 = P_k1_k * transpose(data_kalman.C) * (1. / K_k1);
		x_k1_k1 = x_k1_k + L_k1 * ((*w_measure) - data_kalman.C * x_k1_k);
		P_k1_k1 = (1 - L_k1 * data_kalman.C) * P_k1_k;	//P_k1_k + L_k1 * K_k1 * transpose(L_k1);

		float prediction = x_k1_k1;

		x_km1_km1 = x_k_k;
		x_k_k = x_k1_k1;
		P_k_k = P_k1_k1;

		iteration++;

		*w_kalman = prediction;
		return prediction;
	}

	float transpose(float A)
	{	return A;	}

	float x_km1_km1;
	float x_k1_k;
	float x_k_k;
	float x_k1_k1;
	float P_k1_k;
	float P_k1_k1;
	float P_k_k;
	float L_k1;
	float K_k1;

	kalman_vars data_kalman;
	unsigned long iteration;
};
}

#endif
