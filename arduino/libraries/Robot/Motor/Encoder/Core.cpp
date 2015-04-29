#include "structs.h"
#include "math.h"
#include "../../HLayer/structs.h"
#include "../../HLayer/HL_Time.h"

#ifndef ENCODERSPACE_CORE
#define ENCODERSPACE_CORE

namespace EncoderSpace
{
class Core
{
public:

	Core( HLayerSpace::adc_data_out *DataInADC, encoder_init_data *DataInput, encoder_output *DataOut)
{
		dataInput = DataInput;
		dataInADC = DataInADC;
		dataOut = DataOut;

		lastStamp = 0;
		lastLecture = 0;
}

	encoder_output ReadEncoder()
	{
		if ((*dataInADC).ticket)
		{
			volatile unsigned long now = (*dataInADC).timeStamp;
			float time_change;

			// Check fot time overflow
			if (now < lastStamp)
				time_change = ((float)((0xFFFFFFFF - lastStamp) + now)) / 1000000.;
			else
				time_change = ((float)(now - lastStamp)) / 1000000.;

			int val_change = (*dataInADC).lecture - lastLecture;

			// We try to avoid steeps due to the analog nature of the encoder and the
			// low pass filter stage
			if ( fabs(val_change) >= (*dataInput).max_steep )
			{
				// Assume speed is constant, then
				// dataOut.w_rads remains unchanged
				// but we may want to correct for possible time delays
				// in the update for delta
				(*dataOut).delta_phi = (*dataOut).w_rads * time_change;
			}
			else
			{
				// If all changes are less than half the encoder range, the modular value with minimal absolute value
				// corrects the wrap of the output value (say we pass from 1023 to cero, we do not wish to
				// take that as a sudden reverse)
				val_change = findAbsMin(val_change + (*dataInput).range_enc, val_change - (*dataInput).range_enc, val_change);

				float rad_change = (6.2831853071 / ((float)(*dataInput).range_enc)) * ((float)val_change); // 6.2831853071 = 2PI
				float speed = rad_change / time_change;

				// If MODE == INVERSE, THEN OUTPUT THE NEGATIVE OF THE SPEED
				if (!((*dataInput).mode)) speed = -speed;

				(*dataOut).w_rads = speed;
				(*dataOut).delta_phi = rad_change;
			}

			(*dataOut).timeStamp = (*dataInADC).timeStamp;
			(*dataOut).delta_t = time_change;
			(*dataOut).ticket = true;

			lastLecture = (*dataInADC).lecture;
			lastStamp = (*dataInADC).timeStamp;

		}
		else
		{
			(*dataOut).ticket = false;
			(*dataOut).timeStamp = (*dataInADC).timeStamp;
		}

		return *dataOut;
	}

	encoder_output fetchEncoder()
	{	return *dataOut; }

	HLayerSpace::adc_data_out *dataInADC;
	encoder_init_data *dataInput;
	encoder_output *dataOut;

	unsigned long lastStamp;
	int lastLecture;

	template<class TYPE>
	TYPE findAbsMin(TYPE a, TYPE b)
	{
		if (fabs(a) < fabs(b)) return a;
		else return b;
	}

	template<class TYPE>
	TYPE findAbsMin(TYPE a, TYPE b, TYPE c)
	{
		return findAbsMin(a, findAbsMin(b, c));
	}
};
}

#endif
