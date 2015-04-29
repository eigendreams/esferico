#include "Core.cpp"
#include "Kalman1D.cpp"
#include "../../HLayer/LinkADC.cpp"

#ifndef ENCODERSPACE_ENCODER
#define ENCODERSPACE_ENCODER

namespace EncoderSpace
{
class Encoder
{
public:

	encoder_init_data dataIn;
	HLayerSpace::adc_data_out dataADC;
	encoder_output encoderOutput;

	bool clean_flag;
	encoder_output sum_encoderOutput;

	/*
	 * Varias clases ayudantes, una para la lectura del ADC a través de la capa
	 * de hardware, con los valores actualizados por referencia, otra para el encoder,
	 * leida y alimentada por referencia, y otra para el filtro kalman usado solamente
	 * para el filtrado de la velocidad angular para propósitos de control, la señal enviada
	 * a ROS es la suma de varios valores de salida de los encoders en el tiempo, y por ello
	 * está ya algo filtrada. EL timeStamp sirve para calcular el momento de salida de la
	 * salida del encoder, y es diferente de los timeStamps usados para el cálculo
	 * en el core
	 */
	Encoder() :
		link_ADC(&dataADC, &(dataIn.pinHardware)),
		encoder_core(&dataADC, &dataIn, &encoderOutput),
		kalman(&(encoderOutput.w_rads), &(encoderOutput.w_rads_kalman))
	{
		encoderOutput.timeStamp = HLayerSpace::Time_micros_HL();
		clean_flag = false;
	}

	// En get deseamos obtener velocidades angulares en ws, estas se suman en sum(), y las
	// operaciones de lectura y procesamiento se realizan antes
	encoder_output get()
	{
		link_ADC.ReadADC();
		encoder_core.ReadEncoder();
		kalman.compute();
		this->sum();
		return encoderOutput;
	}

	// A diferencia de get, pass no realiza la suma de salidas, lo que ahorra algo de
	// recursos, pero el procesamiento se realiza de todos modos
	encoder_output pass()
	{
		link_ADC.ReadADC();
		encoder_core.ReadEncoder();
		kalman.compute();
		return encoderOutput;
	}

	encoder_output fetch() {
		return encoderOutput;
	}

	// Sum suma las multiples salidas del encoder del bucle de velocidad fija, para dar la
	// ilusión de una velocidad de salida variable, pero algo más suave entre menor sea esta
	encoder_output sum()
	{
		if (clean_flag)
		{
			sum_encoderOutput.delta_phi = 0;
			sum_encoderOutput.delta_t = 0;
			clean_flag = false;
		}
		sum_encoderOutput.delta_phi += encoderOutput.delta_phi;
		sum_encoderOutput.delta_t += encoderOutput.delta_t;
		sum_encoderOutput.timeStamp = encoderOutput.timeStamp;
		sum_encoderOutput.ticket = encoderOutput.ticket;
		return sum_encoderOutput;
	}

	// Pedimos la información se sum con yield en una clase superior
	encoder_output yield()
	{
		clean_flag = true;
		sum_encoderOutput.w_rads = sum_encoderOutput.delta_phi / sum_encoderOutput.delta_t;
		return sum_encoderOutput;
	}

	HLayerSpace::LinkADC link_ADC;
	Core encoder_core;
	Kalman1D kalman;
};
}

#endif
