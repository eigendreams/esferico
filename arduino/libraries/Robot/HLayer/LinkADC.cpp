#include "Arduino.h"
#include "structs.h"

#ifdef __ENCODER_DIGITAL__
#include "AS5043.h"
extern AS5043Class encoders_spi;
#endif

#ifndef HLAYERSPACE_LINKADC
#define HLAYERSPACE_LINKADC

namespace HLayerSpace
{
	class LinkADC
	{
		public:

		LinkADC(adc_data_out *Data_ADC, int *PinHardware)
		{
			data_ADC = Data_ADC;
			pinHardware = PinHardware;
		}

		adc_data_out ReadADC()
		{
			unsigned long time_stamp = micros();

#ifdef __ENCODER_DIGITAL__
			encoders_spi.read(*pinHardware);
			int lecture = encoders_spi.getAngle();
#else
			int lecture = analogRead(*pinHardware);
#endif

			// Una zona muerta limita algo el efecto del ruido pero introduce
			// errores a muy bajas velocidades
			if (abs((*data_ADC).lecture - lecture) > 2)
				(*data_ADC).lecture = lecture;

			// Un timeStamp devuelve el momento de la lextura en micros, y el ticket asegura
			// que no llamemos dos veces a la lectura en la misma ventana de sampling del ADC
			((time_stamp - (*data_ADC).timeStamp) >= 100) ? (*data_ADC).ticket = true : (*data_ADC).ticket = false;
			(*data_ADC).timeStamp = time_stamp;

			return *data_ADC;
		}

		adc_data_out fetchADC()
		{	return *data_ADC;	}

		int *pinHardware;
		adc_data_out *data_ADC;
	};
}

#endif
