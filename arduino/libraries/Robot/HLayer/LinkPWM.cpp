#include "Arduino.h"
#include "structs.h"

#ifdef __OUTPUT_SERIAL__
#include "SaberSerial.cpp"
extern SaberSerialClass motors_serial;
#endif

#ifndef HLAYERSPACE_LINKPWM
#define HLAYERSPACE_LINKPWM

namespace HLayerSpace
{
	// Un template permite que una sola definición abarque todas
    // las clases "válidas" de tipos (numéricos)
	template<class TYPE>
	bool OutPWM(int pinHardware, TYPE pwm)
	{
		if ((pwm >= 0) && (pwm <= 255))
		{
#ifdef __OUTPUT_SERIAL__
			motors_serial.motorSet(pinHardware, (int)pwm);
#else
			analogWrite(pinHardware, (int)pwm);
#endif
			return true;
		}
		return false;
	}
}

#endif
