#include <stdio.h>
#include <stdlib.h>

#include <ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "Motor/Motor_w_PID.cpp"
#include "HLayer/HL_Time.h"
#include "HLayer/structs.h"
#include "structs.h"

#ifndef ROBOTSPACE_ROBOT
#define ROBOTSPACE_ROBOT

namespace RobotSpace
{
class Robot
{
public:

	Robot() :
		/*
		 * Esta clase instancia algunas clases "ayudantes", siendo estas las propias clases
		 * de los motores. Algunas cosas se pasan por referencia, principalmente para ahorrar en
		 * de los motores. Algunas cosas se pasan por referencia, principalmente para ahorrar en
		 * memoria y tamaño del código que por otra razón. No es muy elegante pero reduce considerablemente
		 * la necesidad de hacer referencias interclases en el código de forma explícita (lease más dificíl de
		 * cambiar, mantener, modificar y extender) y sirven como pseudo variables globales para todas las
		 * clases en las que estas se propagen por referencia, en el caso de los motores, solo se pasan la
		 * bandera de apagado por timeOut "forceOff" y las bandera del modo de control y modo de encoders.
		 * Sin embargo, la toma de datos de los encoders se realiza en este nivel, porque es aquì donde
		 * se construye el MultiArray de salida para ROS, y la publicación se hace en el nivel donde se halla
		 * definido el publisher
		 */
		motorR(&forceOff, &modePassControl, &modePassEncoder),
		motorL(&forceOff, &modePassControl, &modePassEncoder)
{
		timeNow = 0;
		timeLast = 0;
		timeLastMsgs = 0;
		timeLastYield = 0;

		sampleTime = 50;
		timeOut = 2000;

		modePassControl = true;
		modePassEncoder = false;

		// Estas declaraciones son necesarias paea poder publicar adecuadamente
		// datos de tipo multiarray, y deben introducirse necesariamente en el
		// tiempo de compilación, por lo que esta clase no puede ser instanciada
		// de forma dinámica
		ws.layout.dim_length = 1;
		ws.data_length = 4;
		ws.layout.dim = &dimws;
		ws.layout.dim[0].label = "ws";
		ws.layout.dim[0].size = 4;
		ws.layout.dim[0].stride = 4;
		ws.layout.data_offset = 4;
		ws.data = &data_ws[0];

		robotData.position_tol = 0.1;
		robotData.theta_tol = 0.1;
}

	bool compute()
	{
		timeNow = HLayerSpace::Time_milis_HL();

		// Bucle fijo de control y lectura a 50 Hz, por supuesto, se puede cambiar, pero
		// pruebas experimentales sugieren que debería ser mayor a 15 ms en sampleo
		if ((timeNow - timeLast) >= 20)
		{
			motorR.compute();
			motorL.compute();

			// Revisando si hay una condición de timeOut, timeLastMsgs se actualiza en el
			// subscriptor asociado a vdes. Si se vuelven a recibir datos en el futuro,
			// el timeOut se desactiva y permite reasumir el control
			((timeNow - timeLastMsgs) > timeOut) ? forceOff = true : forceOff = false;

			timeLast = timeNow;
		}

		if ((timeNow - timeLastYield) >= sampleTime)
		{
			// yield se ejecuta aquì, porque es aquí donde decimos cuando recolectamos
			// la información que se ha ido acumulando en la salida sumada de los encoders.
			// Se ejecuta aunque no usemos la suma, porque garantiza un estado "limpio" si
			// cambiamos el modo del encoder y evita overflows
			motorR.encoder.yield();
			motorL.encoder.yield();

			if (modePassEncoder) {
				ws.data[0] = motorR.encoder.dataADC.lecture;
				ws.data[1] = motorR.encoder.dataADC.timeStamp;
				ws.data[2] = motorL.encoder.dataADC.lecture;
				ws.data[3] = motorL.encoder.dataADC.timeStamp;
			} else {
				ws.data[0] = motorR.encoder.sum_encoderOutput.delta_phi;
				ws.data[1] = motorR.encoder.sum_encoderOutput.delta_t;
				ws.data[2] = motorL.encoder.sum_encoderOutput.delta_phi;
				ws.data[3] = motorL.encoder.sum_encoderOutput.delta_t;
			}

			timeLastYield = timeNow;
			return true;
		}
		return false;
	}

	std_msgs::Float32MultiArray ws;
	std_msgs::MultiArrayDimension dimws;
	float data_ws[4];

	bool forceOff;
	bool modePassControl;
	bool modePassEncoder;

	MotorSpace::Motor_w_PID motorR, motorL;

	robot_data robotData;

	unsigned long timeNow, timeLast, timeLastYield, timeLastMsgs;
	unsigned int sampleTime, timeOut;
};
}
#endif
