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

#include "Robot.cpp"

extern RobotSpace::Robot robot;
extern ros::NodeHandle nh;

/*
 * SÍ es posible incluir todas estas feas declaraciones dentro de una clase, pero
 * hay ciertas limitaciones en el manejo de punteros a funciones, preprocesamiento,
 * scope, cast y orden de inclusión... no se si vale la pena.
 */
void vdes(const std_msgs::Float32MultiArray& array)
{
	robot.motorR.w_set_point = array.data[0];
	robot.motorL.w_set_point = array.data[1];
	robot.timeLastMsgs = HLayerSpace::Time_milis_HL();
}

void range_0(const std_msgs::Float32& number)
{
	robot.motorR.pid.data_init.maxOut = number.data;
	robot.motorL.pid.data_init.maxOut = number.data;
}

void mode_control(const std_msgs::Bool& dat)
{	robot.modePassControl = dat.data;	}

void time_sample(const std_msgs::UInt16& number)
{	robot.sampleTime = number.data;	}

void mode_encoder(const std_msgs::Bool& dat)
{	robot.modePassEncoder = dat.data;	}

void param_radii(const std_msgs::Float32& number)
{	robot.robotData.wheel_radii = number.data;	}

void param_width(const std_msgs::Float32& number)
{	robot.robotData.wheel_separation = number.data;	}

/*void kpid(const std_msgs::Float32MultiArray &array)
{
	robot.motorR.pid.data_init.kp = array.data[0];
	robot.motorR.pid.data_init.ki = array.data[1];
	robot.motorR.pid.data_init.kd = array.data[2];
	robot.motorR.pid.data_init.km = array.data[3];

	robot.motorL.pid.data_init.kp = array.data[0];
	robot.motorL.pid.data_init.ki = array.data[1];
	robot.motorL.pid.data_init.kd = array.data[2];
	robot.motorL.pid.data_init.km = array.data[3];
}*/

/*
 * Como se hace un uso más o menos intensivo de las capacidades del uC, parece necesario tanto
 * el limitar el número de subscritores y publicadores en el código. En particular, algunos de los
 * tipos de datos que maneja rosserial son más convenientes en una forma más "compacta". Una forma
 * de hacer ello es usar multiarray, pero solo para una relativamente larga secuencia de datos del
 * mismo tipo. En otro caso, un Bool de 8 bits es el tipo de datos más elemental que se puede
 * transmitir, y todos los demás datos deberían obedecer el principio de mínimo uso de bits
 */
ros::Publisher ws_pub("ws", &robot.ws);
ros::Subscriber<std_msgs::Float32MultiArray> vdes_sub("vdes", vdes);
//ros::Subscriber<std_msgs::Float32> range_0_sub("R0", range_0);
ros::Subscriber<std_msgs::Bool> mode_control_sub("Mc", mode_control);
ros::Subscriber<std_msgs::UInt16> time_sample_sub("Ts", time_sample);
ros::Subscriber<std_msgs::Bool> mode_encoder_sub("Me", mode_encoder);
//ros::Subscriber<std_msgs::Float32> param_radii_sub("par_rd", param_radii);
//ros::Subscriber<std_msgs::Float32> param_width_sub("par_wd", param_width);
//ros::Subscriber<std_msgs::Float32MultiArray> kpid_sub("kpid", kpid);

void robotInit()
{
	delay(1);
	nh.advertise(ws_pub);
	delay(1);
	nh.subscribe(vdes_sub);
	delay(1);
//	nh.subscribe(range_0_sub);
//	delay(1);
	nh.subscribe(mode_control_sub);
	delay(1);
	nh.subscribe(time_sample_sub);
	delay(1);
	nh.subscribe(mode_encoder_sub);
	delay(1);
//	nh.subscribe(param_radii_sub);
//	delay(1);
//	nh.subscribe(param_width_sub);
//	delay(1);
//  nh.subscribe(kpid_sub);
}

void robotPublish()
{
	// robot.compute DEBE devolver true solo cuando se quiera publicar, en el
	// código, esto se refiere a cuando se quiera hacer un YIELD de los datos
	// almacenados en el multiarray vdes
	if (robot.compute()){
		ws_pub.publish(&(robot.ws));
	}
}
