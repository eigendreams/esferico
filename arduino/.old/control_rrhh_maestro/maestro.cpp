//#include <Arduino.h>
//
//#include <MsTimer2.h>
//#include <PID_Beta6.h>
//
//#include <ros.h>
//#include "std_msgs/Bool.h"
//#include "std_msgs/Int16.h"
//#include "std_msgs/Int16MultiArray.h"
//#include "std_msgs/MultiArrayLayout.h"
//#include "std_msgs/MultiArrayDimension.h"
//
//#include "utils.h"
//
//
//
//// The four gruesome, POSITION PIDs used here for speeds... mmm
//PID myPID1(&Input1, &Output1, &Setpoint1, 1, 1, 0);
//PID myPID2(&Input2, &Output2, &Setpoint2, 1, 1, 0);
//PID myPID3(&Input3, &Output3, &Setpoint3, 1, 1, 0);
//PID myPID4(&Input4, &Output4, &Setpoint4, 1, 1, 0);
//
//
//
//// ********************************************************************************************************************
//ros::NodeHandle nh;
//
//bool mode_control_val = true;
//bool mode_relay_val = false;
//
//int input_vdes_1 = 0;
//int input_vdes_2 = 0;
//int input_vdes_3 = 0;
//int input_vdes_4 = 0;
//
//int sign_vdes_1 = 0;
//int sign_vdes_2 = 0;
//int sign_vdes_3 = 0;
//int sign_vdes_4 = 0;
//
//void vdes(const std_msgs::Int16MultiArray& array)
//{
//	if (mode_relay_val)
//	{
//		// The direction selection is done in the program lobig, to save time. Anyway, this
//		// callback should not be called more than 3 times per second, or something like that
//		input_vdes_1 = array.data[0];
//		input_vdes_2 = array.data[1];
//		input_vdes_3 = array.data[2];
//		input_vdes_4 = array.data[3];
//	}
//	else
//	{
//		input_vdes_1 = 127;
//		input_vdes_2 = 127;
//		input_vdes_3 = 127;
//		input_vdes_4 = 127;
//	}
//}
//
//void mode_control(const std_msgs::Bool& dat)
//{
//	// Specifies the control mode, if true, then the vdes values are taken as PWM outputs to send
//	// directly to the Sabertooth, otherwise they are taken as velocities. For the sake of sanity,
//	// and since we can think of a middle zero, both things should cover 0 to 255... or not,
//	// I don't know...
//	mode_control_val = dat.data;
//}
//
//void mode_relay (const std_msgs::Bool& dat)
//{
//	// Shut off should be fast, so it's done here. If done in the loop, the delay could be as big
//	// as 200 ms, because of the use of delays, constant publishing and 4 PID loops, anyway, the
//	// data is also saved to a global variable. A true is needed to turn on the relay and thus the motors.
//	// Since the analogWrite assign for turning on the motors is not done here, there will be a small delay
//	// till the motors start to move
//	mode_relay_val = dat.data;
//
//	if (mode_relay_val == true)
//		digitalWrite(7, HIGH);
//	else
//	{
//		analogWrite(motor1Pin, 127);
//		analogWrite(motor2Pin, 127);
//		analogWrite(motor3Pin, 127);
//		analogWrite(motor4Pin, 127);
//		digitalWrite(7, LOW);
//	}
//}
//
///*
//int data_ws[4];
//std_msgs::Int16MultiArray ws;
//std_msgs::MultiArrayDimension dimws;
//ros::Publisher ws_pub("ws", &ws);
//*/
//
//ros::Subscriber<std_msgs::Int16MultiArray> vdes_sub("vdes", vdes);
//ros::Subscriber<std_msgs::Bool> mode_control_sub("Mc", mode_control);
//ros::Subscriber<std_msgs::Bool> mode_relay_sub("Mr", mode_relay);
//
//void set_ros()
//{
//	// Estas declaraciones son necesarias paea poder publicar adecuadamente
//	// datos de tipo multiarray, y deben introducirse necesariamente en el
//	// tiempo de compilación, por lo que esta clase no puede ser instanciada
//	// de forma dinámica
//	/*
//	ws.layout.dim_length = 1;
//	ws.data_length = 4;
//	ws.layout.dim = &dimws;
//	ws.layout.dim[0].label = "ws";
//	ws.layout.dim[0].size = 4;
//	ws.layout.dim[0].stride = 4;
//	ws.layout.data_offset = 4;
//	ws.data = &data_ws[0];
//	*/
//
//	nh.initNode();
//	//nh.advertise(ws_pub);
//	nh.subscribe(vdes_sub);
//	nh.subscribe(mode_control_sub);
//	nh.subscribe(mode_relay_sub);
//}
//// ********************************************************************************************************************
//
//
//
//void setup()
//{
//	// setup the ros hooks et al
//	set_ros();
//
//	// Is this really needed?
//	Input1 = 0;
//	Input2 = 0;
//	Input3 = 0;
//	Input4 = 0;
//
//	Setpoint1 = 0;
//	Setpoint2 = 0;
//	Setpoint3 = 0;
//	Setpoint4 = 0;
//
//	// turn off the relay at the start of the sketch
//	digitalWrite(7, LOW);
//	pinMode(7, OUTPUT);
//
//	//**************************************************************************
//	attachInterrupt(1, suma3, CHANGE);
//	attachInterrupt(0, suma4, CHANGE);
//	MsTimer2::set(300, vel);
//	MsTimer2::start();
//
//	myPID1.SetMode(AUTO);
//	myPID1.SetSampleTime(50);
//	myPID1.SetInputLimits(0, 255);
//
//	myPID2.SetMode(AUTO);
//	myPID2.SetSampleTime(50);
//	myPID2.SetInputLimits(0, 255);
//
//	myPID3.SetMode(AUTO);
//	myPID3.SetSampleTime(50);
//	myPID3.SetInputLimits(0, 255);
//
//	myPID4.SetMode(AUTO);
//	myPID4.SetSampleTime(50);
//	myPID4.SetInputLimits(0, 255);
//	//**************************************************************************
//}
//
//void loop()
//{
//	nh.spinOnce();
//
//	// Lee los sensores por bit banging y delays...
//	// sensores();
//
//	// El metodo en interrupcion por timer2 vel que se ejecuta cada 300 ms por defecto
//	// se encarga de actualizar los valores de Input*, por lo que puede ser que se
//	// envieen varias veces los mismos valores. Como los Input* solo se actualizan en
//	// la interrupción, se garantiza sincronia de los datos.
//	sign_vdes_1 = sign(input_vdes_1 - 127);
//	sign_vdes_2 = sign(input_vdes_2 - 127);
//	sign_vdes_3 = sign(input_vdes_3 - 127);
//	sign_vdes_4 = sign(input_vdes_4 - 127);
//	/*
//	ws.data[0] = sign_vdes_1 * int(Input1);
//	ws.data[1] = sign_vdes_2 * int(Input2);
//	ws.data[2] = sign_vdes_3 * int(Input3);
//	ws.data[3] = sign_vdes_4 * int(Input4);
//	ws_pub.publish(&ws);
//	*/
//
//	// El Setpoint va de 0 a 255, pero recibimos valores de 0 a 127 alrededor del
//	// cero, aunque perdemos algo de resolución, ganamos en simpleza
//	Setpoint1 = 2 * abs(input_vdes_1 - 127);
//	Setpoint2 = 2 * abs(input_vdes_2 - 127);
//	Setpoint3 = 2 * abs(input_vdes_3 - 127);
//	Setpoint4 = 2 * abs(input_vdes_4 - 127);
//
//	// Los inputs van de 0 a 255, y ya corregimos los setpoints para que también
//	// vayan de 0 a 255. EL SaberTooth solo tiene 256 niveles de salida, así que no
//	// es que la pérdida de precisión importa mucho.
//	error1 = abs( Input1 - Setpoint1 );
//	error2 = abs( Input2 - Setpoint2 );
//	error3 = abs( Input3 - Setpoint3 );
//	error4 = abs( Input4 - Setpoint4 );
//
//	// Corrigiendo las ganancias del controlador:
//	if( error1 < 9 )
//	{
//		P1 = PP2_;
//		I1 = II2_;
//		D1 = DD2_;
//	}
//	else
//	{
//		P1 = PP1_;
//		I1 = II1_;
//		D1 = DD1_;
//	}
//
//	if( error2 < 9 )
//	{
//		P2 = PP2_;
//		I2 = II2_;
//		D2 = DD2_;
//	}
//	else
//	{
//		P2 = PP1_;
//		I2 = II1_;
//		D2 = DD1_;
//	}
//	if( error3 < 9 )
//	{
//		P3 = PP2_;
//		I3 = II2_;
//		D3 = DD2_;
//	}
//	else
//	{
//		P3 = PP1_;
//		I3 = II1_;
//		D3 = DD1_;
//	}
//	if( error4 < 9 )
//	{
//		P4 = PP2_;
//		I4 = II2_;
//		D4 = DD2_;
//	}
//	else
//	{
//		P4 = PP1_;
//		I4 = II1_;
//		D4 = DD1_;
//	}
//
//	myPID1.SetTunings(P1, I1, D1);
//	myPID2.SetTunings(P2, I2, D2);
//	myPID3.SetTunings(P3, I3, D3);
//	myPID4.SetTunings(P4, I4, D4);
//
//	// El PID deberia calcularse siempre, para evitar overflows
//	myPID1.Compute();
//	myPID2.Compute();
//	myPID3.Compute();
//	myPID4.Compute();
//
//	// Pero como da valores absolutos, debemos darle salida con respecto a cero
//	Salida1 = map( Output1, 0, 255, 0, 127 );
//	Salida2 = map( Output2, 0, 255, 0, 127 );
//	Salida3 = map( Output3, 0, 255, 0, 127 );
//	Salida4 = map( Output4, 0, 255, 0, 127 );
//
//	if( sign_vdes_1 > 0 )
//	{
//		Salida1 = 128 + Salida1;
//	}
//	else
//	{
//		Salida1 = 127 - Salida1;
//	}
//
//	if( sign_vdes_2 > 0 )
//	{
//		Salida2 = 128 + Salida2;
//	}
//	else
//	{
//		Salida2 = 127 - Salida2;
//	}
//
//	if( sign_vdes_3 > 0 )
//	{
//		Salida3 = 128 + Salida3;
//	}
//	else
//	{
//		Salida3 = 127 - Salida3;
//	}
//
//	if( sign_vdes_4 > 0 )
//	{
//		Salida4 = 128 + Salida4;
//	}
//	else
//	{
//		Salida4 = 127 - Salida4;
//	}
//
//
//	if (mode_relay_val == false)
//	{
//		analogWrite(motor1Pin, 127);
//		analogWrite(motor2Pin, 127);
//		analogWrite(motor3Pin, 127);
//		analogWrite(motor4Pin, 127);
//	}
//	else
//	{
//		if (mode_control_val == false)
//		{
//			analogWrite(motor1Pin, Salida1);
//			analogWrite(motor2Pin, Salida2);
//			analogWrite(motor3Pin, Salida3);
//			analogWrite(motor4Pin, Salida4);
//		}
//		else
//		{
//			analogWrite(motor1Pin, input_vdes_1);
//			analogWrite(motor2Pin, input_vdes_2);
//			analogWrite(motor3Pin, input_vdes_3);
//			analogWrite(motor4Pin, input_vdes_4);
//		}
//	}
//}



// The code is edited and compiled in Eclipse, using the Arduino Core library and AVR compiler
//
// The following code turns a "switching" relay of the RRHH on and off, and replicates the desired PWM values in the
// corresponding pins. Two SaberTooths (25 A x 2 channels) are used as motor controllers, using the analog input mode.
// A single stage filter is used to transform the PWM outputs to continuous voltage values.
// A motor speed of zero maps to an analog output of 2.5 V to the SaberTooth

#include <Arduino.h>

#include <ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
// ********************************************************************************************************************
int motor1Pin = 5;
int motor2Pin = 6;
int motor3Pin = 9;
int motor4Pin = 10;
// ********************************************************************************************************************
ros::NodeHandle nh;

bool mode_control_val = true;
bool mode_relay_val = false;

int input_vdes_1 = 0;
int input_vdes_2 = 0;
int input_vdes_3 = 0;
int input_vdes_4 = 0;

void turn_off() {
	analogWrite(motor1Pin, 127);
	analogWrite(motor2Pin, 127);
	analogWrite(motor3Pin, 127);
	analogWrite(motor4Pin, 127);
}

void tf_vdes(const std_msgs::Int16MultiArray& array) {
	if (mode_relay_val) {
		// The direction selection is done in the program loop, to save time. Anyway, this
		// callback should not be called more than 10 times per second
		input_vdes_1 = array.data[0];
		input_vdes_2 = array.data[1];
		input_vdes_3 = array.data[2];
		input_vdes_4 = array.data[3];
	} else {
		// Else the desired values are "zeroed", just in case
		input_vdes_1 = 127;
		input_vdes_2 = 127;
		input_vdes_3 = 127;
		input_vdes_4 = 127;
	}
}

void mode_control(const std_msgs::Bool& data) {
	// Specifies the control mode, if true, the vdes values are taken as PWM outputs to send
	// directly to the Sabertooth, otherwise they are taken as velocities
	// NOT USED/IMPLEMENTED YET
	mode_control_val = data.data;
}

void mode_relay(const std_msgs::Bool& data) {
	// Shut off should be fast, so it's done here. If done in the loop, the delay could be as big
	// as 200 ms. A True is needed to turn on the relay and thus the motors. Since the analogWrite
	// call for actually "moving" the motors is in loop(), there will be a small delay when turning on
	mode_relay_val = data.data;
	if (mode_relay_val)
		digitalWrite(7, HIGH);
	else {
		digitalWrite(7, LOW);
		turn_off();
	}
}

ros::Subscriber<std_msgs::Int16MultiArray> tf_vdes_sub("tf_vdes", tf_vdes);
ros::Subscriber<std_msgs::Bool> mode_control_sub("mc", mode_control);
ros::Subscriber<std_msgs::Bool> mode_relay_sub("mr", mode_relay);

void start_ros() {
	nh.initNode();
	nh.subscribe(tf_vdes_sub);
	nh.subscribe(mode_control_sub);
	nh.subscribe(mode_relay_sub);
}
// ********************************************************************************************************************
void setup() {
	start_ros();

	// Turn off the relay at the start
	digitalWrite(7, LOW);
	pinMode(7, OUTPUT);
}

void loop() {
	nh.spinOnce();

	if (mode_relay_val == false)
		turn_off();
	else {
		analogWrite(motor1Pin, input_vdes_1);
		analogWrite(motor2Pin, input_vdes_2);
		analogWrite(motor3Pin, input_vdes_3);
		analogWrite(motor4Pin, input_vdes_4);
	}
}
