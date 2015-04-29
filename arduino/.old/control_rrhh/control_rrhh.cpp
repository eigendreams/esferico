// The code is edited and compiled in Eclipse, using the Arduino Core library and AVR compiler
//
// The following code turns a "switching" relay of the RRHH on and off, and replicates the desired PWM values in the
// corresponding pins. Two SaberTooths (25 A x 2 channels) are used as motor controllers, using the analog input mode.
// A single stage filter is used to transform the PWM outputs to continuous voltage values.
// A motor speed of zero maps to an analog output of 2.5 V to the SaberTooth

#include <Arduino.h>
#include "SimpleSaber.h"

#include "ros.h"
#include "rrhh/RRHH.h"
#include "std_msgs/Bool.h"

SimpleSaberClass motor1(5, 2, 127);
SimpleSaberClass motor2(6, 2, 127);
SimpleSaberClass motor3(9, 2, 127);
SimpleSaberClass motor4(10, 2, 127);

ros::NodeHandle nh;

bool mode_control_val = true;
bool mode_relay_val = false;
int input_vdes_1 = 0;
int input_vdes_2 = 0;
int input_vdes_3 = 0;
int input_vdes_4 = 0;

void turn_off() {
	digitalWrite(7, LOW);
	// Desired values are "zeroed", just in case
	input_vdes_1 = 0;
	input_vdes_2 = 0;
	input_vdes_3 = 0;
	input_vdes_4 = 0;
	// All motors are stopped
	motor1.stop();
	motor2.stop();
	motor3.stop();
	motor4.stop();
}

void tf_vdes(const rrhh::RRHH& dmsg) {
	if (mode_relay_val) {
		input_vdes_1 = dmsg.wheel1;
		input_vdes_2 = dmsg.wheel2;
		input_vdes_3 = dmsg.wheel3;
		input_vdes_4 = dmsg.wheel4;
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
	if (mode_relay_val) digitalWrite(7, HIGH);
	else turn_off();
}

ros::Subscriber<rrhh::RRHH> tf_vdes_sub("tf_vdes", tf_vdes);
ros::Subscriber<std_msgs::Bool> mode_control_sub("mc", mode_control);
ros::Subscriber<std_msgs::Bool> mode_relay_sub("mr", mode_relay);

void setup() {
	nh.initNode();
	nh.subscribe(tf_vdes_sub);
	nh.subscribe(mode_control_sub);
	nh.subscribe(mode_relay_sub);

	// Turn off at the start (ensure 0 values)
	pinMode(7, OUTPUT);
	turn_off();
}

void loop() {
	nh.spinOnce();

	if (mode_relay_val) {
		motor1.write(input_vdes_1);
		motor2.write(input_vdes_2);
		motor3.write(input_vdes_3);
		motor4.write(input_vdes_4);
	}
}
