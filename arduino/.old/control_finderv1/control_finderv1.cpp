#include <Arduino.h>

#include "ros.h"
#include "finder/TwoWheelInt16.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int16.h"

#include "SimpleSaber.h"
#include "AS5043.h"
#include "Encoder.h"
#include "SimplePID.h"

#include "SimpleDriver.h"

// (pinpwm, umbral, maxpwm_sense) outputs below umbral are taken as 0
SimpleSaberClass motor1(10, 1, 127);
SimpleSaberClass motor2(11, 1, 127);

// (CLK, DO, PROG)
AS5043Class AS5043obj(13, 12, 11);
// (AS5043, CSn, input_min, input_max, output_max_abs_sense) output goes from -511 to + 511 in this case
EncoderClass ENC1(&AS5043obj, A0, 0, 1023, 511);
EncoderClass ENC2(&AS5043obj, A1, 0, 1023, 511);

SimpleDriverClass DOF1(5, 3, 2, 127);
SimpleDriverClass DOF4(6, 4, 2, 127);

// (kp, ki, kd, km, umbral) errors below umbral are taken as 0
SimplePID pid1(.1, .1, .1, .1, 0);
SimplePID pid2(.1, .1, .1, .1, 0);

ros::NodeHandle nh;

bool mode_control_val = false;	// if true use pid control, if false pass PWMs directly

int input_vdes_1 = 0;			// speed in motor1
int input_vdes_2 = 0;			// speed in motor2

unsigned long sampleTime = 50;	// 20 Hz is good
unsigned long milisLast = 0;

unsigned long timeLastMsg = 0;	// used to watch for timeOut
bool timedOut = false;

void vdes(const finder::TwoWheelInt16& dmsg) {
	input_vdes_1 = dmsg.wheel1;
	input_vdes_2 = dmsg.wheel2;
	timeLastMsg = millis();
	timedOut = false;
}

void pid_max(const std_msgs::Int16& data) {
	int pidmax = min(abs(data.data), MAX_PID_OUT);
	pid1.setRange(pidmax);
	pid2.setRange(pidmax);
}

void mode_control(const std_msgs::Bool& data) {
	mode_control_val = data.data;
}

void time_sample(const std_msgs::UInt16& data) {
	sampleTime = abs(data.data);
}

ros::Subscriber<finder::TwoWheelInt16> vdes_sub("vdes", vdes);
ros::Subscriber<std_msgs::Int16> pid_max_sub("pid_max", pid_max);
ros::Subscriber<std_msgs::Bool> mode_control_sub("mc", mode_control);
ros::Subscriber<std_msgs::UInt16> time_sample_sub("ts", time_sample);

// All the pubs msgs, keep names short
finder::TwoWheelInt16 finderOut;
ros::Publisher finderOut_f("finder_lecture", &finderOut);
ros::Publisher finderOutPID_f("finder_output", &finderOut);

void setup() {
	nh.initNode();

	nh.subscribe(vdes_sub);
	nh.subscribe(pid_max_sub);
	nh.subscribe(mode_control_sub);
	nh.subscribe(time_sample_sub);
	nh.advertise(finderOut_f);
	nh.advertise(finderOutPID_f);

	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);

	// Perform first encoders read, to get valid changes later
	ENC1.readChange();
	ENC2.readChange();
}

void loop() {
	nh.spinOnce();

	unsigned long milisNow = millis();

	// 20 Hz / 50 ms operation is best, it seems
	if (milisNow - milisLast >= sampleTime) {

		// Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
		// to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
		if (milisNow - timeLastMsg > 5000) {
			input_vdes_1 = 0;
			input_vdes_2 = 0;
			timedOut = true;
		}

		// Calculate change in encoders
		int change1 = ENC1.read();
		int change2 = ENC2.readChange();

		/* // in case the encoders are analog, adapt this code:
		 * if (abs(change1) > max_steep) change1 = lastchange1 else read
		 * if (abs(change2) > max_steep) change2 = lastchange2 else read
		 * ...
		 * ...
		 * ...
		 * lastchange1 = change1;
		 * lastchange2 = change2;
		 */

		// if mode_control is true and not timed out, pass speeds as pid inputs etc
		// if mode_control is false or we timed out, pass speeds as PWM values (if timedOut speeds were set as 0)
		if (mode_control_val && !timedOut) {
			pid1.compute(change1, input_vdes_1);
			pid1.compute(change2, input_vdes_2);
			motor1.write(pid1.get());
			motor2.write(pid2.get());
		} else {
			motor1.write(input_vdes_1);
			motor2.write(input_vdes_2);
		}

		// Send data to PC, first changes and then pid outputs
		finderOut.wheel1 = change1;
		finderOut.wheel2 = change2;
		finderOut_f.publish(&finderOut);
		finderOut.wheel1 = pid1.get();
		finderOut.wheel2 = pid2.get();
		finderOutPID_f.publish(&finderOut);

		milisLast = milisNow;
	}
}
