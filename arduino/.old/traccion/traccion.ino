#include "ros.h"
#include "finder/TwoWheelInt16.h"
#include "std_msgs/Int16.h"

#include "Servo.h"
#include "OSMC.h"
#include "AS5043.h"
#include "Encoder.h"
#include "SimplePID.h"

// (pinpwm1, pinpwm2, umbral, maxpwm_sense) outputs below umbral are taken as 0
OSMCClass LEFT(5, 3, 1, 127);
OSMCClass RIGHT(9, 6, 1, 127);

// (CLK, DO, PROG)
// (AS5043, CSn, input_min, input_max, output_max_abs_sense) output goes from -511 to + 511 in this case
AS5043Class AS5043obj(13, 12, 11);
EncoderClass ENCLEFT(&AS5043obj, A0, 0, 1023, 511);
EncoderClass ENCRIGHT(&AS5043obj, A1, 0, 1023, 511);

// (kp, ki, kd, km, umbral) errors below umbral are taken as 0
SimplePID PIDLEFT(2, .1, .0, .1, 0, 50);
SimplePID PIDRIGHT(2, .1, .0, .1, 0, 50);

ros::NodeHandle nh;

int control_var  = 0;
int left_des = 0;			// speed in LEFT
int right_des = 0;			// speed in RIGHT
unsigned long milisLast = 0;
unsigned long timeLastMsg = 0;	// used to watch for timeOut
bool timedOut = false;

/*
void traction_des_cb(const finder::TwoWheelInt16& dmsg) {
	left_des = dmsg.left;
	right_des = dmsg.right;
}
ros::Subscriber<finder::TwoWheelInt16> traction_des_sub("traction_motors", traction_des_cb);
*/

void left_des_cb(const std_msgs::Int16& dmsg) {
	left_des = dmsg.data;
	timeLastMsg = millis();
	timedOut = false;
}
void right_des_cb(const std_msgs::Int16& dmsg) {
	right_des = dmsg.data;
	timeLastMsg = millis();
	timedOut = false;
}
void control_mode_cb(const std_msgs::Int16& dmsg) {
	control_var = dmsg.data;
}
ros::Subscriber<std_msgs::Int16> left_des_sub("motor_traction_left", left_des_cb);
ros::Subscriber<std_msgs::Int16> right_des_sub("motor_traction_right", right_des_cb);
ros::Subscriber<std_msgs::Int16> control_mode_sub("control_mode", control_mode_cb);

// All the pubs msgs, keep names short
finder::TwoWheelInt16 finder_out;
finder::TwoWheelInt16 finder_lec;
ros::Publisher finder_out_pub("finder_output", &finder_out);
ros::Publisher finder_lec_pub("finder_lecture", &finder_lec);

void setup() {
	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);

	// Aseugrando que los OSMC funcionen
	digitalWrite(2, LOW);
	digitalWrite(4, LOW);

	nh.initNode();

	nh.subscribe(control_mode_sub);
	nh.subscribe(left_des_sub);
	nh.subscribe(right_des_sub);
	nh.advertise(finder_out_pub);
	nh.advertise(finder_lec_pub);

	// Perform first encoders read, to get valid changes later
	ENCLEFT.begin();
	ENCRIGHT.begin();
}

int select(int value) {
  if (value > 512)	return value - 1023;
  if (value < -512)	return value + 1023;
  return value;
}

int left_lec;
int right_lec;

void loop() {
	nh.spinOnce();
	unsigned long milisNow = millis();

	// 20 Hz / 50 ms operation is best, it seems
	if (milisNow - milisLast >= 50) {

		// Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
		// to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
		if (milisNow - timeLastMsg >= 5000) {
			left_des = 0;
			right_des = 0;
			timedOut = true;
		}

		// Calculate change in encoders
		int left_lecNow = ENCLEFT.read();
		int right_lecNow = ENCRIGHT.read();

		int change_left = left_lecNow - left_lec;
		int change_right = right_lecNow - right_lec;

		left_lec = left_lecNow;
		right_lec = right_lecNow;

		change_left = select(change_left);
		change_right = select(change_right);

		change_left = (change_left * 200. / 1023.);
    	change_right = (change_right * 200. / 1023.);

		PIDLEFT.compute(change_left, left_des);
		PIDLEFT.compute(change_right, right_des);

		if (control_var) {
			LEFT.write(PIDLEFT.get());
			RIGHT.write(PIDRIGHT.get());
		}
		else {
			LEFT.write(left_des);
			RIGHT.write(right_des);
		}

		finder_out.left = PIDLEFT.get();
    	finder_out.right = PIDRIGHT.get();
    	finder_out_pub.publish(&finder_out);

		finder_lec.left = change_left;
		finder_lec.right = change_right;
		finder_lec_pub.publish(&finder_lec);

		milisLast = milisNow;
	}
}
