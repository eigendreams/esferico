//////////////////////////////////////////////////////////////////////////////////////////////
#define MIN_PULSE_WIDTH       500     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2000     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1000     // default pulse width when servo is attached
#define REFRESH_INTERVAL    12500     // minumim time to refresh servos in microseconds 
#define SERVOS_PER_TIMER       8      // the maximum number of servos controlled by one timer 
//////////////////////////////////////////////////////////////////////////////////////////////

#include "Arduino.h"
#include "Servo.h"
#include "ros.h"
#include "std_msgs/Int16.h"

//////////////////////////////////////////////////////////////////////////////////////////////
#define MIN_PULSE_WIDTH       500     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2000     // the longest pulse sent to a servo 
#define DEFAULT_PULSE_WIDTH  1000     // default pulse width when servo is attached
#define REFRESH_INTERVAL    12500     // minumim time to refresh servos in microseconds 
#define SERVOS_PER_TIMER       8      // the maximum number of servos controlled by one timer 
//////////////////////////////////////////////////////////////////////////////////////////////

ros::NodeHandle nh;

Servo servoobj1;

///////////////
Servo rcservo1;
Servo rcservo2;
Servo rcservo3;
Servo rcservo4;
Servo rcservo5;
///////////////

unsigned long milisLast = 0;
unsigned long timeLastMsg = 0;	// used to watch for timeOut
bool timedOut = false;

int motor1des = 0;
int motor2des = 0;
int servo1des = 0;

// All the subs handlers functions, update is asynchronous (done in the loop), keep names short
void motor1_h(const std_msgs::Int16& d_msg) {	motor1des = d_msg.data;	timedOut = false;	timeLastMsg = millis();	}
void motor2_h(const std_msgs::Int16& d_msg) {	motor2des = d_msg.data;	timedOut = false;	timeLastMsg = millis();	}
void servo1_h(const std_msgs::Int16& d_msg) {	servo1des = d_msg.data;	timedOut = false;	timeLastMsg = millis();	}
ros::Subscriber<std_msgs::Int16> motor_sub1("motor1", motor1_h);
ros::Subscriber<std_msgs::Int16> motor_sub2("motor2", motor2_h);
ros::Subscriber<std_msgs::Int16> servo_sub1("servo1", servo1_h);

void setup() {

	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);

	servoobj1.attach(9);

	////////////////////
	rcservo1.attach(10);
	rcservo2.attach(13);
	rcservo3.attach(10);
	rcservo4.attach(13);
	rcservo5.attach(10);
	////////////////////
	digitalWrite(10, 0);
	////////////////////

	digitalWrite(3, 0);
	digitalWrite(4, 0);

	analogWrite(5, 0);
	analogWrite(6, 0);

	digitalWrite(7, 0);
	digitalWrite(8, 0);
	
	digitalWrite(9, 0);

	nh.initNode();

	nh.subscribe(motor_sub1);
	nh.subscribe(motor_sub2);
	nh.subscribe(servo_sub1);
}

void loop() {
	unsigned long milisNow = millis();

	// 20 Hz / 50 ms operation is best, it seems
	if (milisNow - milisLast >= 200) {

		nh.spinOnce();

		// Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
		// to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
		if (milisNow - timeLastMsg >= 5000) {
			motor1des = 0;
			motor2des = 0;
			servo1des = 0;
			timedOut = true;
		}

        //valores de -255 a + 255
        digitalWrite(3, motor1des > 0);
        digitalWrite(4, motor1des < 0);
        analogWrite(5, map(abs(motor1des), 0, 100, 0, 255));

        analogWrite(6, map(abs(motor2des), 0, 100, 0, 255));
        digitalWrite(7, motor2des > 0);
        digitalWrite(8, motor2des < 0);

        //////////////////////////////////////////////////////////////
        rcservo1.write(544);
        rcservo2.write(   (456 * motor1des) / 100 + 1000   );
        rcservo3.write(544);
        rcservo4.write(   (456 * motor2des) / 100 + 1000   )
        rcservo5.write(544)
        //////////////////////////////////////////////////////////////
		servoobj1.write(   (1000 * (servo1des + 90)) / 180 + 1000   );
		//////////////////////////////////////////////////////////////

		milisLast = milisNow;
	}
}
