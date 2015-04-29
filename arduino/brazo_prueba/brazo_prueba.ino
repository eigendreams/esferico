/*#define USE_USBCON
#include "ros.h"
#include "std_msgs/Int16.h"

#include "Servo.h"
#include "DynamixelSerial.h"
#include "SimpleDynamixel.h"
#include "SimpleDriver.h"
#include "Talon.h"
#include "SimpleServo.h"
#include "AS5043.h"
#include "Encoder.h"

ros::NodeHandle nh;
Servo munnieca_servo;
EncoderClass ENCMUNNIECA(A0, 0, 1023, 100, 100);
int wrist_out= 0;
int wrist_lec;
unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
bool timedOut = false;
void wrist_out_cb(const std_msgs::Int16& dmsg) {wrist_out = dmsg.data;}
void alive_cb(const std_msgs::Int16& dmsg) {
	if (dmsg.data) {
  	milisLastMsg = millis();
  	timedOut = false;
	}
}
ros::Subscriber<std_msgs::Int16> wrist_out_sub("wrist_out", wrist_out_cb);
ros::Subscriber<std_msgs::Int16> alive_sub("alive", alive_cb);
std_msgs::Int16 wrist_lec_msg;
ros::Publisher wrist_lec_pub("wrist_lec", &wrist_lec_msg);

void setup() {
	munnieca_servo.attach(9);
	pinMode(A0, INPUT);
	nh.initNode();
	nh.subscribe(wrist_out_sub);
	nh.subscribe(alive_sub);
	nh.advertise(wrist_lec_pub);
}

void loop() {
	nh.spinOnce();
	unsigned long milisNow = millis();
	munnieca_servo.write(wrist_out * 5 + 1500);
	// 10 Hz operation
	if (milisNow - milisLast >= 100) {
		milisLast = milisNow;
	    if (milisNow - milisLastMsg >= 3500) {
	      wrist_out = 0;		// CHECK THIS ONE!!!
	      timedOut = true;
	    }
	    wrist_lec = ENCMUNNIECA.read();
		munnieca_servo.write(wrist_out * 5 + 1500);
		wrist_lec_msg.data = wrist_lec;
		wrist_lec_pub.publish(&wrist_lec_msg); delay(1);
	}
}*/

#define USE_USBCON
#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"

#include "Servo.h"
#include "DynamixelSerial.h"
#include "SimpleDynamixel.h"
#include "SimpleDriver.h"
#include "Talon.h"
#include "SimpleServo.h"
#include "AS5043.h"
#include "Encoder.h"

ros::NodeHandle nh;

// Pins 0, 1 (Serial1 in the LEONARDO) and 2 (DirectionPin) for communication with Dynamixel AX-12 servo (sixth DOF)
// DynamixelClass(Serial, direction_pin)
// SimpleDynamixelClass(DynamixelClass, min_angle_0_1000, max_angle_0_1000, map_abs_angle) // min maps to -angle and max to +angle, min > max is possible
DynamixelClass Dynamixelobj(&Serial1, 2);
SimpleDynamixelClass DYNAMIXEL(&Dynamixelobj, 480, 666, 100);

// Pins 3, 5 for DC Motor control (first DOF) and pins 4, 6 for DC Motor Control (fourth DOF)
// SimpleDriverClass(pin_en, pin_dir, umbral, val) // sense == sign(val), max_angle == abs(val)
SimpleDriverClass BASE(5, 3, 2, 100);
SimpleDriverClass MUNNIECA(6, 4, 2, 100);

// Pins 7 for Talon control (second DOF) and 8 for Talon control (third DOF)
// TalonClass(pin, umbral, val) // sense == sign(val), max_angle == abs(val)
//TalonClass BRAZO(2, 50);
//TalonClass ANTEBRAZO(2, 50);

// Pin 9 for standard servo control (fifth DOF)
// SimpleServoClass(pin, val) // sense == sign(val), max_angle == abs(val) (from -90 to +90)
//Servo brazo_servo;
//Servo antebrazo_servo;
Servo gripper_servo;
Servo munnieca_servo;
//SimpleServoClass PALMA(&gripper_servo, 100);

// Pins 13, 12, 11 for SPI single reads of magnetic encoders (second, third and fourth DOFs) in software emulation mode
// The CS pin must be specified at each constructor of the EncoderClaIP Address:ss, pin 10 is best left ALONE in the UNO!
// AS5043Class(pin_clk, pin_do, pin_prog)
AS5043Class AS5043obj(13, 12, 11);

// TODO Fix the numbers!!!
// Pin A0 for pot read (first DOF), specifies min and max values in EFFECTIVE TURN, positive sense, sampling size of 1
// EncoderClass(pin_An, min, max, val) // min > max is possible (not recommended), sense == sign(val), max_angle == abs(val)
// Pins A1...A3 as CS for SPI read of each sensor, positive sense, sampling size of 1
// EncoderClass(AS5043Class, pin_csn, min, max, val), as before
EncoderClass ENCBASE(&AS5043obj, A0, 485, 1004, -100);
EncoderClass ENCANTEBRAZO(& AS5043obj, A1, 550, 90, -100);
EncoderClass ENCBRAZO(&AS5043obj, A2, 350, 820, -100);
EncoderClass ENCMUNNIECA(A3, 350, 895, 100, 100);

// The incoming 6 DOF int16 information from ROS
int base_out = 0;
//int arm_out = 0;
//int forearm_out = 0;
int wrist_out= 0;
int palm_out = 0;
int gripper_out = 0;

int base_lec;
int arm_lec;
int forearm_lec;
int wrist_lec;
int palm_lec;
int gripper_lec;

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
bool timedOut = false;

void base_out_cb(const std_msgs::Int16& dmsg) {base_out = dmsg.data;}
//void arm_out_cb(const std_msgs::Int16& dmsg) {arm_out = dmsg.data;}
//void forearm_out_cb(const std_msgs::Int16& dmsg) {forearm_out = dmsg.data;}
void wrist_out_cb(const std_msgs::Int16& dmsg) {wrist_out = dmsg.data;}
void palm_out_cb(const std_msgs::Int16& dmsg) {palm_out = dmsg.data;}
void gripper_out_cb(const std_msgs::Int16& dmsg) {gripper_out = dmsg.data;}
void alive_cb(const std_msgs::Int16& dmsg) {
	if (dmsg.data) {
  		milisLastMsg = millis();
  		timedOut = false;
	}
}

ros::Subscriber<std_msgs::Int16> base_out_sub("base_out", base_out_cb);
//ros::Subscriber<std_msgs::Int16> arm_out_sub("arm_out", arm_out_cb);
//ros::Subscriber<std_msgs::Int16> forearm_out_sub("forearm_out", forearm_out_cb);
ros::Subscriber<std_msgs::Int16> wrist_out_sub("wrist_out", wrist_out_cb);
ros::Subscriber<std_msgs::Int16> palm_out_sub("palm_out", palm_out_cb);
ros::Subscriber<std_msgs::Int16> gripper_out_sub("gripper_out", gripper_out_cb);
ros::Subscriber<std_msgs::Int16> alive_sub("alive", alive_cb);

void gripper_reset_cb(const std_msgs::Int16& dmsg) {
	if (dmsg.data) {
		Dynamixelobj.reset(1);
		Dynamixelobj.begin(1000000UL, 2);
		Dynamixelobj.reset(1);
		Dynamixelobj.setMaxTorque(1, 1023);
	}
}
ros::Subscriber<std_msgs::Int16> gripper_reset_sub("gripper_reset", gripper_reset_cb);

std_msgs::Int16 base_lec_msg;
std_msgs::Int16 arm_lec_msg;
std_msgs::Int16 forearm_lec_msg;
std_msgs::Int16 wrist_lec_msg;
//std_msgs::Int16 palm_lec_msg;
//std_msgs::Int16 gripper_lec_msg;
ros::Publisher base_lec_pub("base_lec", &base_lec_msg);
ros::Publisher arm_lec_pub("arm_lec", &arm_lec_msg);
ros::Publisher forearm_lec_pub("forearm_lec", &forearm_lec_msg);
ros::Publisher wrist_lec_pub("wrist_lec", &wrist_lec_msg);
//ros::Publisher palm_lec_pub("palm_lec", &palm_lec_msg);
//ros::Publisher gripper_lec_pub("gripper_lec", &gripper_lec_msg);

void setup() {



	BASE.begin();
	MUNNIECA.begin();

	//pinMode(7, OUTPUT);
	//pinMode(8, OUTPUT);
	//pinMode(9, OUTPUT);

	//brazo_servo.attach(7);
	//antebrazo_servo.attach(8);
	//BRAZO.attach(7);
	//ANTEBRAZO.attach(8);
	munnieca_servo.attach(7);
	gripper_servo.attach(9);

	// Select encoders as digital
	pinMode(A0, OUTPUT);
	pinMode(A1, OUTPUT);
	pinMode(A2, OUTPUT);

	nh.initNode();

	nh.subscribe(base_out_sub);
	//nh.subscribe(arm_out_sub);
	//nh.subscribe(forearm_out_sub);
	nh.subscribe(wrist_out_sub);
	nh.subscribe(palm_out_sub);
	nh.subscribe(gripper_out_sub);

	nh.subscribe(alive_sub);

	nh.subscribe(gripper_reset_sub);

	nh.advertise(base_lec_pub);
	nh.advertise(arm_lec_pub);
	nh.advertise(forearm_lec_pub);
	nh.advertise(wrist_lec_pub);
	//nh.advertise(palm_lec_pub);
	//nh.advertise(gripper_lec_pub);

	// Comm at 1 Mhz and ENC initialization
	Dynamixelobj.begin(1000000UL, 2);
	Dynamixelobj.setMaxTorque(1, 1023);
	AS5043obj.begin();
}

void loop() {
	nh.spinOnce();

	unsigned long milisNow = millis();

	//brazo_servo.write(arm_out * 5 + 1500);
	//antebrazo_servo.write(forearm_out * 5 + 1500);
	gripper_servo.write(palm_out * 5 + 1500);
	munnieca_servo.write(wrist_out * 5 + 1500);

	if (Dynamixelobj.ping(1) == -1) {
		Dynamixelobj.begin(1000000UL, 2);
		Dynamixelobj.setMaxTorque(1, 1023);
	}

	// 20 Hz operation
	if (milisNow - milisLast >= 100) {

		milisLast = milisNow;

		// Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
	    // to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
	    if (milisNow - milisLastMsg >= 3500) {
	      base_out = 0;
	      //arm_out = 0;
	      //forearm_out = 0;
	      wrist_out = 0;		// CHECK THIS ONE!!!
	      palm_out = 0;
	      gripper_out = 0;
	      timedOut = true;
	    }

	    // Obten los valores absolutos de los encoders
	    base_lec = ENCBASE.read();
	    arm_lec = ENCBRAZO.read();
	    forearm_lec = ENCANTEBRAZO.read();
	    wrist_lec = ENCMUNNIECA.read();
	    //palm_lec = PALMA.read();
	    //gripper_lec = DYNAMIXEL.read();

		BASE.write(base_out);
		//BRAZO.write(arm_out);
		//ANTEBRAZO.write(forearm_out);
		MUNNIECA.write(wrist_out);
		//gripper_servo.write(palm_out * 5 + 1500);
		DYNAMIXEL.write(gripper_out);

		//brazo_servo.write(arm_out * 5 + 1500);
		//antebrazo_servo.write(forearm_out * 5 + 1500);
		gripper_servo.write(palm_out * 5 + 1500);
		munnieca_servo.write(wrist_out * 5 + 1500);


		base_lec_msg.data = base_lec;
		arm_lec_msg.data = arm_lec;
		forearm_lec_msg.data = forearm_lec;
		wrist_lec_msg.data = wrist_lec;
		//palm_lec_msg.data = palm_lec;
		//gripper_lec_msg.data = gripper_lec;

		base_lec_pub.publish(&base_lec_msg); delay(1);
		arm_lec_pub.publish(&arm_lec_msg); delay(1);
		forearm_lec_pub.publish(&forearm_lec_msg); delay(1);
		wrist_lec_pub.publish(&wrist_lec_msg); delay(1);
		//palm_lec_pub.publish(&palm_lec_msg); delay(1);
		//gripper_lec_pub.publish(&gripper_lec_msg); delay(1);
	}
}

