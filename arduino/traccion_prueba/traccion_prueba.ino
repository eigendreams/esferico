#define USE_USBCON
#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Empty.h"
#include "Servo.h"
#include "OSMC.h"
#include "AS5043.h"
#include "Encoder.h"

// (pinpwm1, pinpwm2, umbral, maxpwm_sense) outputs below umbral are taken as 0
OSMCClass LEFT(5, 3, 2, 1, 127);
OSMCClass RIGHT(9, 6, 4, 1, 127);

// (CLK, DO, PROG)
// (AS5043, CSn, input_min, input_max, output_max_abs_sense) output goes from -511 to + 511 in this case
AS5043Class AS5043obj(13, 12, 11);
EncoderClass ENCLEFT(&AS5043obj, A0, 0, 1023, 100);
EncoderClass ENCRIGHT(&AS5043obj, A1, 0, 1023, 100);

ros::NodeHandle nh;

int left_out = 0;			
int right_out = 0;		

int left_lec = 0;
int right_lec = 0;

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
bool timedOut = false;

void left_out_cb(const std_msgs::Int16& dmsg) {left_out = dmsg.data;}
void right_out_cb(const std_msgs::Int16& dmsg) {right_out = dmsg.data;}
void alive_cb(const std_msgs::Int16& dmsg) {
	if (dmsg.data) {
	milisLastMsg = millis();
	timedOut = false;
	}
}
ros::Subscriber<std_msgs::Int16> left_out_sub("left_out", left_out_cb);
ros::Subscriber<std_msgs::Int16> right_out_sub("right_out", right_out_cb);
ros::Subscriber<std_msgs::Int16> alive_sub("alive", alive_cb);

std_msgs::Int16 left_lec_msg;
std_msgs::Int16 right_lec_msg;
ros::Publisher left_lec_pub("left_lec", &left_lec_msg);
ros::Publisher right_lec_pub("right_lec", &right_lec_msg);

std_msgs::Int16 batt_v_msg;
std_msgs::Int16 mot_a_msg;
ros::Publisher batt_v_pub("volt", &batt_v_msg);
ros::Publisher mot_a_pub("amp", &mot_a_msg);

void setup() {
	// Configurando pines
	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);

	// Iniciando nodo de ROS
	nh.initNode();

	// Suscribiendo y publicando
	nh.subscribe(left_out_sub);
	delay(1);
	nh.subscribe(right_out_sub);
	delay(1);
	nh.subscribe(alive_sub);
	delay(1);
	nh.advertise(left_lec_pub);
	delay(1);
	nh.advertise(right_lec_pub);

    delay(1);
	nh.advertise(batt_v_pub);
	delay(1);
	nh.advertise(mot_a_pub);
	delay(1);

	// Inicializa los motores y encoders
	//LEFT.begin();
	//RIGHT.begin();
	//ENCLEFT.begin();
	//ENCRIGHT.begin();
}

void loop() {
	nh.spinOnce();

	unsigned long milisNow = millis();

	// 20 Hz / 1 ms operation is best, it seems
	if (milisNow - milisLast >= 100) {

		// Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
		// to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
		if (milisNow - milisLastMsg >= 3500) {
			left_out = 0;
			right_out = 0;
			timedOut = true;
		}

		//left_lec = ENCLEFT.read();
		//right_lec = ENCRIGHT.read();

		//LEFT.write(left_out);
		//RIGHT.write(right_out);

		left_lec_msg.data = left_lec;
		right_lec_msg.data = right_lec;

		left_lec_pub.publish(&left_lec_msg);
		right_lec_pub.publish(&right_lec_msg);

		int lec_v  = analogRead(A0);
		int lec_a1 = analogRead(A2);
		int lec_a2 = analogRead(A4);

		batt_v_msg.data = lec_v; // 1000 * ((5. * lec_v) / 1024.)  / (0.06369);
		mot_a_msg.data = 1000 * ((5. * lec_a1) / 1024.)  / (0.01830) + 1000 * ((5. * lec_a2) / 1024.)  / (0.01830);

 		delay(1);
		batt_v_pub.publish(&batt_v_msg);
		mot_a_pub.publish(&mot_a_msg);

		milisLast = milisNow;
	}
}
