/*
 * New robotic arm control library, using the LEONARDO ONLY (needs more than 2K RAM)
 *
 * *****************************************************************************
 *
 * Device map:
 *
 * 	DOF		MOTOR		DRIVER		NEEDS		PINS		SENSOR		MODE
 * 	1		DC			SHIELD		EN, SE		3, 5		POT, MGN	A0, SPI
 * 	2		BIG DC		TALON		SERVO		7			MGN			A1, SPI
 * 	3		BIG DC		TALON		SERVO		8			MGN			A2, SPI
 * 	4		DC			SHIELD		EN, SE		4, 6		MGN			A3, SPI
 * 	5		SERVO		-			SERVO		9			-			-
 * 	6		DYN			-			DYN			0, 1, 2		-			-
 *
 * SPI takes pins 11, 12, 13 (in software mode because it's a LEONARDO, use 10 as CS if encoders are chained
 * DYN takes pins 0, 1, 2, NEEDS A LEONARDO!!! (because of Hardware Serial1)
 *
 * *****************************************************************************
 *
 * Pin map:
 *
 * 	PIN		DESC
 * 	0		Dynamixel RX-TX
 * 	1		Dynamixel RX-TX
 * 	2		Dynamixel direction select
 * 	3		DC Motor 1 Direction
 * 	4		DC Motor 2 Direction
 * 	5		DC Motor 1 Enable
 * 	6		DC Motor 2 Enable
 * 	7		Talon 1
 * 	8		Talon 2
 * 	9		Servo
 * 	10		AS5043 CS if chained encoders OR extra Servo
 * 	11		AS5043 PROG
 * 	12		AS5043 DO
 * 	13		AS5043 CLK
 *
 * 	A0		DC Motor ANALOG INPUT if potentiometer, if MGN then CS0 or AnalogOut
 * 	A1		AS5043 CS1 or AnalogOut
 * 	A2		AS5043 CS2 or AnalogOut
 * 	A3		AS5043 CS3 or AnalogOut
 *
 * *****************************************************************************
 *
 * The following libraries are needed:
 *
 *	Servo
 *	ros
 *	DynamixelSerial
 *	SimpleDynamixel
 *	SimpleDriver
 *	Talon
 *	SimpleServo
 *	AS5043
 *	Encoder
 *	SimplePID
 *
 * The ros library is a modified version for the Leonardo
 * The DynamixelSerial class needs a HardwareSerial object passed in the constructor
 * The SimpleDynamixel is a wrapper class for DynamixelSerial and needs one instance of it passed
 * The SimpleDriver is a wrapper class for motor control
 * The Talon is a simple class for Talon control (uses the Servo class)
 * The SimpleServo is a simple class for servo control (to ensure consistency with other libs)
 * The AS5043 class needs the pin numbers when used in software mode (software mode needed in LEONARDO)
 * The Encoder class is a wrapper for the AS5043 class and for an analogInput "equivalent" functionality
 * The SimplePID is a simple controller class
 *
 * *****************************************************************************
 *
 * Now, all sensors (and all sensor should be magnetic encoders, though the base sensor
 * could be a simple multiturn or endless pot) have a resolution of 10 bits, from 0
 * to 1023, now, only the first 4 DOF need any form of control, since the last 2 DOF,
 * being servos, can simply be written to with the desired angle value.
 *
 * However, there are differences between the use of small DC motors and the BIG DC motors. The
 * DC motors use a shield, and can be set with a PWM from 2 * (-127 to + 127). The Talon
 * motors, being "servo-like", could be called with an angle, but the encapsulating Talon
 * class deals with them as "motor-like" devices, going from -127 to +127. This is done
 * to ensure "orthogonality", and to create general control functions. Since -128 * 2 = -256 is
 * outside the range of PWM output, we must restrict values to [-127, +127].
 *
 * The max "angle" absolute value of the Servo output classes SimpleDynamixel and SimpleServo
 * can be specified in the constructor. This way, we can set "angles" for each servo as
 * however we like, -90 to +90, -127 to +127, etcetera, mapping to "standard" 0 to 180
 *
 * All motors, then, are set with values from -angle to +angle, be it speed (SimpleDriver, Talon)
 * or position (SimpleDynamixel, SimpleServo). Another class, SaberSerial, also follows this
 * convention (set in hardware too) with -127 to +127. The operation range is covered with Int16
 *
 * This way all sensors are 10 bits, and all actuators are 8/9/10 bits. Even though this loses
 * resolution in the PWM output of the simple motor driver and servo output of the Talon wrapper, we
 * consider that 255 values are enough for most non high precision scenarios, and that
 * the PWM signal does not loses appreciable effect. But we gain uniformity, consistency,
 * and less hassle when dealing with all the DOF for control tasks.
 *
 * *****************************************************************************
 *
 * All encoders use (or should use) the same AS5043Class instance. The "proper" way to read them
 * would be to use the read() method of the AS5043Class instance, and then get the information of each
 * encoder with the get() method of each EncoderClass instance. There is a read() method in the
 * EncoderClass, but using it is expensive (ALL sensor would be reread at each call, this takes
 * much more time and scrambles timeStamps and angle changes).
 *
 * *****************************************************************************
 *
 * Finally, since a resolution reduction is necessary in some of the DOFs, the same -127 to +127
 * convention can also be applied to encoder reads, if the max angle of the encoder is set to +127 at
 * construction, but could be set to something else too. This is particularly important if using a multiturn
 * pot to read position since resolution is constrained anyway. Or for example, when using a restricted
 * DOF, a resolution reduction is convenient. Also reduces noise, somewhat. Range is covered with Int16
 *
 * *****************************************************************************
 *
 * Keep in ming that the "best" initial request for each DOF should be something on the line of:
 *
 * 	BASE -> 0		(goes from -180 to +180)
 * 	BRAZO -> -90		(goes from -90 to +90)
 * 	ANTEBRAZO -> -90		(goes from -90 to +90)
 * 	MUNNIECA -> 0		(goes from -90 to +90)
 * 	PALMA -> 0		(goes from -90 to +90)
 * 	DYNAMIXEL -> 0		(goes from -90 to +90)
 */
#define USE_USBCON
#include "ros.h"
#include "arm_interface/Arm.h"
#include "std_msgs/Int16.h"

#include "Servo.h"
#include "DynamixelSerial.h"
#include "SimpleDynamixel.h"
#include "SimpleDriver.h"
#include "Talon.h"
#include "SimpleServo.h"
#include "AS5043.h"
#include "Encoder.h"
#include "SimplePID.h"
#include "SimpleOpen.h"

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
TalonClass BRAZO(2, 50);
TalonClass ANTEBRAZO(2, 50);

// Pin 9 for standard servo control (fifth DOF)
// SimpleServoClass(pin, val) // sense == sign(val), max_angle == abs(val) (from -90 to +90)
Servo servoobj;
SimpleServoClass PALMA(&servoobj, 100);

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
EncoderClass ENCBRAZO(&AS5043obj, A1, 350, 820, -100);
EncoderClass ENCANTEBRAZO(& AS5043obj, A2, 550, 90, -100);
EncoderClass ENCMUNNIECA(A3, 350, 895, 100, 100);

// TODO Fix the numbers!!!
// The set of PID controllers for each controllable DOF, set kp, ki, kd, km IN SECONDS!!!, last value is umbral for zero error/ zero output
SimplePID PIDBASE(5, 0, 0, 0, 0, 100); // NEEDS an umbral of zero to avoid vanishing of the ITerm contribution (if set), because motor cannot hold itself
SimplePID PIDBRAZO(4, 0, 0, 0, 7, 50);
SimplePID PIDANTE(4, 0, 0, 0, 7, 50);
SimplePID PIDMUNNIECA(5, 0, 0, 0, 0, 100);
// TODO fix description!!! Not PID yet
// float KScale, int StartValue, int OutputFixed (+/-)
// SimpleOpen OPENMUNNIECA(0.9, 0, 127);

// The incoming 6 DOF int16 information from ROS
int base_des = 0;
int brazo_des = 0;
int antebrazo_des = 0;
int munnieca_des = 0;
int palma_des = 0;
int dynamixel_des = 0;

// // All the subs handlers functions, update is asynchronous (done in the loop), keep names short
// void arm_des_cb(const arm_interface::Arm& d_msg) {
// 	base_des = d_msg.base;
// 	brazo_des = d_msg.brazo;
// 	antebrazo_des = d_msg.antebrazo;
// 	munnieca_des = d_msg.munnieca;
// 	palma_des = d_msg.palma;
// 	dynamixel_des = d_msg.dynamixelBASE;
// }
// ros::Subscriber<arm_interface::Arm> arms_des_sub("arm_motors", arm_des_cb);

void arm_des_cb(const std_msgs::Int16& d_msg) {
	base_des = d_msg.data;
 	/*brazo_des = 0;
 	antebrazo_des = 0;
 	munnieca_des = 0;
 	palma_des = 0;
 	dynamixel_des = 0;*/
}
ros::Subscriber<std_msgs::Int16> arms_des_sub("motor_arm_base", arm_des_cb);

// All the pubs msgs, keep names short
arm_interface::Arm arms_lec;
ros::Publisher arms_lec_pub("arm_lectures", &arms_lec);

unsigned long milisLast = 0;

void setup() {
	BASE.begin();
	MUNNIECA.begin();

	BRAZO.attach(7);
	ANTEBRAZO.attach(8);
	PALMA.attach(10);

	pinMode(A0, OUTPUT);
	pinMode(A1, OUTPUT);

	//servoobj.attach(10);

	nh.initNode();

	nh.subscribe(arms_des_sub);
	nh.advertise(arms_lec_pub);

	// Comm at 1 Mhz
	Dynamixelobj.begin(1000000UL, 2);
	DYNAMIXEL.begin();
	AS5043obj.begin();

	// Initialize desired values to ACTUAL values (to avoid jerk)
	base_des = ENCBASE.read();
	brazo_des = ENCBRAZO.read();
	antebrazo_des = ENCANTEBRAZO.read();
	munnieca_des = 0;
	palma_des = 0;
	dynamixel_des = 0;
}

void loop() {
	nh.spinOnce();
	unsigned long milisNow = millis();

	// Maybe in here
	// MUNNIECA.write(OPENMUNNIECA.check());

	// 20 Hz operation
	if (milisNow - milisLast >= 50) {

		milisLast = milisNow;

		int base_lec = ENCBASE.readAngle();
		int brazo_lec = ENCBRAZO.readAngle();
		int antebrazo_lec = ENCANTEBRAZO.readAngle();
		int munnieca_lec = ENCMUNNIECA.readAngle();

		PIDBASE.compute(base_lec, base_des);
		PIDBRAZO.compute(brazo_lec, brazo_des);
		PIDANTE.compute(antebrazo_lec, antebrazo_des);
		PIDMUNNIECA.compute(munnieca_lec, munnieca_des);
		// OPENMUNNIECA.setGoal(munnieca_des);

		/*
		BASE.write(PIDBASE.get());
		BRAZO.write(PIDBRAZO.get());
		ANTEBRAZO.write(PIDANTE.get());
		MUNNIECA.write(OPENMUNNIECA.check());
		PALMA.write(palma_des);
		DYNAMIXEL.write(dynamixel_des);
		*/

		BASE.write(PIDBASE.get());
		BRAZO.write(PIDBRAZO.get());//brazo_des);
		ANTEBRAZO.write(PIDANTE.get());//antebrazo_des);
		MUNNIECA.write(0*PIDMUNNIECA.get());
		PALMA.write(palma_des);
		DYNAMIXEL.write(dynamixel_des);

		arms_lec.base = base_lec;
		arms_lec.brazo = brazo_lec;
		arms_lec.antebrazo = antebrazo_lec;
		arms_lec.munnieca = munnieca_lec;
		arms_lec.palma = PALMA.read();
		arms_lec.dynamixel = DYNAMIXEL.read();

		arms_lec_pub.publish(&arms_lec);
	}
}

