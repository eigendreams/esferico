//#include "rosSerial5.h"
#include "ros.h"
#include "std_msgs/Int16.h"
#include "volti/float32_12.h"
#include "volti/float32_3.h"
#include "Servo.h"
#include "AS5043.h"
#include "Encoder.h"
#include "Wire.h"
#include "IMU.h"

ros::NodeHandle nh;

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
unsigned long millisTock = 0;
bool timedOut = false;

//volti::float32_12 imu_data1;
volti::float32_3 imu_data1;
ros::Publisher imu_pub1("i1", &imu_data1);
IMU imu1(1);

//volti::float32_12 imu_data2;
volti::float32_3 imu_data2;
ros::Publisher imu_pub2("i2", &imu_data2);
IMU imu2(3);

// vol1 vol2 mot1 mot2 motv
//Servo servo_11;
//Servo servo_12;
Servo servo_13;
Servo servo_14;
//Servo servo_15;

// CLK, DO, PROG
AS5043Class AS5043obj(15, 25, 26);
// CSN
EncoderClass gear1(&AS5043obj, 34, 485, 1004, -100);
EncoderClass gear2(&AS5043obj, 33, 485, 1004, -100);
//EncoderClass gearv(&AS5043obj, 32, 485, 1004, -100);

//volatile int s11_out = 0;  // v1
//volatile int s12_out = 0;  // v2
volatile int s13_out = 0;  // m1
volatile int s14_out = 0;  // m2
//volatile int s15_out = 0;  // mv

/*void s11_out_cb(const std_msgs::Int16& dmsg) {	
  s11_out = dmsg.data;	
}
ros::Subscriber<std_msgs::Int16> s11_out_sub("v1", s11_out_cb);
void s12_out_cb(const std_msgs::Int16& dmsg) {	
  s12_out = dmsg.data;	
}
ros::Subscriber<std_msgs::Int16> s12_out_sub("v2", s12_out_cb);*/
void s13_out_cb(const std_msgs::Int16& dmsg) {	
  s13_out = dmsg.data;	
}
ros::Subscriber<std_msgs::Int16> s13_out_sub("m1", s13_out_cb);
void s14_out_cb(const std_msgs::Int16& dmsg) {	
  s14_out = dmsg.data;	
}
ros::Subscriber<std_msgs::Int16> s14_out_sub("m2", s14_out_cb);
/*void s15_out_cb(const std_msgs::Int16& dmsg) {	
  s15_out = dmsg.data;	
}
ros::Subscriber<std_msgs::Int16> s15_out_sub("mv", s15_out_cb);*/

void alive_cb( const std_msgs::Int16& dmsg) {
  if (dmsg.data){
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    milisLastMsg = millis();
    timedOut = false;
  }
} 
ros::Subscriber<std_msgs::Int16> alive_sub("al", alive_cb);

int gear1_lec;
int gear2_lec;
//int gearv_lec;

std_msgs::Int16 gear1_lec_msg;
std_msgs::Int16 gear2_lec_msg;
//std_msgs::Int16 gearv_lec_msg;
ros::Publisher gear1_lec_pub("e1", &gear1_lec_msg);
ros::Publisher gear2_lec_pub("e2", &gear2_lec_msg);
//ros::Publisher gearv_lec_pub("ev", &gearv_lec_msg);

void setup() {

  nh.initNode();

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);

  pinMode(15, OUTPUT);
  pinMode(25, INPUT);
  pinMode(26, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(32, OUTPUT);

  //servo_11.attach(11);
  //servo_12.attach(12);
  servo_13.attach(13);
  servo_14.attach(14);
  //servo_15.attach(15);

  //nh.subscribe(s11_out_sub);
  //nh.subscribe(s12_out_sub);
  nh.subscribe(s13_out_sub);
  nh.subscribe(s14_out_sub);
  //nh.subscribe(s15_out_sub);
  nh.subscribe(alive_sub);

  nh.advertise(gear1_lec_pub);
  nh.advertise(gear2_lec_pub);
  //nh.advertise(gearv_lec_pub);

  delay(50);

  AS5043obj.begin();

  nh.advertise(imu_pub1);
  nh.advertise(imu_pub2);
  
  imu1.setup(); 
  imu2.setup(); 

  millisTock = millis();
}

void commloop() {

  if (millisTock <= millis()) {

    millisTock += 50;

    if (millis() - milisLastMsg >= 3000) {

      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);

      //s11_out = 0;  // v1
      //s12_out = 0;  // v2
      s13_out = 0;  // m1
      s14_out = 0;  // m2
      //s15_out = 0;  // mv

      timedOut = true;
    }

    gear1_lec = gear1.read();
    gear2_lec = gear2.read();
    //
    if (gear1.isValid() != 1) {
      gear1.restartComm();
      gear1_lec = gear1.read();
    }
    //
    if (gear2.isValid() != 1) {
      gear2.restartComm();
      gear2_lec = gear2.read();
    }
    //gearv_lec = gearv.read();

    gear1_lec_msg.data = gear1_lec;
    gear2_lec_msg.data = gear2_lec;
    //gearv_lec_msg.data = gearv_lec;

    gear1_lec_pub.publish(&gear1_lec_msg);
    gear2_lec_pub.publish(&gear2_lec_msg);
    //gearv_lec_pub.publish(&gearv_lec_msg);

    //servo_11.writeMicroseconds(100 * s11_out + 1000); // v1
    //servo_12.writeMicroseconds(100 * s12_out + 1000); // v2
    servo_13.writeMicroseconds( s13_out / 2 + 1500); // m1
    servo_14.writeMicroseconds( s14_out / 2 + 1500); // m2
    //servo_15.writeMicroseconds( 50 * s15_out + 1500); // mv

    imu_data1.data[0] = TO_DEG(imu1.roll);
    imu_data1.data[1] = TO_DEG(imu1.pitch);
    imu_data1.data[2] = TO_DEG(imu1.yaw);
    /*imu_data1.data[3] = imu1.accel[0];
    imu_data1.data[4] = imu1.accel[1];
    imu_data1.data[5] = imu1.accel[2];
    imu_data1.data[6] = imu1.magnetom[0];
    imu_data1.data[7] = imu1.magnetom[1]; 
    imu_data1.data[8] = imu1.magnetom[2];
    imu_data1.data[9] = imu1.gyro[0];
    imu_data1.data[10] = imu1.gyro[1];
    imu_data1.data[11] = imu1.gyro[2];*/

    imu_pub1.publish(&imu_data1);

    imu_data2.data[0] = TO_DEG(imu2.roll);
    imu_data2.data[1] = TO_DEG(imu2.pitch);
    imu_data2.data[2] = TO_DEG(imu2.yaw);
    /*imu_data2.data[3] = imu2.accel[0];
    imu_data2.data[4] = imu2.accel[1];
    imu_data2.data[5] = imu2.accel[2];
    imu_data2.data[6] = imu2.magnetom[0];
    imu_data2.data[7] = imu2.magnetom[1]; 
    imu_data2.data[8] = imu2.magnetom[2];
    imu_data2.data[9] = imu2.gyro[0];
    imu_data2.data[10] = imu2.gyro[1];
    imu_data2.data[11] = imu2.gyro[2];*/

    imu_pub2.publish(&imu_data2);
  }
}

void loop() {

  nh.spinOnce();
  imu1.loop(); delayMicroseconds(222);
  imu2.loop(); delayMicroseconds(222);
  commloop();

}

