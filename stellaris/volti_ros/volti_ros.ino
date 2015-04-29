#include "rosSerial5.h"
//#include "ros.h"

#include "std_msgs/Int16.h"
#include "volti/float32_12.h"
#include "volti/float32_3.h"
#include "Servo.h"
#include "AS5043.h"
#include "Encoder.h"

ros::NodeHandle nh;

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
unsigned long millisTock = 0;
bool timedOut = false;

unsigned long timescont = 0;

// vol1 vol2 mot1 mot2 motv
//Servo servo_11;
//Servo servo_12;
Servo servo_13;
Servo servo_14;
//Servo servo_15;

// CLK, DO, PROG
AS5043Class AS5043obj(23, 25, 26);
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
    timescont++;
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
  
  //servo_11.attach(11);
  //servo_12.attach(12);
  servo_13.attach(13);
  servo_14.attach(14);
  //servo_15.attach(15);
  
  servo_13.writeMicroseconds( 1500 ); // m1
  servo_14.writeMicroseconds( 1500 ); // m2

  timedOut = true;

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

  millisTock = millis();
}

void commloop() {

  if (millisTock <= millis()) {
    
    millisTock += 50;

    if (millis() - milisLastMsg > 2000) {

      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);

      //s11_out = 0;  // v1
      //s12_out = 0;  // v2
      s13_out = 0;  // m1
      s14_out = 0;  // m2
      //s15_out = 0;  // mv

      timedOut = true;
      
      timescont == 0;
    }

    gear1_lec = gear1.read();
    gear2_lec = gear2.read();
    
    if (gear1.isValid() != 1) {
      gear1.restartComm();
      gear1_lec = gear1.read();
      if (gear1.isValid() != 1) {
        gear1_lec = -1; // raise error code
      }
    }
    //
    if (gear2.isValid() != 1) {
      gear2.restartComm();
      gear2_lec = gear2.read();
      if (gear2.isValid() != 1) {
        gear2_lec = -1; // raise error code
      }
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
    //////servo_13.writeMicroseconds( s13_out / 20 + 1500); // m1
    //////servo_14.writeMicroseconds( s14_out / 20 + 1500); // m2
    //servo_15.writeMicroseconds( 50 * s15_out + 1500); // mv

    if (timedOut) {
      servo_13.writeMicroseconds( 1500 ); // m1
      servo_14.writeMicroseconds( 1500 ); // m2
    }
    else {
      servo_13.writeMicroseconds( s13_out / 20 + 1500); // m1 // 100 points would be a 1 percent, 5 of 500 us, min of 0.2 percent
      servo_14.writeMicroseconds( s14_out / 20 + 1500); // m2 // 100 points would be a 1 percent, 5 of 500 us, min of 0.2 percent
    }
  }
}

void loop() {

  nh.spinOnce();
  commloop();

}

