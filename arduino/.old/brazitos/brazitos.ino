#include "ros.h"
#include "finder/FourArmsInt16.h"

#include "Servo.h"
#include "Talon.h"
#include "Encoder.h"
#include "SimplePID.h"

ros::NodeHandle nh;

// umbral, max
TalonClass MTFR(2, 50); // NEW MOTOR
TalonClass MTFL(2, 35);
TalonClass MTBR(2, 35);
TalonClass MTBL(2, 35);

// pin, min, max, max_change, map
EncoderClass ENCFR(A0, 0, 1023, 255, 100);
EncoderClass ENCFL(A1, 0, 1023, 255, 100);
EncoderClass ENCBR(A2, 0, 1023, 255, 100);
EncoderClass ENCBL(A3, 0, 1023, 255, 100);

// P, I, D, Km, umbral, max
SimplePID PIDFR(4, .5, 0, 0, 5, 50);
SimplePID PIDFL(2, .5, 0, 0, 5, 35);
SimplePID PIDBR(2, .5, 0, 0, 5, 35);
SimplePID PIDBL(2, .5, 0, 0, 5, 35);

// Encoder lectures
int mtfr_lec;
int mtfl_lec;
int mtbr_lec;
int mtbl_lec;

// The incoming 6 DOF int16 information from ROS
int mtfr_des = 0;
int mtfl_des = 0;
int mtbr_des = 0;
int mtbl_des = 0;

// All the subs handlers functions, update is asynchronous (done in the loop), keep names short
void arms_des_cb(const finder::FourArmsInt16& d_msg) {
  mtfr_des = d_msg.mtfr;  
  mtfl_des = d_msg.mtfl;  
  mtbr_des = d_msg.mtbr;  
  mtbl_des = d_msg.mtbl;  
}
ros::Subscriber<finder::FourArmsInt16> arms_des_sub("traction_arms", arms_des_cb);

// All the subs handlers functions, update is asynchronous (done in the loop), keep names short
void pid_des_cb(const finder::FourArmsInt16& d_msg) {
  float pid_p = (d_msg.mtfr / 1000.);
  float pid_i = (d_msg.mtfl / 1000.);
  float pid_d = (d_msg.mtbr / 1000.);
  float pid_m = (d_msg.mtbl / 1000.);

  PIDFR.setTunings(pid_p, pid_i, pid_d, pid_m);
  PIDFL.setTunings(pid_p, pid_i, pid_d, pid_m);
  PIDBR.setTunings(pid_p, pid_i, pid_d, pid_m);
  PIDBL.setTunings(pid_p, pid_i, pid_d, pid_m);
}
ros::Subscriber<finder::FourArmsInt16> pid_des_sub("pid_des", pid_des_cb);

// All the pubs msgs, keep names short
finder::FourArmsInt16 arms_lec;
ros::Publisher arms_lec_pub("traction_arms_lec", &arms_lec);

finder::FourArmsInt16 arms_out;
ros::Publisher arms_out_pub("traction_arms_out", &arms_out);

unsigned long milisLast = 0;

void setup() {
  MTFR.attach(3);
  MTFL.attach(5);
  MTBR.attach(6);
  MTBL.attach(9);

  nh.initNode();

  nh.subscribe(pid_des_sub);
  nh.subscribe(arms_des_sub);
  nh.advertise(arms_lec_pub);
  nh.advertise(arms_out_pub);
}

int select(int value) {
  if (abs(value) > 251) return 0;
  if (value > 503) return value - 1006;
  if (value < -503) return value + 1006;
  return value;
}

void loop() {
  nh.spinOnce();
  unsigned long milisNow = millis();

  if (milisNow - milisLast >= 100) {

    // Obten los valores absolutos de los encoders
    int mtfr_lecNow = ENCFR.read();
    int mtfl_lecNow = ENCFL.read();
    int mtbr_lecNow = ENCBR.read();
    int mtbl_lecNow = ENCBL.read();

    // Calcula el cambio obtenido de los encoders? 
    int change_fr = mtfr_lecNow - mtfr_lec;
    int change_fl = mtfl_lecNow - mtfl_lec;
    int change_br = mtbr_lecNow - mtbr_lec;
    int change_bl = mtbl_lecNow - mtbl_lec;

    // Actualiza variables
    mtfr_lec = mtfr_lecNow;
    mtfl_lec = mtfl_lecNow;
    mtbr_lec = mtbr_lecNow;
    mtbl_lec = mtbl_lecNow;

    // Pero si el cambio se sale de rango de 0 a 1006? Ajustando modulo 1006
    change_fr = select(change_fr);
    change_fl = select(change_fl);
    change_br = select(change_br);
    change_bl = select(change_bl)*-1;

    // Y este cambio debe mapearse a algo de -100 a 100 (el signo se conserva)
    change_fr = (change_fr * 200. / 1006.);
    change_fl = (change_fl * 200. / 1006.);
    change_br = (change_br * 200. / 1006.);
    change_bl = (change_bl * 200. / 1006.);

    // Y publicar los changes
    arms_lec.mtfr = change_fr;
    arms_lec.mtfl = change_fl;
    arms_lec.mtbr = change_br;
    arms_lec.mtbl = change_bl;
    arms_lec_pub.publish(&arms_lec);

    // Y ese cambio es el valor de entrada del PIDm
    PIDFR.compute(change_fr, mtfr_des);
    PIDFL.compute(change_fl, mtfl_des);
    PIDBR.compute(change_br, mtbr_des);
    PIDBL.compute(change_bl, mtbl_des);

    // Y darle salida a cada motor en cada DOF
    MTFR.write(-PIDFR.get()); //CHANGED!!!
    MTFL.write(PIDFL.get());
    MTBR.write(PIDBR.get());
    MTBL.write(PIDBL.get());
    
    arms_out.mtfr = PIDFR.get();
    arms_out.mtfl = PIDFL.get();
    arms_out.mtbr = PIDBR.get();
    arms_out.mtbl = PIDBL.get();
    arms_out_pub.publish(&arms_out);

    milisLast = milisNow;
  }
}
