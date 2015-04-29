#include "Servo.h"
#include "AS5043.h"
#include "Encoder.h"

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
unsigned long millisTock = 0;
unsigned long timescont = 0;
bool timedOut = false;

Servo servo_13;
Servo servo_14;

volatile int s13_out = 0;  // m1
volatile int s14_out = 0;  // m2

void setup() {
  
  Serial.begin(115200);
  Serial5.begin(115200);
 
  servo_13.attach(13);
  servo_14.attach(14);
  
  servo_13.writeMicroseconds( 1500 ); // m1
  servo_14.writeMicroseconds( 1500 ); // m2

  timedOut = true;

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);

  delay(50);

  millisTock = millis();
  
}

void alive() {
  
    // show status
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    // record msg time
    milisLastMsg = millis();
    timedOut = false;
    timescont++;
} 

uint16_t word1 = 0;
int debug = 0;
int lec1 = 0;
int lec2 = 0;

void commloop() {
  
  if (Serial.available()) {
   if (Serial.read() == 'r') {
    debug = 1;
   } 
   else {
    debug = 0; 
   }
  }
  
  // PROT FF AL HL HL CHKSUM = 10 bytes
  if (Serial5.available() >= 10){
    
    // 0x7530 = 30000
    lec1 = Serial5.read();
    word1 = (word1 << 8) | lec1;
    if (word1 != 0x7530) {
      while(Serial5.available()) {
        lec1 = Serial5.read();
        word1 = (word1 << 8) | lec1;
        if (word1 == 0x7530) {
          if (Serial5.available() < 8) {
           delay(10);
          }
          break;
        }
      }
    }
    
    if (Serial5.available() < 8) {
      return; 
    }
    
    // read alive status
    lec1 = Serial5.read();
    lec2 = Serial5.read();
    uint16_t aldata = (lec1 << 8) | lec2;
    // read the servos data, in microseconds
    lec1 = Serial5.read();
    lec2 = Serial5.read();
    int16_t s1data = (lec1 << 8) | lec2;
    lec1 = Serial5.read();
    lec2 = Serial5.read();
    int16_t s2data = (lec1 << 8) | lec2;
    // verify the checksum
    lec1 = Serial5.read();
    lec2 = Serial5.read();
    uint16_t checksumH = lec1;
    uint16_t checksumL = lec2;
    //
    uint16_t localchecksumH = (((long)aldata + (long)s1data + (long)s2data) >> 8) & 255;
    uint16_t localchecksumL = (((long)aldata + (long)s1data + (long)s2data) >> 0) & 255;
    //
    uint16_t checksum = (checksumH << 8) | checksumL;
    uint16_t localchecksum = (localchecksumH << 8) | localchecksumL;
    
    if (debug) {
     Serial.print(" w "); 
     Serial.print((word1));
     Serial.print(" a "); 
     Serial.print((aldata));
     Serial.print(" s1 "); 
     Serial.print((s1data));
     Serial.print(" s2 "); 
     Serial.print((s2data));
     Serial.print(" c "); 
     Serial.print(checksum);;
     Serial.print(" l "); 
     Serial.print(localchecksum);
     Serial.print(" b "); 
     Serial.print(Serial5.available());
     Serial.println(""); 
    }
    
    // TODO add more aldata rules
    if (checksum == localchecksum) {
      if (aldata == 1) {
        s13_out = s1data;
        s14_out = s2data;
        alive();
      }
    }
  }
}

void loop() {
  
  commloop();
  
  if (millisTock <= millis()) {
    
    millisTock += 50;
    
    if (millis() - milisLastMsg > 1500) {
      // shut down all
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      s13_out = 0;  // m1
      s14_out = 0;  // m2
      timedOut = true;
      timescont = 0;
    }
    
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
