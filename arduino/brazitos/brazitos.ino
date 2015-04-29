#include "ros.h"
#include "std_msgs/Int16.h"
#include "Servo.h"
#include "Talon.h"
#include "Encoder.h"

////////////////////////////////////////////////////////////////////////////////
Servo brazo_servo;
Servo antebrazo_servo;
int arm_out = 0;
int forearm_out = 0;
void arm_out_cb(const std_msgs::Int16& dmsg) {arm_out = dmsg.data;}
void forearm_out_cb(const std_msgs::Int16& dmsg) {forearm_out = dmsg.data;}
ros::Subscriber<std_msgs::Int16> arm_out_sub("arm_out", arm_out_cb);
ros::Subscriber<std_msgs::Int16> forearm_out_sub("forearm_out", forearm_out_cb);
////////////////////////////////////////////////////////////////////////////////
/*#include "i2cmaster.h"
#include "MLX90620_registers.h"
#include "finder/int16_64.h"
#include "finder/uint8_64.h"
//int ir_req;
//void ir_req_cb( const std_msgs::Int16& dmsg) {  ir_req = dmsg.data; }
//ros::Subscriber<std_msgs::Int16> ir_req_sub("ir_req", ir_req_cb);
finder::uint8_64 ir_data;
ros::Publisher ir_pub("ir_data", &ir_data);
int refreshRate = 16; //Set this value to your desired refresh frequency
int conta = 0;
byte loopCount = 0; //Used in main loop
void setConfiguration(int irRefreshRateHZ)
{
  byte Hz_LSB;
  switch(irRefreshRateHZ)
  {
  case 0:
    Hz_LSB = 0b00001111;
    break;
  case 1:
    Hz_LSB = 0b00001110;
    break;
  case 2:
    Hz_LSB = 0b00001101;
    break;
  case 4:
    Hz_LSB = 0b00001100;
    break;
  case 8:
    Hz_LSB = 0b00001011;
    break;
  case 16:
    Hz_LSB = 0b00001010;
    break;
  case 32:
    Hz_LSB = 0b00001001;
    break;
  default:
    Hz_LSB = 0b00001110;
  }
  byte defaultConfig_H = 0b01110100; // x111.01xx, Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz

  i2c_start_wait(MLX90620_WRITE);
  i2c_write(0x03); //Command = configuration value
  i2c_write((byte)Hz_LSB - 0x55);
  i2c_write(Hz_LSB);
  i2c_write(defaultConfig_H - 0x55); //Assumes NA = 0, ADC low reference enabled, Ta Refresh rate of 2Hz
  i2c_write(defaultConfig_H);
  i2c_stop();
}
void writeTrimmingValue(byte val)
{
  i2c_start_wait(MLX90620_WRITE); //Write to the sensor
  i2c_write(0x04); //Command = write oscillator trimming value
  i2c_write((byte)val - 0xAA);
  i2c_write(val);
  i2c_write(0x56); //Always 0x56
  i2c_write(0x00); //Always 0x00
  i2c_stop();
}
unsigned int readPTAT_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read PTAT
  i2c_write(0x90); //Start address is 0x90
  i2c_write(0x00); //Address step is 0
  i2c_write(0x01); //Number of reads is 1
  i2c_rep_start(MLX90620_READ);

  byte ptatLow = i2c_readAck(); //Grab the lower and higher PTAT bytes
  byte ptatHigh = i2c_readAck();

  i2c_stop();
  
  return( (unsigned int)(ptatHigh << 8) | ptatLow); //Combine bytes and return
}
void readIR_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read a register
  i2c_write(0x00); //Start address = 0x00
  i2c_write(0x01); //Address step = 1
  i2c_write(0x40); //Number of reads is 64
  i2c_rep_start(MLX90620_READ);

  for(int i = 0 ; i < 64 ; i++)
  {
    byte pixelDataLow = i2c_readAck();
    byte pixelDataHigh = i2c_readAck();
    ir_data.data[i] = (byte)(constrain(((int)(pixelDataHigh << 8) | pixelDataLow) + 50, 0, 255));
    //ir_data.data[i] = (int)(pixelDataHigh << 8) | pixelDataLow;
  }

  i2c_stop();
}
int readCPIX_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE);
  i2c_write(CMD_READ_REGISTER); //Command = read register
  i2c_write(0x91);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(MLX90620_READ);

  byte cpixLow = i2c_readAck(); //Grab the two bytes
  byte cpixHigh = i2c_readAck();
  i2c_stop();

  return ( (int)(cpixHigh << 8) | cpixLow);
}
unsigned int readConfig_MLX90620()
{
  i2c_start_wait(MLX90620_WRITE); //The MLX configuration is in the MLX, not EEPROM
  i2c_write(CMD_READ_REGISTER); //Command = read configuration register
  i2c_write(0x92); //Start address
  i2c_write(0x00); //Address step of zero
  i2c_write(0x01); //Number of reads is 1

    i2c_rep_start(MLX90620_READ);

  byte configLow = i2c_readAck(); //Grab the two bytes
  byte configHigh = i2c_readAck();

  i2c_stop();

  return( (unsigned int)(configHigh << 8) | configLow); //Combine the configuration bytes and return as one unsigned int
}
boolean checkConfig_MLX90620()
{
  if ( (readConfig_MLX90620() & (unsigned int)1<<POR_TEST) == 0)
    return true;
  else
    return false;
}*/
////////////////////////////////////////////////////////////////////////////////

ros::NodeHandle nh;

// umbral, max
TalonClass MTFR(2, 35);
TalonClass MTFL(2, 35);
TalonClass MTBR(2, 50);
TalonClass MTBL(2, 50);

// pin, min, max, max_change, map
EncoderClass ENCFR(A0, 0, 1023, 255, 100);
EncoderClass ENCFL(A1, 0, 1023, 255, 100);
EncoderClass ENCBR(A2, 0, 1023, 255, 100);
EncoderClass ENCBL(A3, 0, 1023, 255, 100);

// The incoming 6 DOF int16 information from ROS
int fr_out = 0;
int fl_out = 0;
int br_out = 0;
int bl_out = 0;

// Encoder lectures
int fr_lec;
int fl_lec;
int br_lec;
int bl_lec;

unsigned long milisLast = 0;
unsigned long milisLastMsg = 0;
bool timedOut = false;

void fr_out_cb(const std_msgs::Int16& dmsg) {fr_out = dmsg.data;}
void fl_out_cb(const std_msgs::Int16& dmsg) {fl_out = dmsg.data;}
void br_out_cb(const std_msgs::Int16& dmsg) {br_out = dmsg.data;}
void bl_out_cb(const std_msgs::Int16& dmsg) {bl_out = dmsg.data;}
void alive_cb( const std_msgs::Int16& dmsg) {
  if (dmsg.data){
    milisLastMsg = millis();
    timedOut = false;
  }
}
ros::Subscriber<std_msgs::Int16> fr_out_sub("fr_out", fr_out_cb);
ros::Subscriber<std_msgs::Int16> fl_out_sub("fl_out", fl_out_cb);
ros::Subscriber<std_msgs::Int16> br_out_sub("br_out", br_out_cb);
ros::Subscriber<std_msgs::Int16> bl_out_sub("bl_out", bl_out_cb);
ros::Subscriber<std_msgs::Int16> alive_sub("alive", alive_cb);

std_msgs::Int16 fr_lec_msg;
std_msgs::Int16 fl_lec_msg;
std_msgs::Int16 br_lec_msg;
std_msgs::Int16 bl_lec_msg;
ros::Publisher fr_lec_pub("fr_lec", &fr_lec_msg);
ros::Publisher fl_lec_pub("fl_lec", &fl_lec_msg);
ros::Publisher br_lec_pub("br_lec", &br_lec_msg);
ros::Publisher bl_lec_pub("bl_lec", &bl_lec_msg);

void setup() {

  nh.initNode();

  //////////////////////////////////////////////////////////////////////////////
  brazo_servo.attach(7);
  antebrazo_servo.attach(8);
  nh.subscribe(arm_out_sub);
  nh.subscribe(forearm_out_sub);
  //////////////////////////////////////////////////////////////////////////////
  /*i2c_init(); //Init the I2C pins
  //PORTC = (1 << PORTC4) | (1 << PORTC5); //Enable pull-ups
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  delay(5); //Init procedure calls for a 5ms delay after power-on
  writeTrimmingValue(100);
  setConfiguration(refreshRate); //Configure the MLX sensor with the user's choice of refresh rate
  //calculate_TA(); //Calculate the current Tambient
  nh.advertise(ir_pub); delay(1);
  //nh.subscribe(ir_req_sub); delay(1);*/
  //////////////////////////////////////////////////////////////////////////////

  MTFR.attach(3);
  MTFL.attach(5);
  MTBR.attach(6);
  MTBL.attach(9);

  nh.subscribe(fr_out_sub); delay(1);
  nh.subscribe(fl_out_sub); delay(1);
  nh.subscribe(br_out_sub); delay(1);
  nh.subscribe(bl_out_sub); delay(1);
  nh.subscribe(alive_sub); delay(1);
  nh.advertise(fr_lec_pub); delay(1);
  nh.advertise(fl_lec_pub); delay(1);
  nh.advertise(br_lec_pub); delay(1);
  nh.advertise(bl_lec_pub); delay(1);
}

void loop() {

  nh.spinOnce();

  unsigned long milisNow = millis();

  if (milisNow - milisLast >= 100) {

    // Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
    // to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
    if (milisNow - milisLastMsg >= 3500) {
      //////////////////////////////////////////////////////////////////////////
      arm_out = 0;
      forearm_out = 0;
      //////////////////////////////////////////////////////////////////////////
      fr_out = 0;
      fl_out = 0;
      br_out = 0;
      bl_out = 0;
      timedOut = true;
    }

    // Obten los valores absolutos de los encoders
    fr_lec = ENCFR.read();
    fl_lec = ENCFL.read();
    br_lec = ENCBR.read();
    bl_lec = ENCBL.read();

    // Y darle salida a cada motor en cada DOF
    MTFR.write(fr_out);
    MTFL.write(fl_out);
    MTBR.write(br_out);
    MTBL.write(bl_out);
    
    fr_lec_msg.data = fr_lec;
    fl_lec_msg.data = fl_lec;
    br_lec_msg.data = br_lec;
    bl_lec_msg.data = bl_lec;

    fr_lec_pub.publish(&fr_lec_msg); delay(1);
    fl_lec_pub.publish(&fl_lec_msg); delay(1);
    br_lec_pub.publish(&br_lec_msg); delay(1);
    bl_lec_pub.publish(&bl_lec_msg); delay(1);

    milisLast = milisNow;
  
    //////////////////////////////////////////////////////////////////////////////
    brazo_servo.write(arm_out * 5 + 1500);
    antebrazo_servo.write(forearm_out * 5 + 1500);
    ////////////////////////////////////////////////////////////////////////////
    /*if(loopCount++ == 10) //Tambient changes more slowly than the pixel readings. Update TA only every 16 loops.
    { 
      //calculate_TA(); //Calculate the new Tambient
      if(checkConfig_MLX90620()) //Every 16 readings check that the POR flag is not set
      {
        setConfiguration(refreshRate); //Re-write the configuration bytes to the MLX
      }
      loopCount = 0; //Reset count
    }
    readIR_MLX90620(); //Get the 64 bytes of raw pixel data into the irData array
    //calculate_TO(); //Run all the large calculations to get the temperature data for each pixel
    conta++;
    if(conta >= 5)//ir_req)//conta >= 10)
    {
      //ir_req = 0;
      conta = 0;
      ir_pub.publish(&ir_data);
    }*/
    //rawPrintTemperatures(); //Print the entire array so it can more easily be read by Processing app
    //////////////////////////////////////////////////////////////////////////////
  }
}
