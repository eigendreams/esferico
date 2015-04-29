/*
 * adxl345.cpp
 *
 *  Created on: 03/11/2014
 *      Author: gabriel
 */

#include "ADXL345.h"
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <math.h>



using namespace std;

#define DEVID 0x53
#define THRESH_TAP 0x1D
#define OFSX 0x1E
#define OFXY 0x1F
#define OFSZ 0x20
#define DUR 0x21
#define LATENT 0x22
#define WINDOW 0x23
#define THRESH_ACT 0x24
#define THRESH_INACT 0x25
#define TIME_INACT 0x26
#define ACT_INACT_CTL 0x27
#define THRESH_FF 0x28
#define TIME_FF 0x29
#define TAP_AXES 0x2A
#define ACT_TAP_STATUS 0x2B
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_MAP 0x2F
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37
#define FIFO_CTL 0x38
#define FIFO_STATUS 0x39

//unsigned short AcelX;
//unsigned short AcelY;
//unsigned short AcelZ;
//static int i2c_file;

static char buf[10] = {0};
char range = 8;
char offset_x = 0;
char offset_y = 0;
char offset_z = 0;

//Put this in .h


Adxl345::Adxl345(int module){
	
	this->I2CBus = module;
	this->I2CAddress = DEVID;
	this->X_g=0;
	this->Y_g=0;
	this->Z_g=0;
	this->AcelX=0;
	this->AcelY=0;
	this->AcelZ=0;
	this->measure_mode();
	this->readFullAcel();
}

int Adxl345::accelerometer_init(){
	
	////////////////////////////////////
	char i2c_filename[40];
	sprintf(i2c_filename,"/dev/i2c-%d", this->I2CBus);

	//Open the I2C bus
	if ((this->i2c_file = open(i2c_filename,O_RDWR)) < 0) {
		//printf("Failed to open the bus.");
		return 0;
	}

	int addr = DEVID;        // The I2C address
	//Talk to a particular chip
	if (ioctl(this->i2c_file,I2C_SLAVE,addr) < 0) {
		//printf("Failed to acquire bus access and/or talk to slave.\n");
		//printf("%s\n\n",strerror(errno));
		return 0;
	}
	return 1;
}

//Write only an address  Escribe para solicitar lectura a partir de reg
int Adxl345::write_address(unsigned char reg){
	buf[0] = reg;
	if (write(this->i2c_file,buf,1) != 1) {
		//printf("Failed to write to the i2c bus.\n");
		//printf("%s\n\n",strerror(errno));
		return 0;
	}
	return 1;
}

//Write a byte to an address
int Adxl345::write_byte(unsigned char reg, unsigned char data){
	buf[0] = reg;
	buf[1] = data;

	if (write(this->i2c_file,buf,2) != 2) {
		//printf("Failed to write to the i2c bus.\n");
		//printf("%s\n\n",strerror(errno));
		return 0;
	}

	return 1;
}

//Read the current register at an address, then change only the masked bytes based on data
int Adxl345::write_masked_byte(unsigned char reg, unsigned char data, char mask){
	unsigned char current_data;
	//Write desired register
	if(this->write_address(reg) == 0)
		return 0;
	//Read current value of register
	if(this->read_current_byte(&current_data) == 0)
		return 0;
	//printf("Current data: %x\n",current_data);
	//Write masked data
	data = (current_data & ~mask) | (data & mask);
	//printf("Writing data: %x\n",data);
	return this->write_byte(reg, data);
}

//Read a byte from the current address
int Adxl345::read_current_byte(unsigned char * data){
	
	if (read(i2c_file,buf,1) != 1) {
		//printf("Failed to read from the i2c bus.\n");
		//printf("%s\n\n",strerror(errno));
		return 0;
	}

	*data = buf[0];
	return 1;
}

//Read a byte from the passed register
int Adxl345::read_byte(unsigned char reg, unsigned char * data){
	//Write the register's address
	if(this->write_address(reg) == 0)
		return 0;

	//Read from that address
	return this->read_current_byte(data);
}

//Go from standby to measurement mode
int Adxl345::measure_mode(){
	if(this->accelerometer_init()!=1){
		return 0;	
	}
	//printf("Go to measure mode... ");
	if(this->write_masked_byte(POWER_CTL,0x08,0x08) == 0)
		return 0;
	//printf("Set to measure mode okay.\n");
	close(this->i2c_file);
	return 1;
}

//Go to standby mode
int Adxl345::standby_mode(){
	if(this->accelerometer_init()!=1){
		return 0;	
	}
	//printf("Go to standby mode... ");
        if(this->write_masked_byte(POWER_CTL,0x00,0x08) == 0)
                return 0;
        //printf("Set to standby mode okay.\n");
        close(this->i2c_file);
    return 1;
}

//Pass 1 to set power mode, 0 to turn off
//0 by default
int Adxl345::set_low_power(unsigned char power){
	if(this->accelerometer_init()!=1){
		return 0;	
	}
	int ret=this->write_masked_byte(BW_RATE, power<<3,0x10);
	close(this->i2c_file);
	return ret;
}

//Pass a value to set sensed range
//Potential values are 2,4,8,16g
int Adxl345::set_range(char range_set){
	if(this->accelerometer_init()!=1){
		return 0;	
	}
	
	unsigned char rate = 0xF;
	switch(range_set){
		case 2: rate = 0x0; break;
		case 4: rate = 0x1; break;
		case 8: rate = 0x2; break;
		case 16: rate = 0x3; break;
		default: printf("Not a valid range.\n"); return 0;
	}
	if(this->write_masked_byte(DATA_FORMAT,rate,0x3) == 0)
		return 0;
	range = range_set;
	close(this->i2c_file);
	return 1;
}

int Adxl345::get_range(){
	if(this->accelerometer_init()!=1){
		return 0;	
	}
	unsigned char data;
	if(this->read_byte(DATA_FORMAT,&data) == 0)
		return 0;
	//Mask off non-range bits
	data = data & 0x3;
        switch(data){
                case 0x0: range = 2; break;
                case 0x1: range = 4; break;
                case 0x2: range = 8; break;
                case 0x3: range = 16; break;
                default: printf("Not a valid range.\n"); return 0;
        }
    close(this->i2c_file);
	return 1;
}


float Adxl345::convert_to_g(unsigned short raw){
	char negative = 0;
	float result;
	//Convert from twos complement
	if((raw >> 15) == 1){
                raw = ~raw + 1;
                negative = 1;
        }
        result = (float)raw;
        if(negative)
                result *= -1;

	//1FF is the maximum value of a 10-bit signed register
	result = (float)range * (result/(0x1FF));
	return result;
}

int Adxl345::readFullAcel(){
    if(this->accelerometer_init()!=1){
		return 0;	
	}
	
	char buf2[1] = { DATAX0};
    unsigned char data[6];
    if(write(this->i2c_file, buf2, 1) !=1){
           std::cout << "Failed to Reset Address in readFullAcel() "<<std::endl;
           return 0;
    }
    int numberBytes = 0x06;
    int bytesRead = read(this->i2c_file, data, numberBytes);
    if (bytesRead == -1){   
          std::cout << "Failure to read Byte Stream in readFullSensorState()"<<endl;
          return 0;
    }
	
	this->AcelX=(float)(short)((((short) data[3]) << 8) | ((short) data[2]));
    this->AcelY=(float)(short)((((short) data[1]) << 8) | ((short) data[0]));
    this->AcelZ=(float)(short)((((short) data[5]) << 8) | ((short) data[4]));
	
	this->X_g=convert_to_g(this->AcelX);
	this->Y_g=convert_to_g(this->AcelY);
	this->Z_g=convert_to_g(this->AcelZ);
	
	close(this->i2c_file);
    return 1;

}

Adxl345::~Adxl345() {
}
/*
int get_data_x(float * result){
	unsigned char data;
	unsigned short raw;
	//char negative = 0;
	//read data0 from X-Axis
	if(read_byte(DATAX0, &data) == 0)
		return 0;
	//printf("X-Axis Data0:  %02d\n",data);
	raw = data;

	//read data1 from X-Axis
	if(read_byte(DATAX1, &data) == 0)
		return 0;

	//printf("X-Axis Data1:  %02d -- %02x\n",data, data);
	raw += data<<8;
        AcelX=raw;
        *result = #include "padre.h"convert_to_g(raw);
	return 1;
}

int get_data_y(float * result){
        unsigned char data;
        unsigned short raw;

        //read data0 from Y-Axis
        if(read_byte(DATAY0, &data) == 0)
                return 0;
        //printf("X-Axis Data0:  %02d\n",data);
        raw = data;

        //read data1 from Y-Axis
        if(read_byte(DATAY1, &data) == 0)
                return 0;

        //printf("Y-Axis Data1:  %02d -- %02x\n",data, data);
	raw += data<<8;
        AcelY=raw;
	*result = convert_to_g(raw);

        return 1;
}

int get_data_z(float * result){
        unsigned char data;
        unsigned short raw;

        //read data0 from Z-Axis
        if(read_byte(DATAZ0, &data) == 0)
                return 0;
        //printf("Z-Axis Data0:  %02d\n",data);
        raw = data;

        //read data1 from Z-Axis
        if(read_byte(DATAZ1, &data) == 0)
                return 0;

        //printf("Z-Axis Data1:  %02d -- %02x\n",data, data);
	raw += data << 8;
	AcelZ=raw;
        *result = convert_to_g(raw);

        return 1;
}
*/



