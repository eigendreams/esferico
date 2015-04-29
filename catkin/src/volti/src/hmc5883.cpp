/*
 * adxl345.cpp
 *
 *  Created on: 03/11/2014
 *      Author: gabriel
 */
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
#include <cmath>
using namespace std;
#include "hmc5883.h"

#define DEVID 0x1e

#define CREG_A  0x00
#define CREG_B  0x01
#define MODE_REG  0x02
#define DOX_MSB  0x03
#define DOX_LSB   0x04
#define DOY_MSB  0x05
#define DOY_LSB   0x06
#define DOZ_MSB  0x07
#define DOZ	_LSB   0x08
#define STATUS_REG  0x09
#define IDE_REG_A 0x0A
#define IDE_REG_B   0x0B
#define IDE_REG_C  0x0C



//char range = 8;
//char offset_x = 0;
//char offset_y = 0;
//char offset_z = 0;
MAG::MAG(int module) {
	this->I2CBus = module;
	this->I2CAddress = DEVID;
	this->MagX=0;
	this->MagY=0;
	this->MagZ=0;
	this->writeReg_Data(MODE_REG,0x00);
	this->writeReg_Data(CREG_A,0b00011000);
}

MAG::~MAG() {
}
int MAG::hmc5883_init(){
	char i2c_filename[40];
	sprintf(i2c_filename,"/dev/i2c-%d", this->I2CBus);

	//Open the I2C bus
	if ((this->i2c_file = open(i2c_filename,O_RDWR)) < 0) {
		//printf("Failed to open the bus.");
		return 0;
	}

	//Talk to a particular chip
	if (ioctl(this->i2c_file,I2C_SLAVE,this->I2CAddress) < 0) {
		//printf("Failed to acquire bus access and/or talk to slave.\n");
		//printf("%s\n\n",strerror(errno));
		return 0;
	}
	//get_range();
	//read_calibration();

	return 1;

}

short int MAG::writeReg_Data( short int reg,short int data)
{ /*
  i2c_smbus_write_byte_data(file,WHO_AM_I_REG,data);

  return data;*/
	this->hmc5883_init();
		char buf[2];
		buf[0] = reg;
		buf[1] = data;

		if (write(this->i2c_file,buf,2) != 2) {
			//printf("Failed to write to the i2c bus.\n");
			//printf("%s\n\n",strerror(errno));
			return 0;
		}
		close(this->i2c_file);
		return 1;

}

int MAG::readfullState(){
	this->hmc5883_init();
	char buf2[1] = { DOX_MSB};
    char data[6];
    if(write(this->i2c_file, buf2, 1) !=1){
           std::cout << "Failed to Reset Address in readFullSensorState() "<<std::endl;
           return 0;
    }
    int numberBytes = 0x06;
    int bytesRead = read(this->i2c_file, data, numberBytes);
    if (bytesRead == -1){   
          std::cout << "Failure to read Byte Stream in readFullSensorState()"<<endl;
          return 0;
    }
	
	this->MagX=(float)(short)((((short) data[0]) << 8) | ((short) data[1]));
    this->MagY=-1 * (float)(short)(((((short) data[4]) << 8) | ((short) data[5])));
    this->MagZ= -1 * (float)(short)(((((short) data[2]) << 8) | ((short) data[3])));
	
    close(this->i2c_file);
    return 1;
}





