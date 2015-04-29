/*
 * gyro.cpp
 *
 *  Created on: 05/11/2014
 *      Author: gabriel
 */

/*
 * gyro.h
 *
 *  Created on: 05/11/2014
 *      Author: gabriel
 */
/*
 * ITG3200.hclose(this->i2c_file);
 *
 *  Created on: Sep 20, 2012
 *      Author: Ruffin White
 */

/**
 * Includes
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
#include "gyro.h"

/**
 * Defines
 */
#define ITG3200_I2C_ADDRESS 0x68 //7-bit address that Gyro is originally configured by the break-out board

//-----------
// Registers
//-----------
#define WHO_AM_I_REG    0x00
#define SMPLRT_DIV_REG  0x15
#define DLPF_FS_REG     0x16
#define INT_CFG_REG     0x17
#define INT_STATUS      0x1A
#define TEMP_OUT_H_REG  0x1B
#define TEMP_OUT_L_REG  0x1C
#define GYRO_XOUT_H_REG 0x1D
#define GYRO_XOUT_L_REG 0x1E
#define GYRO_YOUT_H_REG 0x1F
#define GYRO_YOUT_L_REG 0x20
#define GYRO_ZOUT_H_REG 0x21
#define GYRO_ZOUT_L_REG 0x22
#define PWR_MGM_REG     0x3E

//----------------------------
// Low Pass Filter Bandwidths
//----------------------------
#define LPFBW_256HZ 0x00
#define LPFBW_188HZ 0x01
#define LPFBW_98HZ  0x02
#define LPFBW_42HZ  0x03
#define LPFBW_20HZ  0x04
#define LPFBW_10HZ  0x05
#define LPFBW_5HZ   0x06

//-----------
// Offsets
//-----------
short int TEMP_OUT_OFFSET = 0;
short int GYRO_XOUT_OFFSET =0;
short int GYRO_YOUT_OFFSET = 0;
short int GYRO_ZOUT_OFFSET = 0;


Gyro::Gyro(int module) {
	this->I2CBus = module;
	this->I2CAddress = ITG3200_I2C_ADDRESS;
	this->GyroX=0;
	this->GyroY=0;
	this->GyroZ=0;
	this->writeFS_SEL(0x1B);
	this->readFullGyro();
}

Gyro::~Gyro() {
	close(this->i2c_file);
}

//This function is used to initialize the gyroscope. The function returns the -errno if an error accrues.
 int Gyro::initialize(){

	char filename[20];

	sprintf(filename, "/dev/i2c-%d", this->I2CBus);

	if ((this->i2c_file = open(filename,O_RDWR)) < 0) {
			//printf("Failed to open the bus.");
			return -errno;
		}
	/*file= open(filename, O_RDWR);

	if (file<0) {
		return -errno;
	}*/

	if (ioctl(this->i2c_file, I2C_SLAVE, this->I2CAddress) < 0) {
			return -errno;
	}
	return 1;

}

//This function is used to initialize the gyroscope. The function returns the -errno if an error accrues.
short int Gyro::zeroGyro(){

	if(this->initialize()!=1){
			return 0;
	}
	this->readFullGyro();
	GYRO_XOUT_OFFSET = this->getGyroX();
	GYRO_YOUT_OFFSET = this->getGyroY();
	short int Z=GYRO_ZOUT_OFFSET = this->getGyroZ();
	close(this->i2c_file);
	return Z;


}

//This function is used to read the WHO_AM_I_REG of the gyroscope.
//Usage: int gyroID = readWhoAmI();
short int Gyro::readWhoAmI()
{       this->initialize();


	    char buf[1] = { WHO_AM_I_REG };
	    char res[10];
	    if(write(this->i2c_file, buf, 1) !=1){
	    	std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
	    }

	    int numberBytes = 0x01;
	    int bytesRead = read(this->i2c_file, res, numberBytes);
	    if (bytesRead == -1){
	    	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
	    }

  short int data=0;
  data = res[0];
  close(this->i2c_file);
  return data;

  /*
  short int data=0;
  data = i2c_smbus_read_byte_data(file, WHO_AM_I_REG);

  return data;*/
}

//This function is used to write the WHO_AM_I_REG of the gyroscope.
//Usage: data = readWhoAmI(data);
short int Gyro::writeWhoAmI( short int data)
{ /*
  i2c_smbus_write_byte_data(file,WHO_AM_I_REG,data);

  return data;*/
	this->initialize();
		char buf[2];
		buf[0] = WHO_AM_I_REG;
		buf[1] = data;

		if (write(this->i2c_file,buf,2) != 2) {
			//printf("Failed to write to the i2c bus.\n");
			//printf("%s\n\n",strerror(errno));
			close(this->i2c_file);
			return 0;
		}
		close(this->i2c_file);
		return 1;

}


//This function is used to read the SMPLRT_DIV_REG of the gyroscope.
short int Gyro::readSmplrtDiv()
{/*
  short int data=0;
  data = i2c_smbus_read_byte_data(file, SMPLRT_DIV_REG);

  return data;*/
	this->initialize();

	char buf[1] = { SMPLRT_DIV_REG };
		    char res[10];
		    if(write(this->i2c_file, buf, 1) !=1){
		    	std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
		    }


		    int numberBytes = 0x01;
		    int bytesRead = read(this->i2c_file, res, numberBytes);
		    if (bytesRead == -1){
		    	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
		    }

	  short int data=0;
	  data = res[0];
	  close(this->i2c_file);
	  return data;
}

//This function is used to write the SMPLRT_DIV_REG of the gyroscope.
short int Gyro::writeSmplrtDiv( short int data)
{ /*
  i2c_smbus_write_byte_data(file,SMPLRT_DIV_REG,data);

  return data;*/
	return 1;
}

//This function is used to read the DLPF_FS_REG of the gyroscope.
short int Gyro::readDlpfFs()
{ /*
  short int data=0;
  data = i2c_smbus_read_byte_data(file, DLPF_FS_REG);

  return data;*/
	this->initialize();
	char buf[1] = { DLPF_FS_REG };
			    char res[10];
			    if(write(this->i2c_file, buf, 1) !=1){
			    	std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
			    }

			    int numberBytes = 0x01;
			    int bytesRead = read(this->i2c_file, res, numberBytes);
			    if (bytesRead == -1){
			    	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
			    }

		  short int data=0;
		  data = res[0];
		  close(this->i2c_file);
		  return data;

}

//This function is used to write the DLPF_FS_REG of the gyroscope.
short int Gyro::writeDlpfFs( short int data)
{ /*
  i2c_smbus_write_byte_data(file,DLPF_FS_REG,data);

  return data;*/
	return 1;
}

//This function is used to read the INT_CFG_REG of the gyroscope.
short int Gyro::readIntCfg()
{ /*
  short int data=0;
  data = i2c_smbus_read_byte_data(file, INT_CFG_REG);

  return data;*/
	this->initialize();
	char buf[1] = { INT_CFG_REG};
		    char res[10];
		    if(write(this->i2c_file, buf, 1) !=1){
		    	std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
		    }

		    int numberBytes = 0x01;
		    int bytesRead = read(this->i2c_file, res, numberBytes);
		    if (bytesRead == -1){
		    	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
		    }

	  short int data=0;
	  data = res[0];
	  close(this->i2c_file);
	  return data;
}

//This function is used to write the INT_CFG_REG of the gyroscope.
short int Gyro::writeIntCfg( short int data)
{
  //i2c_smbus_write_byte_data(file,INT_CFG_REG,data);

  //return data;

	return 1;
}

//This function is used to read the INT_STATUS of the gyroscope.
short int Gyro::readIntStatus()
{/*
  short int data=0;
  data = i2c_smbus_read_byte_data(file, INT_STATUS);

  return data;*/
	this->initialize();
	char buf[1] = { INT_STATUS };
	char res[10];
	if(write(this->i2c_file, buf, 1) !=1){
		std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
	}

    int numberBytes = 0x01;
	int bytesRead = read(this->i2c_file, res, numberBytes);
	if (bytesRead == -1){
	 	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
    }

	short int data=0;
	data = res[0];
	close(this->i2c_file);
	return data;

}


//This function is used to read the temperature of the gyroscope.
//Usage: int gyroTemp = readTemp();
short int Gyro::readTemp()
{ /*
  short int data=0;
  data = i2c_smbus_read_byte_data(file, TEMP_OUT_H_REG)<<8;
  data |= i2c_smbus_read_byte_data(file, TEMP_OUT_L_REG);

  return data;*/
	this->initialize();
	char buf[1] = { TEMP_OUT_H_REG };
		char res[10];
		if(write(this->i2c_file, buf, 1) !=1){
			std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
		}

	    int numberBytes = 0x02;
		int bytesRead = read(this->i2c_file, res, numberBytes);
		if (bytesRead == -1){
		 	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
	    }

		short int data=0;
		data = res[0]<<8;
		data |=res[1];
		//data=(data+13200)/280;
		close(this->i2c_file);
		return data;


}

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int xRate = readX();

int Gyro::readFullGyro() {

	this->initialize();
	char buf[1] = { GYRO_XOUT_H_REG};
	char res[6];
	if(write(this->i2c_file, buf, 1) !=1){
		std::cout << "Ojo Failed to Reset Address in readFullSensorState() " << std::endl;
	}

	int numberBytes = 0x06;
	int bytesRead = read(this->i2c_file, res, numberBytes);
	if (bytesRead == -1){
		std::cout << "Ojo 2 Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
    }
    /*
	short int data=0;
	data = res[0]<<8;
	data |=res[1];
    */

	this->GyroX=-1 * (float)(short)(((((short) res[2]) << 8) | ((short) res[3])));
	this->GyroY=-1 * (float)(short)(((((short) res[0]) << 8) | ((short) res[1])));
	this->GyroZ= -1 * (float)(short)(((((short) res[4]) << 8) | ((short) res[5])));
	close(this->i2c_file);


	return 1;
}

/*
short int Gyro::readX()
{
  //short int data=0;
  //data = i2c_smbus_read_byte_data(file, GYRO_XOUT_H_REG)<<8;
  //data |= i2c_smbus_read_byte_data(file, GYRO_XOUT_L_REG);

  //return data - GYRO_XOUT_OFFSET;
	this->initialize();
	char buf[1] = { GYRO_XOUT_H_REG};
			char res[10];
			if(write(this->i2c_file, buf, 1) !=1){
				std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
			}

		    int numberBytes = 0x02;
			int bytesRead = read(this->i2c_file, res, numberBytes);
			if (bytesRead == -1){
			 	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
		    }

			short int data=0;
			data = res[0]<<8;
			data |=res[1];

			close(this->i2c_file);
			return data - GYRO_XOUT_OFFSET;

}

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int yRate = readY();
short int Gyro::readY()
{
  //short int data=0;
  //data = i2c_smbus_read_byte_data(file, GYRO_YOUT_H_REG)<<8;
  //data |= i2c_smbus_read_byte_data(file, GYRO_YOUT_L_REG);

  //return data - GYRO_YOUT_OFFSET;
	this->initialize();
	char buf[1] = { GYRO_YOUT_H_REG};
			char res[10];
			if(write(this->i2c_file, buf, 1) !=1){
				std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
			}

		    int numberBytes = 0x02;
			int bytesRead = read(this->i2c_file, res, numberBytes);
			if (bytesRead == -1){
			 	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
		    }

			short int data=0;
			data = res[0]<<8;
			data |=res[1];

			close(this->i2c_file);
			return data - GYRO_YOUT_OFFSET;
}

//This fuwriteWhoAmInction is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int zRate = readZ();
short int Gyro::readZ()
{
  //short int data=0;
  //data = i2c_smbus_read_byte_data(file, GYRO_ZOUT_H_REG)<<8;
  //data |= i2c_smbus_read_byte_data(file, GYRO_ZOUT_L_REG);

  //return data - GYRO_ZOUT_OFFSET;
	this->initialize();
	char buf[1] = { GYRO_ZOUT_H_REG};
			char res[10];
			if(write(this->i2c_file, buf, 1) !=1){
				std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
			}

		    int numberBytes = 0x02;
			int bytesRead = read(this->i2c_file, res, numberBytes);
			if (bytesRead == -1){
			 	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
		    }

			short int data=0;
			data = res[0]<<8;
			data |=res[1];


			close(this->GyroX=-1 * (short)((((short) res[2]) << 8) | res[3]);this->i2c_file);
			return data - GYRO_ZOUT_OFFSET;
}
*/
//This function is used to read the PWR_MGM_REG of the gyroscope.
short int Gyro::readPwrMgm()
{ /*
  short int data=0;
  data = i2c_smbus_read_byte_data(file, PWR_MGM_REG);

  return data;*/
	this->initialize();
	char buf[1] = { PWR_MGM_REG };
		char res[10];
		if(write(this->i2c_file, buf, 1) !=1){
			std::cout << "Failed to Reset Address in readFullSensorState() " << std::endl;
		}

	    int numberBytes = 0x01;
		int bytesRead = read(this->i2c_file, res, numberBytes);
		if (bytesRead == -1){
		 	std::cout << "Failure to read Byte Stream in readFullSensorState()" <<std:: endl;
	    }

		short int data=0;
		data = res[0];
		return data;////////////////////////////////////

}

//This function is used to write the PWR_MGM_REG of the gyroscope.
short int Gyro::writePwrMgm( short int data)
{ /*
  i2c_smbus_write_byte_data(file,PWR_MGM_REG,data);

  return data;*/
  return 1;
}


short int Gyro::writeFS_SEL( short int data)
{ /*
  i2c_smbus_write_byte_data(file,WHO_AM_I_REG,data);

  return data;*/this->initialize();
                char buf[2];
                buf[0] = 0x16;
                buf[1] = data;

                if (write(this->i2c_file,buf,2) != 2) {
                        //printf("Failed to write to the i2c bus.\n");
                        //printf("%s\n\n",strerror(errno));
                	    close(this->i2c_file);
                        return 0;
                }
                close(this->i2c_file);
                return 1;

}



