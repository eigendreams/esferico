/*
 * adxl345.h
 *
 *  Created on: 03/11/2014
 *      Author: gabriel
 */

#ifndef ADXL345_H_
#define ADXL345_H_

class Adxl345 {

private:

	int I2CBus, I2CAddress;
	//char dataBuffer[BMA180_I2C_BUFFER];
    int i2c_file;
    
	double AcelX;
	double AcelY;
	double AcelZ;
	
	float X_g;
	float Y_g;
	float Z_g;

    int write_address(unsigned char reg);
    int write_byte(unsigned char reg, unsigned char data);
    int write_masked_byte(unsigned char reg, unsigned char data, char mask);
    int read_byte(unsigned char reg, unsigned char * data);
    int set_low_power(unsigned char power);
    float convert_to_g(unsigned short raw);
	//double pitch;  //in degrees
	//double roll;   //in degrees


	//float temperature; //accurate to 0.5C
	
	//BMA180_RANGE range;
	//BMA180_BANDWIDTH bandwidth;
	//BMA180_MODECONFIG modeConfig;

	

public:
    Adxl345(int module);
    int accelerometer_init();
	int measure_mode();
	int standby_mode();
	int set_range(char range);
	int get_range();
	int read_current_byte(unsigned char * data);
	
	/*
	int get_data_x(float * x_result);
	int get_data_y(float * y_result);
	int get_data_z(float * z_result);
	*/

    //
	//BMA180Accelerometer(int bus, int address);
	
	int readFullAcel();
	
	double getAcelX() { return AcelX; }
	double getAcelY() { return AcelY; }
	double getAcelZ() { return AcelZ; }
	
	float getX_g() { return X_g; }
	float getY_g() { return Y_g; }
	float getZ_g() { return Z_g; }

	virtual ~Adxl345();
};




#endif /* ADXL345_H_ */
