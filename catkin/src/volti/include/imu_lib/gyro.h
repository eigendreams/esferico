/*
 * gyro.h
 *
 *  Created on: 05/11/2014
 *      Author: gabriel
 */

#ifndef GYRO_H_
#define GYRO_H_



//----------------
// Read Functions
//----------------
class Gyro {
private:
	int I2CBus, I2CAddress;
	double GyroX;
    double GyroY;
	double GyroZ;
	int i2c_file;

//This function is used to initialize the gyroscope. The function returns the -errno if an error accrues.
    int initialize();

//This function is used to initialize the gyroscope. The function returns the -errno if an error accrues.

    short int zeroGyro();

//This function is used to read the WHO_AM_I_REG of the gyroscope.
//Usage: int gyroID = readWhoAmI();
    short int readWhoAmI();

//This function is used to write the WHO_AM_I_REG of the gyroscope.
//Usage: data = readWhoAmI(data);
    short int writeWhoAmI(short int data);


//This function is used to read the SMPLRT_DIV_REG of the gyroscope.
    short int readSmplrtDiv();

//This function is used to write the SMPLRT_DIV_REG of the gyroscope.
    short int writeSmplrtDiv(short int data);

//This function is used to read the DLPF_FS_REG of the gyroscope.
    short int readDlpfFs();

//This function is used to write the DLPF_FS_REG of the gyroscope.
    short int writeDlpfFs(short int data);

//This function is used to read the INT_CFG_REG of the gyroscope.
    short int readIntCfg();

//This function is used to write the INT_CFG_REG of the gyroscope.
    short int writeIntCfg( short int data);

//This function is used to read the INT_STATUS of the gyroscope.
    short int readIntStatus();


//This function is used to read the temperature of the gyroscope.
//Usage: int gyroTemp = readTemp();
    short int readTemp();

//This function is used to read the X-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int xRate = readX();
    short int readX();

//This function is used to read the Y-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int yRate = readY();
    short int readY();

//This function is used to read the Z-Axis rate of the gyroscope. The function returns the ADC value from the Gyroscope
//NOTE: This value is NOT in degrees per second.
//Usage: int zRate = readZ();
    short int readZ();

//This function is used to read the PWR_MGM_REG of the gyroscope.
    short int readPwrMgm();

//This function is used to write the PWR_MGM_REG of the gyroscope.
    short int writePwrMgm( short int data);

    short int writeFS_SEL( short int data);

public:
    Gyro(int module);
    ~Gyro();

    int readFullGyro();

    float getGyroX() { return GyroX; }
    float getGyroY() { return GyroY; }
    float getGyroZ() { return GyroZ; }

};
#endif /* GYRO_H_ */
