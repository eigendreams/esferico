#ifndef HMC5883_H_
#define HMC5883_H_

	
    
class MAG {

private:
	int I2CBus, I2CAddress;
	float MagX;
	float MagY;
	float MagZ;

	int i2c_file;

	short int writeReg_Data( short int reg,short int data);

	int hmc5883_init();
public:
	MAG(int module);
	~MAG();

	int readfullState();
	float getMagX() { return MagX; }
	float getMagY() { return MagY; }
	float getMagZ() { return MagZ; }
	
};
#endif /* HMC5883_H_ */
