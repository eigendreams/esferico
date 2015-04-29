#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)
#define OUTPUT__BAUD_RATE 57600
#define OUTPUT__DATA_INTERVAL 20 // in milliseconds
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
#define OUTPUT__MODE_CALIBRATE_BOTH_SENSORS_ANGLES 5
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float
#define OUTPUT__STARTUP_STREAM_ON true  // true or false
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false

#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)

#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

#define DEBUG__NO_DRIFT_CORRECTION false
#define DEBUG__PRINT_LOOP_TIME false

#ifndef HW__VERSION_CODE
  #error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.pde (or .ino)!
#endif

#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

#define ACCEL_ADDRESS ((short) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((short) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((short) 0x68) // 0x68 = 0xD0 / 2

#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif

#ifndef IMU_h
#define IMU_h

class IMU
{
	public:

    unsigned long times;

    // Select your startup output mode and format here!
    short output_mode;
    short output_format;

    // If set true, an error message will be output if we fail to read sensor data.
    // Message format: "!ERR: reading <sensor>", followed by "\r\n".
    boolean output_errors;  // true or false

    // Sensor variables
    float accel[3] = {0, 0, 0};  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
    float accel_min[3] = {0, 0, 0};
    float accel_max[3] = {0, 0, 0};

    float magnetom[3] = {0, 0, 0 };
    float magnetom_min[3] = {0, 0, 0};
    float magnetom_max[3] = {0, 0, 0};
    float magnetom_tmp[3] = {0, 0,0 };

    float gyro[3] = {0, 0, 0};
    float gyro_average[3] = {0, 0, 0};
    short gyro_num_samples = 0;

    // DCM variables
    float MAG_Heading = 0;
    float Accel_Vector[3] = {0, 0, 0}; // Store the acceleration in a vector
    float Gyro_Vector[3] = {0, 0, 0}; // Store the gyros turn rate in a vector
    float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
    float Omega_P[3] = {0, 0, 0}; // Omega Proportional correction
    float Omega_I[3] = {0, 0, 0}; // Omega Integrator
    float Omega[3] = {0, 0, 0};
    float errorRollPitch[3] = {0, 0, 0};
    float errorYaw[3] = {0, 0, 0};
    float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
    float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    // Euler angles
    float yaw;
    float pitch;
    float roll;

    // DCM timing in the main loop
    unsigned long timestamp;
    unsigned long timestamp_old;
    float G_Dt; // Integration time for DCM algorithm

    // More output-state variables
    boolean output_stream_on;
    boolean output_single_on;
    short curr_calibration_sensor;
    boolean reset_calibration_session_flag;
    short num_accel_errors;
    short num_magn_errors;
    short num_gyro_errors;

    short module;
    IMU(short module);
    
    void Compass_Heading();
    void Normalize(void);
    void Drift_correction(void);
    void Matrix_update(void);
    void Euler_angles(void);
    float Vector_Dot_Product(const float v1[3], const float v2[3]);
    void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3]);
    void Vector_Scale(float out[3], const float v[3], float scale);
    void Vector_Add(float out[3], const float v1[3], const float v2[3]);
    void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3]);
    void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
    void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);
	void I2C_Init();
    void Accel_Init();
    void Read_Accel();
    void Magn_Init();
    void Read_Magn();
    void Gyro_Init();
    void Read_Gyro();
    void read_sensors();
    void reset_sensor_fusion();
    void compensate_sensor_errors();
    void check_reset_calibration_session();
    void turn_output_stream_on();
    void turn_output_stream_off();
    char readChar();
    void setup();
    void loop();
};
#endif