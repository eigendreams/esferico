void imu_setup() {
    // Init node
  nh.initNode();
  nh.advertise(imu_pub);

  // Init sensors
  delay(50);    // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
}

void imu_loop() {
    // Read incoming control messages
  //if (Serial.available() >= 2)
  //{
    //if (Serial.read() == '#') // Start of new control message
    //{
      //int command = Serial.read(); // Commands
      //if (command == 'f') // request one output _f_rame
        //output_single_on = true;
      //else if (command == 's') // _s_ynch request
      //{
        // Read ID
        //byte id[2];
        //id[0] = readChar();
        //id[1] = readChar();
        
        // Reply with synch message
        //Serial.print("#SYNCH");
        //Serial.write(id, 2);
        //Serial.println();
      //}
      //else if (command == 'o') // Set _o_utput mode
      //{
        //char output_param = readChar();
        //if (output_param == 'n')  // Calibrate _n_ext sensor
        //{
          //curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          //reset_calibration_session_flag = true;
        //}
        //else if (output_param == 't') // Output angles as _t_ext
        //{
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_TEXT;
        //}
        //else if (output_param == 'b') // Output angles in _b_inary format
        //{
          //output_mode = OUTPUT__MODE_ANGLES;
          //output_format = OUTPUT__FORMAT_BINARY;
        //}
        //else if (output_param == 'c') // Go to _c_alibration mode
        //{
          //output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          //reset_calibration_session_flag = true;
        //}
        //else if (output_param == 's') // Output _s_ensor values
        //{
          //char values_2param = readChar();
          //char format_param = readChar();
          //if (values_param == 'r')  // Output _r_aw sensor values
            //output_mode = OUTPUT__MODE_SENSORS_RAW;
          //else if (values_param == 'c')  // Output _c_alibrated sensor values
            //output_mode = OUTPUT__MODE_SENSORS_CALIB;
          //else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
            //output_mode = OUTPUT__MODE_SENSORS_BOTH;

          //if (format_param == 't') // Output values as _t_text
            //output_format = OUTPUT__FORMAT_TEXT;
          //else if (format_param == 'b') // Output values in _b_inary format
            //output_format = OUTPUT__FORMAT_BINARY;
        //}
        //else if (output_param == '0') // Disable continuous streaming output
        //{
          //turn_output_stream_off();
          //reset_calibration_session_flag = true;
        //}
        //else if (output_param == '1') // Enable continuous streaming output
        //{
          //reset_calibration_session_flag = true;
          //turn_output_stream_on();
        //}
        //else if (output_param == 'e') // _e_rror output settings
        //{
          //char error_param = readChar();
          //if (error_param == '0') output_errors = false;
          //else if (error_param == '1') output_errors = true;
          //else if (error_param == 'c') // get error count
          //{
            //Serial.print("#AMG-ERR:");
            //Serial.print(num_accel_errors); Serial.print(",");
            //Serial.print(num_magn_errors); Serial.print(",");
            //Serial.println(num_gyro_errors);
          //}
        //}
      //}
//#if OUTPUT__HAS_RN_BLUETOOTH == true
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      //else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
        //turn_output_stream_on();
      //else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
        //turn_output_stream_off();
//#endif // OUTPUT__HAS_RN_BLUETOOTH == true
    //}
    //else
    //{ 
    //} // Skip character
  //}

  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();  // Check if this session needs a reset
      //if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {
      // Apply sensor calibration
      compensate_sensor_errors();
    
      // Run DCM algorithm
      Compass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      
      if (output_stream_on || output_single_on) output_angles();
    }
    else  // Output sensor values
    {      
      //if (output_stream_on || output_single_on) output_sensors();
    }
    
    output_single_on = false;
    
//#if DEBUG__PRINT_LOOP_TIME == true
    //Serial.print("loop time (ms) = ");
    //Serial.println(millis() - timestamp);
//#endif
  }
//#if DEBUG__PRINT_LOOP_TIME == true
  //else
  //{
    //Serial.println("waiting...");
  //}
//#endif
}

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
//char readChar()
//{
  //while (Serial.available() < 1) { } // Block
  //return Serial.read();
//}