void setupMPU() {
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x6B);  
  //Set the requested starting register                                                  
  Wire.write(0x00);
  //End the transmission                                                    
  Wire.endTransmission(); 
                                              
  //Configure the accelerometer (+/-8g)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68); 
  //Send the requested starting register                                       
  Wire.write(0x1C);   
  //Set the requested starting register                                                 
  Wire.write(0x10); 
  //End the transmission                                                   
  Wire.endTransmission(); 
                                              
  //Configure the gyro (500dps full scale)
  
  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register                                        
  Wire.write(0x1B);
  //Set the requested starting register                                                    
  Wire.write(0x08); 
  //End the transmission                                                  
  Wire.endTransmission(); 
}

void readMPU() {                                         
  Wire.beginTransmission(0x68);  
  //Send the requested starting register                                      
  Wire.write(0x3B);
  //End the transmission                                                    
  Wire.endTransmission(); 
  //Request 14 bytes from the MPU-6050                                  
  Wire.requestFrom(0x68, 14);    
  //Wait until all the bytes are received                                       
  while(Wire.available() < 14);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
  a_x = Wire.read() << 8 | Wire.read();                                  
  a_y = Wire.read() << 8 | Wire.read();                                  
  a_z = Wire.read() << 8 | Wire.read();                                  
  temp = Wire.read() << 8 | Wire.read();                                   
  g_x = Wire.read() << 8 | Wire.read();                                 
  g_y = Wire.read() << 8 | Wire.read();                                 
  g_z = Wire.read() <<8 | Wire.read(); 
}

void convertMPUVals() {
  g_x -= g_x_abs;                                                
  g_y -= g_y_abs;                                                
  g_z -= g_z_abs;                                                

  g_x_avg = ALPHA * g_x + (1-ALPHA) * g_x_avg;
  g_y_avg = ALPHA * g_y + (1-ALPHA) * g_y_avg; 
  g_z_avg = ALPHA * g_z + (1-ALPHA) * g_z_avg; 

  //Gyro angle calculations . Note 0.0000611 = 1 / (250.0Hz x 65.5)
  
  // calculate the traveled pitch angle and add this to the angle_pitch variable
  d_pitch += g_x_avg * DISP_TO_ANGLE;
  d_roll += g_y_avg * DISP_TO_ANGLE;

  // consider yaw and add to proper meas
  d_pitch += d_roll * sin(g_z_avg * DISP_TO_ANGLE * PI / 180);            
  d_roll -= d_pitch * sin(g_z_avg * DISP_TO_ANGLE * PI / 180);               
  
  // future accelerometer calcs
  
  //Calculate the total accelerometer vector
  //acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
   
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  //angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
  //Calculate the roll angle      
  //angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Accelerometer calibration value for pitch
  //angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll                                              
  //angle_roll_acc -= 0.0;                                               
}

