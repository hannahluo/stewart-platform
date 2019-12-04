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
  float g_x_cur = g_x - g_x_abs;                                                
  float g_y_cur = g_y - g_y_abs;                                                
  float g_z_cur = g_z - g_z_abs;
  float a_x_cur = a_x; // - a_x_abs;                                                
  float a_y_cur = a_y; // - a_y_abs;                                                
  float a_z_cur = a_z; // - a_z_abs;                    

 /*
  g_x_avg = ALPHA * (float)g_x + (1-ALPHA) * (float)g_x_avg;
  g_y_avg = ALPHA * (float)g_y + (1-ALPHA) * (float)g_y_avg; 
  g_z_avg = ALPHA * (float)g_z + (1-ALPHA) * (float)g_z_avg; 
  a_x_avg = ALPHA * (float)a_x + (1-ALPHA) * (float)a_x_avg;
  a_y_avg = ALPHA * (float)a_y + (1-ALPHA) * (float)a_y_avg; 
  a_z_avg = ALPHA * (float)a_z + (1-ALPHA) * (float)a_z_avg; 
*/
  d_roll -= d_roll_offset;
  d_pitch -= d_pitch_offset;

  d_roll = ALPHA * atan2(a_y_cur, a_z_cur) + (1-ALPHA) * d_roll;
  d_pitch = ALPHA * atan2(-a_x_cur, sqrt(a_y_cur*a_y_cur + a_z_cur*a_z_cur)) + (1-ALPHA) * d_pitch;

  /*
  // calculate the traveled pitch angle and add this to the angle_pitch variable
  d_pitch += g_x_avg * disp_to_angle * PI / 180;
  d_roll += g_y_avg * disp_to_angle * PI / 180;

  // consider yaw and add to proper meas
  d_pitch += d_roll * sin(g_z_avg * disp_to_angle * PI / 180);            
  d_roll -= d_pitch * sin(g_z_avg * disp_to_angle * PI / 180);               
  
  // future accelerometer calcs

  a_mag = sqrt((a_x*a_x)+(a_y*a_y)+(a_z*a_z)); 
  
  pitch_acc = asin((float)a_y/a_mag)/(PI/180); 
  roll_acc = asin((float)a_x/a_mag)/(-PI/180);                             
  
  d_pitch = d_pitch * 0.9996 + pitch_acc * 0.0004;  
  d_roll = d_roll * 0.9996 + roll_acc * 0.0004;      
  */                                          
}

