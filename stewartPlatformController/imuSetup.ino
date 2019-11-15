#include <Wire.h>

// to do: design and implement regression algo
// take running average of mean 90% of data ?

#define I2C_ADDR 0x3F // confirm value
#define MPU_SAMPLE_SIZE 1000
#define DISP_TO_ANGLE (1 / (250.0 x 65.5)) // 250 Hz

//Variables for Gyroscope - UPDATE!!!
int g_x, g_y, g_z;
long g_x_abs, g_y_abs, g_z_abs;
float d_pitch, d_roll;

// CORREctION MATh FOR PITCH AND ROLL CALCS
// boolean set_gyro_angles; 
// long a_x, a_y, a_z, acc_total_vector;
// float angle_roll_acc, angle_pitch_acc;
// int angle_pitch_buffer, angle_roll_buffer;
// float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
long loop_timer;
int temp;

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
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250.0Hz x 65.5)
  
  // calculate the traveled pitch angle and add this to the angle_pitch variable
  d_pitch += g_x * DISP_TO_ANGLE;
  d_roll += g_y * DISP_TO_ANGLE;

  // consider yaw and add to proper meas
  d_itch += angle_roll * sin(g_z * DISP_TO_ANGLE * PI / 180);            
  d_roll -= angle_pitch * sin(g_z * DISP_TO_ANGLE * PI / 180);               
  
  // future accelerometer calcs
  
  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z)); 
   
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296; 
  //Calculate the roll angle      
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       
  
  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll                                              
  angle_roll_acc -= 0.0;                                               
}

