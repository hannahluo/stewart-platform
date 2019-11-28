#include <Servo.h>
#include <math.h>
#include <Wire.h>

// #define PRINT_DEBUG
#define ANGLE_PRINT_DEBUG

#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define JOY_BTN_PIN 52
#define LED_PIN 13

#define SERVO_0_PIN 2
#define SERVO_1_PIN 3
#define SERVO_2_PIN 4
#define SERVO_3_PIN 5
#define SERVO_4_PIN 6
#define SERVO_5_PIN 7

#define IMU_VCC_PIN

#define PRESSED 0

#define SERVO_MAX 180
#define SERVO_MIN 0
#define X 0 
#define Y 1
#define Z 2

#define HEIGHT 16.8 // 15.8
#define NUM_LEGS 6
#define HORN_LENGTH 2.0

#define SERVO_ANGLE_SENSITIVITY 2
#define INPUT_ANGLE_SENSITIVITY 0.01

#define I2C_ADDR 0x3F // confirm value
#define MPU_SAMPLE_SIZE 1000
#define ALPHA 0.18 // test
#define MICROS_PER_LOOP 125000

Servo servo_0;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;

//Variables for Gyroscope - UPDATE!!!
int g_x, g_y, g_z;
float g_x_avg, g_y_avg, g_z_avg;
float g_x_abs, g_y_abs, g_z_abs;
float d_pitch, d_roll; // change in roll and pitch
float d_pitch_offset, d_roll_offset;
float disp_to_angle = (1 / ((1000000 / MICROS_PER_LOOP) * 65.5));

float zero_pitch, zero_roll;

// CORREctION MATh FOR PITCH AND ROLL CALCS
// boolean set_gyro_angles; 
long a_x, a_y, a_z, a_mag;
float a_x_avg, a_y_avg, a_z_avg;
float a_x_abs, a_y_abs, a_z_abs;
float roll_acc, pitch_acc;

// Setup timers and temp variables
long loop_timer;
int temp;

int servo_min[NUM_LEGS] = {180,0,180,0,180,0};
int servo_max[NUM_LEGS] = {0,180,0,180,0,180};
int current_servo_angles[NUM_LEGS] = {0,0,0,0,0,0};
float rod_length[NUM_LEGS] = {16.33, 16.51, 16.44, 16.51, 16.56, 16.28};
float servo_angle[NUM_LEGS] = {4*PI/3,2*PI/3,2*PI/3,0,0,4*PI/3};
Servo servos[NUM_LEGS] = {servo_0,servo_1,servo_2,servo_3,servo_4,servo_5};

float DistanceToLegsFromOrigin[NUM_LEGS][3] =
{
  {3.83794,-5.32765,0},
  {-3.83794,-5.32765,0},
  {-6.53288,-0.659892,0},
  {-2.694686,5.987542,0},
  {2.694686,5.987542,0},
  {6.53288,-0.659892,0}
};
float RotationMatrix[3][3];
float T[3];
float P[3];
float LegVectors[NUM_LEGS][3];
float Lengths[NUM_LEGS];

float getLengthOfVector3(const float vector[3])
{
  return sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

int sign(float x)
{
  return ((x>0)-(x<0));
}

void fillRotationMatrix(float roll, float pitch, float yaw)
{
    RotationMatrix[0][0] = cos(yaw)*cos(pitch);
    RotationMatrix[0][1] = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll);
    RotationMatrix[0][2] = sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
    RotationMatrix[1][0] = sin(yaw)*cos(pitch);
    RotationMatrix[1][1] = cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
    RotationMatrix[1][2] = -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
    RotationMatrix[2][0] = -sin(pitch);
    RotationMatrix[2][1] = cos(pitch)*sin(roll);
    RotationMatrix[2][2] = cos(pitch)*cos(roll);
}

void computeTVector(float roll, float pitch, float yaw)
{
    T[Z] = HEIGHT;
    T[Y] = 0;
    T[X] = 0;
}

void computeResultantRotatedPVectorForLeg(int i)
{
  P[X] = RotationMatrix[0][0] * DistanceToLegsFromOrigin[i][X] + RotationMatrix[0][1] * DistanceToLegsFromOrigin[i][Y] + RotationMatrix[0][2] * DistanceToLegsFromOrigin[i][Z];
  P[Y] = RotationMatrix[1][0] * DistanceToLegsFromOrigin[i][X] + RotationMatrix[1][1] * DistanceToLegsFromOrigin[i][Y] + RotationMatrix[1][2] * DistanceToLegsFromOrigin[i][Z];
  P[Z] = RotationMatrix[2][0] * DistanceToLegsFromOrigin[i][X] + RotationMatrix[2][1] * DistanceToLegsFromOrigin[i][Y] + RotationMatrix[2][2] * DistanceToLegsFromOrigin[i][Z];
}

void computeVectorForLeg(int i)
{
  LegVectors[i][X] = T[X] + P[X] - DistanceToLegsFromOrigin[i][X];
  LegVectors[i][Y] = T[Y] + P[Y] - DistanceToLegsFromOrigin[i][Y];
  LegVectors[i][Z] = T[Z] + P[Z] - DistanceToLegsFromOrigin[i][Z];

#ifdef PRINT_DEBUG
  Serial.print("LEG VECTOR ");
  Serial.print(i);
  Serial.print(": [");
  Serial.print(LegVectors[i][X]);
  Serial.print(", ");
  Serial.print(LegVectors[i][Y]);
  Serial.print(", ");
  Serial.print(LegVectors[i][Z]);
  Serial.println("] =");
  Serial.print("P VECTOR   ");
  Serial.print(i);
  Serial.print(": [");
  Serial.print(P[X]);
  Serial.print(", ");
  Serial.print(P[Y]);
  Serial.print(", ");
  Serial.print(P[Z]);
  Serial.println("] -");
  Serial.print("LEGS2OG    ");
  Serial.print(i);
  Serial.print(": [");
  Serial.print(DistanceToLegsFromOrigin[i][X]);
  Serial.print(", ");
  Serial.print(DistanceToLegsFromOrigin[i][Y]);
  Serial.print(", ");
  Serial.print(DistanceToLegsFromOrigin[i][Z]);
  Serial.println("]");
  Serial.println();
#endif
}

// Returns a float from 0.0 - 1.0, representing the fraction of the current height that the leg is
float getPercentHeightLeg(int i)
{
  return LegVectors[i][Z] / T[Z];
}

// Test function to see if the leg positions actually make a viable hexagon - Not optimized because it won't matter in production
float computePlatformLengthBetweenLegs(int a, int b)
{
  float aPos[3] = 
  {
    DistanceToLegsFromOrigin[a][X] + LegVectors[a][X],
    DistanceToLegsFromOrigin[a][Y] + LegVectors[a][Y],
    DistanceToLegsFromOrigin[a][Z] + LegVectors[a][Z]
  };
  
  float bPos[3] = 
  {
    DistanceToLegsFromOrigin[b][X] + LegVectors[b][X],
    DistanceToLegsFromOrigin[b][Y] + LegVectors[b][Y],
    DistanceToLegsFromOrigin[b][Z] + LegVectors[b][Z]
  };

  return sqrt( (aPos[X] - bPos[X])*(aPos[X] - bPos[X]) + (aPos[Y] - bPos[Y])*(aPos[Y] - bPos[Y]) + (aPos[Z] - bPos[Z])*(aPos[Z] - bPos[Z]) );
}

void calculateLegLengths(float roll, float pitch, float yaw, float surgeAngle, float swayAngle, float heaveAngle)
{
  if (abs(roll) < INPUT_ANGLE_SENSITIVITY && abs(pitch) < INPUT_ANGLE_SENSITIVITY) return;
  if(abs(roll) > 30 * 180 / PI || abs(pitch) > 30 * 180 / PI) return;
  fillRotationMatrix(roll,pitch,yaw);
  computeTVector(surgeAngle, swayAngle, heaveAngle);
  for(int i = 0; i < NUM_LEGS; ++i)
    {
      computeResultantRotatedPVectorForLeg(i);
      computeVectorForLeg(i);
    }
}

void writeToServos() {
  float e, f, g;
  float legLength, legX, legY, legZ;
  float alpha;
  for(int i = 0; i < NUM_LEGS; ++i) {
    legLength = sqrt(LegVectors[i][X]*LegVectors[i][X] + LegVectors[i][Y]*LegVectors[i][Y] + LegVectors[i][Z]*LegVectors[i][Z]);
    e = 2*HORN_LENGTH*abs(LegVectors[i][Z]);
    f = 2*HORN_LENGTH*(LegVectors[i][X]*cos(servo_angle[i]) + LegVectors[i][Y]*sin(servo_angle[i]));
    g = legLength*legLength - (rod_length[i]*rod_length[i] - HORN_LENGTH*HORN_LENGTH);
    
#ifdef PRINT_DEBUG
    Serial.print("asin input: ");
    Serial.println((g/sqrt(e*e + f*f)));
#endif

    float asinNum;
    if( abs(g) > sqrt(e*e + f*f)) asinNum = sign(g) * PI/2;
    else asinNum = asin(g/sqrt(e*e + f*f));
    alpha = (asinNum - atan2(f, e))*180/PI;
    
#ifdef PRINT_DEBUG
    Serial.print("pre linear alpha ");
    Serial.println(alpha);
    Serial.print("e: ");
    Serial.print(e);
    Serial.print(", f: ");
    Serial.print(f);
    Serial.print(", g: ");
    Serial.print(g);
    Serial.print(", alpha: ");
    Serial.println((int)alpha);
#endif
    int servoPos;
    /*
    if (i % 2) {
      alpha = (servo_max[i] - servo_min[i])*(alpha - SERVO_MIN)/(SERVO_MAX - SERVO_MIN) + servo_min[i];
      servoPos = constrain(90 - (int)alpha, servo_min[i], servo_max[i]);
    }
    else { // 1 3 5 go backwards
      alpha = (servo_min[i] - servo_max[i])*(alpha - SERVO_MIN)/(SERVO_MAX - SERVO_MIN) + servo_max[i];
      servoPos = constrain(90 + (int)alpha, servo_max[i], servo_min[i]);
    }
    */
    servoPos = 90 + (int)alpha;
    servoPos = constrain(servoPos, SERVO_MIN, SERVO_MAX);

#ifdef PRINT_DEBUG
    Serial.println(servoPos);
#endif
    if(current_servo_angles[i] + SERVO_ANGLE_SENSITIVITY < servoPos || current_servo_angles[i] - SERVO_ANGLE_SENSITIVITY > servoPos)
    {
      int servoPosUs = getMsForAngle(servoPos, i);
      servos[i].writeMicroseconds(servoPosUs);
      current_servo_angles[i] = servoPos;
    }
  }
}

void setup()
{
  Wire.begin();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);    

  servo_0.attach(SERVO_0_PIN);
  servo_1.attach(SERVO_1_PIN);
  servo_2.attach(SERVO_2_PIN);
  servo_3.attach(SERVO_3_PIN);
  servo_4.attach(SERVO_4_PIN);
  servo_5.attach(SERVO_5_PIN);

  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  
  Serial.begin(115200);
  Serial.println("START");

#ifdef PRINT_DEBUG
  Serial.println("Print Debugging Enabled");
#else
  Serial.println("Print Debugging Disabled");
#endif

  for(int i = 0; i < 6; ++i) {
    servos[i].writeMicroseconds(getMsForAngle(90,i));
    delay(1000);
  }

  setupMPU();                                
  for (int i = 0; i < MPU_SAMPLE_SIZE; ++i){                  
    readMPU();                                    
    g_x_abs += g_x;                                         
    g_y_abs += g_y;                                        
    g_z_abs += g_z;
    a_x_abs += a_x;                                         
    a_y_abs += a_y;                                        
    a_z_abs += a_z;                                                
    delay(1);                                                          
  }
 
  g_x_abs /= MPU_SAMPLE_SIZE;                                                 
  g_y_abs /= MPU_SAMPLE_SIZE;                                                 
  g_z_abs /= MPU_SAMPLE_SIZE;
  a_x_abs /= MPU_SAMPLE_SIZE;                                                 
  a_y_abs /= MPU_SAMPLE_SIZE;                                                 
  a_z_abs /= MPU_SAMPLE_SIZE;

  g_x_avg = g_x_abs;
  g_y_avg = g_y_abs;
  g_z_avg = g_z_abs;
  a_x_avg = a_x_abs;
  a_y_avg = a_y_abs;
  a_z_avg = a_z_abs;

  //d_roll_offset = atan2(a_y_abs, a_z_abs);
  //d_pitch_offset = atan2(-a_x_abs, sqrt(a_y_abs*a_y_abs + a_z_abs*a_z_abs));
  
  d_roll_offset = 0;
  d_pitch_offset = 0;
  d_roll = 0;
  d_pitch = 0;
  
  digitalWrite(LED_PIN, LOW);
}

void loop()
{
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  float surgeAngle = 0;
  float swayAngle = 0;
  float heaveAngle = 0;

  while (digitalRead(JOY_BTN_PIN) != PRESSED) {
    loop_timer = micros(); 
    readMPU();
    convertMPUVals();
    
#ifdef ANGLE_PRINT_DEBUG
    Serial.print("Roll: ");
    Serial.print(d_roll * 180 / PI);
    Serial.print(",  Pitch: ");
    Serial.println(d_pitch * 180 / PI);
#endif

    calculateLegLengths(-1.1*d_roll, -1.1*d_pitch, yaw, surgeAngle, swayAngle, heaveAngle);
    writeToServos();

    // Wait to keep the timing consistent
    while(micros() - loop_timer < MICROS_PER_LOOP){}
  }

  while (digitalRead(JOY_BTN_PIN) == PRESSED) {
    delay(100);
    // // Serial.println("Button pushed");
    Serial.println(digitalRead(JOY_BTN_PIN));
  }
}
