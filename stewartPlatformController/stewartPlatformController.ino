#include <Servo.h>
#include <math.h>

// #define PRINT_DEBUG

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

#define DEAD_ZONE_MIN 485
#define DEAD_ZONE_MAX 530
#define MAX_INPUT 1023
#define MIN_INPUT 0
#define MAX_CONVERTED_INPUT 50
#define MIN_CONVERTED_INPUT -50

#define PRESSED 0

#define SERVO_MAX 180
#define SERVO_MIN 0
#define X 0 
#define Y 1
#define Z 2

#define HEIGHT 10.8
#define DISTANCE_TO_LEG 7.1
#define NUM_LEGS 6
#define HORN_LENGTH 1.2
#define ROD_LENGTH 11.2083
#define PLATFORM_LENGTH 10
Servo servo_0;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;

int servo_min[6] = {135,0,180,0,135,0};
//int servo_max[6] = {135,135,180,175,135,175};
int servo_max[6] = {0,135,0,175,0,175};
float rod_length[6] = {11.20, 11.23, 11.23, 11.05, 11.24, 11.32};
// WE NEED TO MEASURE THESE OKAY PLEASE MEASURE THEM BEFORE U TRY TO RUN THE CODE SOMEONE!!
// https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/
// IT'S THE ANGLE BETA FOUND HERE
// DO THIS FOR EACH SERVO AND RECORD THE RESULTS IN THE ARRAY BELOW THANKS
float servo_angle[6] = {0,4*PI/3,4*PI/3,2*PI/3,2*PI/3,0};
Servo servos[6] = {servo_0,servo_1,servo_2,servo_3,servo_4,servo_5};

float DistanceToLegsFromOrigin[NUM_LEGS][3] =
{
  {3.327,6.467,0},
  {7.265,-0.350,0},
  {3.935,-6.117,0},
  {-3.935,-6.117,0},
  {-7.265,-0.350,0},
  {-3.327,6.467,0}
};
float RotationMatrix[3][3];
float T[3];
float P[3];
float LegVectors[NUM_LEGS][3];
float Lengths[NUM_LEGS];

float MaxInputL = (MAX_CONVERTED_INPUT - MIN_CONVERTED_INPUT) / 2.0; // This makes the max in the corners equal to circles
float MaxTilt = atan2(2 * HORN_LENGTH, PLATFORM_LENGTH);
float DeltaInputZ = HEIGHT * ( 1.0 - cos(MaxTilt));

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
  fillRotationMatrix(roll,pitch,yaw);
  computeTVector(surgeAngle, swayAngle, heaveAngle);
  for(int i = 0; i < NUM_LEGS; ++i)
    {
      computeResultantRotatedPVectorForLeg(i);
      computeVectorForLeg(i);
    }
}

void getAngles(int x, int y, float& roll, float& pitch)
{
  // Calculate a z for the vector defining the rotation - The lower the z the more tilted the platform
  // First maps the intensity of the joystick angle to a fraction of the change in height
  // At its lowest the fraction will be 0 and the height equal to base Height
  // At its highest the fraction will be 1.0 and the height will be lowered by DeltaInputZ
  // All values in between are approximately linearized 
  float zv = HEIGHT - min(sqrt(pow(x,2) + pow(y,2)) / MaxInputL, 1.0) * DeltaInputZ;
  float xv = x *1.0 / MAX_CONVERTED_INPUT;
  float yv = y *1.0 / MAX_CONVERTED_INPUT;
  roll = atan2(yv, zv);
  float len = sqrt(pow(zv,2) + pow(yv,2));
  pitch = atan2(xv,len) ;
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
    Serial.print("pre linaear alpha ");
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
    // alpha = (servo_max[i] - servo_min[i])*(alpha - SERVO_MIN)/(SERVO_MAX - SERVO_MIN) + servo_min[i];
    
    int servoPos;
//    if (i == 1) 
//      servoPos = constrain(90 + (int)alpha, servo_min[i], servo_max[i]);
//    else
//      servoPos = constrain(90 - (int)alpha, servo_min[i], servo_max[i]);
    if (i % 2) {
      alpha = (servo_max[i] - servo_min[i])*(alpha - SERVO_MIN)/(SERVO_MAX - SERVO_MIN) + servo_min[i];
      servoPos = constrain(90 - (int)alpha, servo_min[i], servo_max[i]);
    }
    else { // 1 3 5 go backwards
      alpha = (servo_min[i] - servo_max[i])*(alpha - SERVO_MIN)/(SERVO_MAX - SERVO_MIN) + servo_max[i];
      servoPos = constrain(90 + (int)alpha, servo_max[i], servo_min[i]);
    }

#ifdef PRINT_DEBUG
    Serial.println(servoPos);
#endif

    servos[i].write(servoPos);
    
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

  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(JOY_BTN_PIN, INPUT_PULLUP); 
  setupMPU();
  // get flat platform values                                        
  for (int i = 0; i < MPU_SAMPLE_SIZE; ++i){                  
    readMPU();
    //Add the gyro x offset to the gyro_x_cal variable                                            
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable                                              
    gyro_y_cal += gyro_y; 
    //Add the gyro z offset to the gyro_z_cal variable                                             
    gyro_z_cal += gyro_z; 
    //Delay 3us to have 250Hz for-loop                                             
    delay(3);                                                          
  }
 
  gyro_x_cal /= MPU_SAMPLE_SIZE;                                                 
  gyro_y_cal /= MPU_SAMPLE_SIZE;                                                 
  gyro_z_cal /= MPU_SAMPLE_SIZE;
  loop_timer = micros(); 
  
  Serial.begin(9600);
  Serial.println("START");

#ifdef PRINT_DEBUG
  Serial.println("Print Debugging Enabled");
#else
  Serial.println("Print Debugging Disabled");
#endif

  for(int i = 0; i < 6; ++i) {
    servos[i].write(servo_min[i]);
    delay(1000);
    servos[i].write((servo_max[i]+servo_min[i])/2);
    delay(1000);

  }
  delay(2500);
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
    int x = analogRead(JOY_X_PIN);
    int y = analogRead(JOY_Y_PIN);
    int x_new = convert_xy_value(x);
    int y_new = convert_xy_value(y);

#ifdef PRINT_DEBUG
    Serial.println("--------------");
    Serial.print("X: ");
    Serial.println(x);
    Serial.print("Y: ");
    Serial.println(y);
    Serial.print("X CONVERTED: ");
    Serial.println(x_new);
    Serial.print("Y CONVERTED: ");
    Serial.println(y_new);
#endif

    getAngles(x_new, y_new, roll, pitch);

#ifdef PRINT_DEBUG
    Serial.print("Roll: ");
    Serial.println(roll * 180 / PI);
    Serial.print("Pitch: ");
    Serial.println(pitch * 180 / PI);
#endif

    calculateLegLengths(roll, pitch, yaw, surgeAngle, swayAngle, heaveAngle);
    writeToServos();
    delay(75);
  }

  while (digitalRead(JOY_BTN_PIN) == PRESSED) {
    delay(100);
    // // Serial.println("Button pushed");
    Serial.println(digitalRead(JOY_BTN_PIN));
  }
}
