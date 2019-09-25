
#include <Servo.h>
#include <math.h>

#define JOY_X_PIN A0
#define JOY_Y_PIN A1
#define JOY_BTN_PIN 2 // *** Pick the pin

#define SERVO_1_PIN 9 // *** Pick the pin
#define SERVO_2_PIN 9 // *** Pick the pin
#define SERVO_3_PIN 9 // *** Pick the pin
#define SERVO_4_PIN 9 // *** Pick the pin
#define SERVO_5_PIN 9 // *** Pick the pin
#define SERVO_6_PIN 9 // *** Pick the pin

#define DEAD_ZONE_MIN 410
#define DEAD_ZONE_MAX 614
#define PRESSED 1

#define HEIGHT 5
#define DISTANCE_TO_LEG 3
#define NUM_LEGS 6

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;

int servo_min[6] = {5,5,5,5,5,5};
int servo_max[6] = {175,175,175,175,175,175};

float DistanceToLegsFromOrigin[NUM_LEGS][3];
float RotationMatrix[3][3];
float T[3];
float P[3];
float LegVectors[NUM_LEGS][3];
float Lengths[NUM_LEGS];

void initDistanceToLegsFromOrigin()
{
  for(int i = 0; i < NUM_LEGS; ++i)
    {
      float angle = ( PI / 3 ) * i;
      DistanceToLegsFromOrigin[i][0] = DISTANCE_TO_LEG * cos(angle);
      DistanceToLegsFromOrigin[i][1] = DISTANCE_TO_LEG * sin(angle);
      DistanceToLegsFromOrigin[i][2] = 0;
  }
}

float getLengthOfVector3(const float vector[3])
{
  return sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
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
    T[2] = HEIGHT * cos(yaw);
    float xyLength = HEIGHT * sin(yaw);
    T[1] = xyLength * sin(roll);
    T[0] = xyLength * cos(roll);
}

void computeResultantRotatedPVectorForLeg(int i)
{
  P[0] = RotationMatrix[0][0] * DistanceToLegsFromOrigin[i][0] + RotationMatrix[0][1] * DistanceToLegsFromOrigin[i][1] + RotationMatrix[0][2] * DistanceToLegsFromOrigin[i][2];
  P[1] = RotationMatrix[1][0] * DistanceToLegsFromOrigin[i][0] + RotationMatrix[1][1] * DistanceToLegsFromOrigin[i][1] + RotationMatrix[1][2] * DistanceToLegsFromOrigin[i][2];
  P[2] = RotationMatrix[2][0] * DistanceToLegsFromOrigin[i][0] + RotationMatrix[2][1] * DistanceToLegsFromOrigin[i][1] + RotationMatrix[2][2] * DistanceToLegsFromOrigin[i][2];
}

void computeVectorForLeg(int i)
{
  LegVectors[i][0] = T[0] + P[0] - DistanceToLegsFromOrigin[i][0];
  LegVectors[i][1] = T[1] + P[1] - DistanceToLegsFromOrigin[i][1];
  LegVectors[i][2] = T[2] + P[2] - DistanceToLegsFromOrigin[i][2];
}

// Returns a float from 0.0 - 1.0, representing the fraction of the current height that the leg is
float getPercentHeightLeg(int i)
{
  return LegVectors[i][2] / T[2];
}

// Test function to see if the leg positions actually make a viable hexagon - Not optimized because it won't matter in production
float computePlatformLengthBetweenLegs(int a, int b)
{
  float aPos[3] = 
  {
    DistanceToLegsFromOrigin[a][0] + LegVectors[a][0],
    DistanceToLegsFromOrigin[a][1] + LegVectors[a][1],
    DistanceToLegsFromOrigin[a][2] + LegVectors[a][2]
  };
  
  float bPos[3] = 
  {
    DistanceToLegsFromOrigin[b][0] + LegVectors[b][0],
    DistanceToLegsFromOrigin[b][1] + LegVectors[b][1],
    DistanceToLegsFromOrigin[b][2] + LegVectors[b][2]
  };

  return sqrt( (aPos[0] - bPos[0])*(aPos[0] - bPos[0]) + (aPos[1] - bPos[1])*(aPos[1] - bPos[1]) + (aPos[2] - bPos[2])*(aPos[2] - bPos[2]) );
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

void setup()
{
  servo_1.attach(SERVO_1_PIN);
  servo_2.attach(SERVO_2_PIN);
  servo_3.attach(SERVO_3_PIN);
  servo_4.attach(SERVO_4_PIN);
  servo_5.attach(SERVO_5_PIN);
  servo_6.attach(SERVO_6_PIN);

  pinMode(JOY_X_PIN, INPUT);
  pinMode(JOY_Y_PIN, INPUT);
  pinMode(JOY_BTN_PIN, INPUT_PULLUP); 

  Serial.begin(9600);

  initDistanceToLegsFromOrigin();
}

void loop()
{
  float yaw = 0;
  float pitch = 0;
  float roll = 0;
  float surgeAngle = PI/6;
  float swayAngle = PI/6;
  float heaveAngle = PI/6;
  
  while (/*digitalRead(buttonPin) != PRESSED*/true)
  {
    int x = analogRead(JOY_X_PIN);
    int y = analogRead(JOY_Y_PIN);
    //int z = calc_z_val(x,y);
    
    // send x,y,z to function
    calculateLegLengths(yaw, pitch, roll, surgeAngle, swayAngle, heaveAngle);
  }
}

