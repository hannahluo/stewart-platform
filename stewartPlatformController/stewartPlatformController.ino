
#include <Servo.h>

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

Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_4;
Servo servo_5;
Servo servo_6;

int servo_min[6] = {5,5,5,5,5,5};
int servo_max[6] = {175,175,175,175,175,175};


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
}



void loop()
{
  while (digitalRead(buttonPin) != PRESSED)
  {
    int x = analogRead(JOY_X_PIN);
    int y = analogRead(JOY_Y_PIN);
    int z = calc_z_val(x,y);
    
    // send x,y,z to function
    
  }
}

