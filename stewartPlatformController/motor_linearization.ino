float motor_m[NUM_LEGS] = {9.31,-9,11.5,-11.5,8.95,-11.1};
float motor_b[NUM_LEGS] = {642,2390,191,2829,656,2776};

int getMsForAngle(int angle, int servo)
{
  return motor_m[servo]*angle + motor_b[servo];
}

