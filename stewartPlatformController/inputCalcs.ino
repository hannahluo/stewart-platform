#define DEAD_ZONE_MIN 485
#define DEAD_ZONE_MAX 530


// Calculate the z angle from the z-axis to the T line
double calc_z_angle_val(int x, int y)
{
  double xy = sqrt(pow(x,2) + pow(y,2));
  return asin(xy / HEIGHT);
}

int convert_xy_value(int input)
{
  if (input > DEAD_ZONE_MAX)
    return map(input, DEAD_ZONE_MAX, MAX_INPUT, 0, 50);
  else if (input < DEAD_ZONE_MIN)
    return map(input, 0, DEAD_ZONE_MIN, -50, 0);   
  else
    return 0;
}

