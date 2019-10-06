// x and y are between -50 and 50
int preset_angles[9][6] = {
  {90, 90, 90, 90, 90, 90},
  {0, 0, 110, 130, 130, 70}, // North
  {130, 130, 70, 0, 0, 110}, // South
  {60, 110, 0, 100, 100, 130}, // East
  {60, 110, 130, 110, 70, 0}, // West
  {50, 50, 0, 50, 130, 130}, // NE
  {50, 50, 130, 130, 50, 0}, // NW
  {50, 50, 0, 0, 130, 130}, // SE
  {50, 130, 130, 50, 0, 0}, // SW
};

void servosWritePresets(int angles[6]){
  for (int i = 0; i < NUM_LEGS; i ++){
    servos[i].write(angles[i]);
  }
}

// ^ 50, < -50, > 50
void setDirections(int x, int y){
  // Home
  if (x == 0 && y == 0)
    servosWritePresets(preset_angles[0]);
    
  // North
  else if (x < 0 && y == 0) 
    servosWritePresets(preset_angles[1]);
  // South
  else if (x > 0 && y == 0) 
    servosWritePresets(preset_angles[2]);
  // East
  else if (x == 0 && y < 0) 
    servosWritePresets(preset_angles[3]);
  // West
  else if (x == 0 && y > 0) 
    servosWritePresets(preset_angles[4]);

  // North-East
  else if (x < 0 && y < 0) 
    servosWritePresets(preset_angles[5]);
  // North-West
  else if (x < 0 && y > 0) 
    servosWritePresets(preset_angles[6]);
  // South-East
  else if (x > 0 && y < 0) 
    servosWritePresets(preset_angles[7]);
  // South-West
  else if (x > 0 && y > 0) 
    servosWritePresets(preset_angles[8]);
}

