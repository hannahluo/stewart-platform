//roll-x w pitch-y theta yaw-z tri
void fillRotationMatrix(float roll, float pitch, float yaw, float[][] rotationMatrix)
{
    rotationMatrix[0][0] = cos(yaw)*cos(pitch);
    rotationMatrix[0][1] = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll);
    rotationMatrix[0][2] = sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
    rotationMatrix[1][0] = sin(yaw)*cos(pitch);
    rotationMatrix[1][1] = cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll)
    rotationMatrix[1][2] = -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
    rotationMatrix[2][0] = -sin(pitch);
    rotationMatrix[2][1] = cos(pitch)*sin(roll);
    rotationMatrix[2][2] = cos(pitch)*cos(roll);
}

void computeTVector(float roll, float pitch, float yaw, float Height, int[] T)
{
    T[2] = Height * cos(yaw);
    float xyLength = Height * sin(yaw);
    //T[1] = xyLength * cos()
}

int main()
{
    float rotationMatrix[][] = new float[3][3];
    float T[] = new int[3];
    float Height = 5;
    fillRotationMatrix(0,0,0,rotationMatrix);
    
    float yaw = 0;
    float pitch = 0;
    float roll = 0;  
    
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

