#include <iostream>
#include <cmath>

using namespace std;

const float PI = 3.14159265;

const float Height = 5;
const float HexDistanceToLeg = 3;
const int NumLegs = 6;
float DistanceToLegs[NumLegs][3];

float RotationMatrix[3][3];
float T[3];
float P[3];
float LegLengths[6];

//roll-x w pitch-y theta yaw-z tri
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
    T[2] = Height * cos(yaw);
    float xyLength = Height * sin(yaw);
    T[1] = xyLength * sin(roll);
    T[0] = xyLength * cos(roll);
}

void computeResultantRotatedPVectorForLeg(int i)
{
	P[0] = RotationMatrix[0][0] * DistanceToLegs[i][0] + RotationMatrix[0][1] * DistanceToLegs[i][1] + RotationMatrix[0][2] * DistanceToLegs[i][2];
	P[1] = RotationMatrix[1][0] * DistanceToLegs[i][0] + RotationMatrix[1][1] * DistanceToLegs[i][1] + RotationMatrix[1][2] * DistanceToLegs[i][2];
	P[2] = RotationMatrix[2][0] * DistanceToLegs[i][0] + RotationMatrix[2][1] * DistanceToLegs[i][1] + RotationMatrix[2][2] * DistanceToLegs[i][2];
}

int main()
{
    fillRotationMatrix(0,0,0);
    
    for(int i = 0; i < NumLegs; ++i)
    {
    	float angle = ( PI / 6 ) * i;
    	DistanceToLegs[i][0] = HexDistanceToLeg * cos(angle);
    	DistanceToLegs[i][1] = HexDistanceToLeg * sin(angle);
    	DistanceToLegs[i][2] = 0;
	}
    
    float yaw = PI/4;
    float pitch = 0;
    float roll = PI/4;
    
    fillRotationMatrix(roll,pitch,yaw);
    computeTVector(roll, pitch, yaw);
    
    for(int i=0;i<3;++i)
    {
    	for(int j=0;j<3;++j)
    	{
    		cout << RotationMatrix[i][j] << " ";
		}
		cout << endl;
	}
	
	for(int i=0;i<3;++i)
    {
    	cout << T[i] << " ";
	}
    return 0;
}
