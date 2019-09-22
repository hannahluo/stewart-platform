#include <iostream>
#include <cmath>

using namespace std;

const float PI = 3.14159265;

const float Height = 5;
const float HexDistanceToLeg = 3;
const int NumLegs = 6;
float DistanceToLegsFromOrigin[NumLegs][3];

float RotationMatrix[3][3];
float T[3];
float P[3];
float LegVectors[NumLegs][3];
float Lengths[NumLegs];

void initDistanceToLegsFromOrigin()
{
	for(int i = 0; i < NumLegs; ++i)
    {
    	float angle = ( PI / 3 ) * i;
    	DistanceToLegsFromOrigin[i][0] = HexDistanceToLeg * cos(angle);
    	DistanceToLegsFromOrigin[i][1] = HexDistanceToLeg * sin(angle);
    	DistanceToLegsFromOrigin[i][2] = 0;
	}
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
    T[2] = Height * cos(yaw);
    float xyLength = Height * sin(yaw);
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
	
	//cout << endl << "    [" << aPos[0] << "," << aPos[1] << "," << aPos[2] << "] [" << bPos[0] << "," << bPos[1] << "," << bPos[2] << "]" << endl;
	
	return sqrt( (aPos[0] - bPos[0])*(aPos[0] - bPos[0]) + (aPos[1] - bPos[1])*(aPos[1] - bPos[1]) + (aPos[2] - bPos[2])*(aPos[2] - bPos[2]) );
}

void CalculateLegLengths(float roll, float pitch, float yaw, float surgeAngle, float swayAngle, float heaveAngle)
{
	fillRotationMatrix(roll,pitch,yaw);
	computeTVector(surgeAngle, swayAngle, heaveAngle);
	for(int i = 0; i < NumLegs; ++i)
    {
    	computeResultantRotatedPVectorForLeg(i);
    	computeVectorForLeg(i);
    }
}

int main()
{
    initDistanceToLegsFromOrigin();
    
    float yaw = PI/6;
    float pitch = 0;
    float roll = PI/4;
    float surgeAngle = 0;
    float swayAngle = 0;
    float heaveAngle = 0;
    
    CalculateLegLengths(roll, pitch, yaw, surgeAngle, swayAngle, heaveAngle);

    cout << "RotationMatrix --------------------------------------------------------" << endl;
    for(int i=0;i<3;++i)
    {
    	for(int j=0;j<3;++j)
    	{
    		cout << RotationMatrix[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl << "T Vector --------------------------------------------------------------" << endl;	
	for(int i=0;i<3;++i)
    {
    	cout << T[i] << ",";
	}
	cout << endl << "Leg Vectors -----------------------------------------------------------" << endl;
	for(int i=0;i<NumLegs;++i)
    {
    	cout <<"Leg " << i << ": " << LegVectors[i][0] << "," << LegVectors[i][1] << "," << LegVectors[i][0] << endl;
	}
	
	cout << endl << "Platform Sizes ---------------------------------------------------------" << endl;
	for(int i=0; i < NumLegs; ++i)
	{
		cout << computePlatformLengthBetweenLegs(i, (i + 1) % NumLegs) << endl;
	}
	
    return 0;
}
