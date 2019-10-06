% contains all the functions needed to calculate servo angles;
NUM_LEGS = 6;

DistanceToLegsFromOrigin =[ 
  3.327,  6.467, 0;
  7.265, -0.350, 0;
  3.935, -6.117, 0;
  -3.935,-6.117, 0;
  -7.265,-0.350, 0;
  -3.327, 6.467, 0;
];


function [RotMat] = fillRotationMatrix(pitch,roll,yaw)
    RotMat(0,0) = cos(yaw)*cos(pitch);
    RotMat(0,1) = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll);
    RotMat(0,2) = sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
    RotMat(1,0) = sin(yaw)*cos(pitch);
    RotMat(1,1) = cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
    RotMat(1,2) = -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
    RotMat(2,0) = -sin(pitch);
    RotMat(2,1) = cos(pitch)*sin(roll);
    RotMat(2,2) = cos(pitch)*cos(roll);
end
% T is a column vector
function [T] = computeTVector(roll,yaw, HEIGHT)
    T(3) = HEIGHT * cos(yaw);
    xyLength = HEIGHT * sin(yaw);
    T(2) = xyLength * sin(roll);
    T(1) = xyLength * cos(roll);
end

function [P] =  computeResultantRotatedPVectorForLeg(RotMat,DistanceToLegsFromOrigin)
    P = RotMat* DistanceToLegsFromOrigin.';
end

function [LegVectors] = computeVectorForLeg(T, P, DistanceToLegsFromOrigin)
    LegVectors = T + P - DistanceToLegsFromOrigin.';
end

function [percentLegHeight] = getPercentLegHeight(LegVectors,T)
    percentLegHeight = LegVectors(:,3) / T(3);
end
