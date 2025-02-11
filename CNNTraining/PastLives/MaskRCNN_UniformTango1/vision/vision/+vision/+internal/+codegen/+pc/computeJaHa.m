function [R,Ja,Ha] = computeJaHa(pose)
% Compute Rotation, Jacobian and Hessian matrices coefficients

% This function computes the Jacobian and Hessian coefficients given 
% the pose matrix. The last three values determine the rotation angles 
% in X,Y & Z directions. Rotation angles are in radians.
% The rotation sequence is 'XYZ', where the
% order of rotation angles is X Axis Rotation, Y Axis Rotation, and Z
% Axis Rotation, i.e., R = Rx * Ry * Rz.
% poseMat(4:6) = [thetaX, thetaY, thetaZ]
% These are used to compute the first-order derivative (Jacobian) 
% coefficients and the second order (Hessian) coefficients.

% [1] Martin Magnusson, The three-Dimensional Normal-Distributions
% Transform - an Efficient Representation for Registration, Surface
% Analysis, and Loop Detection, Thesis, 2013

% $matlabroot/toolbox/vision/builtins/src/vision/visionNDTComputeScoreDerivatives.cpp
% $matlabroot/toolbox/vision/builtins/src/vision/include/ndtUtils.hpp

% Copyright 2021 The MathWorks, Inc.

%#codegen

R = vision.internal.eul.eulerAngleToRotationMatrix(pose(4:6));

[Ja,Ha] = computeAngleDerivatives(pose);
end

function [Ja,Ha] = computeAngleDerivatives(pose)
% Compute the Jacobian and Hessian coefficients given the angle position
% information. pose(4:6) = [thetaX, thetaY, thetaZ]
% The Jacobian and Hessian are computed using the equations 6.18, 6.19 and
% 6.20, 6.21 respectively from [1].

% Define angleThreshold as 0.1745 radians (10 degrees)
angleThreshold = 0.1745;

% Extract rotation (theta) in each axis.
thetaX = pose(4);
thetaY = pose(5);
thetaZ = pose(6);

% For angles less than angleThreshold (0.1745 radians), sin(x) = x & cos(x) = x
% approximately.
if abs(thetaX) < angleThreshold
    thetaX = 0;
end
if abs(thetaY) < angleThreshold
    thetaY = 0;
end
if abs(thetaZ) < angleThreshold
    thetaZ = 0;
end

% Pre-compute sine and cosine values of the angles.
cx = cos(thetaX);
sx = sin(thetaX);
cy = cos(thetaY);
sy = sin(thetaY);
cz = cos(thetaZ);
sz = sin(thetaZ);

% The Jacobian computed here is a 3x3 matrix containing only the necessary
% coefficients in each axis.
% Jax =  | 0  cx fx |   Jay =  | 0  cy fy |   Jaz =  | 0  cz fz |
%        | ax dx gx |          | ay dy gy |          | az dz gz |
%        | bx ex hx |          | by ey hy |          | bz ez hz |
Ja = coder.nullcopy(zeros(3, 3, 3, 'like', pose));
Ja(1,1,1) = 0.0;
Ja(1,2,1) = -sy*cz;
Ja(1,3,1) = -cy*sz;
Ja(2,1,1) = -sx*sz + cx*sy*cz;
Ja(2,2,1) = sx*cy*cz;
Ja(2,3,1) = cx*cz - sx*sy*sz;
Ja(3,1,1) = cx*sz + sx*sy*cz;
Ja(3,2,1) = -cx*cy*cz;
Ja(3,3,1) = sx*cz + cx*sy*sz;

Ja(1,1,2) = 0.0;
Ja(1,2,2) = sy*sz;
Ja(1,3,2) = -cy*cz;
Ja(2,1,2) = -sx*cz - cx*sy*sz;
Ja(2,2,2) = -sx*cy*sz;
Ja(2,3,2) = -cx*sz - sx*sy*cz;
Ja(3,1,2) = cx*cz - sx*sy*sz;
Ja(3,2,2) = cx*cy*sz;
Ja(3,3,2) = -sx*sz + cx*sy*cz;

Ja(1,1,3) = 0.0;
Ja(1,2,3) = cy;
Ja(1,3,3) = 0.0;
Ja(2,1,3) = -cx*cy;
Ja(2,2,3) = sx*sy;
Ja(2,3,3) = 0.0;
Ja(3,1,3) = -sx*cy;
Ja(3,2,3) = -cx*sy;
Ja(3,3,3) = 0.0;

% The Hessian computed here is a 3x3x6 matrix where the six channels
% represent the coefficents values of the Hessian.
% Actual Hessian =  | 0 0 0 0 0 0 |
%                   | 0 0 0 0 0 0 |
%                   | 0 0 0 0 0 0 |
%                   | 0 0 0 a b c |
%                   | 0 0 0 b d e |
%                   | 0 0 0 c e f |
% where, a = Ha(:,:,1)*t, b = Ha(:,:,2)*t, c = Ha(:,:,3)*t, d = Ha(:,:,4)*t,
% e = Ha(:,:,5)*t, f = Ha(:,:,6)*t and t = [x,y,z] is the transformed
% coordinate
Ha = coder.nullcopy(zeros(3, 3, 6, 'like', pose));

% Coeff a
Ha(1,1,1) = 0.0;
Ha(1,2,1) = 0.0;
Ha(1,3,1) = 0.0;
Ha(2,1,1) = -cx*sz - sx*sy*cz;
Ha(2,2,1) = -cx*cz + sx*sy*sz;
Ha(2,3,1) = sx*cy;
Ha(3,1,1) = -sx*sz + cx*sy*cz;
Ha(3,2,1) = -sx*cz - cx*sy*sz;
Ha(3,3,1) = -cx*cy;

% Coeff b
Ha(1,1,2) = 0;
Ha(1,2,2) = 0;
Ha(1,3,2) = 0;
Ha(2,1,2) = cx*cy*cz;
Ha(2,2,2) = -cx*cy*sz;
Ha(2,3,2) = cx*sy;
Ha(3,1,2) = sx*cy*cz;
Ha(3,2,2) = -sx*cy*sz;
Ha(3,3,2) = sx*sy;

% Coeff c
Ha(1,1,3) = 0.0;
Ha(1,2,3) = 0.0;
Ha(1,3,3) = 0.0;
Ha(2,1,3) = -sx*cz - cx*sy*sz;
Ha(2,2,3) = sx*sz - cx*sy*cz;
Ha(2,3,3) = 0.0;
Ha(3,1,3) = cx*cz - sx*sy*sz;
Ha(3,2,3) = -sx*sy*cz - cx*sz;
Ha(3,3,3) = 0.0;

% Coeff d
Ha(1,1,4) = -cy*cz;
Ha(1,2,4) = cy*sz;
Ha(1,3,4) = -sy;
Ha(2,1,4) = -sx*sy*cz;
Ha(2,2,4) = sx*sy*sz;
Ha(2,3,4) = sx*cy;
Ha(3,1,4) = cx*sy*cz;
Ha(3,2,4) = -cx*sy*sz;
Ha(3,3,4) = -cx*cy;

% Coeff e
Ha(1,1,5) = sy*sz;
Ha(1,2,5) = sy*cz;
Ha(1,3,5) = 0.0;
Ha(2,1,5) = -sx*cy*sz;
Ha(2,2,5) = -sx*cy*cz;
Ha(2,3,5) = 0.0;
Ha(3,1,5) = cx*cy*sz;
Ha(3,2,5) = cx*cy*cz;
Ha(3,3,5) = 0.0;

% Coeff f
Ha(1,1,6) = -cy*cz;
Ha(1,2,6) = cy*sz;
Ha(1,3,6) = 0.0;
Ha(2,1,6) = -cx*sz - sx*sy*cz;
Ha(2,2,6) = -cx*cz + sx*sy*sz;
Ha(2,3,6) = 0.0;
Ha(3,1,6) = -sx*sz + cx*sy*cz;
Ha(3,2,6) = -cx*sy*sz - sx*cz;
Ha(3,3,6) = 0.0;

end
