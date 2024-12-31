function eul = rotationMatrixToEulerAngle(R, seq)
%rotationMatrixToEulerAngle Convert rotation matrix to Euler angles
%   eul = rotationMatrixToEulerAngle(R, seq) converts a 3D rotation matrix,
%   R, into the corresponding Euler angles, EUL. R is a 3-by-3 rotation
%   matrix. The output, EUL, is a 1-by-3 vector of Euler rotation angles.
%   Rotation angles are in radians. The rotation sequence is 'XYZ' by
%   default, where the order of rotation angles is X Axis Rotation, Y Axis
%   Rotation, and Z Axis Rotation, i.e., R = Rx * Ry * Rz. 'ZYX' rotation
%   order is also supported.

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen

% Only supported euler angle sequences are XYZ and ZYX.
isXYZ = nargin<2 || strcmpi(seq, 'XYZ');

vision.internal.inputValidation.validateRotationMatrix(R, 'rotationMatrixToEulerAngle', 'R');

% Set indices for accessing rotation matrix
if isXYZ
    % Handle X-Y-Z rotation order

    i = 3;
    j = 2;
    k = 1;
else
    % Handle Z-Y-X rotation order

    i = 1;
    j = 2;
    k = 3;
end

% Find special cases of rotation matrix values that correspond to Euler
% angle singularities.
sy = sqrt(R(i,i)*R(i,i) + R(j,i)*R(j,i));
singular = sy < 10 * eps(class(R));

% Calculate Euler angles
if ~singular
    eul = [atan2(R(k,j), R(k,k)), atan2(-R(k,i), sy), atan2(R(j,i), R(i,i))];
else
    % Singular matrices need special treatment
    eul = [atan2(-R(j,k), R(j,j)), atan2(-R(k,i), sy), 0];
end


% Invert the result
if isXYZ
    eul = -eul;
end

% Swap the X and Z columns
eul(:,[1,3]) = eul(:,[3,1]);
