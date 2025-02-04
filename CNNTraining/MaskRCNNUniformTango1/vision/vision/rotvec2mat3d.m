function rotationMatrix = rotvec2mat3d(rotationVector)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

validateattributes(rotationVector, {'single', 'double'}, ...
    {'real', 'nonsparse', 'vector', 'numel', 3}, mfilename, 'rotationVector');

rotationMatrix = vision.internal.calibration.rodriguesVectorToMatrix(rotationVector);
