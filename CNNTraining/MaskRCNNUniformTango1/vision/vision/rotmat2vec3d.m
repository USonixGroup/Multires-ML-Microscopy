function rotationVector = rotmat2vec3d(rotationMatrix)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

validateattributes(rotationMatrix, {'single', 'double'}, ...
    {'real', 'nonsparse', 'size', [3 3]}, mfilename, 'rotationMatrix');

rotationVector = vision.internal.calibration.rodriguesMatrixToVector(rotationMatrix)';
