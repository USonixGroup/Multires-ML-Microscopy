function checkFeatures(features, fileName, varName)
% checkFeatures Function to validate image features.

% Copyright 2019-2020 The MathWorks, Inc.

%#codegen

validateattributes(features, {'logical', 'int8', 'uint8', 'int16', ...
    'uint16', 'int32', 'uint32', 'single', 'double', 'binaryFeatures'}, ...
    {'2d', 'nonsparse', 'real'}, fileName, varName);
