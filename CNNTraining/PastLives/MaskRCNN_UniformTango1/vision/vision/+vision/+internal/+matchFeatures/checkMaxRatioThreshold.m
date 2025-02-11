function threshold = checkMaxRatioThreshold(threshold, fileName)
%checkMaxRatioThreshold Check the maximum ratio parameter in feature matching

%   Copyright 2020 The MathWorks, Inc.

%#codegen

validateattributes(threshold, {'numeric'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'positive', 'scalar', '<=', 1.0}, ...
    fileName, 'MaxRatio');