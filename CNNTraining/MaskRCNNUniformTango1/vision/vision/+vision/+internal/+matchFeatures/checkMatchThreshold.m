function threshold = checkMatchThreshold(threshold, fileName)
%checkMatchThreshold Check the match threshold for feature matching

%   Copyright 2020 The MathWorks, Inc.

%#codegen
validateattributes(threshold, {'numeric'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'positive', 'scalar', '<=', 100}, ...
    fileName, 'MatchThreshold');