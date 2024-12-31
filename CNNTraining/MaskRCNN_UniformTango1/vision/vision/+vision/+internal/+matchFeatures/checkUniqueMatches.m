function checkUniqueMatches(uniqueMatches, fileName)
%checkUniqueMatches Check the unique match flag

%   Copyright 2020 The MathWorks, Inc.

%#codegen

validateattributes(uniqueMatches, {'logical', 'numeric'}, ...
    {'nonempty', 'scalar', 'real', 'nonnan', 'nonsparse'}, ...
    fileName, 'Unique');
