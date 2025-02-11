function val = checkAcceleration(val, callername)
%

%   Copyright 2019-2020 The MathWorks, Inc.

val = validatestring(val, {'auto', 'mex', 'none'}, ...
    callername, 'Acceleration');
end
