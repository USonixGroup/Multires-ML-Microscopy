function val = checkExecutionEnvironment(val, callername)
%

%   Copyright 2017-2020 The MathWorks, Inc.

val = validatestring(val, {'auto', 'cpu', 'gpu'}, ...
    callername, 'ExecutionEnvironment');
end
