%vision.internal.ndt.validateTolerance Validate 'Tolerance' parameter

% Copyright 2020 The MathWorks, Inc.

%#codegen
function validateTolerance(in)

validateattributes(in, {'single', 'double'}, ...
    {'real','nonnegative','finite','numel', 2}, '', 'Tolerance');
end
