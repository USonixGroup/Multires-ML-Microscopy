%vision.internal.ndt.validateLogical Validate logical

% Copyright 2020 The MathWorks, Inc.

%#codegen
function validateLogical(in)

validateattributes(in, {'numeric', 'logical'}, {'scalar', 'binary'}, ...
    '', 'Verbose');
end