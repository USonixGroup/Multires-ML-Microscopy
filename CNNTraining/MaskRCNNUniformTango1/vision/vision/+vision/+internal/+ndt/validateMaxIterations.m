%vision.internal.ndt.validateMaxIterations Validate 'MaxIterations' parameter

% Copyright 2020 The MathWorks, Inc.

%#codegen
function validateMaxIterations(in)

validateattributes(in, {'numeric'}, {'scalar','integer','positive'},...
    '', 'MaxIterations');
end