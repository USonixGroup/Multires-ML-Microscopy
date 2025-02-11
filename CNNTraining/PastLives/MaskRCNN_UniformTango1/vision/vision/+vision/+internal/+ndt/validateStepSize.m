%vision.internal.ndt.validateStepSize Validate 'StepSize' parameter

% Copyright 2020 The MathWorks, Inc.

%#codegen
function validateStepSize(in)

validateattributes(in, {'single', 'double'}, {'real','scalar','positive'},...
    '', 'StepSize');
end