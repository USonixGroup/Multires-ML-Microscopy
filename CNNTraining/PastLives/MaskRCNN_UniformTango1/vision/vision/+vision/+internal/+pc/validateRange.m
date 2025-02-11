%vision.internal.pc.validateRange Validate radialRange

% Copyright 2020 The MathWorks, Inc.

%#codegen
function validateRange(radialRange)
validateattributes(radialRange, {'single','double'}, {'real','nonnegative','numel',2, 'nonnan', 'increasing'}, '', 'RadialRange');
validateattributes(radialRange(1), {class(radialRange)}, {'finite'}, '', 'first element of RadialRange');
end