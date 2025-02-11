function validateFocalLength(focalLength, filename)
% Validation of focal length in the camera intrinsic matrix

% Copyright 2023 The MathWorks, Inc.

%#codegen

    validateattributes(focalLength, {'double', 'single'}, ...
        {'vector','real', 'nonsparse', 'finite', 'positive'}, ...
        filename, 'focalLength');
    
    ne = numel(focalLength);            
    coder.internal.errorIf(ne ~= 1 &&  ne ~= 2, ...
        'vision:dims:twoElementVector', 'focalLength');
end