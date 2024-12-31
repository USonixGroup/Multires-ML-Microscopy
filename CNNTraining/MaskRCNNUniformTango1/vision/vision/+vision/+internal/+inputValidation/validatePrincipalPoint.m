function validatePrincipalPoint(principalPoint, filename)
% Validation of principal point in the camera intrinsic matrix

% Copyright 2023 The MathWorks, Inc.

%#codegen

    validateattributes(principalPoint, {'double', 'single'}, ...
        {'vector','real', 'nonsparse','numel', 2, 'finite', 'positive'}, ...
        filename, 'principalPoint');
end