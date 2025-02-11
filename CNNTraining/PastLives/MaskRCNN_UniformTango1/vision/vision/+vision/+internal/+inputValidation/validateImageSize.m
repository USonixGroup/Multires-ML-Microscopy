function validateImageSize(imageSize, filename)
% Validation of image size inputs in camera intrinsics container objects.

% Copyright 2023 The MathWorks, Inc.

%#codegen

    validateattributes(imageSize, {'double', 'single'}, ...
        {'vector','real', 'nonsparse','numel', 2, 'integer', 'positive'}, ...
        filename, 'imageSize');
end  