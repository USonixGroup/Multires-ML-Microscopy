function validateStringScalar(str)
%validateStringScalar Verify that input is a string scalar.
%
% Note: Validation fails if the input is character vector.
%
% This function is for internal use only. It may be removed in the future.

%   Copyright 2022 The MathWorks, Inc.

    validateattributes(str, {'string'}, {'scalar'});
end