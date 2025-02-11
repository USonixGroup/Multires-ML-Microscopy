function validateStringVector(str)
%validateStringVector Verify that input is a string vector
%
% Note: Validation fails if the input is character array or a cell array of
% character vectors.
%
% This function is for internal use only. It may be removed in the future.

%   Copyright 2022 The MathWorks, Inc.

    validateattributes(str, {'string'}, {'vector'});
end