function validateTranslationVector(t, fileName, varName)
%#codegen

%   Copyright 2014-2020 The MathWorks, Inc.

validateattributes(t, {'numeric'}, ...
    {'finite', 'vector', 'real', 'nonsparse', 'numel', 3}, fileName, varName);    
