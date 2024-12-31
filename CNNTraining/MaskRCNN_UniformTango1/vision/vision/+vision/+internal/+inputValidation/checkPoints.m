function checkPoints(pointsIn, fileName, varName, allowGPUArrays)
% Checks if points are valid. Allows points to be any numeric type.

%   Copyright 2014-2020 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

if nargin ~= 4
    allowGPUArrays = false;
end

coder.internal.errorIf( ~isnumeric(pointsIn)...
    && ~vision.internal.inputValidation.isValidPointObj(pointsIn), ...
    'vision:points:ptsClassInvalid', varName);

if isnumeric(pointsIn)
    checkPointArray(pointsIn, fileName, varName, allowGPUArrays);    
else         
    checkPointObject(pointsIn, fileName, varName, allowGPUArrays);      
end

%--------------------------------------------------------------------------
function checkPointArray(value, fileName, varName, allowGPUArrays)

if ~allowGPUArrays && isa(value, 'gpuArray')
    str = class(value);
    cmd = sprintf('<a href="matlab:help %s/gather">gather</a>',str);
    error(message('vision:points:gpuArrayNotSupportedForPtArr',cmd));
end
checkPtsAttributes(value, fileName, varName);


%--------------------------------------------------------------------------
function checkPointObject(value, fileName, varName, allowGPUArrays)

if ~allowGPUArrays && isa(value.Location,'gpuArray')    
    str = class(value);
    cmd = sprintf('<a href="matlab:help %s/gather">gather</a>',str);
    error(message('vision:points:gpuArrayNotSupportedForPtObj',str,cmd));
end
checkPtsAttributes(value.Location, fileName, varName);


%--------------------------------------------------------------------------
function checkPtsAttributes(value, fileName, varName)
validateattributes(value, {'numeric'}, ...
    {'2d', 'nonsparse', 'real', 'size', [NaN, 2]}, fileName, varName);
