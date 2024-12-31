%==========================================================================
% This function handles changes in behavior of color specification and
% maintains backwards compatibility for insertX functions.
%
% Note: There is also a simulation version of this function in +internal
%       folder.
%==========================================================================

% Copyright 2022 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>
function out = parseInserterColor(oldName, ...
    newName, cgparser, fcnName, inpClass, defaults, varargin)

wasOldUsed = cgparser.(oldName);
wasNewUsed = cgparser.(newName);

areBothUsed = wasOldUsed && wasNewUsed;
coder.internal.errorIf(areBothUsed, 'vision:obsolete:twoColorParams', oldName, newName);

if wasOldUsed
    val = (eml_get_parameter_value(cgparser.(oldName), ...
        defaults.(oldName), varargin{:}));
    out = checkColor(val, oldName, fcnName, inpClass, wasNewUsed);
else
    val = (eml_get_parameter_value(cgparser.(newName), ...
        defaults.(newName), varargin{:}));    
    out = checkColor(val, newName, fcnName, inpClass, wasNewUsed);
end 

%==========================================================================
function colorOut = checkColor(color, paramName, fcnName, inpClass, isNew)

% Validate color
if isnumeric(color) || islogical(color)
    % No objects allowed.
    vision.internal.inputValidation.validateNotObject(color, 'vision', 'color');
    
    if ~isempty(color)
        validateattributes(color, ...
            {'uint8','uint16','int16','double','single'},...
            {'real','nonsparse','nonnan', 'finite', '2d', 'size', [NaN 3]}, ...
            fcnName, paramName);
    end

    if isNew
        colorOut = convertToInputImageDataRange(color, inpClass);
    else
        colorOut = color;
    end
else
       % codegen works only for numeric color input
       coder.internal.errorIf(true, ['vision:' fcnName ':colorNotNumeric']);
end

%==========================================================================
function colorOut = convertToInputImageDataRange(color, inpClass)

switch (inpClass)
    case 'uint8'
        colorOut = im2uint8(color);
    case 'uint16'
        colorOut = im2uint16(color);
    case 'int16'
        colorOut = im2int16(color);
    case 'logical'
        colorOut = im2logical(color);
    case {'single', 'double'}
        colorOut = color;
    otherwise
        assert(true, "Unsupported data type conversion");
end
