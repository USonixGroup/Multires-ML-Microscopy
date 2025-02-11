%==========================================================================
% This function handles changes in behavior of color specification and
% maintains backwards compatibility for insertX functions.
%
% Note: There is also a code generation version of this function in +codegen
%       folder.
%==========================================================================

% Copyright 2022 The MathWorks, Inc.

function out = parseInserterColor(oldName, ...
    newName, parser, fcnName, inpClass)

wasOldUsed = ~any(parser.UsingDefaults == oldName);
wasNewUsed = ~any(parser.UsingDefaults == newName);

if wasOldUsed && wasNewUsed % take care of backward compatibility
    error(message("vision:obsolete:twoColorParams", oldName, newName));
elseif wasOldUsed
    out = checkColor(parser.Results.(oldName), oldName, ...
        fcnName, inpClass, wasNewUsed);
else
    out = checkColor(parser.Results.(newName), newName, ...
        fcnName, inpClass, wasNewUsed);
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

   if ischar(color) || isstring(color)
       colorCell = cellstr(color);
   else
       validateattributes(color, {'cell'}, {}, fcnName, paramName);
       colorCell = color;
   end
   supportedColorStr = {'blue','green','red','cyan','magenta', ...
       'yellow','black','white','b','k'};
   numCells = length(colorCell);
   colorOut = cell(1, numCells);
   for ii=1:numCells
       colorOut{ii} =  validatestring(colorCell{ii}, ...
           supportedColorStr, fcnName, paramName);
   end

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
