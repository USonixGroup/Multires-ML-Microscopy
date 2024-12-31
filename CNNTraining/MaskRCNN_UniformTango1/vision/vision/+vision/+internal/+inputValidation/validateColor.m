function colors = validateColor(colors,paramName)
% Validate and manage colors. Return colors as floating point RGB triplets.

% Copyright 2022 The MathWorks, Inc.

validateattributes(colors,{'numeric','string','char'},{'nonempty'},paramName,mfilename);

if isnumeric(colors)
    iValidateNumericColor(colors);
    
    % Numeric color input can be double, single, uint8, uint16, or
    % int16. Convert to values in range [0 1].
    colors = im2double(colors);
    
elseif isstring(colors) || ischar(colors) 
    colors = string(colors);
    if numel(colors) > 1
        error(message('vision:validation:InvalidColor'));
    end
    colors = iConvertToNumericColor(colors);
else
    error(message('vision:validation:InvalidColor'));
end

end

%--------------------------------------------------------------------------
function iValidateNumericColor(color)
attrb = {'nonnegative','finite','size',[1 3],'real','nonsparse'};

if isfloat(color)
    % Floating point values must be within [0 1].
    attrb = [attrb '<=', 1];
end
validateattributes(color,{'double','single','uint8','uint16','int16'},...
    attrb,'Color',mfilename);
end

%--------------------------------------------------------------------------
function color = iConvertToNumericColor(colorString)
persistent colorSpecConverter
if isempty(colorSpecConverter)
    colorSpecConverter = images.internal.ColorSpecToRGBConverter;
end
color = colorSpecConverter.convertColorSpec(colorString(numel(colorString)));
end