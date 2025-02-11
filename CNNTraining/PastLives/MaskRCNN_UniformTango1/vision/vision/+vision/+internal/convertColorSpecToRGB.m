% vision.internal.convertColorSpecToRGB Convert MATLAB ColorSpec to RGB triplet
%
% This is a codegenable alternative to images.internal.ColorSpecToRGBConverter.
%
% Example
% -------
% rgbFill = vision.internal.convertColorSpecToRGB('r')

%  Copyright 2022 The MathWorks, Inc.

%#codegen

function rgb = convertColorSpecToRGB(spec, varargin)
   
    narginchk(1,4)

    if nargin > 1
        outputClass = varargin{1};
    else
        outputClass = 'double';
    end
    
    if nargin > 2
        functionName = varargin{2};
    else
        functionName = mfilename;
    end
    
    if nargin == 3
        argName = varargin{3};
    else
        argName = 'Color';
    end
    
    rgb = colorRGBValue(spec, outputClass, functionName, argName);
end

%--------------------------------------------------------------------------
function outColor = colorRGBValue(inColor, inpClass, functionName, argName)
    
    if isnumeric(inColor)
        outColor = cast(inColor, inpClass);
    else
        if iscell(inColor)
            colorCell = inColor;
        else
            colorCell = {inColor};
        end
        
        numColors = length(colorCell);
        outColor = zeros(numColors, 3, inpClass);
        
        for ii = 1:numColors
            supportedColorStr = {'blue', 'green', 'red', 'cyan', 'magenta', 'yellow',...
                'black', 'white','b','k'};
            
            % validatestring to get partial matches.
            colorString = validatestring(colorCell{ii}, supportedColorStr, ...
                functionName, argName);
            idx = strcmp(colorString, supportedColorStr);
    
            % http://www.mathworks.com/help/techdoc/ref/colorspec.html
            colorValuesFloat = [0 0 1; 0 1 0; 1 0 0; 0 1 1; 1 0 1; 1 1 0; 0 0 0; 1 1 1; 0 0 1; 0 0 0];
            switch inpClass
                case {'double', 'single'}
                    outColor(ii, :) = colorValuesFloat(idx, :);
                case {'uint8', 'uint16'}
                    colorValuesUint = colorValuesFloat*double(intmax(inpClass));
                    outColor(ii, :) = colorValuesUint(idx, :);
                case 'int16'
                    colorValuesInt16 = im2int16(colorValuesFloat);
                    outColor(ii, :) = colorValuesInt16(idx, :);
            end
        end
    end
end