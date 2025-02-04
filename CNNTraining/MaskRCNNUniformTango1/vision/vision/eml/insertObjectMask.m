function Bout = insertObjectMask(A, maskstack, varargin) %#codegen
%

%   Copyright 2020-2022 The MathWorks, Inc.

% Codegen limitations:
% - 'LineWidth' should be a compile time constant
% - 'Color' and 'LineColor' only accept numeric inputs
% - Portable codegen only

% Store image class
imgClass = class(A);

% Validate
A = validateAndManageImage(A);
validateMaskstack(maskstack, A);

if isempty(maskstack)
    if ismatrix(A)
        B = repmat(A,[1 1 3]);
    else
        B = A;
    end
    switch(imgClass)
        case 'uint8'
          Bout = im2uint8(B);
        case 'double'
          Bout = im2double(B);
        case 'uint16'
          Bout = im2uint16(B);
        case 'int16'
          Bout = im2int16(B);
        otherwise
          % Single case.
          Bout = B;
    end
    return
end

numMasks = size(maskstack,3);
[color, opacity, lineColor, lineWidth, lineOpacity] = parseNVPairs(numMasks, varargin{:});

L = flattenMaskstack(maskstack);

B = A;
colormap = color;
alphamap = opacity;

if lineOpacity(1) ~= 0
    maxLabel = numMasks;
    boundaryLabel = computeBoundaryLabel(L, lineWidth);
    
    idx = boundaryLabel ~= 0;
    boundaryLabel(idx) = boundaryLabel(idx) + maxLabel;
    L(idx) = boundaryLabel(idx);
    
    colormap = [colormap; lineColor];
    alphamap = [alphamap lineOpacity];
end

B = overlayImage(A, B, L, colormap, alphamap);

% Set output type to be the same as input image type.
switch(imgClass)
    case 'uint8'
      Bout = im2uint8(B);
    case 'double'
      Bout = im2double(B);
    case 'uint16'
      Bout = im2uint16(B);
    case 'int16'
      Bout = im2int16(B);
    otherwise
      % Single case.
      Bout = B;
end

end

function B = overlayImage(A, B, L, color, opacity)

planeOffset = prod(size(A,[1,2]));

pixelIndexList = label2idx(L);
oneMinusOpacity = 1 - opacity;

for i = 1:length(pixelIndexList)
    
    B(pixelIndexList{i}) = oneMinusOpacity(i) * A(pixelIndexList{i}) + opacity(i)*color(i,1);
    B(pixelIndexList{i}+planeOffset) = oneMinusOpacity(i) * A(pixelIndexList{i}+planeOffset) + opacity(i)*color(i,2);
    B(pixelIndexList{i}+2*planeOffset) = oneMinusOpacity(i) * A(pixelIndexList{i}+2*planeOffset) + opacity(i)*color(i,3);
    
end

end

function L = flattenMaskstack(maskstack)

% For each pixel break as soon as the first true value is found in the Z
% dimension as the first mask gets preference when more than one mask
% occupy the same pixel

numRows = size(maskstack,1);
numCols = size(maskstack,2);
numPages = size(maskstack,3);

L = zeros(numRows, numCols);

parfor col = 1:numCols
    for row = 1:numRows
        for page = 1:numPages
            
            if maskstack(row,col,page)
                L(row,col) = page;
                break
            end
            
        end
    end
end

end

function boundaryL = computeBoundaryLabel(L, lineWidth)

nhoodSize = 2 * lineWidth + 1;
nhoodArray = ones(nhoodSize);

nhood = coder.const(nhoodArray);
BW = (imdilate(L,nhood) > L) | (imerode(L,nhood) < L);
boundaryL = L;
boundaryL(imcomplement(BW)) = 0;

end

function Aout = validateAndManageImage(img)

isGrayOrRGBImage = ismatrix(img) || ((ndims(img) == 3) && (size(img,3) == 3));
coder.internal.errorIf(~isGrayOrRGBImage, 'vision:dims:imageNot2DorRGB');

validateattributes(img, {'single', 'double', 'uint8', 'uint16', 'int16'},...
    {'real','nonempty','nonsparse'}, mfilename, 'A', 1);

A = im2single(img);
if ismatrix(A)
    Aout = repmat(A,[1 1 3]);
else
    Aout  = A;
end

end

function validateMaskstack(maskstack, A)

if isempty(maskstack)
    return
end

validSize = [size(A,[1,2]),NaN];

validateattributes(maskstack, {'logical'}, {'nonsparse', 'size',validSize},...
    mfilename, 'MASKSTACK',2);

end

function [colorOut, opacityOut, lineColorOut, lineWidthOut, lineOpacityOut] = parseNVPairs(numMasks, varargin)

% default Color is lines(1) which is [0, 0.4470, 0.7410]
% LineColor default is same as Color
defaults.Color       = [0, 0.4470, 0.7410];  % Old NV name
defaults.MaskColor   = [0, 0.4470, 0.7410];  % Preferred NV name 
defaults.Opacity     = 0.6;
defaults.LineWidth   = 1;
defaults.LineOpacity = 1;

params = struct(...
    'Color',       uint32(0),...
    'MaskColor',   uint32(0),...    
    'Opacity',     uint32(0),...
    'LineColor',   uint32(0),...
    'LineWidth',   uint32(0),...
    'LineOpacity', uint32(0));

optionsParams = struct(...
    'CaseSensitivity',false, ...
    'StructExpand',   true, ...
    'PartialMatching',true);

parser = eml_parse_parameter_inputs(params, optionsParams, varargin{:});

% Handle backward compatible color parsing
wasOldUsed = parser.Color;
wasNewUsed = parser.MaskColor;
areBothUsed = wasOldUsed && wasNewUsed;
coder.internal.errorIf(areBothUsed, 'vision:obsolete:twoColorParams', 'Color', 'MaskColor');

if wasOldUsed
    color = eml_get_parameter_value(parser.Color, defaults.Color, varargin{:});
else
    color = eml_get_parameter_value(parser.MaskColor, defaults.MaskColor, varargin{:});
end 

opacity = eml_get_parameter_value(parser.Opacity, defaults.Opacity, varargin{:});
lineWidth = eml_get_parameter_value(parser.LineWidth, defaults.LineWidth, varargin{:});
lineOpacity = eml_get_parameter_value(parser.LineOpacity, defaults.LineOpacity, varargin{:});
% default value foe LineColor should be same value as Color input
lineColor = eml_get_parameter_value(parser.LineColor, color, varargin{:});

colorOut = validateAndManageColor(color, numMasks);
opacityOut = validateAndManageOpacity(opacity, numMasks, 'Opacity');
lineColorOut = validateAndManageLineColor(lineColor, numMasks);
lineWidthOut = validateAndManageLineWidth(lineWidth);
lineOpacityOut = validateAndManageOpacity(lineOpacity, numMasks, 'LineOpacity');

end

function colorOut = validateAndManageColor(colorIn, numMasks)

% For codegen, color can only be numeric

validateattributes(colorIn, {'double','single','uint8','uint16','int16'},...
    {'nonsparse', 'real', 'finite','real', 'ncols',3}, mfilename, 'Color');

if isrow(colorIn)
    colorInSingle = im2single(colorIn);
    colorOut = repmat(colorInSingle, numMasks, 1);
    
elseif size(colorIn,1) == numMasks
    colorOut = im2single(colorIn);
    
else
    coder.internal.errorIf(true, 'vision:insertObjectMask:numValuesMustMatchNumMasks','Color');
end

end

function lineColor = validateAndManageLineColor(lineColorIn, numMasks)

% For codegen, color can only be numeric

validateattributes(lineColorIn, {'double','single','uint8','uint16','int16'},...
    {'nonsparse', 'finite','real', 'ncols',3}, mfilename, 'LineColor');

lineColor = zeros(numMasks,3, 'single');
if isrow(lineColorIn)
    lineColorInSingle = im2single(lineColorIn);
    lineColor = repmat(lineColorInSingle, numMasks, 1);
    
elseif size(lineColorIn,1) == numMasks
    lineColor = im2single(lineColorIn);
    
else
    coder.internal.errorIf(true,'vision:insertObjectMask:numValuesMustMatchNumMasks','LineColor');
end

end

function opacity = validateAndManageOpacity(opacityIn, numMasks, paramName)

validateattributes(opacityIn, {'numeric'}, {'real', 'nonsparse', 'nonempty', 'finite',...
    'scalar', '>=',0, '<=',1}, mfilename, paramName)
opacity = single(opacityIn);
opacity = repmat(opacity, [1,numMasks]);

end

function lineWidth = validateAndManageLineWidth(lineWidthIn)

eml_invariant(eml_is_const(lineWidthIn),...
    eml_message('MATLAB:images:validate:codegenInputNotConst','LineWidth'),...
    'IfNotConst','Fail');

validateattributes(lineWidthIn, {'numeric'}, {'real', 'nonsparse', 'nonempty', 'finite',...
    'scalar', 'integer', 'positive'}, mfilename, 'LineWidth');
lineWidth = single(lineWidthIn);

end
