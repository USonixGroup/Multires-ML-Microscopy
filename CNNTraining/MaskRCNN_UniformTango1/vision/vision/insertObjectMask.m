function RGB = insertObjectMask(I, maskstack, namedArgs)
%insertObjectMask Insert masks in image or video stream.
%
%   RGB = insertObjectMask(I, BW) returns a truecolor image RGB, in which
%   image I is fused with mask BW. I is an M-by-N grayscale or an
%   M-by-N-by-3 truecolor image. BW is an M-by-N logical matrix with width
%   and height same as I.
%
%   RGB = insertObjectMask(I, MASKSTACK) returns a truecolor image RGB, in
%   which image I is fused with masks in MASKSTACK. I is an M-by-N
%   grayscale or an M-by-N-by-3 truecolor image. MASKSTACK is an
%   M-by-N-by-P logical matrix with width and height same as I, where P
%   is total number of masks.
%
%   RGB = insertObjectMask(..., NAME1=VALUE1) specifies additional
%   name-value pair arguments described below.
%
%   Parameters include:
%
%   MaskColor           Specify the color of each mask as one of the
%                       following values:
%                          - P-by-3 matrix of RGB triplets, where P is
%                            total number of masks.
%                          - P-element vector of MATLAB ColorSpec names,
%                            where P is total number of masks.
%                          - A scalar MATLAB ColorSpec name or 1-by-3 RGB
%                            triplet to use the same color for all masks.
%
%                       Default: lines(1)
%
%   Opacity             Specify a scalar value in the range of 0 to 1
%                       defining the opacity of the mask over input image I.
%                          - Completely opaque       : 1
%                          - Completely transparent  : 0
%
%                       Default: 0.6
%
%   LineColor           Specify the color of border for each mask as one of
%                       the following values:
%
%                          - "auto" : LineColor is same as "MaskColor".
%                          - P-by-3 matrix of RGB triplets, where P is
%                            total number of masks.
%                          - P-element vector of MATLAB ColorSpec names,
%                            where P is total number of masks.
%                          - A scalar MATLAB ColorSpec name or 1-by-3 RGB
%                            triplet to use the same color for all masks.
%
%                       Default: "auto"
%
%   LineOpacity         Specify a scalar value in the range of 0 to 1
%                       defining the opacity of the border over input image
%                       I. Setting LineOpacity to 0 gives better performance.
%                          - Completely opaque       : 1
%                          - Completely transparent  : 0
%
%                       Default: 1
%
%   LineWidth           Specify a positive scalar value to define the line
%                       width of the mask border. LineWidth is specified in
%                       pixel units.
%
%                       Default: 1
%
%   Class Support
%   -------------
%   The input image I can be truecolor or grayscale image of type uint8,
%   uint16, int16, single or double. MASKSTACK is a stack of logical masks
%   of size M-by-N-by-P. Output RGB is truecolor image of the same type 
%   as input image I.
%
%   NOTES
%   -----
%   - When masks overlap and the same pixel is in more than one mask,
%   MASKSTACK(:,:,i) takes precedence over MASKSTACK(:,:,j) where i<j.
%
%   - To increase performance, set LineOpacity to 0 to disable drawing the
%   edges.
%
%  Example 1: Insert masks with white border to differentiate close objects
%  ------------------------------------------------------------------------
%  I = imread("visionteam1.jpg");
%  load("visionteam1Maskstack.mat");
%  RGB = insertObjectMask(I, maskstack, LineColor="white", LineWidth=2);
%  figure; imshow(RGB);
%
%
%  Example 2: Insert masks colored by labels
%  -----------------------------------------
%  % Read image.
%  I = imread("boats.png");
% 
%  % Load instance segmentation results
%  instances = load("boatInstances.mat");
%  
%  numClasses = length(unique(instances.labels));
% 
%  cmap = lines(numClasses);
%  % Assign color from colormap to each object label.
%  colors = label2rgb(instances.labels,cmap,OutputFormat="triplets");
% 
%  % Visualize overlayed masks
%  figure
%  overlayedmask = insertObjectMask(I, instances.masks, MaskColor=colors);
%  imshow(overlayedmask)
% 
%  Example 3: Insert masks with separate color for each mask
%  ---------------------------------------------------------
%  I = imread("visionteam1.jpg");
%  load("visionteam1Maskstack.mat");
%  numMasks = size(maskstack,3);
%  RGB = insertObjectMask(I, maskstack, MaskColor=lines(numMasks));
%  figure; imshow(RGB);
%
%   See also insertObjectAnnotation, insertShape, showShape, labeloverlay

%   Copyright 2020-2022 The MathWorks, Inc.

arguments
    I (:,:,:) {mustBe2DImage, mustBeNonsparse, mustBeNonempty, mustBeReal, validateNotObject(I,'I')}
    maskstack (:,:,:) {mustBeNonsparse, validateMaskstack(maskstack,I), validateNotObject(maskstack,'maskstack')}
end

arguments
    
    namedArgs.Color = lines(1);

    namedArgs.MaskColor = lines(1);
    
    namedArgs.Opacity (1,1) {mustBeNumeric, mustBeReal, mustBeNonempty, mustBeNonsparse, mustBeFinite,...
        mustBeGreaterThanOrEqual(namedArgs.Opacity,0), mustBeLessThanOrEqual(namedArgs.Opacity,1)} = 0.6;
    
    namedArgs.LineColor = 'auto';
    
    namedArgs.LineWidth (1,1) {mustBeNumeric, mustBeFinite, mustBeInteger, mustBeNonsparse, mustBePositive, ...
        vision.internal.inputValidation.validateNotObject(namedArgs.LineWidth,'vision','LineWidth')} = 1;
    
    namedArgs.LineOpacity (1,1) {mustBeNumeric, mustBeReal, mustBeNonempty, mustBeNonsparse, mustBeFinite,...
        mustBeGreaterThanOrEqual(namedArgs.LineOpacity,0), mustBeLessThanOrEqual(namedArgs.LineOpacity,1)} = 1;
    
end

% Store the image datatype.
imgClass = class(I);

if isempty(maskstack)
    if ismatrix(I)
        RGB = repmat(I,[1 1 3]);
    else
        RGB = I;
    end
    return
end

numMasks = size(maskstack, 3);
maxLabel = numMasks;

isDefaultColor     = isequal(namedArgs.Color, lines(1));
isDefaultMaskColor = isequal(namedArgs.MaskColor, lines(1));

if ~isDefaultColor && ~isDefaultMaskColor
    error(message("vision:obsolete:twoColorParams", "Color", "MaskColor"));
elseif isDefaultMaskColor
    namedArgs.Color = validateAndManageColor(namedArgs.Color, numMasks); % Still respect the old NV pair for backward compatibility
else
    namedArgs.Color = validateAndManageColor(namedArgs.MaskColor, numMasks);
end

namedArgs.Opacity      = validateAndManageOpacity(namedArgs.Opacity, numMasks, 'Opacity');
namedArgs.LineColor    = validateAndManageLineColor(namedArgs.LineColor, namedArgs.Color);
namedArgs.LineOpacity  = validateAndManageOpacity(namedArgs.LineOpacity, numMasks, 'LineOpacity');

if numMasks == 1
    % Performance: Special case for single mask
    L = zeros(size(maskstack,1), size(maskstack,2));
    L(maskstack) = 1;
else
    L = images.internal.builtins.maskstack2label(maskstack, 1:size(maskstack,3), 0);
end

colormap = namedArgs.Color;
alphamap = namedArgs.Opacity;


if namedArgs.LineOpacity ~= 0
    
    boundaryLabel = computeBoundaryLabel(L, namedArgs.LineWidth);
    
    idx = boundaryLabel ~= 0;
    boundaryLabel(idx) = boundaryLabel(idx) + maxLabel;
    L(idx) = boundaryLabel(idx);
    maxLabel = maxLabel * 2;
    
    colormap = [colormap; namedArgs.LineColor];
    alphamap = [alphamap namedArgs.LineOpacity];
end

% Computation is done in single. 
if isa(I,'single')
    RGB = images.internal.labeloverlayalgo(I, L, colormap, alphamap, 1:maxLabel);
else
    RGB = images.internal.labeloverlayalgo(im2single(I), L, colormap, alphamap, 1:maxLabel);
end

% Convert back to original input class.
switch(imgClass)
    case 'uint8'
      RGB = im2uint8(RGB);
    case 'double'
      RGB = im2double(RGB);
    case 'uint16'
      RGB = im2uint16(RGB);
    case 'int16'
      RGB = im2int16(RGB);
    otherwise
      % single case.
      return;
end
end

function boundaryLabel = computeBoundaryLabel(L, lineWidth)

nhoodSize = 2 * lineWidth + 1;
nhood = ones(nhoodSize);
BW = (imdilate(L,nhood) > L) | (imerode(L,nhood) < L);
boundaryLabel = L;
boundaryLabel(imcomplement(BW)) = 0;
    
end

function TF = mustBe2DImage(I)

isGrayOrRGBImage = ismatrix(I) || ((ndims(I) == 3) && (size(I,3) == 3));
if ~isGrayOrRGBImage
    error(message('vision:dims:imageNot2DorRGB'));
end

validateattributes(I, {'single', 'double', 'uint8', 'uint16', 'int16'}, {}, mfilename, 'I', 1);
TF = true;

end

function TF = validateNotObject(x,name)
vision.internal.inputValidation.validateNotObject(x, 'vision', name);
TF = true;
end

function TF = validateMaskstack(maskstack, I)

if isempty(maskstack)
    TF = true;
    return
end

if ismatrix(maskstack)
    validSize = size(I,[1,2]);
else
    validSize = [size(I,[1,2]),NaN];
end
validateattributes(maskstack, {'logical'}, {'size',validSize}, mfilename, 'MASKSTACK', 2);
TF = true;

end

function color = validateAndManageColor(colorIn, numMasks)

validateattributes(colorIn, {'double','single','uint8','uint16','int16','string','cell','char'},...
    {'nonsparse', 'real'}, mfilename, 'Color');

persistent colorSpecConverter
if isempty(colorSpecConverter)
    colorSpecConverter = images.internal.ColorSpecToRGBConverter;
end

if isnumeric(colorIn)
    vision.internal.inputValidation.validateNotObject(colorIn, 'vision', 'Color');
    
    validateattributes(colorIn, {'numeric'}, {'finite','real', 'ncols',3}, mfilename, 'Color');
    color = colorIn;
    
    if isrow(color)
        color = im2single(color);
        color = repelem(color, numMasks, 1);
        
    elseif size(color,1) == numMasks
        color = im2single(color);
        
    else
        error(message('vision:insertObjectMask:numValuesMustMatchNumMasks','Color'));
    end
        
elseif isstring(colorIn) || ischar(colorIn) || iscellstr(colorIn)
    colorIn = string(colorIn);
    
    if isscalar(colorIn)
        color = convertColorSpec(colorSpecConverter, colorIn);
        color = im2single(color);
        color = repelem(color, numMasks, 1);
        
    elseif numel(colorIn) == numMasks
        color = zeros(numMasks, 3, 'single');
        for idx = 1:numMasks
            color(idx, :) = convertColorSpec(colorSpecConverter, colorIn(idx));
        end
        color = im2single(color);
        
    else
        error(message('vision:insertObjectMask:numValuesMustMatchNumMasks','Color'));
    end
    
else
    error(message('vision:insertObjectMask:invalidColorFormat','Color'));
end

end

function opacity = validateAndManageOpacity(opacityIn, numMasks, name)
% Do not allow numeric objects (e.g. gpuArray).
vision.internal.inputValidation.validateNotObject(opacityIn, 'vision', name);
opacity = single(opacityIn);
opacity = repmat(opacity, [1,numMasks]);
end

function lineColor = validateAndManageLineColor(lineColorIn, color)
validateattributes(lineColorIn, {'double','single','uint8','uint16','int16','string','cell','char'},...
    {'nonsparse', 'real'}, mfilename, 'Color');

numMasks = size(color,1);

persistent colorSpecConverter
if isempty(colorSpecConverter)
    colorSpecConverter = images.internal.ColorSpecToRGBConverter;
end

if isnumeric(lineColorIn)
    vision.internal.inputValidation.validateNotObject(lineColorIn, 'vision', 'LineColor');
    
    validateattributes(lineColorIn, {'numeric'}, {'finite','real', 'ncols',3}, mfilename, 'LineColor');
    lineColor = lineColorIn;
    
    if isrow(lineColor)
        lineColor = im2single(lineColor);
        lineColor = repelem(lineColor, numMasks, 1);
        
    elseif size(lineColor,1) == numMasks
        lineColor = im2single(lineColor);
        
    else
        error(message('vision:insertObjectMask:numValuesMustMatchNumMasks','LineColor'));
    end
        
elseif isstring(lineColorIn) || ischar(lineColorIn) || iscellstr(lineColorIn)
    
    lineColorIn = string(lineColorIn);
    
    if isscalar(lineColorIn) &&  strncmpi('auto',lineColorIn,strlength(lineColorIn))
        lineColor = color;
        return
    end
    
    if isscalar(lineColorIn)
        lineColor = convertColorSpec(colorSpecConverter, lineColorIn);
        lineColor = im2single(lineColor);
        lineColor = repelem(lineColor, numMasks, 1);
        
    elseif numel(lineColorIn) == numMasks
        lineColor = zeros(numMasks, 3, 'single');
        for idx = 1:numMasks
            lineColor(idx, :) = convertColorSpec(colorSpecConverter, lineColorIn(idx));
        end
        lineColor = im2single(lineColor);
        
    else
        error(message('vision:insertObjectMask:numValuesMustMatchNumMasks','LineColor'));
    end
    
else
    error(message('vision:insertObjectMask:invalidColorFormat','LineColor'));
end

end
