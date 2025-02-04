function bboxes = detectTextCRAFT(I, varargin)
%
%   This function is used to implement code generation support for the
%   detectTextCRAFT function.

% Copyright 2023-2024 The MathWorks, Inc.
%#codegen
coder.allowpcode('plain');

coder.extrinsic('vision.internal.codegen.checkTextDetectionSpkg');
coder.extrinsic('constructFilePath')
n = ndims(I);
coder.internal.errorIf(isscalar(I)||(n > 3), 'vision:dims:imageNot2DorRGB');

% Check for varsize inputs images
isFirstDimVarSize = ~coder.internal.isConst(size(I, 1));
isSecondDimVarSize = ~coder.internal.isConst(size(I, 2));

coder.internal.errorIf(((n==2) &&(isFirstDimVarSize || isSecondDimVarSize)),'vision:detectText:VarDimImageNotSupported');

if n == 3
    isThirdDimVarSize = ~coder.internal.isConst(size(I, 3));
    coder.internal.errorIf((isFirstDimVarSize||isSecondDimVarSize||isThirdDimVarSize),'vision:detectText:VarDimImageNotSupported');
end

if( ~isempty(varargin) && (isa(varargin{1}, 'numeric')) )
    roi = double(varargin{1});

    % Error out if roi is not a constant.
    coder.internal.assert(coder.internal.isConst(roi), 'vision:detectText:roiConstant');

    % Check whether ROI is fully contained in image.
    coder.const(feval('vision.internal.detector.checkROI', roi, size(I)));
    useROI = true;
    [params]  = parseInputsCodegen(I, roi, useROI, varargin{2:end});
else
    roi = coder.nullcopy(zeros(1,4));
    useROI = false;
    [params] = parseInputsCodegen(I, roi, useROI, varargin{:});
end

% Check if text detection support package is installed
isSpkgInstalled = coder.const(vision.internal.codegen.checkTextDetectionSpkg());
name     = 'Computer Vision Toolbox Model for Text Detection';
basecode = 'TEXT_DETECTION';
coder.internal.errorIf(~isSpkgInstalled, 'vision:supportpackages:InstallRequired', mfilename, name, basecode);

% Load craftNet model
fp = coder.const(constructFilePath());

persistent craftNet
if isempty(craftNet)
    craftNet = coder.loadDeepLearningNetwork(fp);
end

% Support gray scale images.
if (size(I,3)==1)
    img = cat(3,I,I,I);
else
    img = I;
end

% Crop image if requested.
Iroi = vision.internal.detector.cropImageIfRequested(img, roi, useROI);

% Preprocess input image.
[Ipreprocessed, info] = iPreprocessCRAFT(Iroi);
IpreprocessedNew = dlarray(Ipreprocessed, 'SSCB');

networkOutput = predict(craftNet, IpreprocessedNew);

% Postprocess features to obtain text regions.
bboxes = iPostprocessCRAFT(networkOutput, info, params, roi, useROI);

end

%--------------------------------------------------------------------------
function [paramValues] = parseInputsCodegen(I, roi, useROI, varargin)
% This function is used for parsing inputs in codegen
coder.inline('always')
coder.internal.prefer_const(I, roi, useROI, varargin);

% Validate input image.
validateChannelSize = false;  % check if the channel size is equal to that of the network
validateImageSize   = false; % support images smaller than input size
networkInputSize = [224 224 3];
[sz,~] = checkDetectionInputImage(...
    networkInputSize, I, validateChannelSize, validateImageSize);

possibleNameValues = struct(...
    'MinSize',uint32(0), ...
    'MaxSize',uint32(0), ...
    'ExecutionEnvironment',uint32(0), ...
    'Acceleration',uint32(0),...
    'CharacterThreshold', uint32(0), ...
    'LinkThreshold', uint32(0));

poptions = struct( ...
    'CaseSensitivity',false, ...
    'PartialMatching','unique', ...
    'StructExpand',false, ...
    'IgnoreNulls',true);

defaults = struct(...
    'MinSize',[6,6], ...
    'MaxSize', sz(1:2), ...
    'ExecutionEnvironment','auto', ...
    'Acceleration','auto',...
    'CharacterThreshold', 0.4, ...
    'LinkThreshold', 0.4);

if (nargin == 1) % only imageSize
    params = defaults;
else
    pstruct = coder.internal.parseParameterInputs(possibleNameValues, poptions, varargin{:});
    params.MinSize = coder.internal.getParameterValue(pstruct.MinSize, defaults.MinSize, varargin{:});
    params.MaxSize = coder.internal.getParameterValue(pstruct.MaxSize, defaults.MaxSize, varargin{:});
    params.ExecutionEnvironment = coder.internal.getParameterValue(pstruct.ExecutionEnvironment, defaults.ExecutionEnvironment, varargin{:});
    params.Acceleration = coder.internal.getParameterValue(pstruct.Acceleration, defaults.Acceleration, varargin{:});
    params.CharacterThreshold = coder.internal.getParameterValue(pstruct.CharacterThreshold, defaults.CharacterThreshold, varargin{:});
    params.LinkThreshold = coder.internal.getParameterValue(pstruct.LinkThreshold, defaults.LinkThreshold, varargin{:});
end

% Validate CharacterThreshold.
iCheckThreshold(params.CharacterThreshold, 'CharacterThreshold');

% Validate LinkThreshold.
iCheckThreshold(params.LinkThreshold, 'LinkThreshold');

% Validate MinSize and MaxSize only if they are set to non-default values
validateMinSize = coder.const(pstruct.MinSize ~= zeros('uint32')) && ~isequal(params.MinSize,[6 6]);
validateMaxSize = coder.const(pstruct.MaxSize ~= zeros('uint32')) && ~isequal(params.MaxSize,[]);

if validateMinSize
    vision.internal.detector.ValidationUtils.checkMinSize(params.MinSize, [1,1], mfilename);
    if useROI
        coder.internal.errorIf(any(params.MinSize > roi([4 3])) , ...
            'vision:detectText:modelMinSizeGTROISize',...
            roi(1,4), roi(1,3));
    else
        coder.internal.errorIf(any(params.MinSize > sz(1:2)) , ...
            'vision:detectText:modelMinSizeGTImgSize',...
            sz(1,1), sz(1,2));
    end
end

if validateMaxSize
    vision.internal.detector.ValidationUtils.checkSize(params.MaxSize, 'MaxSize', mfilename);
    if useROI
        coder.internal.errorIf(any(params.MaxSize > roi([4 3])) , ...
            'vision:detectText:modelMaxSizeGTROISize',...
            roi(1,4), roi(1,3));
    else
        coder.internal.errorIf(any(params.MaxSize > sz(1:2)) , ...
            'vision:detectText:modelMaxSizeGTImgSize',...
            sz(1,1), sz(1,2));
    end
end

if validateMinSize && validateMaxSize
    coder.internal.errorIf(any(params.MinSize >= params.MaxSize) , ...
        'vision:ObjectDetector:minSizeGTMaxSize');
end

% Ignore ExecutionEnvironment
if coder.const(pstruct.ExecutionEnvironment ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:detectText:IgnoreInputArg');
end

% Ignore Acceleration
if coder.const(pstruct.Acceleration ~= zeros('uint32'))
    coder.internal.compileWarning(...
        'vision:detectText:IgnoreInputArg');
end

paramValues.MinSize              = single(params.MinSize);
paramValues.MaxSize              = double(params.MaxSize);
paramValues.CharacterThreshold   = double(params.CharacterThreshold);
paramValues.LinkThreshold        = double(params.LinkThreshold);
paramValues.ExecutionEnvironment = params.ExecutionEnvironment;
paramValues.Acceleration         = params.Acceleration;
end

% -------------------------------------------------------------------------
function iCheckThreshold(threshold,varName)
validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
    mfilename, varName);
end

% --------------------------------------------------------------------------
function [imSz, isBatchOfImages] = checkDetectionInputImage(networkInputSize,...
    sampleImage, validateChannelSize, validateImageSize)

imSz = coder.nullcopy(zeros(1,4));
[imSz(1), imSz(2), imSz(3), imSz(4)] = size(sampleImage);

networkChannelSize = coder.const(networkInputSize(3));
imageChannelSize = coder.const(imSz(3));

isBatchOfImages =  coder.const(imSz(4) > 1);

if isBatchOfImages
    % Pass the first image in the batch for internal validation.
    img = sampleImage(:,:,:,1);
else
    img = sampleImage;
end

% Multi-Channel or grayScale or RGB images allowed
if coder.const(networkChannelSize > 3 || networkChannelSize == 2)
    vision.internal.inputValidation.validateImage(img, 'I', 'multi-channel');
else
    vision.internal.inputValidation.validateImage(img, 'I');
end

coder.internal.errorIf(validateImageSize && any(imSz(1:2) < networkInputSize(1:2)) , ...
    'vision:ObjectDetector:imageSmallerThanNetwork', networkInputSize(1), networkInputSize(2));

% Validate number of channels for input image and network input
coder.internal.errorIf(validateChannelSize && imageChannelSize ~= networkChannelSize,...
    'vision:ObjectDetector:invalidInputImageChannelSize', imageChannelSize, networkChannelSize);
end

% -------------------------------------------------------------------------
function [preprocessedImageNew, info] = iPreprocessCRAFT(I)
% This function computes the pre processing of input image
coder.inline('always');
coder.internal.prefer_const(I);

magRatio = 1.5;

% Image size for inference.
canvasSize = 1280;

INew = im2single(I);
[height, width, channel] = size(INew);

if (all([height,width] < canvasSize))
    % Limit the maximum dimension to the canvas size.
    targetSize = min((magRatio * max(height, width)), canvasSize);

    imgScale = targetSize / max(height, width);
    targetH = fix(single(height * imgScale));
    targetW = fix(single(width * imgScale));
    processedImage = imresize(INew, [targetH targetW], 'bilinear');
else
    imgScale = 1;
    targetH = height;
    targetW = width;
    processedImage = INew;
end

% Canvas that holds the preprocessed image should be divisible by 32.
targetH32 = targetH;
targetW32 = targetW;
if mod(targetH, 32) ~= 0
    targetH32 = targetH + (32 - mod(targetH, 32));
end
if mod(targetW, 32) ~= 0
    targetW32 = targetW + (32 - mod(targetW, 32));
end

% Preprocessed image of size [h w].
preprocessedImageNew = zeros(targetH32, targetW32, channel, 'like', INew);
preprocessedImageNew(1:targetH, 1:targetW,:,:) = processedImage;
info.imageScale = imgScale;
info.imageSize = [height, width];
end

%--------------------------------------------------------------------------
function bboxes = iPostprocessCRAFT(networkOutput, info, params, roi, useROI)
% This function returns the bounding boxes baed on activations
coder.internal.prefer_const(networkOutput, info, params, roi, useROI);

networkOutputData = extractdata(networkOutput);

% Extract region and affinity score maps.
regionScore = networkOutputData(:,:,1);
affinityScore= networkOutputData(:,:,2);

% Generate bounding boxes based on network output.
bboxes = iFormAxisAlignedBoxes(regionScore, affinityScore, params);

if ~isempty(bboxes)
    % Adjust bounding box coordinates.
    scale = 2 * (1/info.imageScale);
    bboxes = bboxes * scale;

    % Rounding the bboxes to return integer valued coordinates.
    bboxes = round(bboxes);

    % Filter bounding boxes based on min size and max size.
    bboxes = vision.internal.cnn.utils.FilterBboxesFunctor.filterSmallBBoxes(params.MinSize, bboxes);
    bboxes = vision.internal.cnn.utils.FilterBboxesFunctor.filterLargeBBoxes(params.MaxSize, bboxes);

    % Limit width detections
    detectionsWd = min((bboxes(:,1) + bboxes(:,3)),info.imageSize(1,2));
    bboxes(:,3) = detectionsWd(:,1) - bboxes(:,1);

    % Limit Height detections
    detectionsHt = min((bboxes(:,2) + bboxes(:,4)),info.imageSize(1,1));
    bboxes(:,4) = detectionsHt(:,1) - bboxes(:,2);

    % Apply ROI offset.
    bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), roi, useROI);
else
    bboxes = zeros(0, 4, 'double');
end
end

%--------------------------------------------------------------------------
function bboxes = iFormAxisAlignedBoxes(textmap, linkmap, params)
% This function generates axis aligned bounding boxes.
coder.inline('always');
coder.internal.prefer_const(textmap, linkmap, params);

characterThreshold = params.CharacterThreshold;
linkThreshold = params.LinkThreshold;

% Extract the text region.
regionScore = textmap >= characterThreshold;

% Extract the affinity region.
affinityScore = linkmap >= linkThreshold;

textScoreComb = regionScore | affinityScore;

bboxes = iConnectedComponents(textScoreComb, textmap);
end

%--------------------------------------------------------------------------
function bboxes = iConnectedComponents(textScoreComb, textmap)
% Computes the connected components labeled image and statistics output
% for each label.
coder.internal.prefer_const(textScoreComb, textmap);
CC = bwconncomp((textScoreComb),4);
[imgH, imgW] = size(textmap);

% Compute label matrix to label connected components in the image.
labels = labelmatrixCodegen(CC);

bboxes = coder.nullcopy(zeros(coder.ignoreConst(0), 4));

for k = 1:CC.NumObjects
    % Create segmentation map.
    segmap = (zeros(size(textmap), 'logical'));
    segmap(labels==k) = 1;

    if coder.gpu.internal.isGpuEnabled
        labelArea = double(CC.RegionLengths(k));
    else
        labelArea = numel(CC.PixelIdxList{1,k});
    end

    % Generally, characters will have a heatmap values ranging from 0.4 to
    % 0.9 but false detections will be less than 0.7, hence text_threshold
    % of 0.7 is used in the below condition to remove the false detections.
    if (max(textmap(labels==k)) > 0.7)

        % Extract bounding box coordinates of segmap.
        [x, y, w, h] = iGetLabelBoundaries(segmap);

        % Dilate each label to enclose within the bounding box.
        niter = fix(sqrt(labelArea * min(w, h) / (w * h)) * 2);

        sx = x - niter;
        ex = x + w + niter + 1;
        sy = y - niter;
        ey = y + h+ niter + 1;

        % Limit the detection boundary.
        sxNew = max(sx, 1);
        syNew = max(sy, 1);
        exNew = min(ex, imgW);
        eyNew = min(ey, imgH);

        kernel = strel('rectangle', [niter + 1, niter + 1]);
        segmap(syNew:eyNew, sxNew:exNew) = imdilate(segmap(syNew:eyNew, sxNew:exNew), kernel);

        % Extract bounding box coordinates of dilated segmap.
        [x, y, w, h] = iGetLabelBoundaries(segmap);

        bbox = [x, y, w, h];
        bboxes = [bboxes; bbox]; %#ok<AGROW>
    end
end
end

%--------------------------------------------------------------------------
function [x, y, w, h]= iGetLabelBoundaries(segmap)
% Extract bounding box locations.
coder.inline('always');
coder.internal.prefer_const(segmap);

vertical = any(segmap, 2);
horizontal = any(segmap, 1);

y1 = find(vertical, 1, 'first');
y2 = find(vertical, 1, 'last');
x1 = find(horizontal, 1, 'first');
x2 = find(horizontal, 1, 'last');

x = x1(1);
y = y1(1);

% In codegen the find fn returns var size output
% Here we require the elements in first position of x1,x2,y1,y2
w = max(x2(1)-x1(1), 1);
h = max(y2(1)-y1(1), 1);
end

%--------------------------------------------------------------------------
function L = labelmatrixCodegen(CC)
% Function to implement code generation support for the
% labelmatrix function.

L = zeros(CC.ImageSize);

if coder.gpu.internal.isGpuEnabled
    regionIndices = CC.RegionIndices;
    idxCount = coder.internal.indexInt([0; cumsum(CC.RegionLengths)]);
    for k = 1:CC.NumObjects
        L(regionIndices(idxCount(k)+1:idxCount(k+1), 1)) = k;
    end
else
    for k = 1:CC.NumObjects
        L(CC.PixelIdxList{k}) = k;
    end
end
end

%--------------------------------------------------------------------------
function fp = constructFilePath()
% This function loads craftNet model mat file from the path
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsTextDetectionInstalled';
fullPath = which(breadcrumbFile);
pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsTextDetectionInstalled.m');
idx     = strfind(fullPath, pattern);
fp = fullfile(fullPath(1:idx), 'data', 'craftNet.mat');
end