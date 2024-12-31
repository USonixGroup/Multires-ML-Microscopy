function bboxes = detectTextCRAFT(I, varargin)

% Copyright 2021-2023 The MathWorks, Inc.
%#codegen

% Check for deep learning toolbox.

if isSimMode
    bboxes = detectTextCRAFTImpl(I, varargin{:});
else
    bboxes = vision.internal.codegen.detectTextCRAFT(I, varargin{:});
end

end
% -------------------------------------------------------------------------
function bboxes = detectTextCRAFTImpl(I, varargin)
% This function is the simulation implementation of detectTextCRAFT

vision.internal.requiresNeuralToolbox(mfilename);

params = parseInputs(I,varargin{:});
nargoutchk(0,1);

% Defining model as persistent to save upon the loading time.
persistent craftNet;
if isempty(craftNet)
    craftNet = iTripwireCRAFTModel();
end

% Support gray scale images.
if (size(I, 3)==1)
    I = cat(3, I, I, I);
end

% If GPU is available, then convert data to gpuArray.
if ((params.ExecutionEnvironment == "auto" && canUseGPU) || params.ExecutionEnvironment == "gpu")
    I = gpuArray(I);
end

% Crop image if requested.
Iroi = vision.internal.detector.cropImageIfRequested(I, params.ROI, params.UseROI);

% Preprocess input image.
[Ipreprocessed, info] = iPreprocessCRAFT(Iroi);
Ipreprocessed = dlarray(Ipreprocessed, 'SSCB');

% Output from CRAFT network for the given image.
networkOutput = predict(craftNet, Ipreprocessed, 'Acceleration', params.Acceleration);

% Postprocess features to obtain text regions.
bboxes = iPostprocessCRAFT(networkOutput, info, params);
end

% -------------------------------------------------------------------------
function params = parseInputs(I, varargin)

% Validate input image.
validateChannelSize = false;  % check if the channel size is equal to that of the network
validateImageSize   = false; % support images smaller than input size
networkInputSize = [224 224 3];
[sz,~] = vision.internal.cnn.validation.checkDetectionInputImage(...
    networkInputSize, I, validateChannelSize, validateImageSize);

n = ndims(I);
coder.internal.errorIf(isscalar(I)||(n > 3),'vision:dims:imageNot2DorRGB');

defaults = iDefaultParams();

p = inputParser;
p.addOptional('roi', defaults.roi);
p.addParameter('MinSize', defaults.MinSize);
p.addParameter('MaxSize', sz(1:2));
p.addParameter('ExecutionEnvironment', defaults.ExecutionEnvironment);
p.addParameter('CharacterThreshold', defaults.CharacterThreshold);
p.addParameter('LinkThreshold', defaults.LinkThreshold);
p.addParameter('Acceleration', 'auto');

parse(p, varargin{:});
userInput = p.Results;

% Validate CharacterThreshold.
iCheckThreshold(userInput.CharacterThreshold, 'CharacterThreshold');

% Validate LinkThreshold.
iCheckThreshold(userInput.LinkThreshold, 'LinkThreshold');

% Validate ROI.
useROI = ~ismember('roi', p.UsingDefaults);
if useROI
    vision.internal.detector.checkROI(userInput.roi, size(I));
end

% Validate minsize and maxsize.
wasMinSizeSpecified = ~ismember('MinSize', p.UsingDefaults);
wasMaxSizeSpecified = ~ismember('MaxSize', p.UsingDefaults);

if wasMinSizeSpecified
    vision.internal.detector.ValidationUtils.checkMinSize(userInput.MinSize, [1,1], mfilename);
    if useROI
        coder.internal.errorIf(any(userInput.MinSize > userInput.roi([4 3])) , ...
            'vision:detectText:modelMinSizeGTROISize',...
            userInput.roi(1,4), userInput.roi(1,3));
    else
        coder.internal.errorIf(any(userInput.MinSize > sz(1:2)) , ...
            'vision:detectText:modelMinSizeGTImgSize',...
            sz(1,1), sz(1,2));
    end
end

if wasMaxSizeSpecified
    vision.internal.detector.ValidationUtils.checkSize(userInput.MaxSize, 'MaxSize', mfilename);
    if useROI
        coder.internal.errorIf(any(userInput.MaxSize > userInput.roi([4 3])) , ...
            'vision:detectText:modelMaxSizeGTROISize',...
            userInput.roi(1,4), userInput.roi(1,3));
    else
        coder.internal.errorIf(any(userInput.MaxSize > sz(1:2)) , ...
            'vision:detectText:modelMaxSizeGTImgSize',...
            sz(1,1), sz(1,2));
    end
end

if wasMinSizeSpecified && wasMaxSizeSpecified
    coder.internal.errorIf(any(userInput.MinSize >= userInput.MaxSize) , ...
        'vision:ObjectDetector:minSizeGTMaxSize');
end

% Validate execution environment.
exeEnv = vision.internal.cnn.validation.checkExecutionEnvironment(...
    userInput.ExecutionEnvironment, mfilename);

% Validate Acceleration.
accel = vision.internal.cnn.validation.checkAcceleration(...
    userInput.Acceleration, mfilename);

params.ROI                  = double(userInput.roi);
params.UseROI               = useROI;
params.MinSize              = single(userInput.MinSize);
params.MaxSize              = double(userInput.MaxSize);
params.CharacterThreshold   = double(userInput.CharacterThreshold);
params.LinkThreshold        = double(userInput.LinkThreshold);
params.ExecutionEnvironment = exeEnv;
params.Acceleration         = accel;
end

%--------------------------------------------------------------------------
% Function to load CRAFT model
function net = iTripwireCRAFTModel()
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsTextDetectionInstalled';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for Text Detection';
    basecode = 'TEXT_DETECTION';

    throwAsCaller(MException(message('vision:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsTextDetectionInstalled.m');
    idx     = strfind(fullPath, pattern);
    matfile = fullfile(fullPath(1:idx), 'data', 'craftNet.mat');
    data    = load(matfile);
    net     = data.craftNet;
end
end

%--------------------------------------------------------------------------
function s = iDefaultParams()
s.roi                  = zeros(0,4);
s.CharacterThreshold   = 0.4;
s.LinkThreshold        = 0.4;
s.MinSize              = [6,6];
s.MaxSize              = [];
s.ExecutionEnvironment = 'auto';
end

%--------------------------------------------------------------------------
function iCheckThreshold(threshold,varName)
validateattributes(threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
    mfilename, varName);
end

%--------------------------------------------------------------------------
function [preprocessedImage, info] = iPreprocessCRAFT(I)
% iPreprocessCRAFT returns preprocessed image and the scale required to
% map the detections back to the size of test image.

% magRatio and canvasSize are hard-coded based on original reference paper,
% Baek, Youngmin, Bado Lee, Dongyoon Han, Sangdoo Yun, and Hwalsuk Lee.
% "Character region awareness for text detection." In Proceedings of the
% IEEE/CVF Conference on Computer Vision and Pattern Recognition, pp.
% 9365-9374. 2019.
magRatio = 1.5;

% Image size for inference.
canvasSize = 1280;

I = im2single(I);
[height, width, channel] = size(I);

if (all([height,width] < canvasSize))
    % Limit the maximum dimension to the canvas size.
    targetSize = min((magRatio * max(height, width)), canvasSize);

    imgScale = targetSize / max(height, width);
    targetH = fix(single(height * imgScale));
    targetW = fix(single(width * imgScale));
    processedImage = imresize(I, [targetH targetW], 'bilinear');
else
    imgScale = 1;
    targetH = height;
    targetW = width;
    processedImage = I;
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
preprocessedImage = zeros(targetH32, targetW32, channel,'like',I);
preprocessedImage(1:targetH, 1:targetW,:,:) = processedImage;
info.imageScale = imgScale;
info.imageSize = [height, width];
end

%--------------------------------------------------------------------------
function bboxes = iPostprocessCRAFT(networkOutput, info, params)
% iPostprocessCRAFT returns bounding boxes based on activations.
%
% Input networkOutput correspond to predictions obtained form activations
% of last layer.
%
% Input params consists of thresholds defined by user.
%
% Output bounding boxes consists of M-by-4 matrix defining M bounding boxes
% with each row containing four-element vector, [x, y, width, height].

networkOutput = extractdata(gather(networkOutput));

% Extract region and affinity score maps.
regionScore = networkOutput(:,:,1);
affinityScore= networkOutput(:,:,2);

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
    bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), params.ROI, params.UseROI);
else
    bboxes = zeros(0,4,'double');
end
end

%--------------------------------------------------------------------------
function bboxes = iFormAxisAlignedBoxes(textmap, linkmap, params)
% This function generates axis aligned bounding boxes.

characterThreshold = params.CharacterThreshold;
linkThreshold = params.LinkThreshold;

% Extract the text region.
regionScore = textmap >= characterThreshold;

% Extract the affinity region.
affinityScore = linkmap >= linkThreshold;

textScoreComb = regionScore | affinityScore;

bboxes = iConnectedComponents(textScoreComb,textmap);
end

%--------------------------------------------------------------------------
function bboxes = iConnectedComponents(textScoreComb, textmap)
% Computes the connected components labeled image and statistics output
% for each label.
CC = bwconncomp((textScoreComb), 4);
[imgH, imgW] = size(textmap);

% compute label matrix to label connected components in the image.
labels = labelmatrix(CC);

bboxes = [];

for k = 1:CC.NumObjects
    % Create segmentation map.
    segmap = (zeros(size(textmap), 'logical'));
    segmap(labels==k) = 1;
    labelArea = numel(CC.PixelIdxList{1,k});

    % Generally, characters will have a heatmap values ranging from 0.4 to
    % 0.9 but false detections will be less than 0.7, hence text_threshold
    % of 0.7 is used in the below condition to remove the false detections.
    if (max(textmap(labels==k)) > 0.7)

        % Extract bounding box coordinates of segmap.
        [x,y,w,h] = iGetLabelBoundaries(segmap);

        % Dilate each label to enclose within the bounding box.
        niter = fix(sqrt(labelArea * min(w, h) / (w * h)) * 2);

        sx = x - niter;
        ex = x + w + niter + 1;
        sy = y - niter;
        ey = y + h+ niter + 1;

        % Limit the detection boundary.
        sx = max(sx,1);
        sy = max(sy,1);
        ex = min(ex,imgW);
        ey = min(ey,imgH);

        kernel = strel('rectangle',[niter + 1, niter + 1]);
        segmap(sy:ey, sx:ex) = imdilate(segmap(sy:ey, sx:ex), kernel);

        % Extract bounding box coordinates of dilated segmap.
        [x,y,w,h] = iGetLabelBoundaries(segmap);

        bbox = [x,y,w,h];
        bboxes = [bboxes; bbox]; %#ok<AGROW>
    end
end
end

%--------------------------------------------------------------------------
function [x,y,w,h]= iGetLabelBoundaries(segmap)
% Extract bounding box locations.
vertical = any(segmap, 2);
horizontal = any(segmap, 1);

y = find(vertical, 1, 'first');
y2 = find(vertical, 1, 'last');
x = find(horizontal, 1, 'first');
x2 = find(horizontal, 1, 'last');

w = max(x2-x,1);
h = max(y2-y,1);
end
%--------------------------------------------------------------------------
function mode = isSimMode()
mode = isempty(coder.target);
end
