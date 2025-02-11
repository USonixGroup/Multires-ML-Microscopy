function [imagePoints, boardSize, imageIdx, userCanceled] = detectCheckerboardPoints(I, varargin)

% Copyright 2013-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

if isempty(coder.target)
    % Convert strings to chars for simulation

    if isstring(I)
        I = convertStringsToChars(I);
    end

    if nargin>1
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    [images2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial] = parseInputs(varargin{:});
else
    coder.internal.errorIf(ischar(I), 'vision:calibrate:codegenFileNamesNotSupported');
    coder.internal.errorIf(isstring(I), 'vision:calibrate:codegenFileNamesNotSupported');
    coder.internal.errorIf(iscell(I), 'vision:calibrate:codegenFileNamesNotSupported');
    coder.internal.errorIf(isnumeric(I) && length(size(I)) > 3,...
        'vision:calibrate:codegenMultipleImagesNotSupported');
    [images2, showProgressBar,minCornerMetric, highDistortion, usePartial] = parseInputsCodegen(varargin{:});
    parent = [];
end

if isempty(images2)
    % single camera
    [imagePoints, boardSize, imageIdx, userCanceledTmp, fullBoardDetected] = detectMono(I, parent,...
        showProgressBar, minCornerMetric, highDistortion, usePartial);
else
    % 2-camera stereo
    images1 = I;
    checkStereoImages(images1, images2);
    
    % Partial checkerboards are disabled for stereo images for now. Will be
    % enabled later when the dependent functionalities are available.
    usePartial = false;
    
    [imagePoints, boardSize, imageIdx, userCanceledTmp] = detectStereo(images1, ...
        images2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial);
    
    % Set to true since partial checkerboard detection is disabled for
    % stereo images
    fullBoardDetected = true; 
end

checkThatBoardIsAsymmetric(boardSize, fullBoardDetected);
if showProgressBar
    userCanceled = userCanceledTmp;
else
    userCanceled = false;
end

%--------------------------------------------------------------------------
function [image2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial] = parseInputs(varargin)

% Check if the second argument is the second set of images
% Need to do this "by hand" because inputParser does not
% handle optional string arguments.

isSecondArgumentString =  ~isempty(varargin) && ischar(varargin{1});
isThirdArgumentValue = ~mod(nargin,2);

% pv-pairs come in pairs
isSecondArgumentNameValuePair = isSecondArgumentString && isThirdArgumentValue;

if isempty(varargin) || isSecondArgumentNameValuePair
    image2 = [];
    args = varargin;
else
    image2 = varargin{1};
    if numel(varargin) > 1
        args = varargin(2:end);
    else
        args = {};
    end
end

% Parse the Name-Value pairs
parser = inputParser;
parser.addParameter('ShowProgressBar', false, @checkShowProgressBar);
parser.addParameter('ProgressBarParent', [], @checkProgressBarParent);

cornerMetricDef = 0.15; % when highDistortion is disabled
parser.addParameter('MinCornerMetric', cornerMetricDef, @checkMinCornerMetric);
parser.addParameter('HighDistortion', false, @checkHighDistortion);
parser.addParameter('PartialDetections', true, @checkPartialDetections);
parser.parse(args{:});
showProgressBar = parser.Results.ShowProgressBar;
parent          = parser.Results.ProgressBarParent;
highDistortion  = parser.Results.HighDistortion;
usePartial      = parser.Results.PartialDetections;
minCornerMetric = parser.Results.MinCornerMetric;

if highDistortion && any(strcmp('MinCornerMetric', parser.UsingDefaults))
    minCornerMetric = 0.12; % Allow lower threshold for high distortion images
end
    
%--------------------------------------------------------------------------
function [image2, showProgressBar, minCornerMetric, highDistortion, usePartial] = parseInputsCodegen(varargin)
showProgressBar = false;

isSecondArgumentString =  ~isempty(varargin) && (ischar(varargin{1}) || isstring(varargin{1}));
isThirdArgumentValue = ~mod(nargin,2);

isSecondArgumentNameValuePair = isSecondArgumentString && isThirdArgumentValue;

if isempty(varargin) || isSecondArgumentNameValuePair
    image2 = [];
    args = varargin;
else
    image2 = varargin{1};
    if numel(varargin) > 1
        args = varargin(2:end);
    else
        args = {};
    end
end

if ~isempty(args)
    params = struct( ...
        'MinCornerMetric', uint32(0), ...
        'HighDistortion', false, ...
        'PartialDetections', true);

    popt = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', true);

    optarg = eml_parse_parameter_inputs(params, popt, args{:});
    highDistortion = eml_get_parameter_value(optarg.HighDistortion, false, args{:});
    usePartial = eml_get_parameter_value(optarg.PartialDetections, true, args{:});

    % Use default value for MinCornerMetric
    if optarg.MinCornerMetric == uint32(0)
        if highDistortion
            minCornerMetric = 0.12;
        else
            minCornerMetric = 0.15;
        end
    else
        minCornerMetric = eml_get_parameter_value(optarg.MinCornerMetric, 0.15, args{:});
    end

    % MinCornerMetric
    checkMinCornerMetric(minCornerMetric);

    % HighDistortion
    checkHighDistortion(highDistortion);
    
    % PartialDetections
    checkPartialDetections(usePartial);
else
    minCornerMetric = 0.15;
    highDistortion = false;
    usePartial = true;
end

%--------------------------------------------------------------------------
% Detect the checkerboards in a single set of images
function [points, boardSize, imageIdx, userCanceled, fullBoardDetected] = ...
    detectMono(I, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)

userCanceled = false;
if iscell(I)
    % detect in a set of images specified by file names
    fileNames = I;
    checkFileNames(fileNames);
    [points, boardSize, imageIdx, userCanceledTmp, fullBoardDetected] = ...
        detectCheckerboardFiles(fileNames, parent, showProgressBar, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        userCanceled = userCanceledTmp;
    end
elseif ischar(I)
    % detect in a single image specified by a file name
    fileName = I;
    checkFileName(I);
    I = imread(fileName);
    [points, boardSize] = detectCheckerboardInOneImage(I, minCornerMetric, highDistortion, usePartial);
    imageIdx = ~isempty(points);
     
    fullBoardDetected = true;
    if imageIdx
        fullBoardDetected = ~any(isnan(points(:,1)));
    end
elseif ndims(I) > 3
    % detect in a stack of images
    checkImageStack(I);
    [points, boardSize, imageIdx, userCanceledTmp, fullBoardDetected] = ...
        detectCheckerboardStack(I, parent, showProgressBar, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        userCanceled = userCanceledTmp;
    end
else
    % detect in a single image
    checkImage(I);
    [points, boardSize] = detectCheckerboardInOneImage(I, minCornerMetric, highDistortion, usePartial);
    imageIdx = ~isempty(points);
    
    fullBoardDetected = true;
    if imageIdx
        fullBoardDetected = ~any(isnan(points(:,1)));
    end
end

%--------------------------------------------------------------------------
% Detect the checkerboards in stereo pairs.
function [points, boardSize, imageIdx, userCanceled] = ...
    detectStereo(images1, images2, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)

if isnumeric(images1) && size(images1, 4) == 1 % pair of single images
    [points1, boardSize1] = detectMono(images1, [], false, minCornerMetric, highDistortion, usePartial);
    [points2, boardSize2] = detectMono(images2, [], false, minCornerMetric, highDistortion, usePartial);

    userCanceled = false;
    if ~isequal(boardSize1, boardSize2)
        points = zeros(0, 2);
        boardSize = [0,0];
        imageIdx = false;
    else
        points = cat(4, points1, points2);
        boardSize = boardSize1;
        imageIdx = true;
    end

    if isempty(points)
        imageIdx = false;
    end
else
    % concatenate the two sets of images into one
    images = concatenateImages(images1, images2);

    % detect the checkerboards in the combined set
    [points, boardSize, imageIdx, userCanceled] = detectMono(images, parent,...
        showProgressBar, minCornerMetric, highDistortion, usePartial);

    if userCanceled
        points = zeros(0, 2);
        boardSize = [0,0];
    else
        % separate the points from images1 and images2
        [points, imageIdx] = vision.internal.calibration.separatePoints(points, imageIdx);

        if isempty(points)
            boardSize = [0 0];
        end
    end
end

%--------------------------------------------------------------------------
function images = concatenateImages(images1, images2)
if iscell(images1)
    images = {images1{:}, images2{:}}; %#ok
elseif ischar(images1)
    images = {images1, images2};
else
    images = cat(4, images1, images2);
end

%--------------------------------------------------------------------------
function tf = checkShowProgressBar(showProgressBar)
validateattributes(showProgressBar, {'logical', 'numeric'},...
    {'scalar'}, mfilename, 'ShowProgressBar');
tf = true;

%--------------------------------------------------------------
function tf = checkProgressBarParent(progressBarParent)
if ~isempty(progressBarParent)
    validateattributes(progressBarParent, {'matlab.ui.container.internal.AppContainer'},...
        {}, mfilename, 'ProgressBarParent');
end
tf = true;

%--------------------------------------------------------------------------
function tf = checkMinCornerMetric(value)
validateattributes(value, {'single', 'double'},...
    {'scalar', 'real', 'nonnegative', 'finite'}, mfilename, 'MinCornerMetric');
tf = true;

%--------------------------------------------------------------------------
function tf = checkHighDistortion(highDistortion)
validateattributes(highDistortion, {'logical', 'numeric'},...
    {'scalar','binary'}, mfilename, 'HighDistortion');
tf = true;

%--------------------------------------------------------------------------
function tf = checkPartialDetections(usePartial)
validateattributes(usePartial, {'logical', 'numeric'},...
    {'scalar','binary'}, mfilename, 'PartialDetections');
tf = true;

%--------------------------------------------------------------------------
function checkImage(I)
vision.internal.inputValidation.validateImage(I, 'I');

%--------------------------------------------------------------------------
function checkImageStack(images)
validClasses = {'double', 'single', 'uint8', 'int16', 'uint16'};
validateattributes(images, validClasses,...
    {'nonempty', 'real', 'nonsparse'},...
    mfilename, 'images');
coder.internal.errorIf(size(images, 3) ~= 1 && size(images, 3) ~= 3,...
    'vision:dims:imageNot2DorRGB');

%--------------------------------------------------------------------------
function checkFileNames(fileNames)
validateattributes(fileNames, {'cell'}, {'nonempty', 'vector'}, mfilename, ...
    'imageFileNames');
for i = 1:numel(fileNames)
    checkFileName(fileNames{i});
end

%--------------------------------------------------------------------------
function checkFileName(fileName)
validateattributes(fileName, {'char'}, {'nonempty'}, mfilename, ...
    'elements of imageFileNames');

try %#ok<EMTC>
    state = warning('off','imageio:tifftagsread:badTagValueDivisionByZero');
    imfinfo(fileName);
catch e
    warning(state);
    throwAsCaller(e);
end
warning(state);

%--------------------------------------------------------------------------
function checkStereoImages(images1, images2)
coder.internal.errorIf(strcmp(class(images1), class(images2)) == 0,...
    'vision:calibrate:stereoImagesMustBeSameClass');

coder.internal.errorIf(~ischar(images1) && any(size(images1) ~= size(images2)),...
    'vision:calibrate:stereoImagesMustBeSameSize');

%--------------------------------------------------------------------------
function checkThatBoardIsAsymmetric(boardSize, fullBoardDetected)
% ideally, a board should be asymmetric: one dimension should be even, and
% the other should be odd.
if isempty(coder.target) && fullBoardDetected
    if ~all(boardSize == 0) && (~xor(mod(boardSize(1), 2), mod(boardSize(2), 2))...
            || boardSize(1) == boardSize(2))
        s = warning('query', 'backtrace');
        warning off backtrace;
        warning(message('vision:calibrate:boardShouldBeAsymmetric'));
        warning(s);
    end
end

%--------------------------------------------------------------------------
% Detect checkerboards in a set of images specified by file names
function [points, boardSize, imageIdx, userCanceled, fullBoardDetected] = ...
    detectCheckerboardFiles(fileNames, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)
numImages = numel(fileNames);
boardPoints = cell(1, numImages);
boardSizes = zeros(numImages, 2);
userCanceled = false;
if showProgressBar
    waitBar = createProgressbar(numImages, parent);
end
for i = 1:numImages
    if showProgressBar && waitBar.Canceled
            points = [];
            boardSize = [0 0];
            imageIdx =[];
            userCanceled = true;
            return;
    end

    im = imread(fileNames{i});
    [boardPoints{i}, boardSizes(i,:)] = detectCheckerboardInOneImage(im, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        waitBar.update();
    end
end
[points, boardSize, imageIdx, fullBoardDetected] = chooseValidBoards(boardPoints, boardSizes, minCornerMetric, ...
    highDistortion, usePartial, fileNames);

%--------------------------------------------------------------------------
% Detect checkerboards in a stack of images
function [points, boardSize, imageIdx, userCanceled, fullBoardDetected] = ...
    detectCheckerboardStack(images, parent, showProgressBar, minCornerMetric, highDistortion, usePartial)
numImages = size(images, 4);
boardPoints = cell(1, numImages);
boardSizes = zeros(numImages, 2);
userCanceled = false;
if showProgressBar
    waitBar = createProgressbar(numImages, parent);
end
for i = 1:numImages
    if showProgressBar && waitBar.Canceled
            points = [];
            boardSize = [0 0];
            imageIdx =[];
            userCanceled = true;
            return;
    end
    im = images(:, :, :, i);
    [boardPoints{i}, boardSizes(i,:)] = detectCheckerboardInOneImage(im, minCornerMetric, highDistortion, usePartial);
    if showProgressBar
        waitBar.update();
    end
end
[points, boardSize, imageIdx, fullBoardDetected] = chooseValidBoards(boardPoints, boardSizes, minCornerMetric, ...
    highDistortion, usePartial, images);

%--------------------------------------------------------------------------
% Determine which board size is the most common in the set.
function [points, boardSize, imageIdx, fullBoardDetected] = chooseValidBoards(boardPoints, boardSizes, ...
    minCornerMetric, highDistortion, usePartial, images)
uniqueBoardIds = 2.^boardSizes(:, 1) .* 3.^boardSizes(:, 2);

% Eliminate images where no board was detected.
% The unique board id in this case is 2^0 + 3^0 = 1.
% Replace all 1's by a sequence of 1:n * 1e10, which will be different from
% all other numbers which are only multiples of 2 and 3.
zeroIdx = (uniqueBoardIds == 1);

% Defaults to true. Set to false when no reference (dominant) board is
% detected.
fullBoardDetected = true;

if all(zeroIdx)
    % When none of images have any boards, return [] for points and [0 0]
    % for the board size. In this case, the input boardPoints is a cell
    % array of empty matrices, which is used to allocate the height of the
    % imageIdx output vector.
    points = [];
    boardSize = [0 0];
    numImages = numel(boardPoints);
    imageIdx = false(numImages,1);
else
    uniqueBoardIds(zeroIdx) = (1:sum(zeroIdx)) * 5;

    % Find the most common value among unique board ids.
    [~, freq, modes] = mode(uniqueBoardIds);
    modeBoardId = max(modes{1});

    if usePartial
        numBoards = size(boardSizes, 1);
        
        % Get min number of same board detections for it to be considered
        % as the reference board
        if numBoards <= 3
            freqThreshold = 2;
        else
            freqThreshold = 3;
        end
        
        if freq >= freqThreshold
            % Use the board corresponding to the mode as the reference board
            refBoardSize = boardSizes(find(uniqueBoardIds == modeBoardId, 1), :);
        else
            % Use the board with the maximum size as the reference board
            fullBoardDetected = false;
            refBoardSize = max(boardSizes, [], 1);
        end
        
        % Accept all non-empty boards which are of the same size or smaller
        % than the reference board along both dimensions
        imageIdx = all(boardSizes <= refBoardSize, 2) & (boardSizes(:,1) > 0);
        
        % Retry detection for failed images without partial boards. At this
        % point, the rejection could be due to the detected board being larger
        % than the reference board along any dimension.
        retryIdx = find(~imageIdx);
        for idx = retryIdx'
            
            if iscell(images)
                im = imread(images{idx});
            else
                im = images(:, :, :, idx);
            end
            
            % Retry detection without looking for partial boards
            usePartial = false;
            [currBoardPoints, currBoardSize] = detectCheckerboardInOneImage(im, minCornerMetric, ...
                highDistortion, usePartial);
            
            if ~isempty(currBoardPoints) && all(currBoardSize <= refBoardSize)
               boardPoints{idx} = currBoardPoints;
               boardSizes(idx,:) = currBoardSize;
               imageIdx(idx) = true;
            end
        end
        
        % Pad smaller boards to be of identical size to the reference board
        boardPoints = padPartialBoards(boardPoints, boardSizes, imageIdx, refBoardSize);
        
        boardSize = refBoardSize;
    else
        imageIdx = (uniqueBoardIds == modeBoardId);
        boardSize = boardSizes(imageIdx, :);
        boardSize = boardSize(1, :);
    end

    % Get the corresponding points
    points = boardPoints(imageIdx);
    points = cat(3, points{:});
end

%--------------------------------------------------------------------------
function boardPoints = padPartialBoards(boardPoints, boardSizes, validBoards, refBoardSize)

validIdx = find(validBoards);
for boardIdx = 1:numel(validIdx)
   
    currBoardSize = boardSizes(validIdx(boardIdx), :);
    
    % Pad zeros assuming the upper left corner point as the origin. If
    % this is missing (not visible/detected), the actual image location of
    % the origin will have to be determined after camera parameter
    % estimation
    padSize = refBoardSize - currBoardSize;
    
    currBoardPoints = boardPoints{validIdx(boardIdx)};
    
    currBoardX = reshape(currBoardPoints(:,1), currBoardSize - 1);
    currBoardX = padarray(currBoardX, padSize, NaN, 'post');
    
    currBoardY = reshape(currBoardPoints(:,2), currBoardSize - 1);
    currBoardY = padarray(currBoardY, padSize, NaN, 'post');

    boardPoints{validIdx(boardIdx)} = [currBoardX(:), currBoardY(:)];

end 

%--------------------------------------------------------------------------
function [points, boardSize] = detectCheckerboardInOneImage(Iin, ...
    minCornerMetric, highDistortion, usePartial)
if ismatrix(Iin)
    Igray = Iin;
else
    Igray = rgb2gray(Iin);
end
I = im2single(Igray);

% Set bandwidth to smooth the image
if highDistortion
    % Use lower standard deviation to reduce smoothing in high distortion
    % images to prevent loss of features at the edges of Field-Of-View.
    sigma = 1.5;
else
    sigma = 2;
end

[points, boardSize] = vision.internal.calibration.checkerboard.detectCheckerboard(...
    I, sigma, minCornerMetric, highDistortion, usePartial);

% Use a larger kernel size if no points are detected
if isempty(points)
    sigma = 4;
    [points, boardSize] = vision.internal.calibration.checkerboard.detectCheckerboard(...
        I, sigma, minCornerMetric, highDistortion, usePartial);
end

% Replace missing corners with NaNs
if ~isempty(points) && usePartial
    zeroIdx = points(:,1) == 0;
    points(zeroIdx, :) = NaN;
end

%--------------------------------------------------------------------------
function waitBar = createProgressbar(numImages, parent)
    titleId = 'vision:calibrate:AnalyzingImagesTitle';
    messageId = 'vision:calibrate:detectCheckerboardWaitbar';
    tag = 'CheckerboardDetectionProgressBar';
    waitBar = vision.internal.uitools.ProgressBar(numImages, messageId, titleId,...
        tag, parent);
