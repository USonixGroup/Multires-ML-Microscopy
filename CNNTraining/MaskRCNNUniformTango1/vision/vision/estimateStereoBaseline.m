function [stereoParams, pairsUsed, estimationErrors] = estimateStereoBaseline(varargin)

%   Copyright 2017-2024 The MathWorks, Inc.

[imagePoints, worldPoints, intrinsics1, intrinsics2, worldUnits, calibrationParams] = ...
    parseInputs(varargin{:});
calibrationParams.shouldComputeErrors = (nargout >= 3);

[stereoParams, pairsUsed, estimationErrors] = calibrateTwoCameras(imagePoints,...
    worldPoints, intrinsics1, intrinsics2, worldUnits, calibrationParams);

%--------------------------------------------------------------------------
function [imagePoints, worldPoints, intrinsics1, intrinsics2, ...
    worldUnits, calibrationParams] = parseInputs(varargin)
parser = inputParser;
parser.addRequired('imagePoints', @checkImagePoints);
parser.addRequired('worldPoints', @checkWorldPoints);
parser.addRequired('intrinsics1', @checkIntrinsics);
parser.addRequired('intrinsics2', @checkIntrinsics);
parser.addParameter('WorldUnits', 'mm', @checkWorldUnits);
parser.addParameter('ShowProgressBar', false, @checkShowProgressBar);

[parent, varargin] = checkProgressbarParent(varargin{:});
parser.parse(varargin{:});

imagePoints = double(parser.Results.imagePoints);
worldPoints = double(parser.Results.worldPoints);
intrinsics1 = parser.Results.intrinsics1;
intrinsics2 = parser.Results.intrinsics2;
worldUnits  = parser.Results.WorldUnits;

if size(imagePoints, 1) ~= size(worldPoints, 1)
    error(message('vision:calibrate:numberOfPointsMustMatch'));
end

if ~isequal(intrinsics1.ImageSize,intrinsics2.ImageSize)
    error(message('vision:calibrate:sizeOfImageMustMatch'));
end

calibrationParams.showProgressBar = parser.Results.ShowProgressBar;
calibrationParams.parent = parent;

%--------------------------------------------------------------------------
function checkImagePoints(imagePoints)
allownan = true;
vision.internal.inputValidation.checkImagePoints(imagePoints, mfilename, allownan);

%--------------------------------------------------------------------------
function checkWorldPoints(worldPoints)
vision.internal.inputValidation.checkWorldPoints(worldPoints, mfilename);

%--------------------------------------------------------------------------
function checkIntrinsics(intrinsics)
vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    intrinsics, true, mfilename);

%--------------------------------------------------------------------------
function checkWorldUnits(worldUnits)
if isstring(worldUnits)
    validateattributes(worldUnits, {'string'}, ...
        {'scalar'}, mfilename, 'WorldUnits');
else
    validateattributes(worldUnits, {'char'}, ...
        {'nonempty','vector'}, mfilename, 'WorldUnits');
end

%--------------------------------------------------------------------------
function checkShowProgressBar(showProgressBar)
vision.internal.inputValidation.validateLogical(showProgressBar, 'ShowProgressBar');

%--------------------------------------------------------------------------
function [cameraParams, imagesUsed] = calibrateOneCamera(imagePoints, ...
    worldPoints, intrinsics, worldUnits, calibrationParams)

progressBar = vision.internal.calibration.createSingleCameraProgressBar(calibrationParams.showProgressBar);

% compute the initial "guess" of extrinsic camera parameters
% in closed form ignoring distortion
[rvecs, tvecs, imagesUsed] = computeExtrinsics(imagePoints, worldPoints, intrinsics.IntrinsicMatrix);

cameraParams = cameraParameters(...
    'IntrinsicMatrix', intrinsics.IntrinsicMatrix, ...
    'RotationVectors', rvecs(imagesUsed, :), ...
    'TranslationVectors', tvecs(imagesUsed, :), ...
    'WorldPoints', worldPoints, ...
    'WorldUnits', worldUnits, ...
    'EstimateSkew', (intrinsics.Skew ~= 0),...
    'NumRadialDistortionCoefficients', numel(intrinsics.RadialDistortion),...
    'RadialDistortion', intrinsics.RadialDistortion,...
    'EstimateTangentialDistortion', ~isempty(intrinsics.TangentialDistortion),...
    'TangentialDistortion', intrinsics.TangentialDistortion, ...
    'ImageSize', intrinsics.ImageSize);

progressBar.update();
progressBar.delete();

%--------------------------------------------------------------------------
function [rotationVectors, translationVectors, imagesUsed] = ...
    computeExtrinsics(imagePoints, worldPoints, A)
% Compute translation and rotation vectors for all images

w1 = warning('Error', 'MATLAB:nearlySingularMatrix'); %#ok
w2 = warning('Error', 'images:maketform:conditionNumberofAIsHigh'); %#ok

numImages = size(imagePoints, 3);
rotationVectors = zeros(3, numImages);
translationVectors = zeros(3, numImages);
imagesUsed = true(numImages, 1);

for i = 1:numImages
    try
        validDetectionsIdx = ~any(isnan(imagePoints(:,:,i)),2);
        validImagePoints = imagePoints(validDetectionsIdx,:,i);
        validWorldPoints = worldPoints(validDetectionsIdx,:);
        [R, t] = vision.internal.calibration.extrinsicsPlanar(...
                validImagePoints, validWorldPoints, A');
        rotationVectors(:, i) = vision.internal.calibration.rodriguesMatrixToVector(R);    
        translationVectors(:, i) = t;
    catch
        imagesUsed(i) = false;
    end
end

warning(w1);
warning(w2);

if ~all(imagesUsed)
    warning(message('vision:calibrate:invalidHomographies', ...
        numImages - sum(imagesUsed), numImages));
end

if sum(imagesUsed) < 2
    error(message('vision:calibrate:notEnoughValidHomographies'));
end

rotationVectors = rotationVectors';
translationVectors = translationVectors';

%--------------------------------------------------------------------------
function [parent, varargin] = checkProgressbarParent(varargin)
if nargin > 4 && (isa(varargin{5}, 'matlab.ui.container.internal.AppContainer') || isempty(varargin{5}))
    parent = varargin{5};
    varargin(5) = [];
else 
    parent = [];
end

%--------------------------------------------------------------------------
function [stereoParams, pairsUsed, errors] = calibrateTwoCameras(imagePoints,...
    worldPoints, intrinsics1, intrinsics2, worldUnits, calibrationParams)

imagePoints1 = imagePoints(:, :, :, 1);
imagePoints2 = imagePoints(:, :, :, 2);

showProgressBar = calibrationParams.showProgressBar;
progressBar = vision.internal.calibration.createStereoCameraProgressBar(...
    showProgressBar, calibrationParams.parent);
calibrationParams.showProgressBar = false;

% Calibrate each camera separately
shouldComputeErrors = calibrationParams.shouldComputeErrors;
calibrationParams.shouldComputeErrors = false;
[cameraParameters1, imagesUsed1] = calibrateOneCamera(imagePoints1, ...
    worldPoints, intrinsics1, worldUnits, calibrationParams);

progressBar.update();

[cameraParameters2, imagesUsed2] = calibrateOneCamera(imagePoints2, ...
    worldPoints, intrinsics2, worldUnits, calibrationParams);

progressBar.update();

% Account for possible mismatched pairs
pairsUsed = imagesUsed1 & imagesUsed2;
cameraParameters1 = vision.internal.calibration.removeUnusedExtrinsics(...
    cameraParameters1, pairsUsed, imagesUsed1);
cameraParameters2 = vision.internal.calibration.removeUnusedExtrinsics(...
    cameraParameters2, pairsUsed, imagesUsed2);

% Compute the initial estimate of translation and rotation of camera 2
[R, t] = vision.internal.calibration.estimateInitialTranslationAndRotation(...
    cameraParameters1, cameraParameters2);

stereoParams = stereoParameters(cameraParameters1, cameraParameters2, R, t);

isIntrinsicsFixed = true;
errors = refine(stereoParams, imagePoints1(:, :, pairsUsed), ...
    imagePoints2(:, :, pairsUsed), shouldComputeErrors, isIntrinsicsFixed);

progressBar.update();
delete(progressBar);
