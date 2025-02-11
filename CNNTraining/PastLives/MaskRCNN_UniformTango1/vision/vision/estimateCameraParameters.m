function [cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(varargin)

%   Copyright 2013-2024 The MathWorks, Inc.

if nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end

[imagePoints, worldPoints, imageSize, worldUnits, cameraModel, calibrationParams] = ...
    parseInputs(varargin{:});
calibrationParams.shouldComputeErrors = (nargout >= 3);

if size(imagePoints, 4) == 1 % single camera
    [cameraParams, imagesUsed, estimationErrors] = calibrateOneCamera(imagePoints, ...
        worldPoints, imageSize, cameraModel, worldUnits, calibrationParams);
else % 2-camera stereo
    [cameraParams, imagesUsed, estimationErrors] = calibrateTwoCameras(imagePoints,...
        worldPoints, imageSize, cameraModel, worldUnits, calibrationParams);
end

validateEstimatedResult(cameraParams);

%--------------------------------------------------------------------------
function [imagePoints, worldPoints, imageSize, worldUnits, cameraModel, calibrationParams] = ...
    parseInputs(varargin)
parser = inputParser;
parser.addRequired('imagePoints', @checkImagePoints);
parser.addRequired('worldPoints', @checkWorldPoints);
parser.addParameter('WorldUnits', 'mm', @checkWorldUnits);
parser.addParameter('EstimateSkew', false, @checkEstimateSkew);
parser.addParameter('EstimateTangentialDistortion', false, ...
    @checkEstimateTangentialDistortion);
parser.addParameter('NumRadialDistortionCoefficients', 2, ...
    @checkNumRadialDistortionCoefficients);
parser.addParameter('InitialK', [], @checkInitialK);
parser.addParameter('InitialIntrinsicMatrix', [], @checkInitialIntrinsicMatrix);
parser.addParameter('InitialRadialDistortion', [], @checkInitialRadialDistortion);
parser.addParameter('ShowProgressBar', false, @checkShowProgressBar);
parser.addParameter('ImageSize', zeros(0,2),...
                    @vision.internal.calibration.CameraParametersImpl.checkImageSize);

[parent, varargin] = checkProgressbarParent(varargin{:});

parser.parse(varargin{:});

imagePoints = parser.Results.imagePoints;
worldPoints = parser.Results.worldPoints;
if size(imagePoints, 1) ~= size(worldPoints, 1)
    error(message('vision:calibrate:numberOfPointsMustMatch'));
end

imageSize = parser.Results.ImageSize;

worldUnits  = parser.Results.WorldUnits;
cameraModel.EstimateSkew = parser.Results.EstimateSkew;
cameraModel.EstimateTangentialDistortion = ...
    parser.Results.EstimateTangentialDistortion;

% Check whether InitialK and InitialIntrinsicMatrix were used as input
% parameters
initialKDefined = ~ismember('InitialK', parser.UsingDefaults);
initialIntrinsicDefined = ~ismember('InitialIntrinsicMatrix', parser.UsingDefaults);

% Define the initiIntrinsics based on the name-value argument used. Only
% one intrinsics format is accepted at a time.
if initialKDefined && initialIntrinsicDefined
    error(message('vision:calibrate:bothIntrinsicFormatsProvided'));
elseif initialKDefined
    initIntrinsics = double(parser.Results.InitialK)';
else
    initIntrinsics = double(parser.Results.InitialIntrinsicMatrix);
end

initRadial = double(parser.Results.InitialRadialDistortion);

if ~isempty(initRadial) && ...
        any(strcmp('NumRadialDistortionCoefficients', parser.UsingDefaults))
    cameraModel.NumRadialDistortionCoefficients = numel(initRadial);
else
    cameraModel.NumRadialDistortionCoefficients = ...
        parser.Results.NumRadialDistortionCoefficients;
end

if ~isempty(initRadial) && ...
        cameraModel.NumRadialDistortionCoefficients ~= numel(initRadial)
    error(message('vision:calibrate:numRadialCoeffsDoesntMatch', ...
        'InitialRadialDistortion', 'NumRadialDistortionCoefficients'));
end
calibrationParams.initIntrinsics  = initIntrinsics;
calibrationParams.initRadial      = initRadial;
calibrationParams.showProgressBar = parser.Results.ShowProgressBar;
calibrationParams.parent = parent;

%--------------------------------------------------------------------------
function checkImagePoints(imagePoints)
% imagePoints can have missing keypoints (as NaNs) for partial detections.
allownan = true;
vision.internal.inputValidation.checkImagePoints(imagePoints, mfilename, allownan);

%--------------------------------------------------------------------------
function checkWorldPoints(worldPoints)
vision.internal.inputValidation.checkWorldPoints(worldPoints, mfilename);

%--------------------------------------------------------------------------
function checkWorldUnits(worldUnits)
validateattributes(worldUnits, {'char'}, {'vector'}, mfilename, 'worldUnits');

%--------------------------------------------------------------------------
function checkEstimateSkew(esitmateSkew)
validateattributes(esitmateSkew, {'logical'}, {'scalar'}, ...
    mfilename, 'EstimateSkew');

%--------------------------------------------------------------------------
function checkEstimateTangentialDistortion(estimateTangential)
validateattributes(estimateTangential, {'logical'}, {'scalar'}, mfilename, ...
    'EstimateTangentialDistortion');

%--------------------------------------------------------------------------
function checkNumRadialDistortionCoefficients(numRadialCoeffs)
validateattributes(numRadialCoeffs, {'numeric'}, ...
   {'scalar', 'integer', '>=', 2, '<=', 3}, ...
   mfilename, 'NumRadialDistortionCoefficients');

%--------------------------------------------------------------------------
function checkInitialK(K)
if ~isempty(K)
    validateattributes(K, {'single', 'double'}, ...
        {'real', 'nonsparse', 'finite', 'size', [3 3]}, ...
        mfilename, 'InitialK');
end

%--------------------------------------------------------------------------
function checkInitialIntrinsicMatrix(K)
if ~isempty(K)
    validateattributes(K, {'single', 'double'}, ...
        {'real', 'nonsparse', 'finite', 'size', [3 3]}, ...
        mfilename, 'InitialIntrisicMatrix');
end

%--------------------------------------------------------------------------
function checkInitialRadialDistortion(P)
if ~isempty(P)
    validateattributes(P, {'single', 'double'},...
        {'real', 'nonsparse', 'finite', 'vector'},...
        mfilename, 'InitialRadialDistortion');
    
    if numel(P) ~= 2
        validateattributes(P, {'single', 'double'}, {'numel', 3}, ...
            mfilename, 'InitialRadialDistortion');
    end
end

%--------------------------------------------------------------------------
function checkShowProgressBar(showProgressBar)
vision.internal.inputValidation.validateLogical(showProgressBar, 'ShowProgressBar');

%--------------------------------------------------------------------------
function [cameraParams, imagesUsed, errors] = calibrateOneCamera(imagePoints, ...
    worldPoints, imageSize, cameraModel, worldUnits, calibrationParams)

progressBar = vision.internal.calibration.createSingleCameraProgressBar(...
    calibrationParams.showProgressBar, calibrationParams.parent);

% compute the initial "guess" of intrinisc and extrinsic camera parameters
% in closed form ignoring distortion
[cameraParams, imagesUsed] = computeInitialParameterEstimate(...
    worldPoints, imagePoints, imageSize, cameraModel, worldUnits, ...
    calibrationParams.initIntrinsics, calibrationParams.initRadial);
imagePoints = imagePoints(:, :, imagesUsed);

progressBar.update();

% refine the initial estimate and compute distortion coefficients using
% non-linear least squares minimization
errors = refine(cameraParams, imagePoints, calibrationParams.shouldComputeErrors);
progressBar.update();
progressBar.delete();
%--------------------------------------------------------------------------
function [iniltialParams, validIdx] = computeInitialParameterEstimate(...
    worldPoints, imagePoints, imageSize, cameraModel, worldUnits, initIntrinsics, initRadial)
% Solve for the camera intriniscs and extrinsics in closed form ignoring
% distortion.

[H, validIdx] = computeHomographies(imagePoints, worldPoints);

if isempty(initIntrinsics)
    if ~isempty(imageSize)
        % assume zero skew and centered principal point for initial guess
        cx = (imageSize(2)-1)/2;
        cy = (imageSize(1)-1)/2;
        [fx, fy] = vision.internal.calibration.computeFocalLength(H, cx, cy);
        A = vision.internal.calibration.constructIntrinsicMatrix(fx, fy, cx, cy, 0);
        if ~isreal(A)
            error(message('vision:calibrate:complexCameraMatrix'));
        end
    else        
        V = computeV(H);
        B = computeB(V);
        A = computeIntrinsics(B);

        % Initialize principal points with centroid of image points if they
        % are negative.
        if A(1,3) < 0 || A(2,3) < 0
            A(1:2,3) = computeImagePointsCentroid(imagePoints);
        end
    end
else
    % initial guess for the intrinsics has been provided. No need to solve.
    A = initIntrinsics';
end

[rvecs, tvecs] = computeExtrinsics(A, H);

if isempty(initRadial)
    radialCoeffs = zeros(1, cameraModel.NumRadialDistortionCoefficients);
else
    radialCoeffs = initRadial;
end

iniltialParams = cameraParameters('K', A, 'RotationVectors', rvecs, ...
    'TranslationVectors', tvecs, 'WorldPoints', worldPoints, ...
    'WorldUnits', worldUnits, 'EstimateSkew', cameraModel.EstimateSkew,...
    'NumRadialDistortionCoefficients', cameraModel.NumRadialDistortionCoefficients,...
    'EstimateTangentialDistortion', cameraModel.EstimateTangentialDistortion,...
    'RadialDistortion', radialCoeffs, 'ImageSize', imageSize);

%--------------------------------------------------------------------------
function H = computeHomography(imagePoints, worldPoints)
% Compute projective transformation from worldPoints to imagePoints

validPointsIdx = ~isnan(imagePoints(:,1));

H = fitgeotrans(worldPoints(validPointsIdx,:), imagePoints(validPointsIdx,:), 'projective');
H = (H.T)';
H = H / H(3,3);


%--------------------------------------------------------------------------
function [homographies, validIdx] = computeHomographies(points, worldPoints)
% Compute homographies for all images

w1 = warning('Error', 'MATLAB:nearlySingularMatrix'); %#ok
w2 = warning('Error', 'images:maketform:conditionNumberofAIsHigh'); %#ok

numImages = size(points, 3);
validIdx = true(numImages, 1);
homographies = zeros(3, 3, numImages);
for i = 1:numImages
    try    
        homographies(:, :, i) = ...
            computeHomography(double(points(:, :, i)), worldPoints);
    catch 
        validIdx(i) = false;
    end
end
warning(w1);
warning(w2);
homographies = homographies(:, :, validIdx);
if ~all(validIdx)
    warning(message('vision:calibrate:invalidHomographies', ...
        numImages - size(homographies, 3), numImages));
end

if size(homographies, 3) < 2
    error(message('vision:calibrate:notEnoughValidHomographies'));
end

%--------------------------------------------------------------------------
function [parent, varargin] = checkProgressbarParent(varargin)
if nargin > 2 && (isa(varargin{3}, 'matlab.ui.container.internal.AppContainer') || isempty(varargin{3}))
    parent = varargin{3};
    varargin(3) = [];
else
    parent = [];
end

%--------------------------------------------------------------------------
function V = computeV(homographies)
% Vb = 0

numImages = size(homographies, 3);
V = zeros(2 * numImages, 6);
for i = 1:numImages
    H = homographies(:, :, i)';
    V(i*2-1,:) = computeLittleV(H, 1, 2);
    V(i*2, :) = computeLittleV(H, 1, 1) - computeLittleV(H, 2, 2);
end

%--------------------------------------------------------------------------
function v = computeLittleV(H, i, j)
    v = [H(i,1)*H(j,1), H(i,1)*H(j,2)+H(i,2)*H(j,1), H(i,2)*H(j,2),...
         H(i,3)*H(j,1)+H(i,1)*H(j,3), H(i,3)*H(j,2)+H(i,2)*H(j,3), H(i,3)*H(j,3)];

%--------------------------------------------------------------------------     
function B = computeB(V)
% lambda * B = inv(A)' * inv(A), where A is the intrinsic matrix

[~, ~, U] = svd(V);
b = U(:, end);

% b = [B11, B12, B22, B13, B23, B33]
B = [b(1), b(2), b(4); b(2), b(3), b(5); b(4), b(5), b(6)];

%--------------------------------------------------------------------------
function A = computeIntrinsics(B)
% Compute the intrinsic matrix

cy = (B(1,2)*B(1,3) - B(1,1)*B(2,3)) / (B(1,1)*B(2,2)-B(1,2)^2);
lambda = B(3,3) - (B(1,3)^2 + cy * (B(1,2)*B(1,3) - B(1,1)*B(2,3))) / B(1,1);
fx = sqrt(lambda / B(1,1));
fy = sqrt(lambda * B(1,1) / (B(1,1) * B(2,2) - B(1,2)^2));
skew = -B(1,2) * fx^2 * fy / lambda;
cx = skew * cy / fy - B(1,3) * fx^2 / lambda;
A = vision.internal.calibration.constructIntrinsicMatrix(fx, fy, cx, cy, skew);
if ~isreal(A)
    error(message('vision:calibrate:complexCameraMatrix'));
end

%--------------------------------------------------------------------------
function centroid = computeImagePointsCentroid(imagePoints)
% Compute the centroid of all image points. 
% imagePoints is a numTargetPoints-by-2-numImages-by-numCameras vector
% centroid is a 1-by-2 vector

% Reorder the dimensions of imagePoints to 
% numTargetPoints-by-numImages-by-numCameras-by-2.
allImagePoints = permute(imagePoints, [1 3 4 2]);

% Reshape the imagePoints to numTargetPoints*numImages*numCameras-by-2
sz = size(allImagePoints);
allImagePoints = reshape(allImagePoints, [prod(sz(1:3)), sz(4)]);

% Remove any NaN points that correspond to partial detections.
validDetectionsIdx = ~any(isnan(allImagePoints),2);
validImagePoints = allImagePoints(validDetectionsIdx,:);

% Compute centroid using valid image points.
centroid = mean(validImagePoints);

%--------------------------------------------------------------------------
function [rotationVectors, translationVectors] = ...
    computeExtrinsics(A, homographies)
% Compute translation and rotation vectors for all images

numImages = size(homographies, 3);
rotationVectors = zeros(3, numImages);
translationVectors = zeros(3, numImages); 
Ainv = inv(A);
for i = 1:numImages
    H = homographies(:, :, i);
    h1 = H(:, 1);
    h2 = H(:, 2);
    h3 = H(:, 3);
    lambda = 1 / norm(Ainv * h1); %#ok
    
    % 3D rotation matrix
    r1 = lambda * Ainv * h1; %#ok
    r2 = lambda * Ainv * h2; %#ok
    r3 = cross(r1, r2);
    R = [r1,r2,r3];
    
    rotationVectors(:, i) = vision.internal.calibration.rodriguesMatrixToVector(R);
    
    % translation vector
    t = lambda * Ainv * h3;  %#ok
    translationVectors(:, i) = t;
end

rotationVectors = rotationVectors';
translationVectors = translationVectors';

%--------------------------------------------------------------------------
function [stereoParams, pairsUsed, errors] = calibrateTwoCameras(imagePoints,...
    worldPoints, imageSize, cameraModel, worldUnits, calibrationParams)

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
    worldPoints, imageSize, cameraModel, worldUnits, calibrationParams);

progressBar.update();

[cameraParameters2, imagesUsed2] = calibrateOneCamera(imagePoints2, ...
    worldPoints, imageSize, cameraModel, worldUnits, calibrationParams);

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

tform = rigidtform3d(R, t);

stereoParams = stereoParameters(cameraParameters1, cameraParameters2, tform);

errors = refine(stereoParams, imagePoints1(:, :, pairsUsed), ...
    imagePoints2(:, :, pairsUsed), shouldComputeErrors);

progressBar.update();
delete(progressBar);

%--------------------------------------------------------------------------
function validateEstimatedResult(result)

if isa(result, 'stereoParameters')
    principalPoint1 = result.CameraParameters1.PrincipalPoint;
    principalPoint2 = result.CameraParameters2.PrincipalPoint;
    isPrincipalPointNegative = any(principalPoint1 < 0) || any(principalPoint2 < 0);
else % result is a cameraParameters object.
    isPrincipalPointNegative = any(result.PrincipalPoint < 0);
end

if isPrincipalPointNegative
    error(message('vision:calibrate:convergedToNegativePrincipalPoint'));
end