function [fisheyeParams, imagesUsed, estimationErrors] = estimateFisheyeParameters(varargin)

%   Copyright 2017-2023 The MathWorks, Inc.

[imagePoints, worldPoints, imageSize, worldUnits, estimateAlignment, ...
    calibrationParams] = parseInputs(varargin{:});
calibrationParams.shouldComputeErrors = (nargout >= 2);

progressBar = vision.internal.calibration.createSingleCameraProgressBar(...
    calibrationParams.showProgressBar, calibrationParams.parent);

% Compute the initial "guess" of intrinsic and extrinsic camera parameters
[fisheyeParams, imagesUsed] = computeInitialParameters(worldPoints, ...
            imagePoints, imageSize, worldUnits, estimateAlignment);

progressBar.update();

% Refine the initial estimate using non-linear least squares minimization
estimationErrors = refine(fisheyeParams, imagePoints, ...
    calibrationParams.shouldComputeErrors);

progressBar.update();
progressBar.delete();

%--------------------------------------------------------------------------
function [imagePoints, worldPoints, imageSize, worldUnits, ...
    estimateAlignment, calibrationParams] = parseInputs(varargin)
parser = inputParser;
parser.addRequired('imagePoints', @checkImagePoints);
parser.addRequired('worldPoints', @checkWorldPoints);
parser.addRequired('ImageSize', @vision.internal.calibration.CameraParametersImpl.checkImageSize);
parser.addParameter('WorldUnits', 'mm', @checkWorldUnits);
parser.addParameter('EstimateAlignment', false, @checkEstimateAlignment);
parser.addParameter('ShowProgressBar', false, @checkShowProgressBar);

[parent, varargin] = checkProgressbarParent(varargin{:});
parser.parse(varargin{:});

imagePoints = parser.Results.imagePoints;
worldPoints = parser.Results.worldPoints;
if size(imagePoints, 1) ~= size(worldPoints, 1)
    error(message('vision:calibrate:numberOfPointsMustMatch'));
end

imageSize = parser.Results.ImageSize;
worldUnits  = parser.Results.WorldUnits;
estimateAlignment = parser.Results.EstimateAlignment;
calibrationParams.showProgressBar = parser.Results.ShowProgressBar;
calibrationParams.parent = parent;

%--------------------------------------------------------------------------
function checkImagePoints(imagePoints)
% Missing image points are NaNs
allownan = true;

vision.internal.inputValidation.checkImagePoints(imagePoints, mfilename, allownan);


%--------------------------------------------------------------------------
function checkWorldPoints(worldPoints)
vision.internal.inputValidation.checkWorldPoints(worldPoints, mfilename);

%--------------------------------------------------------------------------
function checkWorldUnits(worldUnits)
if isstring(worldUnits)
    validateattributes(worldUnits, {'string'}, ...
        {'scalar'}, mfilename, 'WorldUnits');
else
    validateattributes(worldUnits, {'char'}, ...
        {'vector'}, mfilename, 'WorldUnits');
end

%--------------------------------------------------------------------------
function checkEstimateAlignment(estimateAlignment)
validateattributes(estimateAlignment, {'logical'}, {'scalar'}, ...
                    mfilename, 'EstimateAlignment');

%--------------------------------------------------------------------------
function checkShowProgressBar(showProgressBar)
vision.internal.inputValidation.validateLogical(showProgressBar, 'ShowProgressBar');

%--------------------------------------------------------------------------
function [parent, varargin] = checkProgressbarParent(varargin)
if nargin > 3 && (isa(varargin{4}, 'matlab.ui.container.internal.AppContainer') || isempty(varargin{4}))
    parent = varargin{4};
    varargin(4) = [];
else
    parent = [];
end

%--------------------------------------------------------------------------
function [params, imagesUsed] = computeInitialParameters(worldPoints, ...
        imagePoints, imageSize, worldUnits, estimateAlignment)
% Find partial extrinsics, r1, r2, tx, ty
[extrinsics, imagesUsed] = computeInitialExtrinsics(imagePoints, ...
                                                worldPoints, imageSize);
imagePoints = imagePoints(:, :, imagesUsed);
extrinsics = extrinsics(:, :, imagesUsed);

% Find intrinsics and partial extrinsics tz
[intrinsics, extrinsics] = estimateIntrinsicsAndTranslationOfZ(...
    worldPoints, imagePoints, imageSize, extrinsics);

% Find r3
[rvecs, tvecs] = computeFullExtrinsics(extrinsics);

numImages = numel(imagesUsed);
allrvecs = zeros(numImages, 3);
alltvecs = allrvecs;
allrvecs(imagesUsed, :) = rvecs;
alltvecs(imagesUsed, :) = tvecs;

obj = fisheyeIntrinsics(intrinsics, imageSize, imageSize([2, 1])/2);
params = fisheyeParameters(obj, ...
    'RotationVectors', allrvecs, ...
    'TranslationVectors', alltvecs, ...
    'WorldPoints', worldPoints, ...
    'WorldUnits', worldUnits, ...
    'EstimateAlignment', estimateAlignment);

%--------------------------------------------------------------------------
function [Es, imagesUsed] = computeInitialExtrinsics(imagePoints, worldPoints, imageSize)
% Initial estimation of camera extrinsics
cx = imageSize(2)/2;
cy = imageSize(1)/2;
X = worldPoints(:, 1);
Y = worldPoints(:, 2);
numImages = size(imagePoints, 3);
Es = zeros(3, 3, numImages);
imagesUsed = true(numImages, 1);
for k = 1 : numImages
    
    % Get valid imagePoints (all NaN coordinates are invalid)
    validImagePointsIdx = ~isnan(imagePoints(:,1,k));
    
    u = imagePoints(validImagePointsIdx, 1, k) - cx;
    v = imagePoints(validImagePointsIdx, 2, k) - cy;
    
    % Set world points to use for the estimation
    currX = X(validImagePointsIdx);
    currY = Y(validImagePointsIdx);
    
    % Build the system of linear equations (Scaramuzza et al, eq. 11)
    M = [currX.*v, currY.*v, -currX.*u, -currY.*u, v, -u];
    [~,~,V] = svd(M);
    
    % Solve the rotation and translation
    r11 = V(1, end);
    r12 = V(2, end);
    r21 = V(3, end);
    r22 = V(4, end);
    t1 = V(5, end);
    t2 = V(6, end);

    % r1, r2 are orthonormal so we can have three equations as follows:
    % |r1| = |r2| = lambda, and r1' * r2 = 0
    % To solve r3, we solve the derived equation:
    % r32^4 + (CC - BB) r32^2 - AA = 0, 
    % where AA = ((r11*r12)+(r21*r22))^2, BB = r11^2 + r21^2, CC = r12^2 + r22^2.
    A = (r11*r12+r21*r22);
    AA = A^2;
    BB = r11^2 + r21^2;
    CC = r12^2 + r22^2;
    
    r32_2 = roots([ 1, CC-BB, -AA]);
    r32_2 = r32_2(r32_2>=0);

    % r3 may have multiple numeric solutions
    r31 = [];
    r32 = [];
    for i = 1 : length(r32_2)
        if r32_2(i) == 0
            temp = sqrt(CC-BB);
            r31 = [r31; temp; -temp];
            r32 = [r32; 0; 0];
        else
            temp = sqrt(r32_2(i));
            r32 = [r32; temp; -temp];
            r31 = [r31; -A/temp; A/temp];
        end        
    end
    
    % Solve lambda: r31^2 + BB = lambda^2
    E = zeros(3, 3, length(r32)*2);
    for i = 1:length(r32)
        lambda = sqrt(BB+r31(i)^2)\1;
        R = lambda * [  r11     r12     t1; ...
                        r21     r22     t2; ...
                        r31(i)  r32(i)  0];
        E(:,:,2*i-1) = R; 
        E(:,:,2*i) = -R; 
    end
    
    % Pick the extrinsics that place the calibration points in front of
    % the camera
    bestE = chooseExtrinsics(E, currX, currY, u, v);
    if ~isempty(bestE)
        Es(:,:,k) = bestE;
    else
        imagesUsed(k) = false;
    end
end

%--------------------------------------------------------------------------
function bestE = chooseExtrinsics(E, X, Y, Xp, Yp)
% Among multiple ambiguous solutions, choose the one that places the
% checkerboard in front of the camera, where the last (highest degree)
% polynomial coefficient is negative.

% Pick t1, t2 that are pointing to calibration points.
tt = squeeze(E(1:2,3,:));
% The first image point corresponds to the origin of the world coordinate
% system.
d = tt - [Xp(1); Yp(1)]; 
d = sum(d.*d);
[~, ind] = min(d);

indices = [];
for i = 1 : size(tt, 2)
    if sign(tt(1, i))==sign(tt(1, ind)) && sign(tt(2, i))==sign(tt(2, ind))
        indices = [indices; i];
    end
end

E = E(:, :, indices);
bestE = [];
for i = 1 : size(E, 3)
    e = E(:,:,i);
    % Use only quadratic polynomial to validate coefficients.
    % (Scaramuzza et al, eq. 13)
    r11 = e(1,1);
    r21 = e(2,1);
    r31 = e(3,1);
    r12 = e(1,2);
    r22 = e(2,2);
    r32 = e(3,2);
    t1  = e(1,3);
    t2  = e(2,3);
    
    A = r21.*X + r22.*Y + t2;
    B = Yp.*( r31.*X + r32.*Y );
    C = r11.*X + r12.*Y + t1;
    D = Xp.*( r31.*X + r32.*Y );
    rho = sqrt(Xp.^2 + Yp.^2);
    rho2 = (Xp.^2 + Yp.^2);
    
    P = [A, A.*rho, A.*rho2, -Yp; ...
         C, C.*rho, C.*rho2, -Xp];        
    Q = [B; D];
    
    s = pinv(P) * Q;
    if s(3) <= 0 % Check the sign of the quadratic polynomial.
        bestE = e;
    end
end

%--------------------------------------------------------------------------
function [intrinsics, extrinsics] = estimateIntrinsicsAndTranslationOfZ(...
    worldPoints, imagePoints, imageSize, extrinsics)
cx = imageSize(2)/2;
cy = imageSize(1)/2;
X = worldPoints(:, 1);
Y = worldPoints(:, 2);
numImages = size(imagePoints, 3);

numPoints = nnz(~isnan(imagePoints(:,1,:)));
P = zeros(numPoints*2, 4+numImages);
Q = zeros(numPoints*2, 1);

% Solve for intrinsics (Scaramuzza et al, eq. 13)
ptCounter = 0; % Point counter
for k = 1 : numImages
    E = extrinsics(:,:,k);
    r11 = E(1,1);
    r21 = E(2,1);
    r31 = E(3,1);
    r12 = E(1,2);
    r22 = E(2,2);
    r32 = E(3,2);
    t1  = E(1,3);
    t2  = E(2,3);

    % Get valid imagePoints (all NaN-valued coordinates are invalid)
    validImagePointsIdx = ~isnan(imagePoints(:,1,k));
    numValidPoints = nnz(validImagePointsIdx);
    
    u = imagePoints(validImagePointsIdx, 1, k) - cx;
    v = imagePoints(validImagePointsIdx, 2, k) - cy;
    
    % Set world points to use for the estimation
    currX = X(validImagePointsIdx);
    currY = Y(validImagePointsIdx);

    A = r21.*currX + r22.*currY + t2;
    B = v.*( r31.*currX + r32.*currY );
    C = r11.*currX + r12.*currY + t1;
    D = u.*( r31.*currX + r32.*currY );
    rho = sqrt(u.^2 + v.^2);
    rho2 = rho.*rho;
    rho3 = rho.^3;
    rho4 = rho.^4;
    
    M = [A, A.*rho2, A.*rho3, A.*rho4; ...
         C, C.*rho2, C.*rho3, C.*rho4];        
    
    firstRow = ptCounter + 1;
    lastRow  = ptCounter + 2 * numValidPoints;
    P(firstRow:lastRow, 1:4) = M;
    P(firstRow:lastRow, 4+k) = [-v; -u];
    Q(firstRow:lastRow) = [B; D];
    
    ptCounter = ptCounter + 2 * numValidPoints;
end

% Set the constraints to make the radial distortion function monotonically
% increasing, and convex. This ensures the distortion gets larger for the
% pixels that are far away from the center of distortion.
numSkips = 5;
rhoSamples = 1:numSkips:round(max(imagePoints(:)));
numSamples = numel(rhoSamples);
rhoMono = [rhoSamples; rhoSamples.^2; rhoSamples.^3]';

% The first derivative is non-negative
% 0*a0 + a1(not needed) + 2*a2*rho + 3*a3*rho^2 + 4*a4*rho^3 >= 0
mono = [zeros(numSamples, 1), 2 * rhoMono(:, 1), 3 * rhoMono(:, 2), 4 * rhoMono(:, 3)];

% The second derivative is non-negative
% 0*a0 + 0*a1(not needed) + 2*a2 + 6*a3*rho + 12*a4*rho^2 >= 0
mono = [mono; zeros(numSamples, 1), 2 * ones(numSamples, 1), 6 * rhoMono(:, 1), 12 * rhoMono(:, 2)]; 

% Set the constraints
A = [mono, zeros(size(mono,1),size(P,2)-size(mono,2))];
b = zeros(size(mono,1),1);   

options = optimset('Display','off');
x = ilslnsh(P,Q,A,b,[],[],[],[],[],options);

intrinsics = x(1:4);
for k = 1 : numImages
    extrinsics(3,3,k) = x(4+k);
end    

%--------------------------------------------------------------------------
function [rotationVectors, translationVectors] = computeFullExtrinsics(extrinsics)
% Compute translation and rotation vectors for all images

numImages = size(extrinsics, 3);
rotationVectors = zeros(numImages, 3);
translationVectors = zeros(numImages, 3); 
for i = 1:numImages
    E = extrinsics(:, :, i);
    t  = E(:, 3);
    
    % 3D rotation matrix
    r1 = E(:, 1);
    r2 = E(:, 2);
    r3 = cross(r1, r2);    
    R = [r1, r2, r3];
    
    rotationVectors(i, :) = vision.internal.calibration.rodriguesMatrixToVector(R);
    
    translationVectors(i, :) = t;
end



