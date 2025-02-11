%estimateEssentialMatrix Estimate essential matrix
%  estimateEssentialMatrix estimates the essential matrix from
%  corresponding points in a pair of images using the M-estimator SAmple
%  Consensus (MSAC) algorithm.
%
%  E = estimateEssentialMatrix(matchedPoints1, matchedPoints2, cameraParams) 
%  returns the 3-by-3 essential matrix E. matchedPoints1 and matchedPoints2
%  can be M-by-2 matrices of [x,y] coordinates or objects of any of the
%  point feature types (cornerPoints, BRISKPoints, SIFTPoints, SURFPoints,
%  ORBPoints, KAZEPoints, MSERRegions). matchedPoints1 and matchedPoints2
%  must contain at least 5 points which are putatively matched by using a
%  function such as matchFeatures. cameraParams is a cameraParameters or
%  cameraIntrinsics object, containing the parameters of the camera used to
%  take the images.
%
%  E = estimateEssentialMatrix(matchedPoints1, matchedPoints2,
%  cameraParams1, cameraParams2) returns the essential matrix relating two
%  images taken by different cameras. cameraParams1 and cameraParams2 are
%  cameraParameters or cameraIntrinsics objects containing the parameters
%  of camera 1 and camera 2 respectively.
%
%  [E, inliersIndex] = estimateEssentialMatrix(...) additionally returns
%  the indices of the inliers used to compute the essential matrix.
%  inliersIndex is an M-by-1 logical vector, with the values of true
%  corresponding to the inliers.
%
%  [E, inliersIndex, status] = estimateEssentialMatrix(...) additionally
%  returns a status code. If the status output is not specified, the
%  function will issue an error if the number of matched points or the
%  number of inliers is less than 5. The status can have the following
%  values:
%
%     0: No error.
%     1: matchedPoints1 and matchedPoints2 do not contain enough points.
%        At least 5 points are required.
%     2: Not enough inliers found. At least 5 inliers are required.
%
%  [...] = estimateEssentialMatrix(..., Name, Value) specifies additional
%  name-value pair arguments described below:
%
%   'MaxNumTrials'       Positive integer scalar.
%                        Specifies the number of random trials for finding
%                        the outliers. The actual number of trials depends
%                        on matchedPoints1, matchedPoints2, and the values
%                        of the MaxDistance and Confidence parameters.
%                        Increasing this value will improve the robustness
%                        of the output at the expense of additional
%                        computation.
%
%                        Default: 1000
%
%   'Confidence'         Scalar value greater than 0 and less than 100.
%                        Specifies the desired confidence (in percentage)
%                        for finding the maximum number of inliers.
%                        Increasing this value will improve the robustness
%                        of the output at the expense of additional
%                        computation.
%
%                        Default: 99
%
%   'MaxDistance'        Positive numeric scalar specifying the Sampson
%                        distance threshold for finding outliers in pixels
%                        squared. The Sampson distance is a first order
%                        approximation of the squared geometric distance
%                        between a point and the epipolar line. Increasing
%                        this value will make the algorithm converge faster,
%                        but may adversely affect the accuracy of the result.
%
%                        Default: 0.1
%
%  Class Support
%  -------------
%  matchedPoints1 and matchedPoints2 can be double or single when specified
%  as an M-by-2 matrix. cameraParams must be a cameraParameters or 
%  cameraIntrinsics object.
%
%  Example: Estimate essential matrix from a pair of images
%  --------------------------------------------------------
%  % Load precomputed camera parameters
%  load upToScaleReconstructionCameraParameters.mat
%
%  % Read and undistort two images
%  imageDir = fullfile(toolboxdir('vision'), 'visiondata','upToScaleReconstructionImages');
%  images = imageDatastore(imageDir);
%  I1 = undistortImage(readimage(images, 1), cameraParams);
%  I2 = undistortImage(readimage(images, 2), cameraParams);
%  I1gray = rgb2gray(I1);
%  I2gray = rgb2gray(I2);
%
%  % Detect feature points
%  imagePoints1 = detectSURFFeatures(I1gray);
%  imagePoints2 = detectSURFFeatures(I2gray);
%
%  % Extract feature descriptors
%  features1 = extractFeatures(I1gray, imagePoints1, 'Upright', true);
%  features2 = extractFeatures(I2gray, imagePoints2, 'Upright', true);
%
%  % Match features
%  indexPairs = matchFeatures(features1, features2);
%  matchedPoints1 = imagePoints1(indexPairs(:, 1));
%  matchedPoints2 = imagePoints2(indexPairs(:, 2));
%  figure
%  showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);
%  title('Putative Matches')
%
%  % Estimate essential matrix
%  [E, inliers] = estimateEssentialMatrix(matchedPoints1, matchedPoints2,...
%    cameraParams);
%
%  % Display inlier matches
%  inlierPoints1 = matchedPoints1(inliers);
%  inlierPoints2 = matchedPoints2(inliers);
%  figure
%  showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
%  title('Inlier Matches')
%
%  See also estimateFundamentalMatrix, relativeCameraPose, cameraCalibrator,
%           estimateCameraParameters, estimateWorldCameraPose

%  Copyright 2015-2024 The MathWorks, Inc.

% References:
% -----------
% [1] Z. Kukelova, M. Bujnak, T. Pajdla, Polynomial eigenvalue solutions to
% the 5-pt and 6-pt relative pose problems, BMVC 2008, Leeds, UK, 2008.
%
% [2] David Nister, An Efficient Solution to the Five-Point Relative Pose
% Problem, IEEE Transactions on Pattern Analysis and Machine Intelligence,
% Volume 26, Issue 6, June 2004.

%#codegen

function [E, inlierIdx, status] = estimateEssentialMatrix(matchedPoints1, ...
    matchedPoints2, varargin)

% List of status codes
statusCode = struct(...
    'NoError',           int32(0),...
    'NotEnoughPts',      int32(1),...
    'NotEnoughInliers',  int32(2));

[points1, points2, K1, K2, params, outputClass] = parseInputs(matchedPoints1, ...
    matchedPoints2, varargin{:});

numPoints = size(points1, 1);
sampleSize = 5;

if numPoints < sampleSize
    % default outputs
    E = eye(3, outputClass);
    inlierIdx = false(numPoints, 1);
    status = statusCode.NotEnoughPts;
else
    [pts1norm, pts2norm] = pixelsToNormalizedCoords(points1, points2, K1, K2);

    params.sampleSize = sampleSize;
    params.recomputeModelFromInliers = true;

    funcs.fitFunc = @fivePointAlgorithm;
    funcs.evalFunc = @evalEssential;
    funcs.checkFunc = @check;

    [isFound, E, inlierIdx] = vision.internal.ransac.msac(...
        cat(3, pts1norm, pts2norm), params, funcs, K1, K2, outputClass);

    if isFound
        status = statusCode.NoError;
        E = cast(normalizeEssentialMatrix(E), outputClass);
    else
        % default outputs
        E = eye(3, outputClass);
        inlierIdx = false(numPoints, 1);
        status = statusCode.NotEnoughInliers;
    end
end

if nargout < 3
    checkRuntimeStatus(statusCode, status)
end
end

%==========================================================================
% Check runtime status and report error if there is one
%==========================================================================
function checkRuntimeStatus(statusCode, status)
coder.internal.errorIf(status==statusCode.NotEnoughPts, ...
    'vision:points:notEnoughMatchedPts', 'matchedPoints1', ...
    'matchedPoints2', 5);

coder.internal.errorIf(status==statusCode.NotEnoughInliers, ...
    'vision:points:notEnoughInlierMatches', 'matchedPoints1', ...
    'matchedPoints2');
end

%--------------------------------------------------------------------------
function distances = evalEssential(Es, x, varargin)
K1 = varargin{1};
K2 = varargin{2};
outputClass = varargin{3};

x1 = (x(:,:,1) * K1)';
x2 = (x(:,:,2) * K2)';

numSolutions = numel(Es);
distances = zeros(size(x1, 2), numSolutions, outputClass);

for i = 1:numSolutions
    F = K2 \ Es{i} / K1';
    Fx1 = F*x1;
    epipolarDists = sum(x2 .* Fx1);
    Ftx2 = F'*x2;

    % Sampson distance
    distances(:, i) =  epipolarDists.^2 ./ ...
        (sum(Fx1(1:2, :).^2) +  sum(Ftx2(1:2,:).^2));
end
end


%--------------------------------------------------------------------------
% Convert the points from pixels no homogeneous normalized image
% coordinates
function [pts1norm, pts2norm] = pixelsToNormalizedCoords(points1, points2,...
    K1, K2)
numPoints = size(points1, 1);
pts1 = [points1, ones(numPoints, 1)];
pts2 = [points2, ones(numPoints, 1)];

pts1norm = pts1 / K1;
pts2norm = pts2 / K2;
end

%--------------------------------------------------------------------------
function E = normalizeEssentialMatrix(E)
if iscell(E)
    E = E{1};
end

E = E / norm(E);
if E(end) < 0
    E = -E;
end
end

%--------------------------------------------------------------------------
function [points1, points2, K1, K2, ransacParams, outputClass] = parseInputs(...
    matchedPoints1, matchedPoints2, varargin)

[pts1, pts2] = ...
    vision.internal.inputValidation.checkAndConvertMatchedPoints(...
    matchedPoints1, matchedPoints2, mfilename, 'matchedPoints1', ...
    'matchedPoints2');

points1 = cast(pts1, 'double');
points2 = cast(pts2, 'double');

outputClass = class(points1);

[K1, K2, idx] = parseCameraIntrinsics(outputClass, varargin{:});


defaults = struct('MaxNumTrials',1000, 'Confidence',99, 'MaxDistance',0.1);
if isempty(coder.target)
    ransacParams = parseRANSACParamsMatlab(defaults, varargin{idx:end});
else
    ransacParams = parseRANSACParamsCodegen(defaults, varargin{idx:end});
end
end


%--------------------------------------------------------------------------
function ransacParams = parseRANSACParamsMatlab(defaults, varargin)
parser = inputParser;
parser.FunctionName = mfilename;
parser.addParameter('MaxNumTrials', defaults.MaxNumTrials, @checkMaxNumTrials);
parser.addParameter('Confidence', defaults.Confidence, @checkConfidence);
parser.addParameter('MaxDistance', defaults.MaxDistance, @checkMaxDistance);

parser.parse(varargin{:});
ransacParams.confidence = parser.Results.Confidence;
ransacParams.maxDistance = parser.Results.MaxDistance;
ransacParams.maxNumTrials = parser.Results.MaxNumTrials;
end

%--------------------------------------------------------------------------
function ransacParams = parseRANSACParamsCodegen(defaults, varargin)

% Instantiate an input parser
parms = struct( ...
    'MaxNumTrials',       uint32(0), ...
    'Confidence',         uint32(0), ...
    'MaxDistance',        uint32(0));

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

% Specify the optional parameters
optarg = eml_parse_parameter_inputs(parms, popt, varargin{:});
ransacParams.maxNumTrials = eml_get_parameter_value(optarg.MaxNumTrials,...
    defaults.MaxNumTrials, varargin{:});
ransacParams.confidence   = eml_get_parameter_value(optarg.Confidence,...
    defaults.Confidence, varargin{:});
ransacParams.maxDistance  = eml_get_parameter_value(optarg.MaxDistance,...
    defaults.MaxDistance, varargin{:});

checkMaxNumTrials(ransacParams.maxNumTrials);
checkConfidence  (ransacParams.confidence);
checkMaxDistance (ransacParams.maxDistance);
end

%--------------------------------------------------------------------------
function [K1, K2, idx] = parseCameraIntrinsics(outputClass, varargin)
camIntrinsics1 = varargin{1};
validateattributes(camIntrinsics1, {'cameraParameters','cameraIntrinsics'}, {'scalar'}, mfilename);

K1 = cast(camIntrinsics1.IntrinsicMatrix, outputClass);
if numel(varargin) > 1 && (isa(varargin{2}, 'cameraParameters') || isa(varargin{2}, 'cameraIntrinsics'))

    validateattributes(varargin{2}, {'cameraParameters','cameraIntrinsics'}, {'scalar'}, mfilename);

    camIntrinsics2 = varargin{2};

    K2 = cast(camIntrinsics2.IntrinsicMatrix, outputClass);
    idx = 3;
else
    K2 = K1;
    idx = 2;
end
end

%--------------------------------------------------------------------------
function tf = checkMaxNumTrials(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'integer', 'positive'}, mfilename, ...
    'MaxNumTrials');
tf = true;
end

%--------------------------------------------------------------------------
function tf = checkConfidence(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'positive', '<', 100}, mfilename, ...
    'Confidence');
tf = true;
end

%--------------------------------------------------------------------------
function tf = checkMaxDistance(value)
validateattributes(value,{'single','double'}, ...
    {'real', 'nonsparse', 'scalar','nonnegative','finite'}, mfilename, ...
    'MaxDistance');
tf = true;
end

%--------------------------------------------------------------------------
function r = check(Es, varargin)
r = ~isempty(Es);
end

%--------------------------------------------------------------------------
function Es = fivePointAlgorithm(x, varargin)
Q1 = x(:,:,1);
Q2 = x(:,:,2);

Q = [Q2(:,1).*Q1(:,1),...
    Q2(:,1).*Q1(:,2),...
    Q2(:,1).*Q1(:,3),...
    Q2(:,2).*Q1(:,1),...
    Q2(:,2).*Q1(:,2),...
    Q2(:,2).*Q1(:,3),...
    Q2(:,3).*Q1(:,1),...
    Q2(:,3).*Q1(:,2),...
    Q2(:,3).*Q1(:,3),...
    ];

[~, ~, V] = svd(Q);
EE = V(:,6:9);
E1 = reshape(EE(:,1), [3,3])';
E2 = reshape(EE(:,2), [3,3])';
E3 = reshape(EE(:,3), [3,3])';
E4 = reshape(EE(:,4), [3,3])';

[C1, C2, C3, C4] = computeCoefficients(E1, E2, E3, E4);

Es = polyeig4(C1,C2,C3,C4,E1,E2,E3,E4);
end

%--------------------------------------------------------------------------
function [C1,C2,C3,C4] = computeCoefficients(E1, E2, E3, E4)

% Unpack the matrices. The code below was generated as follows:
% for i = 1:4
%     for j = 1:3
%         for k = 1:3
%             fprintf('e%d_%d_%d = E%d(%d,%d);\n', i, j, k, i, j, k);
%         end
%     end
% end

e1_1_1 = E1(1,1);
e1_1_2 = E1(1,2);
e1_1_3 = E1(1,3);
e1_2_1 = E1(2,1);
e1_2_2 = E1(2,2);
e1_2_3 = E1(2,3);
e1_3_1 = E1(3,1);
e1_3_2 = E1(3,2);
e1_3_3 = E1(3,3);
e2_1_1 = E2(1,1);
e2_1_2 = E2(1,2);
e2_1_3 = E2(1,3);
e2_2_1 = E2(2,1);
e2_2_2 = E2(2,2);
e2_2_3 = E2(2,3);
e2_3_1 = E2(3,1);
e2_3_2 = E2(3,2);
e2_3_3 = E2(3,3);
e3_1_1 = E3(1,1);
e3_1_2 = E3(1,2);
e3_1_3 = E3(1,3);
e3_2_1 = E3(2,1);
e3_2_2 = E3(2,2);
e3_2_3 = E3(2,3);
e3_3_1 = E3(3,1);
e3_3_2 = E3(3,2);
e3_3_3 = E3(3,3);
e4_1_1 = E4(1,1);
e4_1_2 = E4(1,2);
e4_1_3 = E4(1,3);
e4_2_1 = E4(2,1);
e4_2_2 = E4(2,2);
e4_2_3 = E4(2,3);
e4_3_1 = E4(3,1);
e4_3_2 = E4(3,2);
e4_3_3 = E4(3,3);

% The code below was generated using the Symbolic Math Toolbox as follows:
%
% %% Create E matrices
% E1 = sym('e1_%d_%d', 3);
% E2 = sym('e2_%d_%d', 3);
% E3 = sym('e3_%d_%d', 3);
% E4 = sym('e4_%d_%d', 3);
%
% %% Create parameter variables
% syms x y z;
%
% %% Define equations
% E = x*E1+y*E2+z*E3+E4;
% eqn = det(E);
% eqns = 2 * (E * E.')*E - trace(E*E.')*E;
%
% eqns = [eqn; eqns(:)];
%
% eqns = collect(expand(eqns), [x, y, z]);
%
% %% Extract coefficients
% for i = 1:numel(eqns)
%     [c, m] = coeffs(eqns(i), [x, y, z]);
%     C(i, :) = c;
% end
%
% % Reorder the columns of C to match the paper [1]
% %      Paper           C
% %      -----           ---
% %1     x^3             x^3
% %2     y^1*x^2         y^1*x^2
% %3     y^2*x^1         z^1*x^2
% %4     y^3             x^2
% %5     z^1*x^2         x*y^2
% %6     z^1*y^1*x^1     x*y*z
% %7     z^1*y^2         x*y
% %8     z^2*x^1         x*z^2
% %9     z^2*y^1         x*z
% %10     z^3             x
% %11    x^2             y^3
% %12     y^1*x^1         y^2*z
% %13     y^2             y^2
% %14     z^1*x^1         y*z^2
% %15     z^1*y^1         y*z
% %16     z^2             y
% %17     x^1             z^3
% %18     y^1             z^2
% %19     z^1             z
% %20     1               1
%
% idx = [1, 2, 5, 11, 3, 6, 12, 8, 14, 17, 4, 7, 13, 9, 15, 18, 10, 16, 19, 20];
% C = C(:, idx);
% C1 = C(:,[1:4,11:13,17:18,20]);
% C2 = C(:,[5:7,14:15,19]);
% C3 = C(:,[8:9,16]);
% C4 = C(:,10);
% %% Generate the function
% matlabFunction(C1,C2,C3,C4, file="computeCoefficients.m");

t2 = e1_1_1.^2;
t3 = e1_1_2.^2;
t4 = e1_1_3.^2;
t5 = e1_2_1.^2;
t6 = e1_2_2.^2;
t7 = e1_2_3.^2;
t8 = e1_3_1.^2;
t9 = e1_3_2.^2;
t10 = e1_3_3.^2;
t11 = e2_1_1.^2;
t12 = e2_1_2.^2;
t13 = e2_1_3.^2;
t14 = e2_2_1.^2;
t15 = e2_2_2.^2;
t16 = e2_2_3.^2;
t17 = e2_3_1.^2;
t18 = e2_3_2.^2;
t19 = e2_3_3.^2;
t20 = e3_1_1.^2;
t21 = e3_1_2.^2;
t22 = e3_1_3.^2;
t23 = e3_2_1.^2;
t24 = e3_2_2.^2;
t25 = e3_2_3.^2;
t26 = e3_3_1.^2;
t27 = e3_3_2.^2;
t28 = e3_3_3.^2;
t29 = e4_1_1.^2;
t30 = e4_1_2.^2;
t31 = e4_1_3.^2;
t32 = e4_2_1.^2;
t33 = e4_2_2.^2;
t34 = e4_2_3.^2;
t35 = e4_3_1.^2;
t36 = e4_3_2.^2;
t37 = e4_3_3.^2;
et1 = e1_1_1.*e2_1_1.*e4_1_1.*6.0+e1_1_1.*e2_1_2.*e4_1_2.*2.0+e1_1_2.*e2_1_1.*e4_1_2.*2.0+e1_1_2.*e2_1_2.*e4_1_1.*2.0+e1_1_1.*e2_1_3.*e4_1_3.*2.0+e1_1_3.*e2_1_1.*e4_1_3.*2.0+e1_1_3.*e2_1_3.*e4_1_1.*2.0+e1_1_1.*e2_2_1.*e4_2_1.*2.0+e1_2_1.*e2_1_1.*e4_2_1.*2.0+e1_2_1.*e2_2_1.*e4_1_1.*2.0-e1_1_1.*e2_2_2.*e4_2_2.*2.0+e1_1_2.*e2_2_1.*e4_2_2.*2.0+e1_1_2.*e2_2_2.*e4_2_1.*2.0+e1_2_1.*e2_1_2.*e4_2_2.*2.0+e1_2_1.*e2_2_2.*e4_1_2.*2.0-e1_2_2.*e2_1_1.*e4_2_2.*2.0+e1_2_2.*e2_1_2.*e4_2_1.*2.0+e1_2_2.*e2_2_1.*e4_1_2.*2.0-e1_2_2.*e2_2_2.*e4_1_1.*2.0-e1_1_1.*e2_2_3.*e4_2_3.*2.0+e1_1_3.*e2_2_1.*e4_2_3.*2.0+e1_1_3.*e2_2_3.*e4_2_1.*2.0+e1_2_1.*e2_1_3.*e4_2_3.*2.0+e1_2_1.*e2_2_3.*e4_1_3.*2.0-e1_2_3.*e2_1_1.*e4_2_3.*2.0+e1_2_3.*e2_1_3.*e4_2_1.*2.0+e1_2_3.*e2_2_1.*e4_1_3.*2.0-e1_2_3.*e2_2_3.*e4_1_1.*2.0+e1_1_1.*e2_3_1.*e4_3_1.*2.0+e1_3_1.*e2_1_1.*e4_3_1.*2.0+e1_3_1.*e2_3_1.*e4_1_1.*2.0-e1_1_1.*e2_3_2.*e4_3_2.*2.0+e1_1_2.*e2_3_1.*e4_3_2.*2.0+e1_1_2.*e2_3_2.*e4_3_1.*2.0+e1_3_1.*e2_1_2.*e4_3_2.*2.0+e1_3_1.*e2_3_2.*e4_1_2.*2.0-e1_3_2.*e2_1_1.*e4_3_2.*2.0+e1_3_2.*e2_1_2.*e4_3_1.*2.0;
et2 = e1_3_2.*e2_3_1.*e4_1_2.*2.0-e1_3_2.*e2_3_2.*e4_1_1.*2.0-e1_1_1.*e2_3_3.*e4_3_3.*2.0+e1_1_3.*e2_3_1.*e4_3_3.*2.0+e1_1_3.*e2_3_3.*e4_3_1.*2.0+e1_3_1.*e2_1_3.*e4_3_3.*2.0+e1_3_1.*e2_3_3.*e4_1_3.*2.0-e1_3_3.*e2_1_1.*e4_3_3.*2.0+e1_3_3.*e2_1_3.*e4_3_1.*2.0+e1_3_3.*e2_3_1.*e4_1_3.*2.0-e1_3_3.*e2_3_3.*e4_1_1.*2.0;
et3 = e1_1_1.*e2_1_1.*e4_2_1.*2.0+e1_1_1.*e2_2_1.*e4_1_1.*2.0+e1_2_1.*e2_1_1.*e4_1_1.*2.0+e1_1_1.*e2_1_2.*e4_2_2.*2.0+e1_1_1.*e2_2_2.*e4_1_2.*2.0+e1_1_2.*e2_1_1.*e4_2_2.*2.0-e1_1_2.*e2_1_2.*e4_2_1.*2.0-e1_1_2.*e2_2_1.*e4_1_2.*2.0+e1_1_2.*e2_2_2.*e4_1_1.*2.0-e1_2_1.*e2_1_2.*e4_1_2.*2.0+e1_2_2.*e2_1_1.*e4_1_2.*2.0+e1_2_2.*e2_1_2.*e4_1_1.*2.0+e1_1_1.*e2_1_3.*e4_2_3.*2.0+e1_1_1.*e2_2_3.*e4_1_3.*2.0+e1_1_3.*e2_1_1.*e4_2_3.*2.0-e1_1_3.*e2_1_3.*e4_2_1.*2.0-e1_1_3.*e2_2_1.*e4_1_3.*2.0+e1_1_3.*e2_2_3.*e4_1_1.*2.0-e1_2_1.*e2_1_3.*e4_1_3.*2.0+e1_2_3.*e2_1_1.*e4_1_3.*2.0+e1_2_3.*e2_1_3.*e4_1_1.*2.0+e1_2_1.*e2_2_1.*e4_2_1.*6.0+e1_2_1.*e2_2_2.*e4_2_2.*2.0+e1_2_2.*e2_2_1.*e4_2_2.*2.0+e1_2_2.*e2_2_2.*e4_2_1.*2.0+e1_2_1.*e2_2_3.*e4_2_3.*2.0+e1_2_3.*e2_2_1.*e4_2_3.*2.0+e1_2_3.*e2_2_3.*e4_2_1.*2.0+e1_2_1.*e2_3_1.*e4_3_1.*2.0+e1_3_1.*e2_2_1.*e4_3_1.*2.0+e1_3_1.*e2_3_1.*e4_2_1.*2.0-e1_2_1.*e2_3_2.*e4_3_2.*2.0+e1_2_2.*e2_3_1.*e4_3_2.*2.0+e1_2_2.*e2_3_2.*e4_3_1.*2.0+e1_3_1.*e2_2_2.*e4_3_2.*2.0+e1_3_1.*e2_3_2.*e4_2_2.*2.0-e1_3_2.*e2_2_1.*e4_3_2.*2.0+e1_3_2.*e2_2_2.*e4_3_1.*2.0;
et4 = e1_3_2.*e2_3_1.*e4_2_2.*2.0-e1_3_2.*e2_3_2.*e4_2_1.*2.0-e1_2_1.*e2_3_3.*e4_3_3.*2.0+e1_2_3.*e2_3_1.*e4_3_3.*2.0+e1_2_3.*e2_3_3.*e4_3_1.*2.0+e1_3_1.*e2_2_3.*e4_3_3.*2.0+e1_3_1.*e2_3_3.*e4_2_3.*2.0-e1_3_3.*e2_2_1.*e4_3_3.*2.0+e1_3_3.*e2_2_3.*e4_3_1.*2.0+e1_3_3.*e2_3_1.*e4_2_3.*2.0-e1_3_3.*e2_3_3.*e4_2_1.*2.0;
et5 = e1_1_1.*e2_1_1.*e4_3_1.*2.0+e1_1_1.*e2_3_1.*e4_1_1.*2.0+e1_3_1.*e2_1_1.*e4_1_1.*2.0+e1_1_1.*e2_1_2.*e4_3_2.*2.0+e1_1_1.*e2_3_2.*e4_1_2.*2.0+e1_1_2.*e2_1_1.*e4_3_2.*2.0-e1_1_2.*e2_1_2.*e4_3_1.*2.0-e1_1_2.*e2_3_1.*e4_1_2.*2.0+e1_1_2.*e2_3_2.*e4_1_1.*2.0-e1_3_1.*e2_1_2.*e4_1_2.*2.0+e1_3_2.*e2_1_1.*e4_1_2.*2.0+e1_3_2.*e2_1_2.*e4_1_1.*2.0+e1_1_1.*e2_1_3.*e4_3_3.*2.0+e1_1_1.*e2_3_3.*e4_1_3.*2.0+e1_1_3.*e2_1_1.*e4_3_3.*2.0-e1_1_3.*e2_1_3.*e4_3_1.*2.0-e1_1_3.*e2_3_1.*e4_1_3.*2.0+e1_1_3.*e2_3_3.*e4_1_1.*2.0-e1_3_1.*e2_1_3.*e4_1_3.*2.0+e1_3_3.*e2_1_1.*e4_1_3.*2.0+e1_3_3.*e2_1_3.*e4_1_1.*2.0+e1_2_1.*e2_2_1.*e4_3_1.*2.0+e1_2_1.*e2_3_1.*e4_2_1.*2.0+e1_3_1.*e2_2_1.*e4_2_1.*2.0+e1_2_1.*e2_2_2.*e4_3_2.*2.0+e1_2_1.*e2_3_2.*e4_2_2.*2.0+e1_2_2.*e2_2_1.*e4_3_2.*2.0-e1_2_2.*e2_2_2.*e4_3_1.*2.0-e1_2_2.*e2_3_1.*e4_2_2.*2.0+e1_2_2.*e2_3_2.*e4_2_1.*2.0-e1_3_1.*e2_2_2.*e4_2_2.*2.0+e1_3_2.*e2_2_1.*e4_2_2.*2.0+e1_3_2.*e2_2_2.*e4_2_1.*2.0+e1_2_1.*e2_2_3.*e4_3_3.*2.0+e1_2_1.*e2_3_3.*e4_2_3.*2.0+e1_2_3.*e2_2_1.*e4_3_3.*2.0-e1_2_3.*e2_2_3.*e4_3_1.*2.0;
et6 = e1_2_3.*e2_3_1.*e4_2_3.*-2.0+e1_2_3.*e2_3_3.*e4_2_1.*2.0-e1_3_1.*e2_2_3.*e4_2_3.*2.0+e1_3_3.*e2_2_1.*e4_2_3.*2.0+e1_3_3.*e2_2_3.*e4_2_1.*2.0+e1_3_1.*e2_3_1.*e4_3_1.*6.0+e1_3_1.*e2_3_2.*e4_3_2.*2.0+e1_3_2.*e2_3_1.*e4_3_2.*2.0+e1_3_2.*e2_3_2.*e4_3_1.*2.0+e1_3_1.*e2_3_3.*e4_3_3.*2.0+e1_3_3.*e2_3_1.*e4_3_3.*2.0+e1_3_3.*e2_3_3.*e4_3_1.*2.0;
et7 = e1_1_1.*e2_1_1.*e4_1_2.*2.0+e1_1_1.*e2_1_2.*e4_1_1.*2.0+e1_1_2.*e2_1_1.*e4_1_1.*2.0+e1_1_2.*e2_1_2.*e4_1_2.*6.0+e1_1_2.*e2_1_3.*e4_1_3.*2.0+e1_1_3.*e2_1_2.*e4_1_3.*2.0+e1_1_3.*e2_1_3.*e4_1_2.*2.0+e1_1_1.*e2_2_1.*e4_2_2.*2.0+e1_1_1.*e2_2_2.*e4_2_1.*2.0-e1_1_2.*e2_2_1.*e4_2_1.*2.0+e1_2_1.*e2_1_1.*e4_2_2.*2.0-e1_2_1.*e2_1_2.*e4_2_1.*2.0-e1_2_1.*e2_2_1.*e4_1_2.*2.0+e1_2_1.*e2_2_2.*e4_1_1.*2.0+e1_2_2.*e2_1_1.*e4_2_1.*2.0+e1_2_2.*e2_2_1.*e4_1_1.*2.0+e1_1_2.*e2_2_2.*e4_2_2.*2.0+e1_2_2.*e2_1_2.*e4_2_2.*2.0+e1_2_2.*e2_2_2.*e4_1_2.*2.0-e1_1_2.*e2_2_3.*e4_2_3.*2.0+e1_1_3.*e2_2_2.*e4_2_3.*2.0+e1_1_3.*e2_2_3.*e4_2_2.*2.0+e1_2_2.*e2_1_3.*e4_2_3.*2.0+e1_2_2.*e2_2_3.*e4_1_3.*2.0-e1_2_3.*e2_1_2.*e4_2_3.*2.0+e1_2_3.*e2_1_3.*e4_2_2.*2.0+e1_2_3.*e2_2_2.*e4_1_3.*2.0-e1_2_3.*e2_2_3.*e4_1_2.*2.0+e1_1_1.*e2_3_1.*e4_3_2.*2.0+e1_1_1.*e2_3_2.*e4_3_1.*2.0-e1_1_2.*e2_3_1.*e4_3_1.*2.0+e1_3_1.*e2_1_1.*e4_3_2.*2.0-e1_3_1.*e2_1_2.*e4_3_1.*2.0-e1_3_1.*e2_3_1.*e4_1_2.*2.0+e1_3_1.*e2_3_2.*e4_1_1.*2.0+e1_3_2.*e2_1_1.*e4_3_1.*2.0+e1_3_2.*e2_3_1.*e4_1_1.*2.0+e1_1_2.*e2_3_2.*e4_3_2.*2.0;
et8 = e1_3_2.*e2_1_2.*e4_3_2.*2.0+e1_3_2.*e2_3_2.*e4_1_2.*2.0-e1_1_2.*e2_3_3.*e4_3_3.*2.0+e1_1_3.*e2_3_2.*e4_3_3.*2.0+e1_1_3.*e2_3_3.*e4_3_2.*2.0+e1_3_2.*e2_1_3.*e4_3_3.*2.0+e1_3_2.*e2_3_3.*e4_1_3.*2.0-e1_3_3.*e2_1_2.*e4_3_3.*2.0+e1_3_3.*e2_1_3.*e4_3_2.*2.0+e1_3_3.*e2_3_2.*e4_1_3.*2.0-e1_3_3.*e2_3_3.*e4_1_2.*2.0;
et9 = e1_1_1.*e2_1_2.*e4_2_1.*2.0-e1_1_1.*e2_1_1.*e4_2_2.*2.0+e1_1_1.*e2_2_1.*e4_1_2.*2.0-e1_1_1.*e2_2_2.*e4_1_1.*2.0+e1_1_2.*e2_1_1.*e4_2_1.*2.0+e1_1_2.*e2_2_1.*e4_1_1.*2.0+e1_2_1.*e2_1_1.*e4_1_2.*2.0+e1_2_1.*e2_1_2.*e4_1_1.*2.0-e1_2_2.*e2_1_1.*e4_1_1.*2.0+e1_1_2.*e2_1_2.*e4_2_2.*2.0+e1_1_2.*e2_2_2.*e4_1_2.*2.0+e1_2_2.*e2_1_2.*e4_1_2.*2.0+e1_1_2.*e2_1_3.*e4_2_3.*2.0+e1_1_2.*e2_2_3.*e4_1_3.*2.0+e1_1_3.*e2_1_2.*e4_2_3.*2.0-e1_1_3.*e2_1_3.*e4_2_2.*2.0-e1_1_3.*e2_2_2.*e4_1_3.*2.0+e1_1_3.*e2_2_3.*e4_1_2.*2.0-e1_2_2.*e2_1_3.*e4_1_3.*2.0+e1_2_3.*e2_1_2.*e4_1_3.*2.0+e1_2_3.*e2_1_3.*e4_1_2.*2.0+e1_2_1.*e2_2_1.*e4_2_2.*2.0+e1_2_1.*e2_2_2.*e4_2_1.*2.0+e1_2_2.*e2_2_1.*e4_2_1.*2.0+e1_2_2.*e2_2_2.*e4_2_2.*6.0+e1_2_2.*e2_2_3.*e4_2_3.*2.0+e1_2_3.*e2_2_2.*e4_2_3.*2.0+e1_2_3.*e2_2_3.*e4_2_2.*2.0+e1_2_1.*e2_3_1.*e4_3_2.*2.0+e1_2_1.*e2_3_2.*e4_3_1.*2.0-e1_2_2.*e2_3_1.*e4_3_1.*2.0+e1_3_1.*e2_2_1.*e4_3_2.*2.0-e1_3_1.*e2_2_2.*e4_3_1.*2.0-e1_3_1.*e2_3_1.*e4_2_2.*2.0+e1_3_1.*e2_3_2.*e4_2_1.*2.0+e1_3_2.*e2_2_1.*e4_3_1.*2.0+e1_3_2.*e2_3_1.*e4_2_1.*2.0+e1_2_2.*e2_3_2.*e4_3_2.*2.0;
et10 = e1_3_2.*e2_2_2.*e4_3_2.*2.0+e1_3_2.*e2_3_2.*e4_2_2.*2.0-e1_2_2.*e2_3_3.*e4_3_3.*2.0+e1_2_3.*e2_3_2.*e4_3_3.*2.0+e1_2_3.*e2_3_3.*e4_3_2.*2.0+e1_3_2.*e2_2_3.*e4_3_3.*2.0+e1_3_2.*e2_3_3.*e4_2_3.*2.0-e1_3_3.*e2_2_2.*e4_3_3.*2.0+e1_3_3.*e2_2_3.*e4_3_2.*2.0+e1_3_3.*e2_3_2.*e4_2_3.*2.0-e1_3_3.*e2_3_3.*e4_2_2.*2.0;
et11 = e1_1_1.*e2_1_2.*e4_3_1.*2.0-e1_1_1.*e2_1_1.*e4_3_2.*2.0+e1_1_1.*e2_3_1.*e4_1_2.*2.0-e1_1_1.*e2_3_2.*e4_1_1.*2.0+e1_1_2.*e2_1_1.*e4_3_1.*2.0+e1_1_2.*e2_3_1.*e4_1_1.*2.0+e1_3_1.*e2_1_1.*e4_1_2.*2.0+e1_3_1.*e2_1_2.*e4_1_1.*2.0-e1_3_2.*e2_1_1.*e4_1_1.*2.0+e1_1_2.*e2_1_2.*e4_3_2.*2.0+e1_1_2.*e2_3_2.*e4_1_2.*2.0+e1_3_2.*e2_1_2.*e4_1_2.*2.0+e1_1_2.*e2_1_3.*e4_3_3.*2.0+e1_1_2.*e2_3_3.*e4_1_3.*2.0+e1_1_3.*e2_1_2.*e4_3_3.*2.0-e1_1_3.*e2_1_3.*e4_3_2.*2.0-e1_1_3.*e2_3_2.*e4_1_3.*2.0+e1_1_3.*e2_3_3.*e4_1_2.*2.0-e1_3_2.*e2_1_3.*e4_1_3.*2.0+e1_3_3.*e2_1_2.*e4_1_3.*2.0+e1_3_3.*e2_1_3.*e4_1_2.*2.0-e1_2_1.*e2_2_1.*e4_3_2.*2.0+e1_2_1.*e2_2_2.*e4_3_1.*2.0+e1_2_1.*e2_3_1.*e4_2_2.*2.0-e1_2_1.*e2_3_2.*e4_2_1.*2.0+e1_2_2.*e2_2_1.*e4_3_1.*2.0+e1_2_2.*e2_3_1.*e4_2_1.*2.0+e1_3_1.*e2_2_1.*e4_2_2.*2.0+e1_3_1.*e2_2_2.*e4_2_1.*2.0-e1_3_2.*e2_2_1.*e4_2_1.*2.0+e1_2_2.*e2_2_2.*e4_3_2.*2.0+e1_2_2.*e2_3_2.*e4_2_2.*2.0+e1_3_2.*e2_2_2.*e4_2_2.*2.0+e1_2_2.*e2_2_3.*e4_3_3.*2.0+e1_2_2.*e2_3_3.*e4_2_3.*2.0+e1_2_3.*e2_2_2.*e4_3_3.*2.0-e1_2_3.*e2_2_3.*e4_3_2.*2.0;
et12 = e1_2_3.*e2_3_2.*e4_2_3.*-2.0+e1_2_3.*e2_3_3.*e4_2_2.*2.0-e1_3_2.*e2_2_3.*e4_2_3.*2.0+e1_3_3.*e2_2_2.*e4_2_3.*2.0+e1_3_3.*e2_2_3.*e4_2_2.*2.0+e1_3_1.*e2_3_1.*e4_3_2.*2.0+e1_3_1.*e2_3_2.*e4_3_1.*2.0+e1_3_2.*e2_3_1.*e4_3_1.*2.0+e1_3_2.*e2_3_2.*e4_3_2.*6.0+e1_3_2.*e2_3_3.*e4_3_3.*2.0+e1_3_3.*e2_3_2.*e4_3_3.*2.0+e1_3_3.*e2_3_3.*e4_3_2.*2.0;
et13 = e1_1_1.*e2_1_1.*e4_1_3.*2.0+e1_1_1.*e2_1_3.*e4_1_1.*2.0+e1_1_3.*e2_1_1.*e4_1_1.*2.0+e1_1_2.*e2_1_2.*e4_1_3.*2.0+e1_1_2.*e2_1_3.*e4_1_2.*2.0+e1_1_3.*e2_1_2.*e4_1_2.*2.0+e1_1_3.*e2_1_3.*e4_1_3.*6.0+e1_1_1.*e2_2_1.*e4_2_3.*2.0+e1_1_1.*e2_2_3.*e4_2_1.*2.0-e1_1_3.*e2_2_1.*e4_2_1.*2.0+e1_2_1.*e2_1_1.*e4_2_3.*2.0-e1_2_1.*e2_1_3.*e4_2_1.*2.0-e1_2_1.*e2_2_1.*e4_1_3.*2.0+e1_2_1.*e2_2_3.*e4_1_1.*2.0+e1_2_3.*e2_1_1.*e4_2_1.*2.0+e1_2_3.*e2_2_1.*e4_1_1.*2.0+e1_1_2.*e2_2_2.*e4_2_3.*2.0+e1_1_2.*e2_2_3.*e4_2_2.*2.0-e1_1_3.*e2_2_2.*e4_2_2.*2.0+e1_2_2.*e2_1_2.*e4_2_3.*2.0-e1_2_2.*e2_1_3.*e4_2_2.*2.0-e1_2_2.*e2_2_2.*e4_1_3.*2.0+e1_2_2.*e2_2_3.*e4_1_2.*2.0+e1_2_3.*e2_1_2.*e4_2_2.*2.0+e1_2_3.*e2_2_2.*e4_1_2.*2.0+e1_1_3.*e2_2_3.*e4_2_3.*2.0+e1_2_3.*e2_1_3.*e4_2_3.*2.0+e1_2_3.*e2_2_3.*e4_1_3.*2.0+e1_1_1.*e2_3_1.*e4_3_3.*2.0+e1_1_1.*e2_3_3.*e4_3_1.*2.0-e1_1_3.*e2_3_1.*e4_3_1.*2.0+e1_3_1.*e2_1_1.*e4_3_3.*2.0-e1_3_1.*e2_1_3.*e4_3_1.*2.0-e1_3_1.*e2_3_1.*e4_1_3.*2.0+e1_3_1.*e2_3_3.*e4_1_1.*2.0+e1_3_3.*e2_1_1.*e4_3_1.*2.0+e1_3_3.*e2_3_1.*e4_1_1.*2.0+e1_1_2.*e2_3_2.*e4_3_3.*2.0;
et14 = e1_1_2.*e2_3_3.*e4_3_2.*2.0-e1_1_3.*e2_3_2.*e4_3_2.*2.0+e1_3_2.*e2_1_2.*e4_3_3.*2.0-e1_3_2.*e2_1_3.*e4_3_2.*2.0-e1_3_2.*e2_3_2.*e4_1_3.*2.0+e1_3_2.*e2_3_3.*e4_1_2.*2.0+e1_3_3.*e2_1_2.*e4_3_2.*2.0+e1_3_3.*e2_3_2.*e4_1_2.*2.0+e1_1_3.*e2_3_3.*e4_3_3.*2.0+e1_3_3.*e2_1_3.*e4_3_3.*2.0+e1_3_3.*e2_3_3.*e4_1_3.*2.0;
et15 = e1_1_1.*e2_1_3.*e4_2_1.*2.0-e1_1_1.*e2_1_1.*e4_2_3.*2.0+e1_1_1.*e2_2_1.*e4_1_3.*2.0-e1_1_1.*e2_2_3.*e4_1_1.*2.0+e1_1_3.*e2_1_1.*e4_2_1.*2.0+e1_1_3.*e2_2_1.*e4_1_1.*2.0+e1_2_1.*e2_1_1.*e4_1_3.*2.0+e1_2_1.*e2_1_3.*e4_1_1.*2.0-e1_2_3.*e2_1_1.*e4_1_1.*2.0-e1_1_2.*e2_1_2.*e4_2_3.*2.0+e1_1_2.*e2_1_3.*e4_2_2.*2.0+e1_1_2.*e2_2_2.*e4_1_3.*2.0-e1_1_2.*e2_2_3.*e4_1_2.*2.0+e1_1_3.*e2_1_2.*e4_2_2.*2.0+e1_1_3.*e2_2_2.*e4_1_2.*2.0+e1_2_2.*e2_1_2.*e4_1_3.*2.0+e1_2_2.*e2_1_3.*e4_1_2.*2.0-e1_2_3.*e2_1_2.*e4_1_2.*2.0+e1_1_3.*e2_1_3.*e4_2_3.*2.0+e1_1_3.*e2_2_3.*e4_1_3.*2.0+e1_2_3.*e2_1_3.*e4_1_3.*2.0+e1_2_1.*e2_2_1.*e4_2_3.*2.0+e1_2_1.*e2_2_3.*e4_2_1.*2.0+e1_2_3.*e2_2_1.*e4_2_1.*2.0+e1_2_2.*e2_2_2.*e4_2_3.*2.0+e1_2_2.*e2_2_3.*e4_2_2.*2.0+e1_2_3.*e2_2_2.*e4_2_2.*2.0+e1_2_3.*e2_2_3.*e4_2_3.*6.0+e1_2_1.*e2_3_1.*e4_3_3.*2.0+e1_2_1.*e2_3_3.*e4_3_1.*2.0-e1_2_3.*e2_3_1.*e4_3_1.*2.0+e1_3_1.*e2_2_1.*e4_3_3.*2.0-e1_3_1.*e2_2_3.*e4_3_1.*2.0-e1_3_1.*e2_3_1.*e4_2_3.*2.0+e1_3_1.*e2_3_3.*e4_2_1.*2.0+e1_3_3.*e2_2_1.*e4_3_1.*2.0+e1_3_3.*e2_3_1.*e4_2_1.*2.0+e1_2_2.*e2_3_2.*e4_3_3.*2.0;
et16 = e1_2_2.*e2_3_3.*e4_3_2.*2.0-e1_2_3.*e2_3_2.*e4_3_2.*2.0+e1_3_2.*e2_2_2.*e4_3_3.*2.0-e1_3_2.*e2_2_3.*e4_3_2.*2.0-e1_3_2.*e2_3_2.*e4_2_3.*2.0+e1_3_2.*e2_3_3.*e4_2_2.*2.0+e1_3_3.*e2_2_2.*e4_3_2.*2.0+e1_3_3.*e2_3_2.*e4_2_2.*2.0+e1_2_3.*e2_3_3.*e4_3_3.*2.0+e1_3_3.*e2_2_3.*e4_3_3.*2.0+e1_3_3.*e2_3_3.*e4_2_3.*2.0;
et17 = e1_1_1.*e2_1_3.*e4_3_1.*2.0-e1_1_1.*e2_1_1.*e4_3_3.*2.0+e1_1_1.*e2_3_1.*e4_1_3.*2.0-e1_1_1.*e2_3_3.*e4_1_1.*2.0+e1_1_3.*e2_1_1.*e4_3_1.*2.0+e1_1_3.*e2_3_1.*e4_1_1.*2.0+e1_3_1.*e2_1_1.*e4_1_3.*2.0+e1_3_1.*e2_1_3.*e4_1_1.*2.0-e1_3_3.*e2_1_1.*e4_1_1.*2.0-e1_1_2.*e2_1_2.*e4_3_3.*2.0+e1_1_2.*e2_1_3.*e4_3_2.*2.0+e1_1_2.*e2_3_2.*e4_1_3.*2.0-e1_1_2.*e2_3_3.*e4_1_2.*2.0+e1_1_3.*e2_1_2.*e4_3_2.*2.0+e1_1_3.*e2_3_2.*e4_1_2.*2.0+e1_3_2.*e2_1_2.*e4_1_3.*2.0+e1_3_2.*e2_1_3.*e4_1_2.*2.0-e1_3_3.*e2_1_2.*e4_1_2.*2.0+e1_1_3.*e2_1_3.*e4_3_3.*2.0+e1_1_3.*e2_3_3.*e4_1_3.*2.0+e1_3_3.*e2_1_3.*e4_1_3.*2.0-e1_2_1.*e2_2_1.*e4_3_3.*2.0+e1_2_1.*e2_2_3.*e4_3_1.*2.0+e1_2_1.*e2_3_1.*e4_2_3.*2.0-e1_2_1.*e2_3_3.*e4_2_1.*2.0+e1_2_3.*e2_2_1.*e4_3_1.*2.0+e1_2_3.*e2_3_1.*e4_2_1.*2.0+e1_3_1.*e2_2_1.*e4_2_3.*2.0+e1_3_1.*e2_2_3.*e4_2_1.*2.0-e1_3_3.*e2_2_1.*e4_2_1.*2.0-e1_2_2.*e2_2_2.*e4_3_3.*2.0+e1_2_2.*e2_2_3.*e4_3_2.*2.0+e1_2_2.*e2_3_2.*e4_2_3.*2.0-e1_2_2.*e2_3_3.*e4_2_2.*2.0+e1_2_3.*e2_2_2.*e4_3_2.*2.0+e1_2_3.*e2_3_2.*e4_2_2.*2.0+e1_3_2.*e2_2_2.*e4_2_3.*2.0;
et18 = e1_3_2.*e2_2_3.*e4_2_2.*2.0-e1_3_3.*e2_2_2.*e4_2_2.*2.0+e1_2_3.*e2_2_3.*e4_3_3.*2.0+e1_2_3.*e2_3_3.*e4_2_3.*2.0+e1_3_3.*e2_2_3.*e4_2_3.*2.0+e1_3_1.*e2_3_1.*e4_3_3.*2.0+e1_3_1.*e2_3_3.*e4_3_1.*2.0+e1_3_3.*e2_3_1.*e4_3_1.*2.0+e1_3_2.*e2_3_2.*e4_3_3.*2.0+e1_3_2.*e2_3_3.*e4_3_2.*2.0+e1_3_3.*e2_3_2.*e4_3_2.*2.0+e1_3_3.*e2_3_3.*e4_3_3.*6.0;
mt1 = [e1_1_1.*e1_2_2.*e1_3_3-e1_1_1.*e1_2_3.*e1_3_2-e1_1_2.*e1_2_1.*e1_3_3+e1_1_2.*e1_2_3.*e1_3_1+e1_1_3.*e1_2_1.*e1_3_2-e1_1_3.*e1_2_2.*e1_3_1,e1_1_1.*t3+e1_1_1.*t4+e1_1_1.*t5-e1_1_1.*t6-e1_1_1.*t7+e1_1_1.*t8-e1_1_1.*t9-e1_1_1.*t10+e1_1_1.^3+e1_1_2.*e1_2_1.*e1_2_2.*2.0+e1_1_3.*e1_2_1.*e1_2_3.*2.0+e1_1_2.*e1_3_1.*e1_3_2.*2.0+e1_1_3.*e1_3_1.*e1_3_3.*2.0,e1_2_1.*t2-e1_2_1.*t3-e1_2_1.*t4+e1_2_1.*t6+e1_2_1.*t7+e1_2_1.*t8-e1_2_1.*t9-e1_2_1.*t10+e1_2_1.^3+e1_1_1.*e1_1_2.*e1_2_2.*2.0+e1_1_1.*e1_1_3.*e1_2_3.*2.0+e1_2_2.*e1_3_1.*e1_3_2.*2.0+e1_2_3.*e1_3_1.*e1_3_3.*2.0,e1_3_1.*t2-e1_3_1.*t3-e1_3_1.*t4+e1_3_1.*t5-e1_3_1.*t6-e1_3_1.*t7+e1_3_1.*t9+e1_3_1.*t10+e1_3_1.^3+e1_1_1.*e1_1_2.*e1_3_2.*2.0+e1_1_1.*e1_1_3.*e1_3_3.*2.0+e1_2_1.*e1_2_2.*e1_3_2.*2.0+e1_2_1.*e1_2_3.*e1_3_3.*2.0];
mt2 = [e1_1_2.*t2+e1_1_2.*t4-e1_1_2.*t5+e1_1_2.*t6-e1_1_2.*t7-e1_1_2.*t8+e1_1_2.*t9-e1_1_2.*t10+e1_1_2.^3+e1_1_1.*e1_2_1.*e1_2_2.*2.0+e1_1_3.*e1_2_2.*e1_2_3.*2.0+e1_1_1.*e1_3_1.*e1_3_2.*2.0+e1_1_3.*e1_3_2.*e1_3_3.*2.0,-e1_2_2.*t2+e1_2_2.*t3-e1_2_2.*t4+e1_2_2.*t5+e1_2_2.*t7-e1_2_2.*t8+e1_2_2.*t9-e1_2_2.*t10+e1_2_2.^3+e1_1_1.*e1_1_2.*e1_2_1.*2.0+e1_1_2.*e1_1_3.*e1_2_3.*2.0+e1_2_1.*e1_3_1.*e1_3_2.*2.0+e1_2_3.*e1_3_2.*e1_3_3.*2.0,-e1_3_2.*t2+e1_3_2.*t3-e1_3_2.*t4-e1_3_2.*t5+e1_3_2.*t6-e1_3_2.*t7+e1_3_2.*t8+e1_3_2.*t10+e1_3_2.^3+e1_1_1.*e1_1_2.*e1_3_1.*2.0+e1_1_2.*e1_1_3.*e1_3_3.*2.0+e1_2_1.*e1_2_2.*e1_3_1.*2.0+e1_2_2.*e1_2_3.*e1_3_3.*2.0];
mt3 = [e1_1_3.*t2+e1_1_3.*t3-e1_1_3.*t5-e1_1_3.*t6+e1_1_3.*t7-e1_1_3.*t8-e1_1_3.*t9+e1_1_3.*t10+e1_1_3.^3+e1_1_1.*e1_2_1.*e1_2_3.*2.0+e1_1_2.*e1_2_2.*e1_2_3.*2.0+e1_1_1.*e1_3_1.*e1_3_3.*2.0+e1_1_2.*e1_3_2.*e1_3_3.*2.0,-e1_2_3.*t2-e1_2_3.*t3+e1_2_3.*t4+e1_2_3.*t5+e1_2_3.*t6-e1_2_3.*t8-e1_2_3.*t9+e1_2_3.*t10+e1_2_3.^3+e1_1_1.*e1_1_3.*e1_2_1.*2.0+e1_1_2.*e1_1_3.*e1_2_2.*2.0+e1_2_1.*e1_3_1.*e1_3_3.*2.0+e1_2_2.*e1_3_2.*e1_3_3.*2.0,-e1_3_3.*t2-e1_3_3.*t3+e1_3_3.*t4-e1_3_3.*t5-e1_3_3.*t6+e1_3_3.*t7+e1_3_3.*t8+e1_3_3.*t9+e1_3_3.^3+e1_1_1.*e1_1_3.*e1_3_1.*2.0+e1_1_2.*e1_1_3.*e1_3_2.*2.0+e1_2_1.*e1_2_3.*e1_3_1.*2.0+e1_2_2.*e1_2_3.*e1_3_2.*2.0];
mt4 = [e1_1_1.*e1_2_2.*e2_3_3-e1_1_1.*e1_2_3.*e2_3_2-e1_1_1.*e1_3_2.*e2_2_3+e1_1_1.*e1_3_3.*e2_2_2-e1_1_2.*e1_2_1.*e2_3_3+e1_1_2.*e1_2_3.*e2_3_1+e1_1_2.*e1_3_1.*e2_2_3-e1_1_2.*e1_3_3.*e2_2_1+e1_1_3.*e1_2_1.*e2_3_2-e1_1_3.*e1_2_2.*e2_3_1-e1_1_3.*e1_3_1.*e2_2_2+e1_1_3.*e1_3_2.*e2_2_1+e1_2_1.*e1_3_2.*e2_1_3-e1_2_1.*e1_3_3.*e2_1_2-e1_2_2.*e1_3_1.*e2_1_3+e1_2_2.*e1_3_3.*e2_1_1+e1_2_3.*e1_3_1.*e2_1_2-e1_2_3.*e1_3_2.*e2_1_1];
mt5 = [e2_1_1.*t2.*3.0+e2_1_1.*t3+e2_1_1.*t4+e2_1_1.*t5-e2_1_1.*t6-e2_1_1.*t7+e2_1_1.*t8-e2_1_1.*t9-e2_1_1.*t10+2.0.*e2_1_2.*e1_1_1.*e1_1_2+2.0.*e2_1_3.*e1_1_1.*e1_1_3+2.0.*e2_2_1.*e1_1_1.*e1_2_1+2.0.*e2_1_2.*e1_2_1.*e1_2_2+2.0.*e2_2_1.*e1_1_2.*e1_2_2+-2.0.*e2_2_2.*e1_1_1.*e1_2_2+2.0.*e2_2_2.*e1_1_2.*e1_2_1+2.0.*e2_1_3.*e1_2_1.*e1_2_3+2.0.*e2_2_1.*e1_1_3.*e1_2_3+-2.0.*e2_2_3.*e1_1_1.*e1_2_3+2.0.*e2_2_3.*e1_1_3.*e1_2_1+2.0.*e2_3_1.*e1_1_1.*e1_3_1+2.0.*e2_1_2.*e1_3_1.*e1_3_2+2.0.*e2_3_1.*e1_1_2.*e1_3_2+-2.0.*e2_3_2.*e1_1_1.*e1_3_2+2.0.*e2_3_2.*e1_1_2.*e1_3_1+2.0.*e2_1_3.*e1_3_1.*e1_3_3+2.0.*e2_3_1.*e1_1_3.*e1_3_3+-2.0.*e2_3_3.*e1_1_1.*e1_3_3+2.0.*e2_3_3.*e1_1_3.*e1_3_1];
mt6 = [e2_2_1.*t2-e2_2_1.*t3-e2_2_1.*t4+e2_2_1.*t5.*3.0+e2_2_1.*t6+e2_2_1.*t7+e2_2_1.*t8-e2_2_1.*t9-e2_2_1.*t10+2.0.*e2_1_1.*e1_1_1.*e1_2_1+2.0.*e2_1_1.*e1_1_2.*e1_2_2+2.0.*e2_1_2.*e1_1_1.*e1_2_2+-2.0.*e2_1_2.*e1_1_2.*e1_2_1+2.0.*e2_2_2.*e1_1_1.*e1_1_2+2.0.*e2_1_1.*e1_1_3.*e1_2_3+2.0.*e2_1_3.*e1_1_1.*e1_2_3+-2.0.*e2_1_3.*e1_1_3.*e1_2_1+2.0.*e2_2_3.*e1_1_1.*e1_1_3+2.0.*e2_2_2.*e1_2_1.*e1_2_2+2.0.*e2_2_3.*e1_2_1.*e1_2_3+2.0.*e2_3_1.*e1_2_1.*e1_3_1+2.0.*e2_2_2.*e1_3_1.*e1_3_2+2.0.*e2_3_1.*e1_2_2.*e1_3_2+-2.0.*e2_3_2.*e1_2_1.*e1_3_2+2.0.*e2_3_2.*e1_2_2.*e1_3_1+2.0.*e2_2_3.*e1_3_1.*e1_3_3+2.0.*e2_3_1.*e1_2_3.*e1_3_3+-2.0.*e2_3_3.*e1_2_1.*e1_3_3+2.0.*e2_3_3.*e1_2_3.*e1_3_1];
mt7 = [e2_3_1.*t2-e2_3_1.*t3-e2_3_1.*t4+e2_3_1.*t5-e2_3_1.*t6-e2_3_1.*t7+e2_3_1.*t8.*3.0+e2_3_1.*t9+e2_3_1.*t10+2.0.*e2_1_1.*e1_1_1.*e1_3_1+2.0.*e2_1_1.*e1_1_2.*e1_3_2+2.0.*e2_1_2.*e1_1_1.*e1_3_2+-2.0.*e2_1_2.*e1_1_2.*e1_3_1+2.0.*e2_3_2.*e1_1_1.*e1_1_2+2.0.*e2_1_1.*e1_1_3.*e1_3_3+2.0.*e2_1_3.*e1_1_1.*e1_3_3+-2.0.*e2_1_3.*e1_1_3.*e1_3_1+2.0.*e2_3_3.*e1_1_1.*e1_1_3+2.0.*e2_2_1.*e1_2_1.*e1_3_1+2.0.*e2_2_1.*e1_2_2.*e1_3_2+2.0.*e2_2_2.*e1_2_1.*e1_3_2+-2.0.*e2_2_2.*e1_2_2.*e1_3_1+2.0.*e2_3_2.*e1_2_1.*e1_2_2+2.0.*e2_2_1.*e1_2_3.*e1_3_3+2.0.*e2_2_3.*e1_2_1.*e1_3_3+-2.0.*e2_2_3.*e1_2_3.*e1_3_1+2.0.*e2_3_3.*e1_2_1.*e1_2_3+2.0.*e2_3_2.*e1_3_1.*e1_3_2+2.0.*e2_3_3.*e1_3_1.*e1_3_3];
mt8 = [e2_1_2.*t2+e2_1_2.*t3.*3.0+e2_1_2.*t4-e2_1_2.*t5+e2_1_2.*t6-e2_1_2.*t7-e2_1_2.*t8+e2_1_2.*t9-e2_1_2.*t10+2.0.*e2_1_1.*e1_1_1.*e1_1_2+2.0.*e2_1_3.*e1_1_2.*e1_1_3+2.0.*e2_1_1.*e1_2_1.*e1_2_2+2.0.*e2_2_1.*e1_1_1.*e1_2_2+-2.0.*e2_2_1.*e1_1_2.*e1_2_1+2.0.*e2_2_2.*e1_1_1.*e1_2_1+2.0.*e2_2_2.*e1_1_2.*e1_2_2+2.0.*e2_1_3.*e1_2_2.*e1_2_3+2.0.*e2_2_2.*e1_1_3.*e1_2_3+-2.0.*e2_2_3.*e1_1_2.*e1_2_3+2.0.*e2_2_3.*e1_1_3.*e1_2_2+2.0.*e2_1_1.*e1_3_1.*e1_3_2+2.0.*e2_3_1.*e1_1_1.*e1_3_2+-2.0.*e2_3_1.*e1_1_2.*e1_3_1+2.0.*e2_3_2.*e1_1_1.*e1_3_1+2.0.*e2_3_2.*e1_1_2.*e1_3_2+2.0.*e2_1_3.*e1_3_2.*e1_3_3+2.0.*e2_3_2.*e1_1_3.*e1_3_3+-2.0.*e2_3_3.*e1_1_2.*e1_3_3+2.0.*e2_3_3.*e1_1_3.*e1_3_2];
mt9 = [-e2_2_2.*t2+e2_2_2.*t3-e2_2_2.*t4+e2_2_2.*t5+e2_2_2.*t6.*3.0+e2_2_2.*t7-e2_2_2.*t8+e2_2_2.*t9-e2_2_2.*t10+-2.0.*e2_1_1.*e1_1_1.*e1_2_2+2.0.*e2_1_1.*e1_1_2.*e1_2_1+2.0.*e2_1_2.*e1_1_1.*e1_2_1+2.0.*e2_2_1.*e1_1_1.*e1_1_2+2.0.*e2_1_2.*e1_1_2.*e1_2_2+2.0.*e2_1_2.*e1_1_3.*e1_2_3+2.0.*e2_1_3.*e1_1_2.*e1_2_3+-2.0.*e2_1_3.*e1_1_3.*e1_2_2+2.0.*e2_2_3.*e1_1_2.*e1_1_3+2.0.*e2_2_1.*e1_2_1.*e1_2_2+2.0.*e2_2_3.*e1_2_2.*e1_2_3+2.0.*e2_2_1.*e1_3_1.*e1_3_2+2.0.*e2_3_1.*e1_2_1.*e1_3_2+-2.0.*e2_3_1.*e1_2_2.*e1_3_1+2.0.*e2_3_2.*e1_2_1.*e1_3_1+2.0.*e2_3_2.*e1_2_2.*e1_3_2+2.0.*e2_2_3.*e1_3_2.*e1_3_3+2.0.*e2_3_2.*e1_2_3.*e1_3_3+-2.0.*e2_3_3.*e1_2_2.*e1_3_3+2.0.*e2_3_3.*e1_2_3.*e1_3_2];
mt10 = [-e2_3_2.*t2+e2_3_2.*t3-e2_3_2.*t4-e2_3_2.*t5+e2_3_2.*t6-e2_3_2.*t7+e2_3_2.*t8+e2_3_2.*t9.*3.0+e2_3_2.*t10+-2.0.*e2_1_1.*e1_1_1.*e1_3_2+2.0.*e2_1_1.*e1_1_2.*e1_3_1+2.0.*e2_1_2.*e1_1_1.*e1_3_1+2.0.*e2_3_1.*e1_1_1.*e1_1_2+2.0.*e2_1_2.*e1_1_2.*e1_3_2+2.0.*e2_1_2.*e1_1_3.*e1_3_3+2.0.*e2_1_3.*e1_1_2.*e1_3_3+-2.0.*e2_1_3.*e1_1_3.*e1_3_2+2.0.*e2_3_3.*e1_1_2.*e1_1_3+-2.0.*e2_2_1.*e1_2_1.*e1_3_2+2.0.*e2_2_1.*e1_2_2.*e1_3_1+2.0.*e2_2_2.*e1_2_1.*e1_3_1+2.0.*e2_3_1.*e1_2_1.*e1_2_2+2.0.*e2_2_2.*e1_2_2.*e1_3_2+2.0.*e2_2_2.*e1_2_3.*e1_3_3+2.0.*e2_2_3.*e1_2_2.*e1_3_3+-2.0.*e2_2_3.*e1_2_3.*e1_3_2+2.0.*e2_3_3.*e1_2_2.*e1_2_3+2.0.*e2_3_1.*e1_3_1.*e1_3_2+2.0.*e2_3_3.*e1_3_2.*e1_3_3];
mt11 = [e2_1_3.*t2+e2_1_3.*t3+e2_1_3.*t4.*3.0-e2_1_3.*t5-e2_1_3.*t6+e2_1_3.*t7-e2_1_3.*t8-e2_1_3.*t9+e2_1_3.*t10+2.0.*e2_1_1.*e1_1_1.*e1_1_3+2.0.*e2_1_2.*e1_1_2.*e1_1_3+2.0.*e2_1_1.*e1_2_1.*e1_2_3+2.0.*e2_2_1.*e1_1_1.*e1_2_3+-2.0.*e2_2_1.*e1_1_3.*e1_2_1+2.0.*e2_2_3.*e1_1_1.*e1_2_1+2.0.*e2_1_2.*e1_2_2.*e1_2_3+2.0.*e2_2_2.*e1_1_2.*e1_2_3+-2.0.*e2_2_2.*e1_1_3.*e1_2_2+2.0.*e2_2_3.*e1_1_2.*e1_2_2+2.0.*e2_2_3.*e1_1_3.*e1_2_3+2.0.*e2_1_1.*e1_3_1.*e1_3_3+2.0.*e2_3_1.*e1_1_1.*e1_3_3+-2.0.*e2_3_1.*e1_1_3.*e1_3_1+2.0.*e2_3_3.*e1_1_1.*e1_3_1+2.0.*e2_1_2.*e1_3_2.*e1_3_3+2.0.*e2_3_2.*e1_1_2.*e1_3_3+-2.0.*e2_3_2.*e1_1_3.*e1_3_2+2.0.*e2_3_3.*e1_1_2.*e1_3_2+2.0.*e2_3_3.*e1_1_3.*e1_3_3];
mt12 = [-e2_2_3.*t2-e2_2_3.*t3+e2_2_3.*t4+e2_2_3.*t5+e2_2_3.*t6+e2_2_3.*t7.*3.0-e2_2_3.*t8-e2_2_3.*t9+e2_2_3.*t10+-2.0.*e2_1_1.*e1_1_1.*e1_2_3+2.0.*e2_1_1.*e1_1_3.*e1_2_1+2.0.*e2_1_3.*e1_1_1.*e1_2_1+2.0.*e2_2_1.*e1_1_1.*e1_1_3+-2.0.*e2_1_2.*e1_1_2.*e1_2_3+2.0.*e2_1_2.*e1_1_3.*e1_2_2+2.0.*e2_1_3.*e1_1_2.*e1_2_2+2.0.*e2_2_2.*e1_1_2.*e1_1_3+2.0.*e2_1_3.*e1_1_3.*e1_2_3+2.0.*e2_2_1.*e1_2_1.*e1_2_3+2.0.*e2_2_2.*e1_2_2.*e1_2_3+2.0.*e2_2_1.*e1_3_1.*e1_3_3+2.0.*e2_3_1.*e1_2_1.*e1_3_3+-2.0.*e2_3_1.*e1_2_3.*e1_3_1+2.0.*e2_3_3.*e1_2_1.*e1_3_1+2.0.*e2_2_2.*e1_3_2.*e1_3_3+2.0.*e2_3_2.*e1_2_2.*e1_3_3+-2.0.*e2_3_2.*e1_2_3.*e1_3_2+2.0.*e2_3_3.*e1_2_2.*e1_3_2+2.0.*e2_3_3.*e1_2_3.*e1_3_3];
mt13 = [-e2_3_3.*t2-e2_3_3.*t3+e2_3_3.*t4-e2_3_3.*t5-e2_3_3.*t6+e2_3_3.*t7+e2_3_3.*t8+e2_3_3.*t9+e2_3_3.*t10.*3.0+-2.0.*e2_1_1.*e1_1_1.*e1_3_3+2.0.*e2_1_1.*e1_1_3.*e1_3_1+2.0.*e2_1_3.*e1_1_1.*e1_3_1+2.0.*e2_3_1.*e1_1_1.*e1_1_3+-2.0.*e2_1_2.*e1_1_2.*e1_3_3+2.0.*e2_1_2.*e1_1_3.*e1_3_2+2.0.*e2_1_3.*e1_1_2.*e1_3_2+2.0.*e2_3_2.*e1_1_2.*e1_1_3+2.0.*e2_1_3.*e1_1_3.*e1_3_3+-2.0.*e2_2_1.*e1_2_1.*e1_3_3+2.0.*e2_2_1.*e1_2_3.*e1_3_1+2.0.*e2_2_3.*e1_2_1.*e1_3_1+2.0.*e2_3_1.*e1_2_1.*e1_2_3+-2.0.*e2_2_2.*e1_2_2.*e1_3_3+2.0.*e2_2_2.*e1_2_3.*e1_3_2+2.0.*e2_2_3.*e1_2_2.*e1_3_2+2.0.*e2_3_2.*e1_2_2.*e1_2_3+2.0.*e2_2_3.*e1_2_3.*e1_3_3+2.0.*e2_3_1.*e1_3_1.*e1_3_3+2.0.*e2_3_2.*e1_3_2.*e1_3_3];
mt14 = [e1_1_1.*e2_2_2.*e2_3_3-e1_1_1.*e2_2_3.*e2_3_2-e1_1_2.*e2_2_1.*e2_3_3+e1_1_2.*e2_2_3.*e2_3_1+e1_1_3.*e2_2_1.*e2_3_2-e1_1_3.*e2_2_2.*e2_3_1-e1_2_1.*e2_1_2.*e2_3_3+e1_2_1.*e2_1_3.*e2_3_2+e1_2_2.*e2_1_1.*e2_3_3-e1_2_2.*e2_1_3.*e2_3_1-e1_2_3.*e2_1_1.*e2_3_2+e1_2_3.*e2_1_2.*e2_3_1+e1_3_1.*e2_1_2.*e2_2_3-e1_3_1.*e2_1_3.*e2_2_2-e1_3_2.*e2_1_1.*e2_2_3+e1_3_2.*e2_1_3.*e2_2_1+e1_3_3.*e2_1_1.*e2_2_2-e1_3_3.*e2_1_2.*e2_2_1];
mt15 = [e1_1_1.*t11.*3.0+e1_1_1.*t12+e1_1_1.*t13+e1_1_1.*t14-e1_1_1.*t15-e1_1_1.*t16+e1_1_1.*t17-e1_1_1.*t18-e1_1_1.*t19+2.0.*e1_1_2.*e2_1_1.*e2_1_2+2.0.*e1_1_3.*e2_1_1.*e2_1_3+2.0.*e1_2_1.*e2_1_1.*e2_2_1+2.0.*e1_1_2.*e2_2_1.*e2_2_2+2.0.*e1_2_1.*e2_1_2.*e2_2_2+-2.0.*e1_2_2.*e2_1_1.*e2_2_2+2.0.*e1_2_2.*e2_1_2.*e2_2_1+2.0.*e1_1_3.*e2_2_1.*e2_2_3+2.0.*e1_2_1.*e2_1_3.*e2_2_3+-2.0.*e1_2_3.*e2_1_1.*e2_2_3+2.0.*e1_2_3.*e2_1_3.*e2_2_1+2.0.*e1_3_1.*e2_1_1.*e2_3_1+2.0.*e1_1_2.*e2_3_1.*e2_3_2+2.0.*e1_3_1.*e2_1_2.*e2_3_2+-2.0.*e1_3_2.*e2_1_1.*e2_3_2+2.0.*e1_3_2.*e2_1_2.*e2_3_1+2.0.*e1_1_3.*e2_3_1.*e2_3_3+2.0.*e1_3_1.*e2_1_3.*e2_3_3+-2.0.*e1_3_3.*e2_1_1.*e2_3_3+2.0.*e1_3_3.*e2_1_3.*e2_3_1];
mt16 = [e1_2_1.*t11-e1_2_1.*t12-e1_2_1.*t13+e1_2_1.*t14.*3.0+e1_2_1.*t15+e1_2_1.*t16+e1_2_1.*t17-e1_2_1.*t18-e1_2_1.*t19+2.0.*e1_1_1.*e2_1_1.*e2_2_1+2.0.*e1_1_1.*e2_1_2.*e2_2_2+2.0.*e1_1_2.*e2_1_1.*e2_2_2+-2.0.*e1_1_2.*e2_1_2.*e2_2_1+2.0.*e1_2_2.*e2_1_1.*e2_1_2+2.0.*e1_1_1.*e2_1_3.*e2_2_3+2.0.*e1_1_3.*e2_1_1.*e2_2_3+-2.0.*e1_1_3.*e2_1_3.*e2_2_1+2.0.*e1_2_3.*e2_1_1.*e2_1_3+2.0.*e1_2_2.*e2_2_1.*e2_2_2+2.0.*e1_2_3.*e2_2_1.*e2_2_3+2.0.*e1_3_1.*e2_2_1.*e2_3_1+2.0.*e1_2_2.*e2_3_1.*e2_3_2+2.0.*e1_3_1.*e2_2_2.*e2_3_2+-2.0.*e1_3_2.*e2_2_1.*e2_3_2+2.0.*e1_3_2.*e2_2_2.*e2_3_1+2.0.*e1_2_3.*e2_3_1.*e2_3_3+2.0.*e1_3_1.*e2_2_3.*e2_3_3+-2.0.*e1_3_3.*e2_2_1.*e2_3_3+2.0.*e1_3_3.*e2_2_3.*e2_3_1];
mt17 = [e1_3_1.*t11-e1_3_1.*t12-e1_3_1.*t13+e1_3_1.*t14-e1_3_1.*t15-e1_3_1.*t16+e1_3_1.*t17.*3.0+e1_3_1.*t18+e1_3_1.*t19+2.0.*e1_1_1.*e2_1_1.*e2_3_1+2.0.*e1_1_1.*e2_1_2.*e2_3_2+2.0.*e1_1_2.*e2_1_1.*e2_3_2+-2.0.*e1_1_2.*e2_1_2.*e2_3_1+2.0.*e1_3_2.*e2_1_1.*e2_1_2+2.0.*e1_1_1.*e2_1_3.*e2_3_3+2.0.*e1_1_3.*e2_1_1.*e2_3_3+-2.0.*e1_1_3.*e2_1_3.*e2_3_1+2.0.*e1_3_3.*e2_1_1.*e2_1_3+2.0.*e1_2_1.*e2_2_1.*e2_3_1+2.0.*e1_2_1.*e2_2_2.*e2_3_2+2.0.*e1_2_2.*e2_2_1.*e2_3_2+-2.0.*e1_2_2.*e2_2_2.*e2_3_1+2.0.*e1_3_2.*e2_2_1.*e2_2_2+2.0.*e1_2_1.*e2_2_3.*e2_3_3+2.0.*e1_2_3.*e2_2_1.*e2_3_3+-2.0.*e1_2_3.*e2_2_3.*e2_3_1+2.0.*e1_3_3.*e2_2_1.*e2_2_3+2.0.*e1_3_2.*e2_3_1.*e2_3_2+2.0.*e1_3_3.*e2_3_1.*e2_3_3];
mt18 = [e1_1_2.*t11+e1_1_2.*t12.*3.0+e1_1_2.*t13-e1_1_2.*t14+e1_1_2.*t15-e1_1_2.*t16-e1_1_2.*t17+e1_1_2.*t18-e1_1_2.*t19+2.0.*e1_1_1.*e2_1_1.*e2_1_2+2.0.*e1_1_3.*e2_1_2.*e2_1_3+2.0.*e1_1_1.*e2_2_1.*e2_2_2+2.0.*e1_2_1.*e2_1_1.*e2_2_2+-2.0.*e1_2_1.*e2_1_2.*e2_2_1+2.0.*e1_2_2.*e2_1_1.*e2_2_1+2.0.*e1_2_2.*e2_1_2.*e2_2_2+2.0.*e1_1_3.*e2_2_2.*e2_2_3+2.0.*e1_2_2.*e2_1_3.*e2_2_3+-2.0.*e1_2_3.*e2_1_2.*e2_2_3+2.0.*e1_2_3.*e2_1_3.*e2_2_2+2.0.*e1_1_1.*e2_3_1.*e2_3_2+2.0.*e1_3_1.*e2_1_1.*e2_3_2+-2.0.*e1_3_1.*e2_1_2.*e2_3_1+2.0.*e1_3_2.*e2_1_1.*e2_3_1+2.0.*e1_3_2.*e2_1_2.*e2_3_2+2.0.*e1_1_3.*e2_3_2.*e2_3_3+2.0.*e1_3_2.*e2_1_3.*e2_3_3+-2.0.*e1_3_3.*e2_1_2.*e2_3_3+2.0.*e1_3_3.*e2_1_3.*e2_3_2];
mt19 = [-e1_2_2.*t11+e1_2_2.*t12-e1_2_2.*t13+e1_2_2.*t14+e1_2_2.*t15.*3.0+e1_2_2.*t16-e1_2_2.*t17+e1_2_2.*t18-e1_2_2.*t19+-2.0.*e1_1_1.*e2_1_1.*e2_2_2+2.0.*e1_1_1.*e2_1_2.*e2_2_1+2.0.*e1_1_2.*e2_1_1.*e2_2_1+2.0.*e1_2_1.*e2_1_1.*e2_1_2+2.0.*e1_1_2.*e2_1_2.*e2_2_2+2.0.*e1_1_2.*e2_1_3.*e2_2_3+2.0.*e1_1_3.*e2_1_2.*e2_2_3+-2.0.*e1_1_3.*e2_1_3.*e2_2_2+2.0.*e1_2_3.*e2_1_2.*e2_1_3+2.0.*e1_2_1.*e2_2_1.*e2_2_2+2.0.*e1_2_3.*e2_2_2.*e2_2_3+2.0.*e1_2_1.*e2_3_1.*e2_3_2+2.0.*e1_3_1.*e2_2_1.*e2_3_2+-2.0.*e1_3_1.*e2_2_2.*e2_3_1+2.0.*e1_3_2.*e2_2_1.*e2_3_1+2.0.*e1_3_2.*e2_2_2.*e2_3_2+2.0.*e1_2_3.*e2_3_2.*e2_3_3+2.0.*e1_3_2.*e2_2_3.*e2_3_3+-2.0.*e1_3_3.*e2_2_2.*e2_3_3+2.0.*e1_3_3.*e2_2_3.*e2_3_2];
mt20 = [-e1_3_2.*t11+e1_3_2.*t12-e1_3_2.*t13-e1_3_2.*t14+e1_3_2.*t15-e1_3_2.*t16+e1_3_2.*t17+e1_3_2.*t18.*3.0+e1_3_2.*t19+-2.0.*e1_1_1.*e2_1_1.*e2_3_2+2.0.*e1_1_1.*e2_1_2.*e2_3_1+2.0.*e1_1_2.*e2_1_1.*e2_3_1+2.0.*e1_3_1.*e2_1_1.*e2_1_2+2.0.*e1_1_2.*e2_1_2.*e2_3_2+2.0.*e1_1_2.*e2_1_3.*e2_3_3+2.0.*e1_1_3.*e2_1_2.*e2_3_3+-2.0.*e1_1_3.*e2_1_3.*e2_3_2+2.0.*e1_3_3.*e2_1_2.*e2_1_3+-2.0.*e1_2_1.*e2_2_1.*e2_3_2+2.0.*e1_2_1.*e2_2_2.*e2_3_1+2.0.*e1_2_2.*e2_2_1.*e2_3_1+2.0.*e1_3_1.*e2_2_1.*e2_2_2+2.0.*e1_2_2.*e2_2_2.*e2_3_2+2.0.*e1_2_2.*e2_2_3.*e2_3_3+2.0.*e1_2_3.*e2_2_2.*e2_3_3+-2.0.*e1_2_3.*e2_2_3.*e2_3_2+2.0.*e1_3_3.*e2_2_2.*e2_2_3+2.0.*e1_3_1.*e2_3_1.*e2_3_2+2.0.*e1_3_3.*e2_3_2.*e2_3_3];
mt21 = [e1_1_3.*t11+e1_1_3.*t12+e1_1_3.*t13.*3.0-e1_1_3.*t14-e1_1_3.*t15+e1_1_3.*t16-e1_1_3.*t17-e1_1_3.*t18+e1_1_3.*t19+2.0.*e1_1_1.*e2_1_1.*e2_1_3+2.0.*e1_1_2.*e2_1_2.*e2_1_3+2.0.*e1_1_1.*e2_2_1.*e2_2_3+2.0.*e1_2_1.*e2_1_1.*e2_2_3+-2.0.*e1_2_1.*e2_1_3.*e2_2_1+2.0.*e1_2_3.*e2_1_1.*e2_2_1+2.0.*e1_1_2.*e2_2_2.*e2_2_3+2.0.*e1_2_2.*e2_1_2.*e2_2_3+-2.0.*e1_2_2.*e2_1_3.*e2_2_2+2.0.*e1_2_3.*e2_1_2.*e2_2_2+2.0.*e1_2_3.*e2_1_3.*e2_2_3+2.0.*e1_1_1.*e2_3_1.*e2_3_3+2.0.*e1_3_1.*e2_1_1.*e2_3_3+-2.0.*e1_3_1.*e2_1_3.*e2_3_1+2.0.*e1_3_3.*e2_1_1.*e2_3_1+2.0.*e1_1_2.*e2_3_2.*e2_3_3+2.0.*e1_3_2.*e2_1_2.*e2_3_3+-2.0.*e1_3_2.*e2_1_3.*e2_3_2+2.0.*e1_3_3.*e2_1_2.*e2_3_2+2.0.*e1_3_3.*e2_1_3.*e2_3_3];
mt22 = [-e1_2_3.*t11-e1_2_3.*t12+e1_2_3.*t13+e1_2_3.*t14+e1_2_3.*t15+e1_2_3.*t16.*3.0-e1_2_3.*t17-e1_2_3.*t18+e1_2_3.*t19+-2.0.*e1_1_1.*e2_1_1.*e2_2_3+2.0.*e1_1_1.*e2_1_3.*e2_2_1+2.0.*e1_1_3.*e2_1_1.*e2_2_1+2.0.*e1_2_1.*e2_1_1.*e2_1_3+-2.0.*e1_1_2.*e2_1_2.*e2_2_3+2.0.*e1_1_2.*e2_1_3.*e2_2_2+2.0.*e1_1_3.*e2_1_2.*e2_2_2+2.0.*e1_2_2.*e2_1_2.*e2_1_3+2.0.*e1_1_3.*e2_1_3.*e2_2_3+2.0.*e1_2_1.*e2_2_1.*e2_2_3+2.0.*e1_2_2.*e2_2_2.*e2_2_3+2.0.*e1_2_1.*e2_3_1.*e2_3_3+2.0.*e1_3_1.*e2_2_1.*e2_3_3+-2.0.*e1_3_1.*e2_2_3.*e2_3_1+2.0.*e1_3_3.*e2_2_1.*e2_3_1+2.0.*e1_2_2.*e2_3_2.*e2_3_3+2.0.*e1_3_2.*e2_2_2.*e2_3_3+-2.0.*e1_3_2.*e2_2_3.*e2_3_2+2.0.*e1_3_3.*e2_2_2.*e2_3_2+2.0.*e1_3_3.*e2_2_3.*e2_3_3];
mt23 = [-e1_3_3.*t11-e1_3_3.*t12+e1_3_3.*t13-e1_3_3.*t14-e1_3_3.*t15+e1_3_3.*t16+e1_3_3.*t17+e1_3_3.*t18+e1_3_3.*t19.*3.0+-2.0.*e1_1_1.*e2_1_1.*e2_3_3+2.0.*e1_1_1.*e2_1_3.*e2_3_1+2.0.*e1_1_3.*e2_1_1.*e2_3_1+2.0.*e1_3_1.*e2_1_1.*e2_1_3+-2.0.*e1_1_2.*e2_1_2.*e2_3_3+2.0.*e1_1_2.*e2_1_3.*e2_3_2+2.0.*e1_1_3.*e2_1_2.*e2_3_2+2.0.*e1_3_2.*e2_1_2.*e2_1_3+2.0.*e1_1_3.*e2_1_3.*e2_3_3+-2.0.*e1_2_1.*e2_2_1.*e2_3_3+2.0.*e1_2_1.*e2_2_3.*e2_3_1+2.0.*e1_2_3.*e2_2_1.*e2_3_1+2.0.*e1_3_1.*e2_2_1.*e2_2_3+-2.0.*e1_2_2.*e2_2_2.*e2_3_3+2.0.*e1_2_2.*e2_2_3.*e2_3_2+2.0.*e1_2_3.*e2_2_2.*e2_3_2+2.0.*e1_3_2.*e2_2_2.*e2_2_3+2.0.*e1_2_3.*e2_2_3.*e2_3_3+2.0.*e1_3_1.*e2_3_1.*e2_3_3+2.0.*e1_3_2.*e2_3_2.*e2_3_3,e2_1_1.*e2_2_2.*e2_3_3-e2_1_1.*e2_2_3.*e2_3_2-e2_1_2.*e2_2_1.*e2_3_3+e2_1_2.*e2_2_3.*e2_3_1+e2_1_3.*e2_2_1.*e2_3_2-e2_1_3.*e2_2_2.*e2_3_1];
mt24 = [e2_1_1.*t12+e2_1_1.*t13+e2_1_1.*t14-e2_1_1.*t15-e2_1_1.*t16+e2_1_1.*t17-e2_1_1.*t18-e2_1_1.*t19+e2_1_1.^3+e2_1_2.*e2_2_1.*e2_2_2.*2.0+e2_1_3.*e2_2_1.*e2_2_3.*2.0+e2_1_2.*e2_3_1.*e2_3_2.*2.0+e2_1_3.*e2_3_1.*e2_3_3.*2.0,e2_2_1.*t11-e2_2_1.*t12-e2_2_1.*t13+e2_2_1.*t15+e2_2_1.*t16+e2_2_1.*t17-e2_2_1.*t18-e2_2_1.*t19+e2_2_1.^3+e2_1_1.*e2_1_2.*e2_2_2.*2.0+e2_1_1.*e2_1_3.*e2_2_3.*2.0+e2_2_2.*e2_3_1.*e2_3_2.*2.0+e2_2_3.*e2_3_1.*e2_3_3.*2.0,e2_3_1.*t11-e2_3_1.*t12-e2_3_1.*t13+e2_3_1.*t14-e2_3_1.*t15-e2_3_1.*t16+e2_3_1.*t18+e2_3_1.*t19+e2_3_1.^3+e2_1_1.*e2_1_2.*e2_3_2.*2.0+e2_1_1.*e2_1_3.*e2_3_3.*2.0+e2_2_1.*e2_2_2.*e2_3_2.*2.0+e2_2_1.*e2_2_3.*e2_3_3.*2.0];
mt25 = [e2_1_2.*t11+e2_1_2.*t13-e2_1_2.*t14+e2_1_2.*t15-e2_1_2.*t16-e2_1_2.*t17+e2_1_2.*t18-e2_1_2.*t19+e2_1_2.^3+e2_1_1.*e2_2_1.*e2_2_2.*2.0+e2_1_3.*e2_2_2.*e2_2_3.*2.0+e2_1_1.*e2_3_1.*e2_3_2.*2.0+e2_1_3.*e2_3_2.*e2_3_3.*2.0,-e2_2_2.*t11+e2_2_2.*t12-e2_2_2.*t13+e2_2_2.*t14+e2_2_2.*t16-e2_2_2.*t17+e2_2_2.*t18-e2_2_2.*t19+e2_2_2.^3+e2_1_1.*e2_1_2.*e2_2_1.*2.0+e2_1_2.*e2_1_3.*e2_2_3.*2.0+e2_2_1.*e2_3_1.*e2_3_2.*2.0+e2_2_3.*e2_3_2.*e2_3_3.*2.0,-e2_3_2.*t11+e2_3_2.*t12-e2_3_2.*t13-e2_3_2.*t14+e2_3_2.*t15-e2_3_2.*t16+e2_3_2.*t17+e2_3_2.*t19+e2_3_2.^3+e2_1_1.*e2_1_2.*e2_3_1.*2.0+e2_1_2.*e2_1_3.*e2_3_3.*2.0+e2_2_1.*e2_2_2.*e2_3_1.*2.0+e2_2_2.*e2_2_3.*e2_3_3.*2.0];
mt26 = [e2_1_3.*t11+e2_1_3.*t12-e2_1_3.*t14-e2_1_3.*t15+e2_1_3.*t16-e2_1_3.*t17-e2_1_3.*t18+e2_1_3.*t19+e2_1_3.^3+e2_1_1.*e2_2_1.*e2_2_3.*2.0+e2_1_2.*e2_2_2.*e2_2_3.*2.0+e2_1_1.*e2_3_1.*e2_3_3.*2.0+e2_1_2.*e2_3_2.*e2_3_3.*2.0,-e2_2_3.*t11-e2_2_3.*t12+e2_2_3.*t13+e2_2_3.*t14+e2_2_3.*t15-e2_2_3.*t17-e2_2_3.*t18+e2_2_3.*t19+e2_2_3.^3+e2_1_1.*e2_1_3.*e2_2_1.*2.0+e2_1_2.*e2_1_3.*e2_2_2.*2.0+e2_2_1.*e2_3_1.*e2_3_3.*2.0+e2_2_2.*e2_3_2.*e2_3_3.*2.0,-e2_3_3.*t11-e2_3_3.*t12+e2_3_3.*t13-e2_3_3.*t14-e2_3_3.*t15+e2_3_3.*t16+e2_3_3.*t17+e2_3_3.*t18+e2_3_3.^3+e2_1_1.*e2_1_3.*e2_3_1.*2.0+e2_1_2.*e2_1_3.*e2_3_2.*2.0+e2_2_1.*e2_2_3.*e2_3_1.*2.0+e2_2_2.*e2_2_3.*e2_3_2.*2.0];
mt27 = [e1_1_1.*e1_2_2.*e4_3_3-e1_1_1.*e1_2_3.*e4_3_2-e1_1_1.*e1_3_2.*e4_2_3+e1_1_1.*e1_3_3.*e4_2_2-e1_1_2.*e1_2_1.*e4_3_3+e1_1_2.*e1_2_3.*e4_3_1+e1_1_2.*e1_3_1.*e4_2_3-e1_1_2.*e1_3_3.*e4_2_1+e1_1_3.*e1_2_1.*e4_3_2-e1_1_3.*e1_2_2.*e4_3_1-e1_1_3.*e1_3_1.*e4_2_2+e1_1_3.*e1_3_2.*e4_2_1+e1_2_1.*e1_3_2.*e4_1_3-e1_2_1.*e1_3_3.*e4_1_2-e1_2_2.*e1_3_1.*e4_1_3+e1_2_2.*e1_3_3.*e4_1_1+e1_2_3.*e1_3_1.*e4_1_2-e1_2_3.*e1_3_2.*e4_1_1];
mt28 = [e4_1_1.*t2.*3.0+e4_1_1.*t3+e4_1_1.*t4+e4_1_1.*t5-e4_1_1.*t6-e4_1_1.*t7+e4_1_1.*t8-e4_1_1.*t9-e4_1_1.*t10+2.0.*e4_1_2.*e1_1_1.*e1_1_2+2.0.*e4_1_3.*e1_1_1.*e1_1_3+2.0.*e4_2_1.*e1_1_1.*e1_2_1+2.0.*e4_1_2.*e1_2_1.*e1_2_2+2.0.*e4_2_1.*e1_1_2.*e1_2_2+-2.0.*e4_2_2.*e1_1_1.*e1_2_2+2.0.*e4_2_2.*e1_1_2.*e1_2_1+2.0.*e4_1_3.*e1_2_1.*e1_2_3+2.0.*e4_2_1.*e1_1_3.*e1_2_3+-2.0.*e4_2_3.*e1_1_1.*e1_2_3+2.0.*e4_2_3.*e1_1_3.*e1_2_1+2.0.*e4_3_1.*e1_1_1.*e1_3_1+2.0.*e4_1_2.*e1_3_1.*e1_3_2+2.0.*e4_3_1.*e1_1_2.*e1_3_2+-2.0.*e4_3_2.*e1_1_1.*e1_3_2+2.0.*e4_3_2.*e1_1_2.*e1_3_1+2.0.*e4_1_3.*e1_3_1.*e1_3_3+2.0.*e4_3_1.*e1_1_3.*e1_3_3+-2.0.*e4_3_3.*e1_1_1.*e1_3_3+2.0.*e4_3_3.*e1_1_3.*e1_3_1];
mt29 = [e4_2_1.*t2-e4_2_1.*t3-e4_2_1.*t4+e4_2_1.*t5.*3.0+e4_2_1.*t6+e4_2_1.*t7+e4_2_1.*t8-e4_2_1.*t9-e4_2_1.*t10+2.0.*e4_1_1.*e1_1_1.*e1_2_1+2.0.*e4_1_1.*e1_1_2.*e1_2_2+2.0.*e4_1_2.*e1_1_1.*e1_2_2+-2.0.*e4_1_2.*e1_1_2.*e1_2_1+2.0.*e4_2_2.*e1_1_1.*e1_1_2+2.0.*e4_1_1.*e1_1_3.*e1_2_3+2.0.*e4_1_3.*e1_1_1.*e1_2_3+-2.0.*e4_1_3.*e1_1_3.*e1_2_1+2.0.*e4_2_3.*e1_1_1.*e1_1_3+2.0.*e4_2_2.*e1_2_1.*e1_2_2+2.0.*e4_2_3.*e1_2_1.*e1_2_3+2.0.*e4_3_1.*e1_2_1.*e1_3_1+2.0.*e4_2_2.*e1_3_1.*e1_3_2+2.0.*e4_3_1.*e1_2_2.*e1_3_2+-2.0.*e4_3_2.*e1_2_1.*e1_3_2+2.0.*e4_3_2.*e1_2_2.*e1_3_1+2.0.*e4_2_3.*e1_3_1.*e1_3_3+2.0.*e4_3_1.*e1_2_3.*e1_3_3+-2.0.*e4_3_3.*e1_2_1.*e1_3_3+2.0.*e4_3_3.*e1_2_3.*e1_3_1];
mt30 = [e4_3_1.*t2-e4_3_1.*t3-e4_3_1.*t4+e4_3_1.*t5-e4_3_1.*t6-e4_3_1.*t7+e4_3_1.*t8.*3.0+e4_3_1.*t9+e4_3_1.*t10+2.0.*e4_1_1.*e1_1_1.*e1_3_1+2.0.*e4_1_1.*e1_1_2.*e1_3_2+2.0.*e4_1_2.*e1_1_1.*e1_3_2+-2.0.*e4_1_2.*e1_1_2.*e1_3_1+2.0.*e4_3_2.*e1_1_1.*e1_1_2+2.0.*e4_1_1.*e1_1_3.*e1_3_3+2.0.*e4_1_3.*e1_1_1.*e1_3_3+-2.0.*e4_1_3.*e1_1_3.*e1_3_1+2.0.*e4_3_3.*e1_1_1.*e1_1_3+2.0.*e4_2_1.*e1_2_1.*e1_3_1+2.0.*e4_2_1.*e1_2_2.*e1_3_2+2.0.*e4_2_2.*e1_2_1.*e1_3_2+-2.0.*e4_2_2.*e1_2_2.*e1_3_1+2.0.*e4_3_2.*e1_2_1.*e1_2_2+2.0.*e4_2_1.*e1_2_3.*e1_3_3+2.0.*e4_2_3.*e1_2_1.*e1_3_3+-2.0.*e4_2_3.*e1_2_3.*e1_3_1+2.0.*e4_3_3.*e1_2_1.*e1_2_3+2.0.*e4_3_2.*e1_3_1.*e1_3_2+2.0.*e4_3_3.*e1_3_1.*e1_3_3];
mt31 = [e4_1_2.*t2+e4_1_2.*t3.*3.0+e4_1_2.*t4-e4_1_2.*t5+e4_1_2.*t6-e4_1_2.*t7-e4_1_2.*t8+e4_1_2.*t9-e4_1_2.*t10+2.0.*e4_1_1.*e1_1_1.*e1_1_2+2.0.*e4_1_3.*e1_1_2.*e1_1_3+2.0.*e4_1_1.*e1_2_1.*e1_2_2+2.0.*e4_2_1.*e1_1_1.*e1_2_2+-2.0.*e4_2_1.*e1_1_2.*e1_2_1+2.0.*e4_2_2.*e1_1_1.*e1_2_1+2.0.*e4_2_2.*e1_1_2.*e1_2_2+2.0.*e4_1_3.*e1_2_2.*e1_2_3+2.0.*e4_2_2.*e1_1_3.*e1_2_3+-2.0.*e4_2_3.*e1_1_2.*e1_2_3+2.0.*e4_2_3.*e1_1_3.*e1_2_2+2.0.*e4_1_1.*e1_3_1.*e1_3_2+2.0.*e4_3_1.*e1_1_1.*e1_3_2+-2.0.*e4_3_1.*e1_1_2.*e1_3_1+2.0.*e4_3_2.*e1_1_1.*e1_3_1+2.0.*e4_3_2.*e1_1_2.*e1_3_2+2.0.*e4_1_3.*e1_3_2.*e1_3_3+2.0.*e4_3_2.*e1_1_3.*e1_3_3+-2.0.*e4_3_3.*e1_1_2.*e1_3_3+2.0.*e4_3_3.*e1_1_3.*e1_3_2];
mt32 = [-e4_2_2.*t2+e4_2_2.*t3-e4_2_2.*t4+e4_2_2.*t5+e4_2_2.*t6.*3.0+e4_2_2.*t7-e4_2_2.*t8+e4_2_2.*t9-e4_2_2.*t10+-2.0.*e4_1_1.*e1_1_1.*e1_2_2+2.0.*e4_1_1.*e1_1_2.*e1_2_1+2.0.*e4_1_2.*e1_1_1.*e1_2_1+2.0.*e4_2_1.*e1_1_1.*e1_1_2+2.0.*e4_1_2.*e1_1_2.*e1_2_2+2.0.*e4_1_2.*e1_1_3.*e1_2_3+2.0.*e4_1_3.*e1_1_2.*e1_2_3+-2.0.*e4_1_3.*e1_1_3.*e1_2_2+2.0.*e4_2_3.*e1_1_2.*e1_1_3+2.0.*e4_2_1.*e1_2_1.*e1_2_2+2.0.*e4_2_3.*e1_2_2.*e1_2_3+2.0.*e4_2_1.*e1_3_1.*e1_3_2+2.0.*e4_3_1.*e1_2_1.*e1_3_2+-2.0.*e4_3_1.*e1_2_2.*e1_3_1+2.0.*e4_3_2.*e1_2_1.*e1_3_1+2.0.*e4_3_2.*e1_2_2.*e1_3_2+2.0.*e4_2_3.*e1_3_2.*e1_3_3+2.0.*e4_3_2.*e1_2_3.*e1_3_3+-2.0.*e4_3_3.*e1_2_2.*e1_3_3+2.0.*e4_3_3.*e1_2_3.*e1_3_2];
mt33 = [-e4_3_2.*t2+e4_3_2.*t3-e4_3_2.*t4-e4_3_2.*t5+e4_3_2.*t6-e4_3_2.*t7+e4_3_2.*t8+e4_3_2.*t9.*3.0+e4_3_2.*t10+-2.0.*e4_1_1.*e1_1_1.*e1_3_2+2.0.*e4_1_1.*e1_1_2.*e1_3_1+2.0.*e4_1_2.*e1_1_1.*e1_3_1+2.0.*e4_3_1.*e1_1_1.*e1_1_2+2.0.*e4_1_2.*e1_1_2.*e1_3_2+2.0.*e4_1_2.*e1_1_3.*e1_3_3+2.0.*e4_1_3.*e1_1_2.*e1_3_3+-2.0.*e4_1_3.*e1_1_3.*e1_3_2+2.0.*e4_3_3.*e1_1_2.*e1_1_3+-2.0.*e4_2_1.*e1_2_1.*e1_3_2+2.0.*e4_2_1.*e1_2_2.*e1_3_1+2.0.*e4_2_2.*e1_2_1.*e1_3_1+2.0.*e4_3_1.*e1_2_1.*e1_2_2+2.0.*e4_2_2.*e1_2_2.*e1_3_2+2.0.*e4_2_2.*e1_2_3.*e1_3_3+2.0.*e4_2_3.*e1_2_2.*e1_3_3+-2.0.*e4_2_3.*e1_2_3.*e1_3_2+2.0.*e4_3_3.*e1_2_2.*e1_2_3+2.0.*e4_3_1.*e1_3_1.*e1_3_2+2.0.*e4_3_3.*e1_3_2.*e1_3_3];
mt34 = [e4_1_3.*t2+e4_1_3.*t3+e4_1_3.*t4.*3.0-e4_1_3.*t5-e4_1_3.*t6+e4_1_3.*t7-e4_1_3.*t8-e4_1_3.*t9+e4_1_3.*t10+2.0.*e4_1_1.*e1_1_1.*e1_1_3+2.0.*e4_1_2.*e1_1_2.*e1_1_3+2.0.*e4_1_1.*e1_2_1.*e1_2_3+2.0.*e4_2_1.*e1_1_1.*e1_2_3+-2.0.*e4_2_1.*e1_1_3.*e1_2_1+2.0.*e4_2_3.*e1_1_1.*e1_2_1+2.0.*e4_1_2.*e1_2_2.*e1_2_3+2.0.*e4_2_2.*e1_1_2.*e1_2_3+-2.0.*e4_2_2.*e1_1_3.*e1_2_2+2.0.*e4_2_3.*e1_1_2.*e1_2_2+2.0.*e4_2_3.*e1_1_3.*e1_2_3+2.0.*e4_1_1.*e1_3_1.*e1_3_3+2.0.*e4_3_1.*e1_1_1.*e1_3_3+-2.0.*e4_3_1.*e1_1_3.*e1_3_1+2.0.*e4_3_3.*e1_1_1.*e1_3_1+2.0.*e4_1_2.*e1_3_2.*e1_3_3+2.0.*e4_3_2.*e1_1_2.*e1_3_3+-2.0.*e4_3_2.*e1_1_3.*e1_3_2+2.0.*e4_3_3.*e1_1_2.*e1_3_2+2.0.*e4_3_3.*e1_1_3.*e1_3_3];
mt35 = [-e4_2_3.*t2-e4_2_3.*t3+e4_2_3.*t4+e4_2_3.*t5+e4_2_3.*t6+e4_2_3.*t7.*3.0-e4_2_3.*t8-e4_2_3.*t9+e4_2_3.*t10+-2.0.*e4_1_1.*e1_1_1.*e1_2_3+2.0.*e4_1_1.*e1_1_3.*e1_2_1+2.0.*e4_1_3.*e1_1_1.*e1_2_1+2.0.*e4_2_1.*e1_1_1.*e1_1_3+-2.0.*e4_1_2.*e1_1_2.*e1_2_3+2.0.*e4_1_2.*e1_1_3.*e1_2_2+2.0.*e4_1_3.*e1_1_2.*e1_2_2+2.0.*e4_2_2.*e1_1_2.*e1_1_3+2.0.*e4_1_3.*e1_1_3.*e1_2_3+2.0.*e4_2_1.*e1_2_1.*e1_2_3+2.0.*e4_2_2.*e1_2_2.*e1_2_3+2.0.*e4_2_1.*e1_3_1.*e1_3_3+2.0.*e4_3_1.*e1_2_1.*e1_3_3+-2.0.*e4_3_1.*e1_2_3.*e1_3_1+2.0.*e4_3_3.*e1_2_1.*e1_3_1+2.0.*e4_2_2.*e1_3_2.*e1_3_3+2.0.*e4_3_2.*e1_2_2.*e1_3_3+-2.0.*e4_3_2.*e1_2_3.*e1_3_2+2.0.*e4_3_3.*e1_2_2.*e1_3_2+2.0.*e4_3_3.*e1_2_3.*e1_3_3];
mt36 = [-e4_3_3.*t2-e4_3_3.*t3+e4_3_3.*t4-e4_3_3.*t5-e4_3_3.*t6+e4_3_3.*t7+e4_3_3.*t8+e4_3_3.*t9+e4_3_3.*t10.*3.0+-2.0.*e4_1_1.*e1_1_1.*e1_3_3+2.0.*e4_1_1.*e1_1_3.*e1_3_1+2.0.*e4_1_3.*e1_1_1.*e1_3_1+2.0.*e4_3_1.*e1_1_1.*e1_1_3+-2.0.*e4_1_2.*e1_1_2.*e1_3_3+2.0.*e4_1_2.*e1_1_3.*e1_3_2+2.0.*e4_1_3.*e1_1_2.*e1_3_2+2.0.*e4_3_2.*e1_1_2.*e1_1_3+2.0.*e4_1_3.*e1_1_3.*e1_3_3+-2.0.*e4_2_1.*e1_2_1.*e1_3_3+2.0.*e4_2_1.*e1_2_3.*e1_3_1+2.0.*e4_2_3.*e1_2_1.*e1_3_1+2.0.*e4_3_1.*e1_2_1.*e1_2_3+-2.0.*e4_2_2.*e1_2_2.*e1_3_3+2.0.*e4_2_2.*e1_2_3.*e1_3_2+2.0.*e4_2_3.*e1_2_2.*e1_3_2+2.0.*e4_3_2.*e1_2_2.*e1_2_3+2.0.*e4_2_3.*e1_2_3.*e1_3_3+2.0.*e4_3_1.*e1_3_1.*e1_3_3+2.0.*e4_3_2.*e1_3_2.*e1_3_3];
mt37 = [e1_1_1.*e2_2_2.*e4_3_3-e1_1_1.*e2_2_3.*e4_3_2-e1_1_1.*e2_3_2.*e4_2_3+e1_1_1.*e2_3_3.*e4_2_2-e1_1_2.*e2_2_1.*e4_3_3+e1_1_2.*e2_2_3.*e4_3_1+e1_1_2.*e2_3_1.*e4_2_3-e1_1_2.*e2_3_3.*e4_2_1+e1_1_3.*e2_2_1.*e4_3_2-e1_1_3.*e2_2_2.*e4_3_1-e1_1_3.*e2_3_1.*e4_2_2+e1_1_3.*e2_3_2.*e4_2_1-e1_2_1.*e2_1_2.*e4_3_3+e1_2_1.*e2_1_3.*e4_3_2+e1_2_1.*e2_3_2.*e4_1_3-e1_2_1.*e2_3_3.*e4_1_2+e1_2_2.*e2_1_1.*e4_3_3-e1_2_2.*e2_1_3.*e4_3_1-e1_2_2.*e2_3_1.*e4_1_3+e1_2_2.*e2_3_3.*e4_1_1-e1_2_3.*e2_1_1.*e4_3_2+e1_2_3.*e2_1_2.*e4_3_1+e1_2_3.*e2_3_1.*e4_1_2-e1_2_3.*e2_3_2.*e4_1_1+e1_3_1.*e2_1_2.*e4_2_3-e1_3_1.*e2_1_3.*e4_2_2-e1_3_1.*e2_2_2.*e4_1_3+e1_3_1.*e2_2_3.*e4_1_2-e1_3_2.*e2_1_1.*e4_2_3+e1_3_2.*e2_1_3.*e4_2_1+e1_3_2.*e2_2_1.*e4_1_3-e1_3_2.*e2_2_3.*e4_1_1+e1_3_3.*e2_1_1.*e4_2_2-e1_3_3.*e2_1_2.*e4_2_1-e1_3_3.*e2_2_1.*e4_1_2+e1_3_3.*e2_2_2.*e4_1_1,et1+et2,et3+et4,et5+et6,et7+et8,et9+et10,et11+et12];
mt38 = [et13+et14,et15+et16,et17+et18,e2_1_1.*e2_2_2.*e4_3_3-e2_1_1.*e2_2_3.*e4_3_2-e2_1_1.*e2_3_2.*e4_2_3+e2_1_1.*e2_3_3.*e4_2_2-e2_1_2.*e2_2_1.*e4_3_3+e2_1_2.*e2_2_3.*e4_3_1+e2_1_2.*e2_3_1.*e4_2_3-e2_1_2.*e2_3_3.*e4_2_1+e2_1_3.*e2_2_1.*e4_3_2-e2_1_3.*e2_2_2.*e4_3_1-e2_1_3.*e2_3_1.*e4_2_2+e2_1_3.*e2_3_2.*e4_2_1+e2_2_1.*e2_3_2.*e4_1_3-e2_2_1.*e2_3_3.*e4_1_2-e2_2_2.*e2_3_1.*e4_1_3+e2_2_2.*e2_3_3.*e4_1_1+e2_2_3.*e2_3_1.*e4_1_2-e2_2_3.*e2_3_2.*e4_1_1];
mt39 = [e4_1_1.*t11.*3.0+e4_1_1.*t12+e4_1_1.*t13+e4_1_1.*t14-e4_1_1.*t15-e4_1_1.*t16+e4_1_1.*t17-e4_1_1.*t18-e4_1_1.*t19+2.0.*e4_1_2.*e2_1_1.*e2_1_2+2.0.*e4_1_3.*e2_1_1.*e2_1_3+2.0.*e4_2_1.*e2_1_1.*e2_2_1+2.0.*e4_1_2.*e2_2_1.*e2_2_2+2.0.*e4_2_1.*e2_1_2.*e2_2_2+-2.0.*e4_2_2.*e2_1_1.*e2_2_2+2.0.*e4_2_2.*e2_1_2.*e2_2_1+2.0.*e4_1_3.*e2_2_1.*e2_2_3+2.0.*e4_2_1.*e2_1_3.*e2_2_3+-2.0.*e4_2_3.*e2_1_1.*e2_2_3+2.0.*e4_2_3.*e2_1_3.*e2_2_1+2.0.*e4_3_1.*e2_1_1.*e2_3_1+2.0.*e4_1_2.*e2_3_1.*e2_3_2+2.0.*e4_3_1.*e2_1_2.*e2_3_2+-2.0.*e4_3_2.*e2_1_1.*e2_3_2+2.0.*e4_3_2.*e2_1_2.*e2_3_1+2.0.*e4_1_3.*e2_3_1.*e2_3_3+2.0.*e4_3_1.*e2_1_3.*e2_3_3+-2.0.*e4_3_3.*e2_1_1.*e2_3_3+2.0.*e4_3_3.*e2_1_3.*e2_3_1];
mt40 = [e4_2_1.*t11-e4_2_1.*t12-e4_2_1.*t13+e4_2_1.*t14.*3.0+e4_2_1.*t15+e4_2_1.*t16+e4_2_1.*t17-e4_2_1.*t18-e4_2_1.*t19+2.0.*e4_1_1.*e2_1_1.*e2_2_1+2.0.*e4_1_1.*e2_1_2.*e2_2_2+2.0.*e4_1_2.*e2_1_1.*e2_2_2+-2.0.*e4_1_2.*e2_1_2.*e2_2_1+2.0.*e4_2_2.*e2_1_1.*e2_1_2+2.0.*e4_1_1.*e2_1_3.*e2_2_3+2.0.*e4_1_3.*e2_1_1.*e2_2_3+-2.0.*e4_1_3.*e2_1_3.*e2_2_1+2.0.*e4_2_3.*e2_1_1.*e2_1_3+2.0.*e4_2_2.*e2_2_1.*e2_2_2+2.0.*e4_2_3.*e2_2_1.*e2_2_3+2.0.*e4_3_1.*e2_2_1.*e2_3_1+2.0.*e4_2_2.*e2_3_1.*e2_3_2+2.0.*e4_3_1.*e2_2_2.*e2_3_2+-2.0.*e4_3_2.*e2_2_1.*e2_3_2+2.0.*e4_3_2.*e2_2_2.*e2_3_1+2.0.*e4_2_3.*e2_3_1.*e2_3_3+2.0.*e4_3_1.*e2_2_3.*e2_3_3+-2.0.*e4_3_3.*e2_2_1.*e2_3_3+2.0.*e4_3_3.*e2_2_3.*e2_3_1];
mt41 = [e4_3_1.*t11-e4_3_1.*t12-e4_3_1.*t13+e4_3_1.*t14-e4_3_1.*t15-e4_3_1.*t16+e4_3_1.*t17.*3.0+e4_3_1.*t18+e4_3_1.*t19+2.0.*e4_1_1.*e2_1_1.*e2_3_1+2.0.*e4_1_1.*e2_1_2.*e2_3_2+2.0.*e4_1_2.*e2_1_1.*e2_3_2+-2.0.*e4_1_2.*e2_1_2.*e2_3_1+2.0.*e4_3_2.*e2_1_1.*e2_1_2+2.0.*e4_1_1.*e2_1_3.*e2_3_3+2.0.*e4_1_3.*e2_1_1.*e2_3_3+-2.0.*e4_1_3.*e2_1_3.*e2_3_1+2.0.*e4_3_3.*e2_1_1.*e2_1_3+2.0.*e4_2_1.*e2_2_1.*e2_3_1+2.0.*e4_2_1.*e2_2_2.*e2_3_2+2.0.*e4_2_2.*e2_2_1.*e2_3_2+-2.0.*e4_2_2.*e2_2_2.*e2_3_1+2.0.*e4_3_2.*e2_2_1.*e2_2_2+2.0.*e4_2_1.*e2_2_3.*e2_3_3+2.0.*e4_2_3.*e2_2_1.*e2_3_3+-2.0.*e4_2_3.*e2_2_3.*e2_3_1+2.0.*e4_3_3.*e2_2_1.*e2_2_3+2.0.*e4_3_2.*e2_3_1.*e2_3_2+2.0.*e4_3_3.*e2_3_1.*e2_3_3];
mt42 = [e4_1_2.*t11+e4_1_2.*t12.*3.0+e4_1_2.*t13-e4_1_2.*t14+e4_1_2.*t15-e4_1_2.*t16-e4_1_2.*t17+e4_1_2.*t18-e4_1_2.*t19+2.0.*e4_1_1.*e2_1_1.*e2_1_2+2.0.*e4_1_3.*e2_1_2.*e2_1_3+2.0.*e4_1_1.*e2_2_1.*e2_2_2+2.0.*e4_2_1.*e2_1_1.*e2_2_2+-2.0.*e4_2_1.*e2_1_2.*e2_2_1+2.0.*e4_2_2.*e2_1_1.*e2_2_1+2.0.*e4_2_2.*e2_1_2.*e2_2_2+2.0.*e4_1_3.*e2_2_2.*e2_2_3+2.0.*e4_2_2.*e2_1_3.*e2_2_3+-2.0.*e4_2_3.*e2_1_2.*e2_2_3+2.0.*e4_2_3.*e2_1_3.*e2_2_2+2.0.*e4_1_1.*e2_3_1.*e2_3_2+2.0.*e4_3_1.*e2_1_1.*e2_3_2+-2.0.*e4_3_1.*e2_1_2.*e2_3_1+2.0.*e4_3_2.*e2_1_1.*e2_3_1+2.0.*e4_3_2.*e2_1_2.*e2_3_2+2.0.*e4_1_3.*e2_3_2.*e2_3_3+2.0.*e4_3_2.*e2_1_3.*e2_3_3+-2.0.*e4_3_3.*e2_1_2.*e2_3_3+2.0.*e4_3_3.*e2_1_3.*e2_3_2];
mt43 = [-e4_2_2.*t11+e4_2_2.*t12-e4_2_2.*t13+e4_2_2.*t14+e4_2_2.*t15.*3.0+e4_2_2.*t16-e4_2_2.*t17+e4_2_2.*t18-e4_2_2.*t19+-2.0.*e4_1_1.*e2_1_1.*e2_2_2+2.0.*e4_1_1.*e2_1_2.*e2_2_1+2.0.*e4_1_2.*e2_1_1.*e2_2_1+2.0.*e4_2_1.*e2_1_1.*e2_1_2+2.0.*e4_1_2.*e2_1_2.*e2_2_2+2.0.*e4_1_2.*e2_1_3.*e2_2_3+2.0.*e4_1_3.*e2_1_2.*e2_2_3+-2.0.*e4_1_3.*e2_1_3.*e2_2_2+2.0.*e4_2_3.*e2_1_2.*e2_1_3+2.0.*e4_2_1.*e2_2_1.*e2_2_2+2.0.*e4_2_3.*e2_2_2.*e2_2_3+2.0.*e4_2_1.*e2_3_1.*e2_3_2+2.0.*e4_3_1.*e2_2_1.*e2_3_2+-2.0.*e4_3_1.*e2_2_2.*e2_3_1+2.0.*e4_3_2.*e2_2_1.*e2_3_1+2.0.*e4_3_2.*e2_2_2.*e2_3_2+2.0.*e4_2_3.*e2_3_2.*e2_3_3+2.0.*e4_3_2.*e2_2_3.*e2_3_3+-2.0.*e4_3_3.*e2_2_2.*e2_3_3+2.0.*e4_3_3.*e2_2_3.*e2_3_2];
mt44 = [-e4_3_2.*t11+e4_3_2.*t12-e4_3_2.*t13-e4_3_2.*t14+e4_3_2.*t15-e4_3_2.*t16+e4_3_2.*t17+e4_3_2.*t18.*3.0+e4_3_2.*t19+-2.0.*e4_1_1.*e2_1_1.*e2_3_2+2.0.*e4_1_1.*e2_1_2.*e2_3_1+2.0.*e4_1_2.*e2_1_1.*e2_3_1+2.0.*e4_3_1.*e2_1_1.*e2_1_2+2.0.*e4_1_2.*e2_1_2.*e2_3_2+2.0.*e4_1_2.*e2_1_3.*e2_3_3+2.0.*e4_1_3.*e2_1_2.*e2_3_3+-2.0.*e4_1_3.*e2_1_3.*e2_3_2+2.0.*e4_3_3.*e2_1_2.*e2_1_3+-2.0.*e4_2_1.*e2_2_1.*e2_3_2+2.0.*e4_2_1.*e2_2_2.*e2_3_1+2.0.*e4_2_2.*e2_2_1.*e2_3_1+2.0.*e4_3_1.*e2_2_1.*e2_2_2+2.0.*e4_2_2.*e2_2_2.*e2_3_2+2.0.*e4_2_2.*e2_2_3.*e2_3_3+2.0.*e4_2_3.*e2_2_2.*e2_3_3+-2.0.*e4_2_3.*e2_2_3.*e2_3_2+2.0.*e4_3_3.*e2_2_2.*e2_2_3+2.0.*e4_3_1.*e2_3_1.*e2_3_2+2.0.*e4_3_3.*e2_3_2.*e2_3_3];
mt45 = [e4_1_3.*t11+e4_1_3.*t12+e4_1_3.*t13.*3.0-e4_1_3.*t14-e4_1_3.*t15+e4_1_3.*t16-e4_1_3.*t17-e4_1_3.*t18+e4_1_3.*t19+2.0.*e4_1_1.*e2_1_1.*e2_1_3+2.0.*e4_1_2.*e2_1_2.*e2_1_3+2.0.*e4_1_1.*e2_2_1.*e2_2_3+2.0.*e4_2_1.*e2_1_1.*e2_2_3+-2.0.*e4_2_1.*e2_1_3.*e2_2_1+2.0.*e4_2_3.*e2_1_1.*e2_2_1+2.0.*e4_1_2.*e2_2_2.*e2_2_3+2.0.*e4_2_2.*e2_1_2.*e2_2_3+-2.0.*e4_2_2.*e2_1_3.*e2_2_2+2.0.*e4_2_3.*e2_1_2.*e2_2_2+2.0.*e4_2_3.*e2_1_3.*e2_2_3+2.0.*e4_1_1.*e2_3_1.*e2_3_3+2.0.*e4_3_1.*e2_1_1.*e2_3_3+-2.0.*e4_3_1.*e2_1_3.*e2_3_1+2.0.*e4_3_3.*e2_1_1.*e2_3_1+2.0.*e4_1_2.*e2_3_2.*e2_3_3+2.0.*e4_3_2.*e2_1_2.*e2_3_3+-2.0.*e4_3_2.*e2_1_3.*e2_3_2+2.0.*e4_3_3.*e2_1_2.*e2_3_2+2.0.*e4_3_3.*e2_1_3.*e2_3_3];
mt46 = [-e4_2_3.*t11-e4_2_3.*t12+e4_2_3.*t13+e4_2_3.*t14+e4_2_3.*t15+e4_2_3.*t16.*3.0-e4_2_3.*t17-e4_2_3.*t18+e4_2_3.*t19+-2.0.*e4_1_1.*e2_1_1.*e2_2_3+2.0.*e4_1_1.*e2_1_3.*e2_2_1+2.0.*e4_1_3.*e2_1_1.*e2_2_1+2.0.*e4_2_1.*e2_1_1.*e2_1_3+-2.0.*e4_1_2.*e2_1_2.*e2_2_3+2.0.*e4_1_2.*e2_1_3.*e2_2_2+2.0.*e4_1_3.*e2_1_2.*e2_2_2+2.0.*e4_2_2.*e2_1_2.*e2_1_3+2.0.*e4_1_3.*e2_1_3.*e2_2_3+2.0.*e4_2_1.*e2_2_1.*e2_2_3+2.0.*e4_2_2.*e2_2_2.*e2_2_3+2.0.*e4_2_1.*e2_3_1.*e2_3_3+2.0.*e4_3_1.*e2_2_1.*e2_3_3+-2.0.*e4_3_1.*e2_2_3.*e2_3_1+2.0.*e4_3_3.*e2_2_1.*e2_3_1+2.0.*e4_2_2.*e2_3_2.*e2_3_3+2.0.*e4_3_2.*e2_2_2.*e2_3_3+-2.0.*e4_3_2.*e2_2_3.*e2_3_2+2.0.*e4_3_3.*e2_2_2.*e2_3_2+2.0.*e4_3_3.*e2_2_3.*e2_3_3];
mt47 = [-e4_3_3.*t11-e4_3_3.*t12+e4_3_3.*t13-e4_3_3.*t14-e4_3_3.*t15+e4_3_3.*t16+e4_3_3.*t17+e4_3_3.*t18+e4_3_3.*t19.*3.0+-2.0.*e4_1_1.*e2_1_1.*e2_3_3+2.0.*e4_1_1.*e2_1_3.*e2_3_1+2.0.*e4_1_3.*e2_1_1.*e2_3_1+2.0.*e4_3_1.*e2_1_1.*e2_1_3+-2.0.*e4_1_2.*e2_1_2.*e2_3_3+2.0.*e4_1_2.*e2_1_3.*e2_3_2+2.0.*e4_1_3.*e2_1_2.*e2_3_2+2.0.*e4_3_2.*e2_1_2.*e2_1_3+2.0.*e4_1_3.*e2_1_3.*e2_3_3+-2.0.*e4_2_1.*e2_2_1.*e2_3_3+2.0.*e4_2_1.*e2_2_3.*e2_3_1+2.0.*e4_2_3.*e2_2_1.*e2_3_1+2.0.*e4_3_1.*e2_2_1.*e2_2_3+-2.0.*e4_2_2.*e2_2_2.*e2_3_3+2.0.*e4_2_2.*e2_2_3.*e2_3_2+2.0.*e4_2_3.*e2_2_2.*e2_3_2+2.0.*e4_3_2.*e2_2_2.*e2_2_3+2.0.*e4_2_3.*e2_2_3.*e2_3_3+2.0.*e4_3_1.*e2_3_1.*e2_3_3+2.0.*e4_3_2.*e2_3_2.*e2_3_3];
mt48 = [e1_1_1.*e4_2_2.*e4_3_3-e1_1_1.*e4_2_3.*e4_3_2-e1_1_2.*e4_2_1.*e4_3_3+e1_1_2.*e4_2_3.*e4_3_1+e1_1_3.*e4_2_1.*e4_3_2-e1_1_3.*e4_2_2.*e4_3_1-e1_2_1.*e4_1_2.*e4_3_3+e1_2_1.*e4_1_3.*e4_3_2+e1_2_2.*e4_1_1.*e4_3_3-e1_2_2.*e4_1_3.*e4_3_1-e1_2_3.*e4_1_1.*e4_3_2+e1_2_3.*e4_1_2.*e4_3_1+e1_3_1.*e4_1_2.*e4_2_3-e1_3_1.*e4_1_3.*e4_2_2-e1_3_2.*e4_1_1.*e4_2_3+e1_3_2.*e4_1_3.*e4_2_1+e1_3_3.*e4_1_1.*e4_2_2-e1_3_3.*e4_1_2.*e4_2_1];
mt49 = [e1_1_1.*t29.*3.0+e1_1_1.*t30+e1_1_1.*t31+e1_1_1.*t32-e1_1_1.*t33-e1_1_1.*t34+e1_1_1.*t35-e1_1_1.*t36-e1_1_1.*t37+2.0.*e1_1_2.*e4_1_1.*e4_1_2+2.0.*e1_1_3.*e4_1_1.*e4_1_3+2.0.*e1_2_1.*e4_1_1.*e4_2_1+2.0.*e1_1_2.*e4_2_1.*e4_2_2+2.0.*e1_2_1.*e4_1_2.*e4_2_2+-2.0.*e1_2_2.*e4_1_1.*e4_2_2+2.0.*e1_2_2.*e4_1_2.*e4_2_1+2.0.*e1_1_3.*e4_2_1.*e4_2_3+2.0.*e1_2_1.*e4_1_3.*e4_2_3+-2.0.*e1_2_3.*e4_1_1.*e4_2_3+2.0.*e1_2_3.*e4_1_3.*e4_2_1+2.0.*e1_3_1.*e4_1_1.*e4_3_1+2.0.*e1_1_2.*e4_3_1.*e4_3_2+2.0.*e1_3_1.*e4_1_2.*e4_3_2+-2.0.*e1_3_2.*e4_1_1.*e4_3_2+2.0.*e1_3_2.*e4_1_2.*e4_3_1+2.0.*e1_1_3.*e4_3_1.*e4_3_3+2.0.*e1_3_1.*e4_1_3.*e4_3_3+-2.0.*e1_3_3.*e4_1_1.*e4_3_3+2.0.*e1_3_3.*e4_1_3.*e4_3_1];
mt50 = [e1_2_1.*t29-e1_2_1.*t30-e1_2_1.*t31+e1_2_1.*t32.*3.0+e1_2_1.*t33+e1_2_1.*t34+e1_2_1.*t35-e1_2_1.*t36-e1_2_1.*t37+2.0.*e1_1_1.*e4_1_1.*e4_2_1+2.0.*e1_1_1.*e4_1_2.*e4_2_2+2.0.*e1_1_2.*e4_1_1.*e4_2_2+-2.0.*e1_1_2.*e4_1_2.*e4_2_1+2.0.*e1_2_2.*e4_1_1.*e4_1_2+2.0.*e1_1_1.*e4_1_3.*e4_2_3+2.0.*e1_1_3.*e4_1_1.*e4_2_3+-2.0.*e1_1_3.*e4_1_3.*e4_2_1+2.0.*e1_2_3.*e4_1_1.*e4_1_3+2.0.*e1_2_2.*e4_2_1.*e4_2_2+2.0.*e1_2_3.*e4_2_1.*e4_2_3+2.0.*e1_3_1.*e4_2_1.*e4_3_1+2.0.*e1_2_2.*e4_3_1.*e4_3_2+2.0.*e1_3_1.*e4_2_2.*e4_3_2+-2.0.*e1_3_2.*e4_2_1.*e4_3_2+2.0.*e1_3_2.*e4_2_2.*e4_3_1+2.0.*e1_2_3.*e4_3_1.*e4_3_3+2.0.*e1_3_1.*e4_2_3.*e4_3_3+-2.0.*e1_3_3.*e4_2_1.*e4_3_3+2.0.*e1_3_3.*e4_2_3.*e4_3_1];
mt51 = [e1_3_1.*t29-e1_3_1.*t30-e1_3_1.*t31+e1_3_1.*t32-e1_3_1.*t33-e1_3_1.*t34+e1_3_1.*t35.*3.0+e1_3_1.*t36+e1_3_1.*t37+2.0.*e1_1_1.*e4_1_1.*e4_3_1+2.0.*e1_1_1.*e4_1_2.*e4_3_2+2.0.*e1_1_2.*e4_1_1.*e4_3_2+-2.0.*e1_1_2.*e4_1_2.*e4_3_1+2.0.*e1_3_2.*e4_1_1.*e4_1_2+2.0.*e1_1_1.*e4_1_3.*e4_3_3+2.0.*e1_1_3.*e4_1_1.*e4_3_3+-2.0.*e1_1_3.*e4_1_3.*e4_3_1+2.0.*e1_3_3.*e4_1_1.*e4_1_3+2.0.*e1_2_1.*e4_2_1.*e4_3_1+2.0.*e1_2_1.*e4_2_2.*e4_3_2+2.0.*e1_2_2.*e4_2_1.*e4_3_2+-2.0.*e1_2_2.*e4_2_2.*e4_3_1+2.0.*e1_3_2.*e4_2_1.*e4_2_2+2.0.*e1_2_1.*e4_2_3.*e4_3_3+2.0.*e1_2_3.*e4_2_1.*e4_3_3+-2.0.*e1_2_3.*e4_2_3.*e4_3_1+2.0.*e1_3_3.*e4_2_1.*e4_2_3+2.0.*e1_3_2.*e4_3_1.*e4_3_2+2.0.*e1_3_3.*e4_3_1.*e4_3_3];
mt52 = [e1_1_2.*t29+e1_1_2.*t30.*3.0+e1_1_2.*t31-e1_1_2.*t32+e1_1_2.*t33-e1_1_2.*t34-e1_1_2.*t35+e1_1_2.*t36-e1_1_2.*t37+2.0.*e1_1_1.*e4_1_1.*e4_1_2+2.0.*e1_1_3.*e4_1_2.*e4_1_3+2.0.*e1_1_1.*e4_2_1.*e4_2_2+2.0.*e1_2_1.*e4_1_1.*e4_2_2+-2.0.*e1_2_1.*e4_1_2.*e4_2_1+2.0.*e1_2_2.*e4_1_1.*e4_2_1+2.0.*e1_2_2.*e4_1_2.*e4_2_2+2.0.*e1_1_3.*e4_2_2.*e4_2_3+2.0.*e1_2_2.*e4_1_3.*e4_2_3+-2.0.*e1_2_3.*e4_1_2.*e4_2_3+2.0.*e1_2_3.*e4_1_3.*e4_2_2+2.0.*e1_1_1.*e4_3_1.*e4_3_2+2.0.*e1_3_1.*e4_1_1.*e4_3_2+-2.0.*e1_3_1.*e4_1_2.*e4_3_1+2.0.*e1_3_2.*e4_1_1.*e4_3_1+2.0.*e1_3_2.*e4_1_2.*e4_3_2+2.0.*e1_1_3.*e4_3_2.*e4_3_3+2.0.*e1_3_2.*e4_1_3.*e4_3_3+-2.0.*e1_3_3.*e4_1_2.*e4_3_3+2.0.*e1_3_3.*e4_1_3.*e4_3_2];
mt53 = [-e1_2_2.*t29+e1_2_2.*t30-e1_2_2.*t31+e1_2_2.*t32+e1_2_2.*t33.*3.0+e1_2_2.*t34-e1_2_2.*t35+e1_2_2.*t36-e1_2_2.*t37+-2.0.*e1_1_1.*e4_1_1.*e4_2_2+2.0.*e1_1_1.*e4_1_2.*e4_2_1+2.0.*e1_1_2.*e4_1_1.*e4_2_1+2.0.*e1_2_1.*e4_1_1.*e4_1_2+2.0.*e1_1_2.*e4_1_2.*e4_2_2+2.0.*e1_1_2.*e4_1_3.*e4_2_3+2.0.*e1_1_3.*e4_1_2.*e4_2_3+-2.0.*e1_1_3.*e4_1_3.*e4_2_2+2.0.*e1_2_3.*e4_1_2.*e4_1_3+2.0.*e1_2_1.*e4_2_1.*e4_2_2+2.0.*e1_2_3.*e4_2_2.*e4_2_3+2.0.*e1_2_1.*e4_3_1.*e4_3_2+2.0.*e1_3_1.*e4_2_1.*e4_3_2+-2.0.*e1_3_1.*e4_2_2.*e4_3_1+2.0.*e1_3_2.*e4_2_1.*e4_3_1+2.0.*e1_3_2.*e4_2_2.*e4_3_2+2.0.*e1_2_3.*e4_3_2.*e4_3_3+2.0.*e1_3_2.*e4_2_3.*e4_3_3+-2.0.*e1_3_3.*e4_2_2.*e4_3_3+2.0.*e1_3_3.*e4_2_3.*e4_3_2];
mt54 = [-e1_3_2.*t29+e1_3_2.*t30-e1_3_2.*t31-e1_3_2.*t32+e1_3_2.*t33-e1_3_2.*t34+e1_3_2.*t35+e1_3_2.*t36.*3.0+e1_3_2.*t37+-2.0.*e1_1_1.*e4_1_1.*e4_3_2+2.0.*e1_1_1.*e4_1_2.*e4_3_1+2.0.*e1_1_2.*e4_1_1.*e4_3_1+2.0.*e1_3_1.*e4_1_1.*e4_1_2+2.0.*e1_1_2.*e4_1_2.*e4_3_2+2.0.*e1_1_2.*e4_1_3.*e4_3_3+2.0.*e1_1_3.*e4_1_2.*e4_3_3+-2.0.*e1_1_3.*e4_1_3.*e4_3_2+2.0.*e1_3_3.*e4_1_2.*e4_1_3+-2.0.*e1_2_1.*e4_2_1.*e4_3_2+2.0.*e1_2_1.*e4_2_2.*e4_3_1+2.0.*e1_2_2.*e4_2_1.*e4_3_1+2.0.*e1_3_1.*e4_2_1.*e4_2_2+2.0.*e1_2_2.*e4_2_2.*e4_3_2+2.0.*e1_2_2.*e4_2_3.*e4_3_3+2.0.*e1_2_3.*e4_2_2.*e4_3_3+-2.0.*e1_2_3.*e4_2_3.*e4_3_2+2.0.*e1_3_3.*e4_2_2.*e4_2_3+2.0.*e1_3_1.*e4_3_1.*e4_3_2+2.0.*e1_3_3.*e4_3_2.*e4_3_3];
mt55 = [e1_1_3.*t29+e1_1_3.*t30+e1_1_3.*t31.*3.0-e1_1_3.*t32-e1_1_3.*t33+e1_1_3.*t34-e1_1_3.*t35-e1_1_3.*t36+e1_1_3.*t37+2.0.*e1_1_1.*e4_1_1.*e4_1_3+2.0.*e1_1_2.*e4_1_2.*e4_1_3+2.0.*e1_1_1.*e4_2_1.*e4_2_3+2.0.*e1_2_1.*e4_1_1.*e4_2_3+-2.0.*e1_2_1.*e4_1_3.*e4_2_1+2.0.*e1_2_3.*e4_1_1.*e4_2_1+2.0.*e1_1_2.*e4_2_2.*e4_2_3+2.0.*e1_2_2.*e4_1_2.*e4_2_3+-2.0.*e1_2_2.*e4_1_3.*e4_2_2+2.0.*e1_2_3.*e4_1_2.*e4_2_2+2.0.*e1_2_3.*e4_1_3.*e4_2_3+2.0.*e1_1_1.*e4_3_1.*e4_3_3+2.0.*e1_3_1.*e4_1_1.*e4_3_3+-2.0.*e1_3_1.*e4_1_3.*e4_3_1+2.0.*e1_3_3.*e4_1_1.*e4_3_1+2.0.*e1_1_2.*e4_3_2.*e4_3_3+2.0.*e1_3_2.*e4_1_2.*e4_3_3+-2.0.*e1_3_2.*e4_1_3.*e4_3_2+2.0.*e1_3_3.*e4_1_2.*e4_3_2+2.0.*e1_3_3.*e4_1_3.*e4_3_3];
mt56 = [-e1_2_3.*t29-e1_2_3.*t30+e1_2_3.*t31+e1_2_3.*t32+e1_2_3.*t33+e1_2_3.*t34.*3.0-e1_2_3.*t35-e1_2_3.*t36+e1_2_3.*t37+-2.0.*e1_1_1.*e4_1_1.*e4_2_3+2.0.*e1_1_1.*e4_1_3.*e4_2_1+2.0.*e1_1_3.*e4_1_1.*e4_2_1+2.0.*e1_2_1.*e4_1_1.*e4_1_3+-2.0.*e1_1_2.*e4_1_2.*e4_2_3+2.0.*e1_1_2.*e4_1_3.*e4_2_2+2.0.*e1_1_3.*e4_1_2.*e4_2_2+2.0.*e1_2_2.*e4_1_2.*e4_1_3+2.0.*e1_1_3.*e4_1_3.*e4_2_3+2.0.*e1_2_1.*e4_2_1.*e4_2_3+2.0.*e1_2_2.*e4_2_2.*e4_2_3+2.0.*e1_2_1.*e4_3_1.*e4_3_3+2.0.*e1_3_1.*e4_2_1.*e4_3_3+-2.0.*e1_3_1.*e4_2_3.*e4_3_1+2.0.*e1_3_3.*e4_2_1.*e4_3_1+2.0.*e1_2_2.*e4_3_2.*e4_3_3+2.0.*e1_3_2.*e4_2_2.*e4_3_3+-2.0.*e1_3_2.*e4_2_3.*e4_3_2+2.0.*e1_3_3.*e4_2_2.*e4_3_2+2.0.*e1_3_3.*e4_2_3.*e4_3_3];
mt57 = [-e1_3_3.*t29-e1_3_3.*t30+e1_3_3.*t31-e1_3_3.*t32-e1_3_3.*t33+e1_3_3.*t34+e1_3_3.*t35+e1_3_3.*t36+e1_3_3.*t37.*3.0+-2.0.*e1_1_1.*e4_1_1.*e4_3_3+2.0.*e1_1_1.*e4_1_3.*e4_3_1+2.0.*e1_1_3.*e4_1_1.*e4_3_1+2.0.*e1_3_1.*e4_1_1.*e4_1_3+-2.0.*e1_1_2.*e4_1_2.*e4_3_3+2.0.*e1_1_2.*e4_1_3.*e4_3_2+2.0.*e1_1_3.*e4_1_2.*e4_3_2+2.0.*e1_3_2.*e4_1_2.*e4_1_3+2.0.*e1_1_3.*e4_1_3.*e4_3_3+-2.0.*e1_2_1.*e4_2_1.*e4_3_3+2.0.*e1_2_1.*e4_2_3.*e4_3_1+2.0.*e1_2_3.*e4_2_1.*e4_3_1+2.0.*e1_3_1.*e4_2_1.*e4_2_3+-2.0.*e1_2_2.*e4_2_2.*e4_3_3+2.0.*e1_2_2.*e4_2_3.*e4_3_2+2.0.*e1_2_3.*e4_2_2.*e4_3_2+2.0.*e1_3_2.*e4_2_2.*e4_2_3+2.0.*e1_2_3.*e4_2_3.*e4_3_3+2.0.*e1_3_1.*e4_3_1.*e4_3_3+2.0.*e1_3_2.*e4_3_2.*e4_3_3];
mt58 = [e2_1_1.*e4_2_2.*e4_3_3-e2_1_1.*e4_2_3.*e4_3_2-e2_1_2.*e4_2_1.*e4_3_3+e2_1_2.*e4_2_3.*e4_3_1+e2_1_3.*e4_2_1.*e4_3_2-e2_1_3.*e4_2_2.*e4_3_1-e2_2_1.*e4_1_2.*e4_3_3+e2_2_1.*e4_1_3.*e4_3_2+e2_2_2.*e4_1_1.*e4_3_3-e2_2_2.*e4_1_3.*e4_3_1-e2_2_3.*e4_1_1.*e4_3_2+e2_2_3.*e4_1_2.*e4_3_1+e2_3_1.*e4_1_2.*e4_2_3-e2_3_1.*e4_1_3.*e4_2_2-e2_3_2.*e4_1_1.*e4_2_3+e2_3_2.*e4_1_3.*e4_2_1+e2_3_3.*e4_1_1.*e4_2_2-e2_3_3.*e4_1_2.*e4_2_1];
mt59 = [e2_1_1.*t29.*3.0+e2_1_1.*t30+e2_1_1.*t31+e2_1_1.*t32-e2_1_1.*t33-e2_1_1.*t34+e2_1_1.*t35-e2_1_1.*t36-e2_1_1.*t37+2.0.*e2_1_2.*e4_1_1.*e4_1_2+2.0.*e2_1_3.*e4_1_1.*e4_1_3+2.0.*e2_2_1.*e4_1_1.*e4_2_1+2.0.*e2_1_2.*e4_2_1.*e4_2_2+2.0.*e2_2_1.*e4_1_2.*e4_2_2+-2.0.*e2_2_2.*e4_1_1.*e4_2_2+2.0.*e2_2_2.*e4_1_2.*e4_2_1+2.0.*e2_1_3.*e4_2_1.*e4_2_3+2.0.*e2_2_1.*e4_1_3.*e4_2_3+-2.0.*e2_2_3.*e4_1_1.*e4_2_3+2.0.*e2_2_3.*e4_1_3.*e4_2_1+2.0.*e2_3_1.*e4_1_1.*e4_3_1+2.0.*e2_1_2.*e4_3_1.*e4_3_2+2.0.*e2_3_1.*e4_1_2.*e4_3_2+-2.0.*e2_3_2.*e4_1_1.*e4_3_2+2.0.*e2_3_2.*e4_1_2.*e4_3_1+2.0.*e2_1_3.*e4_3_1.*e4_3_3+2.0.*e2_3_1.*e4_1_3.*e4_3_3+-2.0.*e2_3_3.*e4_1_1.*e4_3_3+2.0.*e2_3_3.*e4_1_3.*e4_3_1];
mt60 = [e2_2_1.*t29-e2_2_1.*t30-e2_2_1.*t31+e2_2_1.*t32.*3.0+e2_2_1.*t33+e2_2_1.*t34+e2_2_1.*t35-e2_2_1.*t36-e2_2_1.*t37+2.0.*e2_1_1.*e4_1_1.*e4_2_1+2.0.*e2_1_1.*e4_1_2.*e4_2_2+2.0.*e2_1_2.*e4_1_1.*e4_2_2+-2.0.*e2_1_2.*e4_1_2.*e4_2_1+2.0.*e2_2_2.*e4_1_1.*e4_1_2+2.0.*e2_1_1.*e4_1_3.*e4_2_3+2.0.*e2_1_3.*e4_1_1.*e4_2_3+-2.0.*e2_1_3.*e4_1_3.*e4_2_1+2.0.*e2_2_3.*e4_1_1.*e4_1_3+2.0.*e2_2_2.*e4_2_1.*e4_2_2+2.0.*e2_2_3.*e4_2_1.*e4_2_3+2.0.*e2_3_1.*e4_2_1.*e4_3_1+2.0.*e2_2_2.*e4_3_1.*e4_3_2+2.0.*e2_3_1.*e4_2_2.*e4_3_2+-2.0.*e2_3_2.*e4_2_1.*e4_3_2+2.0.*e2_3_2.*e4_2_2.*e4_3_1+2.0.*e2_2_3.*e4_3_1.*e4_3_3+2.0.*e2_3_1.*e4_2_3.*e4_3_3+-2.0.*e2_3_3.*e4_2_1.*e4_3_3+2.0.*e2_3_3.*e4_2_3.*e4_3_1];
mt61 = [e2_3_1.*t29-e2_3_1.*t30-e2_3_1.*t31+e2_3_1.*t32-e2_3_1.*t33-e2_3_1.*t34+e2_3_1.*t35.*3.0+e2_3_1.*t36+e2_3_1.*t37+2.0.*e2_1_1.*e4_1_1.*e4_3_1+2.0.*e2_1_1.*e4_1_2.*e4_3_2+2.0.*e2_1_2.*e4_1_1.*e4_3_2+-2.0.*e2_1_2.*e4_1_2.*e4_3_1+2.0.*e2_3_2.*e4_1_1.*e4_1_2+2.0.*e2_1_1.*e4_1_3.*e4_3_3+2.0.*e2_1_3.*e4_1_1.*e4_3_3+-2.0.*e2_1_3.*e4_1_3.*e4_3_1+2.0.*e2_3_3.*e4_1_1.*e4_1_3+2.0.*e2_2_1.*e4_2_1.*e4_3_1+2.0.*e2_2_1.*e4_2_2.*e4_3_2+2.0.*e2_2_2.*e4_2_1.*e4_3_2+-2.0.*e2_2_2.*e4_2_2.*e4_3_1+2.0.*e2_3_2.*e4_2_1.*e4_2_2+2.0.*e2_2_1.*e4_2_3.*e4_3_3+2.0.*e2_2_3.*e4_2_1.*e4_3_3+-2.0.*e2_2_3.*e4_2_3.*e4_3_1+2.0.*e2_3_3.*e4_2_1.*e4_2_3+2.0.*e2_3_2.*e4_3_1.*e4_3_2+2.0.*e2_3_3.*e4_3_1.*e4_3_3];
mt62 = [e2_1_2.*t29+e2_1_2.*t30.*3.0+e2_1_2.*t31-e2_1_2.*t32+e2_1_2.*t33-e2_1_2.*t34-e2_1_2.*t35+e2_1_2.*t36-e2_1_2.*t37+2.0.*e2_1_1.*e4_1_1.*e4_1_2+2.0.*e2_1_3.*e4_1_2.*e4_1_3+2.0.*e2_1_1.*e4_2_1.*e4_2_2+2.0.*e2_2_1.*e4_1_1.*e4_2_2+-2.0.*e2_2_1.*e4_1_2.*e4_2_1+2.0.*e2_2_2.*e4_1_1.*e4_2_1+2.0.*e2_2_2.*e4_1_2.*e4_2_2+2.0.*e2_1_3.*e4_2_2.*e4_2_3+2.0.*e2_2_2.*e4_1_3.*e4_2_3+-2.0.*e2_2_3.*e4_1_2.*e4_2_3+2.0.*e2_2_3.*e4_1_3.*e4_2_2+2.0.*e2_1_1.*e4_3_1.*e4_3_2+2.0.*e2_3_1.*e4_1_1.*e4_3_2+-2.0.*e2_3_1.*e4_1_2.*e4_3_1+2.0.*e2_3_2.*e4_1_1.*e4_3_1+2.0.*e2_3_2.*e4_1_2.*e4_3_2+2.0.*e2_1_3.*e4_3_2.*e4_3_3+2.0.*e2_3_2.*e4_1_3.*e4_3_3+-2.0.*e2_3_3.*e4_1_2.*e4_3_3+2.0.*e2_3_3.*e4_1_3.*e4_3_2];
mt63 = [-e2_2_2.*t29+e2_2_2.*t30-e2_2_2.*t31+e2_2_2.*t32+e2_2_2.*t33.*3.0+e2_2_2.*t34-e2_2_2.*t35+e2_2_2.*t36-e2_2_2.*t37+-2.0.*e2_1_1.*e4_1_1.*e4_2_2+2.0.*e2_1_1.*e4_1_2.*e4_2_1+2.0.*e2_1_2.*e4_1_1.*e4_2_1+2.0.*e2_2_1.*e4_1_1.*e4_1_2+2.0.*e2_1_2.*e4_1_2.*e4_2_2+2.0.*e2_1_2.*e4_1_3.*e4_2_3+2.0.*e2_1_3.*e4_1_2.*e4_2_3+-2.0.*e2_1_3.*e4_1_3.*e4_2_2+2.0.*e2_2_3.*e4_1_2.*e4_1_3+2.0.*e2_2_1.*e4_2_1.*e4_2_2+2.0.*e2_2_3.*e4_2_2.*e4_2_3+2.0.*e2_2_1.*e4_3_1.*e4_3_2+2.0.*e2_3_1.*e4_2_1.*e4_3_2+-2.0.*e2_3_1.*e4_2_2.*e4_3_1+2.0.*e2_3_2.*e4_2_1.*e4_3_1+2.0.*e2_3_2.*e4_2_2.*e4_3_2+2.0.*e2_2_3.*e4_3_2.*e4_3_3+2.0.*e2_3_2.*e4_2_3.*e4_3_3+-2.0.*e2_3_3.*e4_2_2.*e4_3_3+2.0.*e2_3_3.*e4_2_3.*e4_3_2];
mt64 = [-e2_3_2.*t29+e2_3_2.*t30-e2_3_2.*t31-e2_3_2.*t32+e2_3_2.*t33-e2_3_2.*t34+e2_3_2.*t35+e2_3_2.*t36.*3.0+e2_3_2.*t37+-2.0.*e2_1_1.*e4_1_1.*e4_3_2+2.0.*e2_1_1.*e4_1_2.*e4_3_1+2.0.*e2_1_2.*e4_1_1.*e4_3_1+2.0.*e2_3_1.*e4_1_1.*e4_1_2+2.0.*e2_1_2.*e4_1_2.*e4_3_2+2.0.*e2_1_2.*e4_1_3.*e4_3_3+2.0.*e2_1_3.*e4_1_2.*e4_3_3+-2.0.*e2_1_3.*e4_1_3.*e4_3_2+2.0.*e2_3_3.*e4_1_2.*e4_1_3+-2.0.*e2_2_1.*e4_2_1.*e4_3_2+2.0.*e2_2_1.*e4_2_2.*e4_3_1+2.0.*e2_2_2.*e4_2_1.*e4_3_1+2.0.*e2_3_1.*e4_2_1.*e4_2_2+2.0.*e2_2_2.*e4_2_2.*e4_3_2+2.0.*e2_2_2.*e4_2_3.*e4_3_3+2.0.*e2_2_3.*e4_2_2.*e4_3_3+-2.0.*e2_2_3.*e4_2_3.*e4_3_2+2.0.*e2_3_3.*e4_2_2.*e4_2_3+2.0.*e2_3_1.*e4_3_1.*e4_3_2+2.0.*e2_3_3.*e4_3_2.*e4_3_3];
mt65 = [e2_1_3.*t29+e2_1_3.*t30+e2_1_3.*t31.*3.0-e2_1_3.*t32-e2_1_3.*t33+e2_1_3.*t34-e2_1_3.*t35-e2_1_3.*t36+e2_1_3.*t37+2.0.*e2_1_1.*e4_1_1.*e4_1_3+2.0.*e2_1_2.*e4_1_2.*e4_1_3+2.0.*e2_1_1.*e4_2_1.*e4_2_3+2.0.*e2_2_1.*e4_1_1.*e4_2_3+-2.0.*e2_2_1.*e4_1_3.*e4_2_1+2.0.*e2_2_3.*e4_1_1.*e4_2_1+2.0.*e2_1_2.*e4_2_2.*e4_2_3+2.0.*e2_2_2.*e4_1_2.*e4_2_3+-2.0.*e2_2_2.*e4_1_3.*e4_2_2+2.0.*e2_2_3.*e4_1_2.*e4_2_2+2.0.*e2_2_3.*e4_1_3.*e4_2_3+2.0.*e2_1_1.*e4_3_1.*e4_3_3+2.0.*e2_3_1.*e4_1_1.*e4_3_3+-2.0.*e2_3_1.*e4_1_3.*e4_3_1+2.0.*e2_3_3.*e4_1_1.*e4_3_1+2.0.*e2_1_2.*e4_3_2.*e4_3_3+2.0.*e2_3_2.*e4_1_2.*e4_3_3+-2.0.*e2_3_2.*e4_1_3.*e4_3_2+2.0.*e2_3_3.*e4_1_2.*e4_3_2+2.0.*e2_3_3.*e4_1_3.*e4_3_3];
mt66 = [-e2_2_3.*t29-e2_2_3.*t30+e2_2_3.*t31+e2_2_3.*t32+e2_2_3.*t33+e2_2_3.*t34.*3.0-e2_2_3.*t35-e2_2_3.*t36+e2_2_3.*t37+-2.0.*e2_1_1.*e4_1_1.*e4_2_3+2.0.*e2_1_1.*e4_1_3.*e4_2_1+2.0.*e2_1_3.*e4_1_1.*e4_2_1+2.0.*e2_2_1.*e4_1_1.*e4_1_3+-2.0.*e2_1_2.*e4_1_2.*e4_2_3+2.0.*e2_1_2.*e4_1_3.*e4_2_2+2.0.*e2_1_3.*e4_1_2.*e4_2_2+2.0.*e2_2_2.*e4_1_2.*e4_1_3+2.0.*e2_1_3.*e4_1_3.*e4_2_3+2.0.*e2_2_1.*e4_2_1.*e4_2_3+2.0.*e2_2_2.*e4_2_2.*e4_2_3+2.0.*e2_2_1.*e4_3_1.*e4_3_3+2.0.*e2_3_1.*e4_2_1.*e4_3_3+-2.0.*e2_3_1.*e4_2_3.*e4_3_1+2.0.*e2_3_3.*e4_2_1.*e4_3_1+2.0.*e2_2_2.*e4_3_2.*e4_3_3+2.0.*e2_3_2.*e4_2_2.*e4_3_3+-2.0.*e2_3_2.*e4_2_3.*e4_3_2+2.0.*e2_3_3.*e4_2_2.*e4_3_2+2.0.*e2_3_3.*e4_2_3.*e4_3_3];
mt67 = [-e2_3_3.*t29-e2_3_3.*t30+e2_3_3.*t31-e2_3_3.*t32-e2_3_3.*t33+e2_3_3.*t34+e2_3_3.*t35+e2_3_3.*t36+e2_3_3.*t37.*3.0+-2.0.*e2_1_1.*e4_1_1.*e4_3_3+2.0.*e2_1_1.*e4_1_3.*e4_3_1+2.0.*e2_1_3.*e4_1_1.*e4_3_1+2.0.*e2_3_1.*e4_1_1.*e4_1_3+-2.0.*e2_1_2.*e4_1_2.*e4_3_3+2.0.*e2_1_2.*e4_1_3.*e4_3_2+2.0.*e2_1_3.*e4_1_2.*e4_3_2+2.0.*e2_3_2.*e4_1_2.*e4_1_3+2.0.*e2_1_3.*e4_1_3.*e4_3_3+-2.0.*e2_2_1.*e4_2_1.*e4_3_3+2.0.*e2_2_1.*e4_2_3.*e4_3_1+2.0.*e2_2_3.*e4_2_1.*e4_3_1+2.0.*e2_3_1.*e4_2_1.*e4_2_3+-2.0.*e2_2_2.*e4_2_2.*e4_3_3+2.0.*e2_2_2.*e4_2_3.*e4_3_2+2.0.*e2_2_3.*e4_2_2.*e4_3_2+2.0.*e2_3_2.*e4_2_2.*e4_2_3+2.0.*e2_2_3.*e4_2_3.*e4_3_3+2.0.*e2_3_1.*e4_3_1.*e4_3_3+2.0.*e2_3_2.*e4_3_2.*e4_3_3,e4_1_1.*e4_2_2.*e4_3_3-e4_1_1.*e4_2_3.*e4_3_2-e4_1_2.*e4_2_1.*e4_3_3+e4_1_2.*e4_2_3.*e4_3_1+e4_1_3.*e4_2_1.*e4_3_2-e4_1_3.*e4_2_2.*e4_3_1];
mt68 = [e4_1_1.*t30+e4_1_1.*t31+e4_1_1.*t32-e4_1_1.*t33-e4_1_1.*t34+e4_1_1.*t35-e4_1_1.*t36-e4_1_1.*t37+e4_1_1.^3+e4_1_2.*e4_2_1.*e4_2_2.*2.0+e4_1_3.*e4_2_1.*e4_2_3.*2.0+e4_1_2.*e4_3_1.*e4_3_2.*2.0+e4_1_3.*e4_3_1.*e4_3_3.*2.0,e4_2_1.*t29-e4_2_1.*t30-e4_2_1.*t31+e4_2_1.*t33+e4_2_1.*t34+e4_2_1.*t35-e4_2_1.*t36-e4_2_1.*t37+e4_2_1.^3+e4_1_1.*e4_1_2.*e4_2_2.*2.0+e4_1_1.*e4_1_3.*e4_2_3.*2.0+e4_2_2.*e4_3_1.*e4_3_2.*2.0+e4_2_3.*e4_3_1.*e4_3_3.*2.0,e4_3_1.*t29-e4_3_1.*t30-e4_3_1.*t31+e4_3_1.*t32-e4_3_1.*t33-e4_3_1.*t34+e4_3_1.*t36+e4_3_1.*t37+e4_3_1.^3+e4_1_1.*e4_1_2.*e4_3_2.*2.0+e4_1_1.*e4_1_3.*e4_3_3.*2.0+e4_2_1.*e4_2_2.*e4_3_2.*2.0+e4_2_1.*e4_2_3.*e4_3_3.*2.0];
mt69 = [e4_1_2.*t29+e4_1_2.*t31-e4_1_2.*t32+e4_1_2.*t33-e4_1_2.*t34-e4_1_2.*t35+e4_1_2.*t36-e4_1_2.*t37+e4_1_2.^3+e4_1_1.*e4_2_1.*e4_2_2.*2.0+e4_1_3.*e4_2_2.*e4_2_3.*2.0+e4_1_1.*e4_3_1.*e4_3_2.*2.0+e4_1_3.*e4_3_2.*e4_3_3.*2.0,-e4_2_2.*t29+e4_2_2.*t30-e4_2_2.*t31+e4_2_2.*t32+e4_2_2.*t34-e4_2_2.*t35+e4_2_2.*t36-e4_2_2.*t37+e4_2_2.^3+e4_1_1.*e4_1_2.*e4_2_1.*2.0+e4_1_2.*e4_1_3.*e4_2_3.*2.0+e4_2_1.*e4_3_1.*e4_3_2.*2.0+e4_2_3.*e4_3_2.*e4_3_3.*2.0,-e4_3_2.*t29+e4_3_2.*t30-e4_3_2.*t31-e4_3_2.*t32+e4_3_2.*t33-e4_3_2.*t34+e4_3_2.*t35+e4_3_2.*t37+e4_3_2.^3+e4_1_1.*e4_1_2.*e4_3_1.*2.0+e4_1_2.*e4_1_3.*e4_3_3.*2.0+e4_2_1.*e4_2_2.*e4_3_1.*2.0+e4_2_2.*e4_2_3.*e4_3_3.*2.0];
mt70 = [e4_1_3.*t29+e4_1_3.*t30-e4_1_3.*t32-e4_1_3.*t33+e4_1_3.*t34-e4_1_3.*t35-e4_1_3.*t36+e4_1_3.*t37+e4_1_3.^3+e4_1_1.*e4_2_1.*e4_2_3.*2.0+e4_1_2.*e4_2_2.*e4_2_3.*2.0+e4_1_1.*e4_3_1.*e4_3_3.*2.0+e4_1_2.*e4_3_2.*e4_3_3.*2.0,-e4_2_3.*t29-e4_2_3.*t30+e4_2_3.*t31+e4_2_3.*t32+e4_2_3.*t33-e4_2_3.*t35-e4_2_3.*t36+e4_2_3.*t37+e4_2_3.^3+e4_1_1.*e4_1_3.*e4_2_1.*2.0+e4_1_2.*e4_1_3.*e4_2_2.*2.0+e4_2_1.*e4_3_1.*e4_3_3.*2.0+e4_2_2.*e4_3_2.*e4_3_3.*2.0,-e4_3_3.*t29-e4_3_3.*t30+e4_3_3.*t31-e4_3_3.*t32-e4_3_3.*t33+e4_3_3.*t34+e4_3_3.*t35+e4_3_3.*t36+e4_3_3.^3+e4_1_1.*e4_1_3.*e4_3_1.*2.0+e4_1_2.*e4_1_3.*e4_3_2.*2.0+e4_2_1.*e4_2_3.*e4_3_1.*2.0+e4_2_2.*e4_2_3.*e4_3_2.*2.0];
C1 = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9,mt10,mt11,mt12,mt13,mt14,mt15,mt16,mt17,mt18,mt19,mt20,mt21,mt22,mt23,mt24,mt25,mt26,mt27,mt28,mt29,mt30,mt31,mt32,mt33,mt34,mt35,mt36,mt37,mt38,mt39,mt40,mt41,mt42,mt43,mt44,mt45,mt46,mt47,mt48,mt49,mt50,mt51,mt52,mt53,mt54,mt55,mt56,mt57,mt58,mt59,mt60,mt61,mt62,mt63,mt64,mt65,mt66,mt67,mt68,mt69,mt70],10,10);

et19 = e1_1_1.*e2_1_1.*e3_1_1.*6.0+e1_1_1.*e2_1_2.*e3_1_2.*2.0+e1_1_2.*e2_1_1.*e3_1_2.*2.0+e1_1_2.*e2_1_2.*e3_1_1.*2.0+e1_1_1.*e2_1_3.*e3_1_3.*2.0+e1_1_3.*e2_1_1.*e3_1_3.*2.0+e1_1_3.*e2_1_3.*e3_1_1.*2.0+e1_1_1.*e2_2_1.*e3_2_1.*2.0+e1_2_1.*e2_1_1.*e3_2_1.*2.0+e1_2_1.*e2_2_1.*e3_1_1.*2.0-e1_1_1.*e2_2_2.*e3_2_2.*2.0+e1_1_2.*e2_2_1.*e3_2_2.*2.0+e1_1_2.*e2_2_2.*e3_2_1.*2.0+e1_2_1.*e2_1_2.*e3_2_2.*2.0+e1_2_1.*e2_2_2.*e3_1_2.*2.0-e1_2_2.*e2_1_1.*e3_2_2.*2.0+e1_2_2.*e2_1_2.*e3_2_1.*2.0+e1_2_2.*e2_2_1.*e3_1_2.*2.0-e1_2_2.*e2_2_2.*e3_1_1.*2.0-e1_1_1.*e2_2_3.*e3_2_3.*2.0+e1_1_3.*e2_2_1.*e3_2_3.*2.0+e1_1_3.*e2_2_3.*e3_2_1.*2.0+e1_2_1.*e2_1_3.*e3_2_3.*2.0+e1_2_1.*e2_2_3.*e3_1_3.*2.0-e1_2_3.*e2_1_1.*e3_2_3.*2.0+e1_2_3.*e2_1_3.*e3_2_1.*2.0+e1_2_3.*e2_2_1.*e3_1_3.*2.0-e1_2_3.*e2_2_3.*e3_1_1.*2.0+e1_1_1.*e2_3_1.*e3_3_1.*2.0+e1_3_1.*e2_1_1.*e3_3_1.*2.0+e1_3_1.*e2_3_1.*e3_1_1.*2.0-e1_1_1.*e2_3_2.*e3_3_2.*2.0+e1_1_2.*e2_3_1.*e3_3_2.*2.0+e1_1_2.*e2_3_2.*e3_3_1.*2.0+e1_3_1.*e2_1_2.*e3_3_2.*2.0+e1_3_1.*e2_3_2.*e3_1_2.*2.0-e1_3_2.*e2_1_1.*e3_3_2.*2.0+e1_3_2.*e2_1_2.*e3_3_1.*2.0;
et20 = e1_3_2.*e2_3_1.*e3_1_2.*2.0-e1_3_2.*e2_3_2.*e3_1_1.*2.0-e1_1_1.*e2_3_3.*e3_3_3.*2.0+e1_1_3.*e2_3_1.*e3_3_3.*2.0+e1_1_3.*e2_3_3.*e3_3_1.*2.0+e1_3_1.*e2_1_3.*e3_3_3.*2.0+e1_3_1.*e2_3_3.*e3_1_3.*2.0-e1_3_3.*e2_1_1.*e3_3_3.*2.0+e1_3_3.*e2_1_3.*e3_3_1.*2.0+e1_3_3.*e2_3_1.*e3_1_3.*2.0-e1_3_3.*e2_3_3.*e3_1_1.*2.0;
et21 = e1_1_1.*e3_1_1.*e4_1_1.*6.0+e1_1_1.*e3_1_2.*e4_1_2.*2.0+e1_1_2.*e3_1_1.*e4_1_2.*2.0+e1_1_2.*e3_1_2.*e4_1_1.*2.0+e1_1_1.*e3_1_3.*e4_1_3.*2.0+e1_1_3.*e3_1_1.*e4_1_3.*2.0+e1_1_3.*e3_1_3.*e4_1_1.*2.0+e1_1_1.*e3_2_1.*e4_2_1.*2.0+e1_2_1.*e3_1_1.*e4_2_1.*2.0+e1_2_1.*e3_2_1.*e4_1_1.*2.0-e1_1_1.*e3_2_2.*e4_2_2.*2.0+e1_1_2.*e3_2_1.*e4_2_2.*2.0+e1_1_2.*e3_2_2.*e4_2_1.*2.0+e1_2_1.*e3_1_2.*e4_2_2.*2.0+e1_2_1.*e3_2_2.*e4_1_2.*2.0-e1_2_2.*e3_1_1.*e4_2_2.*2.0+e1_2_2.*e3_1_2.*e4_2_1.*2.0+e1_2_2.*e3_2_1.*e4_1_2.*2.0-e1_2_2.*e3_2_2.*e4_1_1.*2.0-e1_1_1.*e3_2_3.*e4_2_3.*2.0+e1_1_3.*e3_2_1.*e4_2_3.*2.0+e1_1_3.*e3_2_3.*e4_2_1.*2.0+e1_2_1.*e3_1_3.*e4_2_3.*2.0+e1_2_1.*e3_2_3.*e4_1_3.*2.0-e1_2_3.*e3_1_1.*e4_2_3.*2.0+e1_2_3.*e3_1_3.*e4_2_1.*2.0+e1_2_3.*e3_2_1.*e4_1_3.*2.0-e1_2_3.*e3_2_3.*e4_1_1.*2.0+e1_1_1.*e3_3_1.*e4_3_1.*2.0+e1_3_1.*e3_1_1.*e4_3_1.*2.0+e1_3_1.*e3_3_1.*e4_1_1.*2.0-e1_1_1.*e3_3_2.*e4_3_2.*2.0+e1_1_2.*e3_3_1.*e4_3_2.*2.0+e1_1_2.*e3_3_2.*e4_3_1.*2.0+e1_3_1.*e3_1_2.*e4_3_2.*2.0+e1_3_1.*e3_3_2.*e4_1_2.*2.0-e1_3_2.*e3_1_1.*e4_3_2.*2.0+e1_3_2.*e3_1_2.*e4_3_1.*2.0;
et22 = e1_3_2.*e3_3_1.*e4_1_2.*2.0-e1_3_2.*e3_3_2.*e4_1_1.*2.0-e1_1_1.*e3_3_3.*e4_3_3.*2.0+e1_1_3.*e3_3_1.*e4_3_3.*2.0+e1_1_3.*e3_3_3.*e4_3_1.*2.0+e1_3_1.*e3_1_3.*e4_3_3.*2.0+e1_3_1.*e3_3_3.*e4_1_3.*2.0-e1_3_3.*e3_1_1.*e4_3_3.*2.0+e1_3_3.*e3_1_3.*e4_3_1.*2.0+e1_3_3.*e3_3_1.*e4_1_3.*2.0-e1_3_3.*e3_3_3.*e4_1_1.*2.0;
et23 = e2_1_1.*e3_1_1.*e4_1_1.*6.0+e2_1_1.*e3_1_2.*e4_1_2.*2.0+e2_1_2.*e3_1_1.*e4_1_2.*2.0+e2_1_2.*e3_1_2.*e4_1_1.*2.0+e2_1_1.*e3_1_3.*e4_1_3.*2.0+e2_1_3.*e3_1_1.*e4_1_3.*2.0+e2_1_3.*e3_1_3.*e4_1_1.*2.0+e2_1_1.*e3_2_1.*e4_2_1.*2.0+e2_2_1.*e3_1_1.*e4_2_1.*2.0+e2_2_1.*e3_2_1.*e4_1_1.*2.0-e2_1_1.*e3_2_2.*e4_2_2.*2.0+e2_1_2.*e3_2_1.*e4_2_2.*2.0+e2_1_2.*e3_2_2.*e4_2_1.*2.0+e2_2_1.*e3_1_2.*e4_2_2.*2.0+e2_2_1.*e3_2_2.*e4_1_2.*2.0-e2_2_2.*e3_1_1.*e4_2_2.*2.0+e2_2_2.*e3_1_2.*e4_2_1.*2.0+e2_2_2.*e3_2_1.*e4_1_2.*2.0-e2_2_2.*e3_2_2.*e4_1_1.*2.0-e2_1_1.*e3_2_3.*e4_2_3.*2.0+e2_1_3.*e3_2_1.*e4_2_3.*2.0+e2_1_3.*e3_2_3.*e4_2_1.*2.0+e2_2_1.*e3_1_3.*e4_2_3.*2.0+e2_2_1.*e3_2_3.*e4_1_3.*2.0-e2_2_3.*e3_1_1.*e4_2_3.*2.0+e2_2_3.*e3_1_3.*e4_2_1.*2.0+e2_2_3.*e3_2_1.*e4_1_3.*2.0-e2_2_3.*e3_2_3.*e4_1_1.*2.0+e2_1_1.*e3_3_1.*e4_3_1.*2.0+e2_3_1.*e3_1_1.*e4_3_1.*2.0+e2_3_1.*e3_3_1.*e4_1_1.*2.0-e2_1_1.*e3_3_2.*e4_3_2.*2.0+e2_1_2.*e3_3_1.*e4_3_2.*2.0+e2_1_2.*e3_3_2.*e4_3_1.*2.0+e2_3_1.*e3_1_2.*e4_3_2.*2.0+e2_3_1.*e3_3_2.*e4_1_2.*2.0-e2_3_2.*e3_1_1.*e4_3_2.*2.0+e2_3_2.*e3_1_2.*e4_3_1.*2.0;
et24 = e2_3_2.*e3_3_1.*e4_1_2.*2.0-e2_3_2.*e3_3_2.*e4_1_1.*2.0-e2_1_1.*e3_3_3.*e4_3_3.*2.0+e2_1_3.*e3_3_1.*e4_3_3.*2.0+e2_1_3.*e3_3_3.*e4_3_1.*2.0+e2_3_1.*e3_1_3.*e4_3_3.*2.0+e2_3_1.*e3_3_3.*e4_1_3.*2.0-e2_3_3.*e3_1_1.*e4_3_3.*2.0+e2_3_3.*e3_1_3.*e4_3_1.*2.0+e2_3_3.*e3_3_1.*e4_1_3.*2.0-e2_3_3.*e3_3_3.*e4_1_1.*2.0;
et25 = e1_1_1.*e2_1_1.*e3_2_1.*2.0+e1_1_1.*e2_2_1.*e3_1_1.*2.0+e1_2_1.*e2_1_1.*e3_1_1.*2.0+e1_1_1.*e2_1_2.*e3_2_2.*2.0+e1_1_1.*e2_2_2.*e3_1_2.*2.0+e1_1_2.*e2_1_1.*e3_2_2.*2.0-e1_1_2.*e2_1_2.*e3_2_1.*2.0-e1_1_2.*e2_2_1.*e3_1_2.*2.0+e1_1_2.*e2_2_2.*e3_1_1.*2.0-e1_2_1.*e2_1_2.*e3_1_2.*2.0+e1_2_2.*e2_1_1.*e3_1_2.*2.0+e1_2_2.*e2_1_2.*e3_1_1.*2.0+e1_1_1.*e2_1_3.*e3_2_3.*2.0+e1_1_1.*e2_2_3.*e3_1_3.*2.0+e1_1_3.*e2_1_1.*e3_2_3.*2.0-e1_1_3.*e2_1_3.*e3_2_1.*2.0-e1_1_3.*e2_2_1.*e3_1_3.*2.0+e1_1_3.*e2_2_3.*e3_1_1.*2.0-e1_2_1.*e2_1_3.*e3_1_3.*2.0+e1_2_3.*e2_1_1.*e3_1_3.*2.0+e1_2_3.*e2_1_3.*e3_1_1.*2.0+e1_2_1.*e2_2_1.*e3_2_1.*6.0+e1_2_1.*e2_2_2.*e3_2_2.*2.0+e1_2_2.*e2_2_1.*e3_2_2.*2.0+e1_2_2.*e2_2_2.*e3_2_1.*2.0+e1_2_1.*e2_2_3.*e3_2_3.*2.0+e1_2_3.*e2_2_1.*e3_2_3.*2.0+e1_2_3.*e2_2_3.*e3_2_1.*2.0+e1_2_1.*e2_3_1.*e3_3_1.*2.0+e1_3_1.*e2_2_1.*e3_3_1.*2.0+e1_3_1.*e2_3_1.*e3_2_1.*2.0-e1_2_1.*e2_3_2.*e3_3_2.*2.0+e1_2_2.*e2_3_1.*e3_3_2.*2.0+e1_2_2.*e2_3_2.*e3_3_1.*2.0+e1_3_1.*e2_2_2.*e3_3_2.*2.0+e1_3_1.*e2_3_2.*e3_2_2.*2.0-e1_3_2.*e2_2_1.*e3_3_2.*2.0+e1_3_2.*e2_2_2.*e3_3_1.*2.0;
et26 = e1_3_2.*e2_3_1.*e3_2_2.*2.0-e1_3_2.*e2_3_2.*e3_2_1.*2.0-e1_2_1.*e2_3_3.*e3_3_3.*2.0+e1_2_3.*e2_3_1.*e3_3_3.*2.0+e1_2_3.*e2_3_3.*e3_3_1.*2.0+e1_3_1.*e2_2_3.*e3_3_3.*2.0+e1_3_1.*e2_3_3.*e3_2_3.*2.0-e1_3_3.*e2_2_1.*e3_3_3.*2.0+e1_3_3.*e2_2_3.*e3_3_1.*2.0+e1_3_3.*e2_3_1.*e3_2_3.*2.0-e1_3_3.*e2_3_3.*e3_2_1.*2.0;
et27 = e1_1_1.*e3_1_1.*e4_2_1.*2.0+e1_1_1.*e3_2_1.*e4_1_1.*2.0+e1_2_1.*e3_1_1.*e4_1_1.*2.0+e1_1_1.*e3_1_2.*e4_2_2.*2.0+e1_1_1.*e3_2_2.*e4_1_2.*2.0+e1_1_2.*e3_1_1.*e4_2_2.*2.0-e1_1_2.*e3_1_2.*e4_2_1.*2.0-e1_1_2.*e3_2_1.*e4_1_2.*2.0+e1_1_2.*e3_2_2.*e4_1_1.*2.0-e1_2_1.*e3_1_2.*e4_1_2.*2.0+e1_2_2.*e3_1_1.*e4_1_2.*2.0+e1_2_2.*e3_1_2.*e4_1_1.*2.0+e1_1_1.*e3_1_3.*e4_2_3.*2.0+e1_1_1.*e3_2_3.*e4_1_3.*2.0+e1_1_3.*e3_1_1.*e4_2_3.*2.0-e1_1_3.*e3_1_3.*e4_2_1.*2.0-e1_1_3.*e3_2_1.*e4_1_3.*2.0+e1_1_3.*e3_2_3.*e4_1_1.*2.0-e1_2_1.*e3_1_3.*e4_1_3.*2.0+e1_2_3.*e3_1_1.*e4_1_3.*2.0+e1_2_3.*e3_1_3.*e4_1_1.*2.0+e1_2_1.*e3_2_1.*e4_2_1.*6.0+e1_2_1.*e3_2_2.*e4_2_2.*2.0+e1_2_2.*e3_2_1.*e4_2_2.*2.0+e1_2_2.*e3_2_2.*e4_2_1.*2.0+e1_2_1.*e3_2_3.*e4_2_3.*2.0+e1_2_3.*e3_2_1.*e4_2_3.*2.0+e1_2_3.*e3_2_3.*e4_2_1.*2.0+e1_2_1.*e3_3_1.*e4_3_1.*2.0+e1_3_1.*e3_2_1.*e4_3_1.*2.0+e1_3_1.*e3_3_1.*e4_2_1.*2.0-e1_2_1.*e3_3_2.*e4_3_2.*2.0+e1_2_2.*e3_3_1.*e4_3_2.*2.0+e1_2_2.*e3_3_2.*e4_3_1.*2.0+e1_3_1.*e3_2_2.*e4_3_2.*2.0+e1_3_1.*e3_3_2.*e4_2_2.*2.0-e1_3_2.*e3_2_1.*e4_3_2.*2.0+e1_3_2.*e3_2_2.*e4_3_1.*2.0;
et28 = e1_3_2.*e3_3_1.*e4_2_2.*2.0-e1_3_2.*e3_3_2.*e4_2_1.*2.0-e1_2_1.*e3_3_3.*e4_3_3.*2.0+e1_2_3.*e3_3_1.*e4_3_3.*2.0+e1_2_3.*e3_3_3.*e4_3_1.*2.0+e1_3_1.*e3_2_3.*e4_3_3.*2.0+e1_3_1.*e3_3_3.*e4_2_3.*2.0-e1_3_3.*e3_2_1.*e4_3_3.*2.0+e1_3_3.*e3_2_3.*e4_3_1.*2.0+e1_3_3.*e3_3_1.*e4_2_3.*2.0-e1_3_3.*e3_3_3.*e4_2_1.*2.0;
et29 = e2_1_1.*e3_1_1.*e4_2_1.*2.0+e2_1_1.*e3_2_1.*e4_1_1.*2.0+e2_2_1.*e3_1_1.*e4_1_1.*2.0+e2_1_1.*e3_1_2.*e4_2_2.*2.0+e2_1_1.*e3_2_2.*e4_1_2.*2.0+e2_1_2.*e3_1_1.*e4_2_2.*2.0-e2_1_2.*e3_1_2.*e4_2_1.*2.0-e2_1_2.*e3_2_1.*e4_1_2.*2.0+e2_1_2.*e3_2_2.*e4_1_1.*2.0-e2_2_1.*e3_1_2.*e4_1_2.*2.0+e2_2_2.*e3_1_1.*e4_1_2.*2.0+e2_2_2.*e3_1_2.*e4_1_1.*2.0+e2_1_1.*e3_1_3.*e4_2_3.*2.0+e2_1_1.*e3_2_3.*e4_1_3.*2.0+e2_1_3.*e3_1_1.*e4_2_3.*2.0-e2_1_3.*e3_1_3.*e4_2_1.*2.0-e2_1_3.*e3_2_1.*e4_1_3.*2.0+e2_1_3.*e3_2_3.*e4_1_1.*2.0-e2_2_1.*e3_1_3.*e4_1_3.*2.0+e2_2_3.*e3_1_1.*e4_1_3.*2.0+e2_2_3.*e3_1_3.*e4_1_1.*2.0+e2_2_1.*e3_2_1.*e4_2_1.*6.0+e2_2_1.*e3_2_2.*e4_2_2.*2.0+e2_2_2.*e3_2_1.*e4_2_2.*2.0+e2_2_2.*e3_2_2.*e4_2_1.*2.0+e2_2_1.*e3_2_3.*e4_2_3.*2.0+e2_2_3.*e3_2_1.*e4_2_3.*2.0+e2_2_3.*e3_2_3.*e4_2_1.*2.0+e2_2_1.*e3_3_1.*e4_3_1.*2.0+e2_3_1.*e3_2_1.*e4_3_1.*2.0+e2_3_1.*e3_3_1.*e4_2_1.*2.0-e2_2_1.*e3_3_2.*e4_3_2.*2.0+e2_2_2.*e3_3_1.*e4_3_2.*2.0+e2_2_2.*e3_3_2.*e4_3_1.*2.0+e2_3_1.*e3_2_2.*e4_3_2.*2.0+e2_3_1.*e3_3_2.*e4_2_2.*2.0-e2_3_2.*e3_2_1.*e4_3_2.*2.0+e2_3_2.*e3_2_2.*e4_3_1.*2.0;
et30 = e2_3_2.*e3_3_1.*e4_2_2.*2.0-e2_3_2.*e3_3_2.*e4_2_1.*2.0-e2_2_1.*e3_3_3.*e4_3_3.*2.0+e2_2_3.*e3_3_1.*e4_3_3.*2.0+e2_2_3.*e3_3_3.*e4_3_1.*2.0+e2_3_1.*e3_2_3.*e4_3_3.*2.0+e2_3_1.*e3_3_3.*e4_2_3.*2.0-e2_3_3.*e3_2_1.*e4_3_3.*2.0+e2_3_3.*e3_2_3.*e4_3_1.*2.0+e2_3_3.*e3_3_1.*e4_2_3.*2.0-e2_3_3.*e3_3_3.*e4_2_1.*2.0;
et31 = e1_1_1.*e2_1_1.*e3_3_1.*2.0+e1_1_1.*e2_3_1.*e3_1_1.*2.0+e1_3_1.*e2_1_1.*e3_1_1.*2.0+e1_1_1.*e2_1_2.*e3_3_2.*2.0+e1_1_1.*e2_3_2.*e3_1_2.*2.0+e1_1_2.*e2_1_1.*e3_3_2.*2.0-e1_1_2.*e2_1_2.*e3_3_1.*2.0-e1_1_2.*e2_3_1.*e3_1_2.*2.0+e1_1_2.*e2_3_2.*e3_1_1.*2.0-e1_3_1.*e2_1_2.*e3_1_2.*2.0+e1_3_2.*e2_1_1.*e3_1_2.*2.0+e1_3_2.*e2_1_2.*e3_1_1.*2.0+e1_1_1.*e2_1_3.*e3_3_3.*2.0+e1_1_1.*e2_3_3.*e3_1_3.*2.0+e1_1_3.*e2_1_1.*e3_3_3.*2.0-e1_1_3.*e2_1_3.*e3_3_1.*2.0-e1_1_3.*e2_3_1.*e3_1_3.*2.0+e1_1_3.*e2_3_3.*e3_1_1.*2.0-e1_3_1.*e2_1_3.*e3_1_3.*2.0+e1_3_3.*e2_1_1.*e3_1_3.*2.0+e1_3_3.*e2_1_3.*e3_1_1.*2.0+e1_2_1.*e2_2_1.*e3_3_1.*2.0+e1_2_1.*e2_3_1.*e3_2_1.*2.0+e1_3_1.*e2_2_1.*e3_2_1.*2.0+e1_2_1.*e2_2_2.*e3_3_2.*2.0+e1_2_1.*e2_3_2.*e3_2_2.*2.0+e1_2_2.*e2_2_1.*e3_3_2.*2.0-e1_2_2.*e2_2_2.*e3_3_1.*2.0-e1_2_2.*e2_3_1.*e3_2_2.*2.0+e1_2_2.*e2_3_2.*e3_2_1.*2.0-e1_3_1.*e2_2_2.*e3_2_2.*2.0+e1_3_2.*e2_2_1.*e3_2_2.*2.0+e1_3_2.*e2_2_2.*e3_2_1.*2.0+e1_2_1.*e2_2_3.*e3_3_3.*2.0+e1_2_1.*e2_3_3.*e3_2_3.*2.0+e1_2_3.*e2_2_1.*e3_3_3.*2.0-e1_2_3.*e2_2_3.*e3_3_1.*2.0;
et32 = e1_2_3.*e2_3_1.*e3_2_3.*-2.0+e1_2_3.*e2_3_3.*e3_2_1.*2.0-e1_3_1.*e2_2_3.*e3_2_3.*2.0+e1_3_3.*e2_2_1.*e3_2_3.*2.0+e1_3_3.*e2_2_3.*e3_2_1.*2.0+e1_3_1.*e2_3_1.*e3_3_1.*6.0+e1_3_1.*e2_3_2.*e3_3_2.*2.0+e1_3_2.*e2_3_1.*e3_3_2.*2.0+e1_3_2.*e2_3_2.*e3_3_1.*2.0+e1_3_1.*e2_3_3.*e3_3_3.*2.0+e1_3_3.*e2_3_1.*e3_3_3.*2.0+e1_3_3.*e2_3_3.*e3_3_1.*2.0;
et33 = e1_1_1.*e3_1_1.*e4_3_1.*2.0+e1_1_1.*e3_3_1.*e4_1_1.*2.0+e1_3_1.*e3_1_1.*e4_1_1.*2.0+e1_1_1.*e3_1_2.*e4_3_2.*2.0+e1_1_1.*e3_3_2.*e4_1_2.*2.0+e1_1_2.*e3_1_1.*e4_3_2.*2.0-e1_1_2.*e3_1_2.*e4_3_1.*2.0-e1_1_2.*e3_3_1.*e4_1_2.*2.0+e1_1_2.*e3_3_2.*e4_1_1.*2.0-e1_3_1.*e3_1_2.*e4_1_2.*2.0+e1_3_2.*e3_1_1.*e4_1_2.*2.0+e1_3_2.*e3_1_2.*e4_1_1.*2.0+e1_1_1.*e3_1_3.*e4_3_3.*2.0+e1_1_1.*e3_3_3.*e4_1_3.*2.0+e1_1_3.*e3_1_1.*e4_3_3.*2.0-e1_1_3.*e3_1_3.*e4_3_1.*2.0-e1_1_3.*e3_3_1.*e4_1_3.*2.0+e1_1_3.*e3_3_3.*e4_1_1.*2.0-e1_3_1.*e3_1_3.*e4_1_3.*2.0+e1_3_3.*e3_1_1.*e4_1_3.*2.0+e1_3_3.*e3_1_3.*e4_1_1.*2.0+e1_2_1.*e3_2_1.*e4_3_1.*2.0+e1_2_1.*e3_3_1.*e4_2_1.*2.0+e1_3_1.*e3_2_1.*e4_2_1.*2.0+e1_2_1.*e3_2_2.*e4_3_2.*2.0+e1_2_1.*e3_3_2.*e4_2_2.*2.0+e1_2_2.*e3_2_1.*e4_3_2.*2.0-e1_2_2.*e3_2_2.*e4_3_1.*2.0-e1_2_2.*e3_3_1.*e4_2_2.*2.0+e1_2_2.*e3_3_2.*e4_2_1.*2.0-e1_3_1.*e3_2_2.*e4_2_2.*2.0+e1_3_2.*e3_2_1.*e4_2_2.*2.0+e1_3_2.*e3_2_2.*e4_2_1.*2.0+e1_2_1.*e3_2_3.*e4_3_3.*2.0+e1_2_1.*e3_3_3.*e4_2_3.*2.0+e1_2_3.*e3_2_1.*e4_3_3.*2.0-e1_2_3.*e3_2_3.*e4_3_1.*2.0;
et34 = e1_2_3.*e3_3_1.*e4_2_3.*-2.0+e1_2_3.*e3_3_3.*e4_2_1.*2.0-e1_3_1.*e3_2_3.*e4_2_3.*2.0+e1_3_3.*e3_2_1.*e4_2_3.*2.0+e1_3_3.*e3_2_3.*e4_2_1.*2.0+e1_3_1.*e3_3_1.*e4_3_1.*6.0+e1_3_1.*e3_3_2.*e4_3_2.*2.0+e1_3_2.*e3_3_1.*e4_3_2.*2.0+e1_3_2.*e3_3_2.*e4_3_1.*2.0+e1_3_1.*e3_3_3.*e4_3_3.*2.0+e1_3_3.*e3_3_1.*e4_3_3.*2.0+e1_3_3.*e3_3_3.*e4_3_1.*2.0;
et35 = e2_1_1.*e3_1_1.*e4_3_1.*2.0+e2_1_1.*e3_3_1.*e4_1_1.*2.0+e2_3_1.*e3_1_1.*e4_1_1.*2.0+e2_1_1.*e3_1_2.*e4_3_2.*2.0+e2_1_1.*e3_3_2.*e4_1_2.*2.0+e2_1_2.*e3_1_1.*e4_3_2.*2.0-e2_1_2.*e3_1_2.*e4_3_1.*2.0-e2_1_2.*e3_3_1.*e4_1_2.*2.0+e2_1_2.*e3_3_2.*e4_1_1.*2.0-e2_3_1.*e3_1_2.*e4_1_2.*2.0+e2_3_2.*e3_1_1.*e4_1_2.*2.0+e2_3_2.*e3_1_2.*e4_1_1.*2.0+e2_1_1.*e3_1_3.*e4_3_3.*2.0+e2_1_1.*e3_3_3.*e4_1_3.*2.0+e2_1_3.*e3_1_1.*e4_3_3.*2.0-e2_1_3.*e3_1_3.*e4_3_1.*2.0-e2_1_3.*e3_3_1.*e4_1_3.*2.0+e2_1_3.*e3_3_3.*e4_1_1.*2.0-e2_3_1.*e3_1_3.*e4_1_3.*2.0+e2_3_3.*e3_1_1.*e4_1_3.*2.0+e2_3_3.*e3_1_3.*e4_1_1.*2.0+e2_2_1.*e3_2_1.*e4_3_1.*2.0+e2_2_1.*e3_3_1.*e4_2_1.*2.0+e2_3_1.*e3_2_1.*e4_2_1.*2.0+e2_2_1.*e3_2_2.*e4_3_2.*2.0+e2_2_1.*e3_3_2.*e4_2_2.*2.0+e2_2_2.*e3_2_1.*e4_3_2.*2.0-e2_2_2.*e3_2_2.*e4_3_1.*2.0-e2_2_2.*e3_3_1.*e4_2_2.*2.0+e2_2_2.*e3_3_2.*e4_2_1.*2.0-e2_3_1.*e3_2_2.*e4_2_2.*2.0+e2_3_2.*e3_2_1.*e4_2_2.*2.0+e2_3_2.*e3_2_2.*e4_2_1.*2.0+e2_2_1.*e3_2_3.*e4_3_3.*2.0+e2_2_1.*e3_3_3.*e4_2_3.*2.0+e2_2_3.*e3_2_1.*e4_3_3.*2.0-e2_2_3.*e3_2_3.*e4_3_1.*2.0;
et36 = e2_2_3.*e3_3_1.*e4_2_3.*-2.0+e2_2_3.*e3_3_3.*e4_2_1.*2.0-e2_3_1.*e3_2_3.*e4_2_3.*2.0+e2_3_3.*e3_2_1.*e4_2_3.*2.0+e2_3_3.*e3_2_3.*e4_2_1.*2.0+e2_3_1.*e3_3_1.*e4_3_1.*6.0+e2_3_1.*e3_3_2.*e4_3_2.*2.0+e2_3_2.*e3_3_1.*e4_3_2.*2.0+e2_3_2.*e3_3_2.*e4_3_1.*2.0+e2_3_1.*e3_3_3.*e4_3_3.*2.0+e2_3_3.*e3_3_1.*e4_3_3.*2.0+e2_3_3.*e3_3_3.*e4_3_1.*2.0;
et37 = e1_1_1.*e2_1_1.*e3_1_2.*2.0+e1_1_1.*e2_1_2.*e3_1_1.*2.0+e1_1_2.*e2_1_1.*e3_1_1.*2.0+e1_1_2.*e2_1_2.*e3_1_2.*6.0+e1_1_2.*e2_1_3.*e3_1_3.*2.0+e1_1_3.*e2_1_2.*e3_1_3.*2.0+e1_1_3.*e2_1_3.*e3_1_2.*2.0+e1_1_1.*e2_2_1.*e3_2_2.*2.0+e1_1_1.*e2_2_2.*e3_2_1.*2.0-e1_1_2.*e2_2_1.*e3_2_1.*2.0+e1_2_1.*e2_1_1.*e3_2_2.*2.0-e1_2_1.*e2_1_2.*e3_2_1.*2.0-e1_2_1.*e2_2_1.*e3_1_2.*2.0+e1_2_1.*e2_2_2.*e3_1_1.*2.0+e1_2_2.*e2_1_1.*e3_2_1.*2.0+e1_2_2.*e2_2_1.*e3_1_1.*2.0+e1_1_2.*e2_2_2.*e3_2_2.*2.0+e1_2_2.*e2_1_2.*e3_2_2.*2.0+e1_2_2.*e2_2_2.*e3_1_2.*2.0-e1_1_2.*e2_2_3.*e3_2_3.*2.0+e1_1_3.*e2_2_2.*e3_2_3.*2.0+e1_1_3.*e2_2_3.*e3_2_2.*2.0+e1_2_2.*e2_1_3.*e3_2_3.*2.0+e1_2_2.*e2_2_3.*e3_1_3.*2.0-e1_2_3.*e2_1_2.*e3_2_3.*2.0+e1_2_3.*e2_1_3.*e3_2_2.*2.0+e1_2_3.*e2_2_2.*e3_1_3.*2.0-e1_2_3.*e2_2_3.*e3_1_2.*2.0+e1_1_1.*e2_3_1.*e3_3_2.*2.0+e1_1_1.*e2_3_2.*e3_3_1.*2.0-e1_1_2.*e2_3_1.*e3_3_1.*2.0+e1_3_1.*e2_1_1.*e3_3_2.*2.0-e1_3_1.*e2_1_2.*e3_3_1.*2.0-e1_3_1.*e2_3_1.*e3_1_2.*2.0+e1_3_1.*e2_3_2.*e3_1_1.*2.0+e1_3_2.*e2_1_1.*e3_3_1.*2.0+e1_3_2.*e2_3_1.*e3_1_1.*2.0+e1_1_2.*e2_3_2.*e3_3_2.*2.0;
et38 = e1_3_2.*e2_1_2.*e3_3_2.*2.0+e1_3_2.*e2_3_2.*e3_1_2.*2.0-e1_1_2.*e2_3_3.*e3_3_3.*2.0+e1_1_3.*e2_3_2.*e3_3_3.*2.0+e1_1_3.*e2_3_3.*e3_3_2.*2.0+e1_3_2.*e2_1_3.*e3_3_3.*2.0+e1_3_2.*e2_3_3.*e3_1_3.*2.0-e1_3_3.*e2_1_2.*e3_3_3.*2.0+e1_3_3.*e2_1_3.*e3_3_2.*2.0+e1_3_3.*e2_3_2.*e3_1_3.*2.0-e1_3_3.*e2_3_3.*e3_1_2.*2.0;
et39 = e1_1_1.*e3_1_1.*e4_1_2.*2.0+e1_1_1.*e3_1_2.*e4_1_1.*2.0+e1_1_2.*e3_1_1.*e4_1_1.*2.0+e1_1_2.*e3_1_2.*e4_1_2.*6.0+e1_1_2.*e3_1_3.*e4_1_3.*2.0+e1_1_3.*e3_1_2.*e4_1_3.*2.0+e1_1_3.*e3_1_3.*e4_1_2.*2.0+e1_1_1.*e3_2_1.*e4_2_2.*2.0+e1_1_1.*e3_2_2.*e4_2_1.*2.0-e1_1_2.*e3_2_1.*e4_2_1.*2.0+e1_2_1.*e3_1_1.*e4_2_2.*2.0-e1_2_1.*e3_1_2.*e4_2_1.*2.0-e1_2_1.*e3_2_1.*e4_1_2.*2.0+e1_2_1.*e3_2_2.*e4_1_1.*2.0+e1_2_2.*e3_1_1.*e4_2_1.*2.0+e1_2_2.*e3_2_1.*e4_1_1.*2.0+e1_1_2.*e3_2_2.*e4_2_2.*2.0+e1_2_2.*e3_1_2.*e4_2_2.*2.0+e1_2_2.*e3_2_2.*e4_1_2.*2.0-e1_1_2.*e3_2_3.*e4_2_3.*2.0+e1_1_3.*e3_2_2.*e4_2_3.*2.0+e1_1_3.*e3_2_3.*e4_2_2.*2.0+e1_2_2.*e3_1_3.*e4_2_3.*2.0+e1_2_2.*e3_2_3.*e4_1_3.*2.0-e1_2_3.*e3_1_2.*e4_2_3.*2.0+e1_2_3.*e3_1_3.*e4_2_2.*2.0+e1_2_3.*e3_2_2.*e4_1_3.*2.0-e1_2_3.*e3_2_3.*e4_1_2.*2.0+e1_1_1.*e3_3_1.*e4_3_2.*2.0+e1_1_1.*e3_3_2.*e4_3_1.*2.0-e1_1_2.*e3_3_1.*e4_3_1.*2.0+e1_3_1.*e3_1_1.*e4_3_2.*2.0-e1_3_1.*e3_1_2.*e4_3_1.*2.0-e1_3_1.*e3_3_1.*e4_1_2.*2.0+e1_3_1.*e3_3_2.*e4_1_1.*2.0+e1_3_2.*e3_1_1.*e4_3_1.*2.0+e1_3_2.*e3_3_1.*e4_1_1.*2.0+e1_1_2.*e3_3_2.*e4_3_2.*2.0;
et40 = e1_3_2.*e3_1_2.*e4_3_2.*2.0+e1_3_2.*e3_3_2.*e4_1_2.*2.0-e1_1_2.*e3_3_3.*e4_3_3.*2.0+e1_1_3.*e3_3_2.*e4_3_3.*2.0+e1_1_3.*e3_3_3.*e4_3_2.*2.0+e1_3_2.*e3_1_3.*e4_3_3.*2.0+e1_3_2.*e3_3_3.*e4_1_3.*2.0-e1_3_3.*e3_1_2.*e4_3_3.*2.0+e1_3_3.*e3_1_3.*e4_3_2.*2.0+e1_3_3.*e3_3_2.*e4_1_3.*2.0-e1_3_3.*e3_3_3.*e4_1_2.*2.0;
et41 = e2_1_1.*e3_1_1.*e4_1_2.*2.0+e2_1_1.*e3_1_2.*e4_1_1.*2.0+e2_1_2.*e3_1_1.*e4_1_1.*2.0+e2_1_2.*e3_1_2.*e4_1_2.*6.0+e2_1_2.*e3_1_3.*e4_1_3.*2.0+e2_1_3.*e3_1_2.*e4_1_3.*2.0+e2_1_3.*e3_1_3.*e4_1_2.*2.0+e2_1_1.*e3_2_1.*e4_2_2.*2.0+e2_1_1.*e3_2_2.*e4_2_1.*2.0-e2_1_2.*e3_2_1.*e4_2_1.*2.0+e2_2_1.*e3_1_1.*e4_2_2.*2.0-e2_2_1.*e3_1_2.*e4_2_1.*2.0-e2_2_1.*e3_2_1.*e4_1_2.*2.0+e2_2_1.*e3_2_2.*e4_1_1.*2.0+e2_2_2.*e3_1_1.*e4_2_1.*2.0+e2_2_2.*e3_2_1.*e4_1_1.*2.0+e2_1_2.*e3_2_2.*e4_2_2.*2.0+e2_2_2.*e3_1_2.*e4_2_2.*2.0+e2_2_2.*e3_2_2.*e4_1_2.*2.0-e2_1_2.*e3_2_3.*e4_2_3.*2.0+e2_1_3.*e3_2_2.*e4_2_3.*2.0+e2_1_3.*e3_2_3.*e4_2_2.*2.0+e2_2_2.*e3_1_3.*e4_2_3.*2.0+e2_2_2.*e3_2_3.*e4_1_3.*2.0-e2_2_3.*e3_1_2.*e4_2_3.*2.0+e2_2_3.*e3_1_3.*e4_2_2.*2.0+e2_2_3.*e3_2_2.*e4_1_3.*2.0-e2_2_3.*e3_2_3.*e4_1_2.*2.0+e2_1_1.*e3_3_1.*e4_3_2.*2.0+e2_1_1.*e3_3_2.*e4_3_1.*2.0-e2_1_2.*e3_3_1.*e4_3_1.*2.0+e2_3_1.*e3_1_1.*e4_3_2.*2.0-e2_3_1.*e3_1_2.*e4_3_1.*2.0-e2_3_1.*e3_3_1.*e4_1_2.*2.0+e2_3_1.*e3_3_2.*e4_1_1.*2.0+e2_3_2.*e3_1_1.*e4_3_1.*2.0+e2_3_2.*e3_3_1.*e4_1_1.*2.0+e2_1_2.*e3_3_2.*e4_3_2.*2.0;
et42 = e2_3_2.*e3_1_2.*e4_3_2.*2.0+e2_3_2.*e3_3_2.*e4_1_2.*2.0-e2_1_2.*e3_3_3.*e4_3_3.*2.0+e2_1_3.*e3_3_2.*e4_3_3.*2.0+e2_1_3.*e3_3_3.*e4_3_2.*2.0+e2_3_2.*e3_1_3.*e4_3_3.*2.0+e2_3_2.*e3_3_3.*e4_1_3.*2.0-e2_3_3.*e3_1_2.*e4_3_3.*2.0+e2_3_3.*e3_1_3.*e4_3_2.*2.0+e2_3_3.*e3_3_2.*e4_1_3.*2.0-e2_3_3.*e3_3_3.*e4_1_2.*2.0;
et43 = e1_1_1.*e2_1_2.*e3_2_1.*2.0-e1_1_1.*e2_1_1.*e3_2_2.*2.0+e1_1_1.*e2_2_1.*e3_1_2.*2.0-e1_1_1.*e2_2_2.*e3_1_1.*2.0+e1_1_2.*e2_1_1.*e3_2_1.*2.0+e1_1_2.*e2_2_1.*e3_1_1.*2.0+e1_2_1.*e2_1_1.*e3_1_2.*2.0+e1_2_1.*e2_1_2.*e3_1_1.*2.0-e1_2_2.*e2_1_1.*e3_1_1.*2.0+e1_1_2.*e2_1_2.*e3_2_2.*2.0+e1_1_2.*e2_2_2.*e3_1_2.*2.0+e1_2_2.*e2_1_2.*e3_1_2.*2.0+e1_1_2.*e2_1_3.*e3_2_3.*2.0+e1_1_2.*e2_2_3.*e3_1_3.*2.0+e1_1_3.*e2_1_2.*e3_2_3.*2.0-e1_1_3.*e2_1_3.*e3_2_2.*2.0-e1_1_3.*e2_2_2.*e3_1_3.*2.0+e1_1_3.*e2_2_3.*e3_1_2.*2.0-e1_2_2.*e2_1_3.*e3_1_3.*2.0+e1_2_3.*e2_1_2.*e3_1_3.*2.0+e1_2_3.*e2_1_3.*e3_1_2.*2.0+e1_2_1.*e2_2_1.*e3_2_2.*2.0+e1_2_1.*e2_2_2.*e3_2_1.*2.0+e1_2_2.*e2_2_1.*e3_2_1.*2.0+e1_2_2.*e2_2_2.*e3_2_2.*6.0+e1_2_2.*e2_2_3.*e3_2_3.*2.0+e1_2_3.*e2_2_2.*e3_2_3.*2.0+e1_2_3.*e2_2_3.*e3_2_2.*2.0+e1_2_1.*e2_3_1.*e3_3_2.*2.0+e1_2_1.*e2_3_2.*e3_3_1.*2.0-e1_2_2.*e2_3_1.*e3_3_1.*2.0+e1_3_1.*e2_2_1.*e3_3_2.*2.0-e1_3_1.*e2_2_2.*e3_3_1.*2.0-e1_3_1.*e2_3_1.*e3_2_2.*2.0+e1_3_1.*e2_3_2.*e3_2_1.*2.0+e1_3_2.*e2_2_1.*e3_3_1.*2.0+e1_3_2.*e2_3_1.*e3_2_1.*2.0+e1_2_2.*e2_3_2.*e3_3_2.*2.0;
et44 = e1_3_2.*e2_2_2.*e3_3_2.*2.0+e1_3_2.*e2_3_2.*e3_2_2.*2.0-e1_2_2.*e2_3_3.*e3_3_3.*2.0+e1_2_3.*e2_3_2.*e3_3_3.*2.0+e1_2_3.*e2_3_3.*e3_3_2.*2.0+e1_3_2.*e2_2_3.*e3_3_3.*2.0+e1_3_2.*e2_3_3.*e3_2_3.*2.0-e1_3_3.*e2_2_2.*e3_3_3.*2.0+e1_3_3.*e2_2_3.*e3_3_2.*2.0+e1_3_3.*e2_3_2.*e3_2_3.*2.0-e1_3_3.*e2_3_3.*e3_2_2.*2.0;
et45 = e1_1_1.*e3_1_2.*e4_2_1.*2.0-e1_1_1.*e3_1_1.*e4_2_2.*2.0+e1_1_1.*e3_2_1.*e4_1_2.*2.0-e1_1_1.*e3_2_2.*e4_1_1.*2.0+e1_1_2.*e3_1_1.*e4_2_1.*2.0+e1_1_2.*e3_2_1.*e4_1_1.*2.0+e1_2_1.*e3_1_1.*e4_1_2.*2.0+e1_2_1.*e3_1_2.*e4_1_1.*2.0-e1_2_2.*e3_1_1.*e4_1_1.*2.0+e1_1_2.*e3_1_2.*e4_2_2.*2.0+e1_1_2.*e3_2_2.*e4_1_2.*2.0+e1_2_2.*e3_1_2.*e4_1_2.*2.0+e1_1_2.*e3_1_3.*e4_2_3.*2.0+e1_1_2.*e3_2_3.*e4_1_3.*2.0+e1_1_3.*e3_1_2.*e4_2_3.*2.0-e1_1_3.*e3_1_3.*e4_2_2.*2.0-e1_1_3.*e3_2_2.*e4_1_3.*2.0+e1_1_3.*e3_2_3.*e4_1_2.*2.0-e1_2_2.*e3_1_3.*e4_1_3.*2.0+e1_2_3.*e3_1_2.*e4_1_3.*2.0+e1_2_3.*e3_1_3.*e4_1_2.*2.0+e1_2_1.*e3_2_1.*e4_2_2.*2.0+e1_2_1.*e3_2_2.*e4_2_1.*2.0+e1_2_2.*e3_2_1.*e4_2_1.*2.0+e1_2_2.*e3_2_2.*e4_2_2.*6.0+e1_2_2.*e3_2_3.*e4_2_3.*2.0+e1_2_3.*e3_2_2.*e4_2_3.*2.0+e1_2_3.*e3_2_3.*e4_2_2.*2.0+e1_2_1.*e3_3_1.*e4_3_2.*2.0+e1_2_1.*e3_3_2.*e4_3_1.*2.0-e1_2_2.*e3_3_1.*e4_3_1.*2.0+e1_3_1.*e3_2_1.*e4_3_2.*2.0-e1_3_1.*e3_2_2.*e4_3_1.*2.0-e1_3_1.*e3_3_1.*e4_2_2.*2.0+e1_3_1.*e3_3_2.*e4_2_1.*2.0+e1_3_2.*e3_2_1.*e4_3_1.*2.0+e1_3_2.*e3_3_1.*e4_2_1.*2.0+e1_2_2.*e3_3_2.*e4_3_2.*2.0;
et46 = e1_3_2.*e3_2_2.*e4_3_2.*2.0+e1_3_2.*e3_3_2.*e4_2_2.*2.0-e1_2_2.*e3_3_3.*e4_3_3.*2.0+e1_2_3.*e3_3_2.*e4_3_3.*2.0+e1_2_3.*e3_3_3.*e4_3_2.*2.0+e1_3_2.*e3_2_3.*e4_3_3.*2.0+e1_3_2.*e3_3_3.*e4_2_3.*2.0-e1_3_3.*e3_2_2.*e4_3_3.*2.0+e1_3_3.*e3_2_3.*e4_3_2.*2.0+e1_3_3.*e3_3_2.*e4_2_3.*2.0-e1_3_3.*e3_3_3.*e4_2_2.*2.0;
et47 = e2_1_1.*e3_1_2.*e4_2_1.*2.0-e2_1_1.*e3_1_1.*e4_2_2.*2.0+e2_1_1.*e3_2_1.*e4_1_2.*2.0-e2_1_1.*e3_2_2.*e4_1_1.*2.0+e2_1_2.*e3_1_1.*e4_2_1.*2.0+e2_1_2.*e3_2_1.*e4_1_1.*2.0+e2_2_1.*e3_1_1.*e4_1_2.*2.0+e2_2_1.*e3_1_2.*e4_1_1.*2.0-e2_2_2.*e3_1_1.*e4_1_1.*2.0+e2_1_2.*e3_1_2.*e4_2_2.*2.0+e2_1_2.*e3_2_2.*e4_1_2.*2.0+e2_2_2.*e3_1_2.*e4_1_2.*2.0+e2_1_2.*e3_1_3.*e4_2_3.*2.0+e2_1_2.*e3_2_3.*e4_1_3.*2.0+e2_1_3.*e3_1_2.*e4_2_3.*2.0-e2_1_3.*e3_1_3.*e4_2_2.*2.0-e2_1_3.*e3_2_2.*e4_1_3.*2.0+e2_1_3.*e3_2_3.*e4_1_2.*2.0-e2_2_2.*e3_1_3.*e4_1_3.*2.0+e2_2_3.*e3_1_2.*e4_1_3.*2.0+e2_2_3.*e3_1_3.*e4_1_2.*2.0+e2_2_1.*e3_2_1.*e4_2_2.*2.0+e2_2_1.*e3_2_2.*e4_2_1.*2.0+e2_2_2.*e3_2_1.*e4_2_1.*2.0+e2_2_2.*e3_2_2.*e4_2_2.*6.0+e2_2_2.*e3_2_3.*e4_2_3.*2.0+e2_2_3.*e3_2_2.*e4_2_3.*2.0+e2_2_3.*e3_2_3.*e4_2_2.*2.0+e2_2_1.*e3_3_1.*e4_3_2.*2.0+e2_2_1.*e3_3_2.*e4_3_1.*2.0-e2_2_2.*e3_3_1.*e4_3_1.*2.0+e2_3_1.*e3_2_1.*e4_3_2.*2.0-e2_3_1.*e3_2_2.*e4_3_1.*2.0-e2_3_1.*e3_3_1.*e4_2_2.*2.0+e2_3_1.*e3_3_2.*e4_2_1.*2.0+e2_3_2.*e3_2_1.*e4_3_1.*2.0+e2_3_2.*e3_3_1.*e4_2_1.*2.0+e2_2_2.*e3_3_2.*e4_3_2.*2.0;
et48 = e2_3_2.*e3_2_2.*e4_3_2.*2.0+e2_3_2.*e3_3_2.*e4_2_2.*2.0-e2_2_2.*e3_3_3.*e4_3_3.*2.0+e2_2_3.*e3_3_2.*e4_3_3.*2.0+e2_2_3.*e3_3_3.*e4_3_2.*2.0+e2_3_2.*e3_2_3.*e4_3_3.*2.0+e2_3_2.*e3_3_3.*e4_2_3.*2.0-e2_3_3.*e3_2_2.*e4_3_3.*2.0+e2_3_3.*e3_2_3.*e4_3_2.*2.0+e2_3_3.*e3_3_2.*e4_2_3.*2.0-e2_3_3.*e3_3_3.*e4_2_2.*2.0;
et49 = e1_1_1.*e2_1_2.*e3_3_1.*2.0-e1_1_1.*e2_1_1.*e3_3_2.*2.0+e1_1_1.*e2_3_1.*e3_1_2.*2.0-e1_1_1.*e2_3_2.*e3_1_1.*2.0+e1_1_2.*e2_1_1.*e3_3_1.*2.0+e1_1_2.*e2_3_1.*e3_1_1.*2.0+e1_3_1.*e2_1_1.*e3_1_2.*2.0+e1_3_1.*e2_1_2.*e3_1_1.*2.0-e1_3_2.*e2_1_1.*e3_1_1.*2.0+e1_1_2.*e2_1_2.*e3_3_2.*2.0+e1_1_2.*e2_3_2.*e3_1_2.*2.0+e1_3_2.*e2_1_2.*e3_1_2.*2.0+e1_1_2.*e2_1_3.*e3_3_3.*2.0+e1_1_2.*e2_3_3.*e3_1_3.*2.0+e1_1_3.*e2_1_2.*e3_3_3.*2.0-e1_1_3.*e2_1_3.*e3_3_2.*2.0-e1_1_3.*e2_3_2.*e3_1_3.*2.0+e1_1_3.*e2_3_3.*e3_1_2.*2.0-e1_3_2.*e2_1_3.*e3_1_3.*2.0+e1_3_3.*e2_1_2.*e3_1_3.*2.0+e1_3_3.*e2_1_3.*e3_1_2.*2.0-e1_2_1.*e2_2_1.*e3_3_2.*2.0+e1_2_1.*e2_2_2.*e3_3_1.*2.0+e1_2_1.*e2_3_1.*e3_2_2.*2.0-e1_2_1.*e2_3_2.*e3_2_1.*2.0+e1_2_2.*e2_2_1.*e3_3_1.*2.0+e1_2_2.*e2_3_1.*e3_2_1.*2.0+e1_3_1.*e2_2_1.*e3_2_2.*2.0+e1_3_1.*e2_2_2.*e3_2_1.*2.0-e1_3_2.*e2_2_1.*e3_2_1.*2.0+e1_2_2.*e2_2_2.*e3_3_2.*2.0+e1_2_2.*e2_3_2.*e3_2_2.*2.0+e1_3_2.*e2_2_2.*e3_2_2.*2.0+e1_2_2.*e2_2_3.*e3_3_3.*2.0+e1_2_2.*e2_3_3.*e3_2_3.*2.0+e1_2_3.*e2_2_2.*e3_3_3.*2.0-e1_2_3.*e2_2_3.*e3_3_2.*2.0;
et50 = e1_2_3.*e2_3_2.*e3_2_3.*-2.0+e1_2_3.*e2_3_3.*e3_2_2.*2.0-e1_3_2.*e2_2_3.*e3_2_3.*2.0+e1_3_3.*e2_2_2.*e3_2_3.*2.0+e1_3_3.*e2_2_3.*e3_2_2.*2.0+e1_3_1.*e2_3_1.*e3_3_2.*2.0+e1_3_1.*e2_3_2.*e3_3_1.*2.0+e1_3_2.*e2_3_1.*e3_3_1.*2.0+e1_3_2.*e2_3_2.*e3_3_2.*6.0+e1_3_2.*e2_3_3.*e3_3_3.*2.0+e1_3_3.*e2_3_2.*e3_3_3.*2.0+e1_3_3.*e2_3_3.*e3_3_2.*2.0;
et51 = e1_1_1.*e3_1_2.*e4_3_1.*2.0-e1_1_1.*e3_1_1.*e4_3_2.*2.0+e1_1_1.*e3_3_1.*e4_1_2.*2.0-e1_1_1.*e3_3_2.*e4_1_1.*2.0+e1_1_2.*e3_1_1.*e4_3_1.*2.0+e1_1_2.*e3_3_1.*e4_1_1.*2.0+e1_3_1.*e3_1_1.*e4_1_2.*2.0+e1_3_1.*e3_1_2.*e4_1_1.*2.0-e1_3_2.*e3_1_1.*e4_1_1.*2.0+e1_1_2.*e3_1_2.*e4_3_2.*2.0+e1_1_2.*e3_3_2.*e4_1_2.*2.0+e1_3_2.*e3_1_2.*e4_1_2.*2.0+e1_1_2.*e3_1_3.*e4_3_3.*2.0+e1_1_2.*e3_3_3.*e4_1_3.*2.0+e1_1_3.*e3_1_2.*e4_3_3.*2.0-e1_1_3.*e3_1_3.*e4_3_2.*2.0-e1_1_3.*e3_3_2.*e4_1_3.*2.0+e1_1_3.*e3_3_3.*e4_1_2.*2.0-e1_3_2.*e3_1_3.*e4_1_3.*2.0+e1_3_3.*e3_1_2.*e4_1_3.*2.0+e1_3_3.*e3_1_3.*e4_1_2.*2.0-e1_2_1.*e3_2_1.*e4_3_2.*2.0+e1_2_1.*e3_2_2.*e4_3_1.*2.0+e1_2_1.*e3_3_1.*e4_2_2.*2.0-e1_2_1.*e3_3_2.*e4_2_1.*2.0+e1_2_2.*e3_2_1.*e4_3_1.*2.0+e1_2_2.*e3_3_1.*e4_2_1.*2.0+e1_3_1.*e3_2_1.*e4_2_2.*2.0+e1_3_1.*e3_2_2.*e4_2_1.*2.0-e1_3_2.*e3_2_1.*e4_2_1.*2.0+e1_2_2.*e3_2_2.*e4_3_2.*2.0+e1_2_2.*e3_3_2.*e4_2_2.*2.0+e1_3_2.*e3_2_2.*e4_2_2.*2.0+e1_2_2.*e3_2_3.*e4_3_3.*2.0+e1_2_2.*e3_3_3.*e4_2_3.*2.0+e1_2_3.*e3_2_2.*e4_3_3.*2.0-e1_2_3.*e3_2_3.*e4_3_2.*2.0;
et52 = e1_2_3.*e3_3_2.*e4_2_3.*-2.0+e1_2_3.*e3_3_3.*e4_2_2.*2.0-e1_3_2.*e3_2_3.*e4_2_3.*2.0+e1_3_3.*e3_2_2.*e4_2_3.*2.0+e1_3_3.*e3_2_3.*e4_2_2.*2.0+e1_3_1.*e3_3_1.*e4_3_2.*2.0+e1_3_1.*e3_3_2.*e4_3_1.*2.0+e1_3_2.*e3_3_1.*e4_3_1.*2.0+e1_3_2.*e3_3_2.*e4_3_2.*6.0+e1_3_2.*e3_3_3.*e4_3_3.*2.0+e1_3_3.*e3_3_2.*e4_3_3.*2.0+e1_3_3.*e3_3_3.*e4_3_2.*2.0;
et53 = e2_1_1.*e3_1_2.*e4_3_1.*2.0-e2_1_1.*e3_1_1.*e4_3_2.*2.0+e2_1_1.*e3_3_1.*e4_1_2.*2.0-e2_1_1.*e3_3_2.*e4_1_1.*2.0+e2_1_2.*e3_1_1.*e4_3_1.*2.0+e2_1_2.*e3_3_1.*e4_1_1.*2.0+e2_3_1.*e3_1_1.*e4_1_2.*2.0+e2_3_1.*e3_1_2.*e4_1_1.*2.0-e2_3_2.*e3_1_1.*e4_1_1.*2.0+e2_1_2.*e3_1_2.*e4_3_2.*2.0+e2_1_2.*e3_3_2.*e4_1_2.*2.0+e2_3_2.*e3_1_2.*e4_1_2.*2.0+e2_1_2.*e3_1_3.*e4_3_3.*2.0+e2_1_2.*e3_3_3.*e4_1_3.*2.0+e2_1_3.*e3_1_2.*e4_3_3.*2.0-e2_1_3.*e3_1_3.*e4_3_2.*2.0-e2_1_3.*e3_3_2.*e4_1_3.*2.0+e2_1_3.*e3_3_3.*e4_1_2.*2.0-e2_3_2.*e3_1_3.*e4_1_3.*2.0+e2_3_3.*e3_1_2.*e4_1_3.*2.0+e2_3_3.*e3_1_3.*e4_1_2.*2.0-e2_2_1.*e3_2_1.*e4_3_2.*2.0+e2_2_1.*e3_2_2.*e4_3_1.*2.0+e2_2_1.*e3_3_1.*e4_2_2.*2.0-e2_2_1.*e3_3_2.*e4_2_1.*2.0+e2_2_2.*e3_2_1.*e4_3_1.*2.0+e2_2_2.*e3_3_1.*e4_2_1.*2.0+e2_3_1.*e3_2_1.*e4_2_2.*2.0+e2_3_1.*e3_2_2.*e4_2_1.*2.0-e2_3_2.*e3_2_1.*e4_2_1.*2.0+e2_2_2.*e3_2_2.*e4_3_2.*2.0+e2_2_2.*e3_3_2.*e4_2_2.*2.0+e2_3_2.*e3_2_2.*e4_2_2.*2.0+e2_2_2.*e3_2_3.*e4_3_3.*2.0+e2_2_2.*e3_3_3.*e4_2_3.*2.0+e2_2_3.*e3_2_2.*e4_3_3.*2.0-e2_2_3.*e3_2_3.*e4_3_2.*2.0;
et54 = e2_2_3.*e3_3_2.*e4_2_3.*-2.0+e2_2_3.*e3_3_3.*e4_2_2.*2.0-e2_3_2.*e3_2_3.*e4_2_3.*2.0+e2_3_3.*e3_2_2.*e4_2_3.*2.0+e2_3_3.*e3_2_3.*e4_2_2.*2.0+e2_3_1.*e3_3_1.*e4_3_2.*2.0+e2_3_1.*e3_3_2.*e4_3_1.*2.0+e2_3_2.*e3_3_1.*e4_3_1.*2.0+e2_3_2.*e3_3_2.*e4_3_2.*6.0+e2_3_2.*e3_3_3.*e4_3_3.*2.0+e2_3_3.*e3_3_2.*e4_3_3.*2.0+e2_3_3.*e3_3_3.*e4_3_2.*2.0;
et55 = e1_1_1.*e2_1_1.*e3_1_3.*2.0+e1_1_1.*e2_1_3.*e3_1_1.*2.0+e1_1_3.*e2_1_1.*e3_1_1.*2.0+e1_1_2.*e2_1_2.*e3_1_3.*2.0+e1_1_2.*e2_1_3.*e3_1_2.*2.0+e1_1_3.*e2_1_2.*e3_1_2.*2.0+e1_1_3.*e2_1_3.*e3_1_3.*6.0+e1_1_1.*e2_2_1.*e3_2_3.*2.0+e1_1_1.*e2_2_3.*e3_2_1.*2.0-e1_1_3.*e2_2_1.*e3_2_1.*2.0+e1_2_1.*e2_1_1.*e3_2_3.*2.0-e1_2_1.*e2_1_3.*e3_2_1.*2.0-e1_2_1.*e2_2_1.*e3_1_3.*2.0+e1_2_1.*e2_2_3.*e3_1_1.*2.0+e1_2_3.*e2_1_1.*e3_2_1.*2.0+e1_2_3.*e2_2_1.*e3_1_1.*2.0+e1_1_2.*e2_2_2.*e3_2_3.*2.0+e1_1_2.*e2_2_3.*e3_2_2.*2.0-e1_1_3.*e2_2_2.*e3_2_2.*2.0+e1_2_2.*e2_1_2.*e3_2_3.*2.0-e1_2_2.*e2_1_3.*e3_2_2.*2.0-e1_2_2.*e2_2_2.*e3_1_3.*2.0+e1_2_2.*e2_2_3.*e3_1_2.*2.0+e1_2_3.*e2_1_2.*e3_2_2.*2.0+e1_2_3.*e2_2_2.*e3_1_2.*2.0+e1_1_3.*e2_2_3.*e3_2_3.*2.0+e1_2_3.*e2_1_3.*e3_2_3.*2.0+e1_2_3.*e2_2_3.*e3_1_3.*2.0+e1_1_1.*e2_3_1.*e3_3_3.*2.0+e1_1_1.*e2_3_3.*e3_3_1.*2.0-e1_1_3.*e2_3_1.*e3_3_1.*2.0+e1_3_1.*e2_1_1.*e3_3_3.*2.0-e1_3_1.*e2_1_3.*e3_3_1.*2.0-e1_3_1.*e2_3_1.*e3_1_3.*2.0+e1_3_1.*e2_3_3.*e3_1_1.*2.0+e1_3_3.*e2_1_1.*e3_3_1.*2.0+e1_3_3.*e2_3_1.*e3_1_1.*2.0+e1_1_2.*e2_3_2.*e3_3_3.*2.0;
et56 = e1_1_2.*e2_3_3.*e3_3_2.*2.0-e1_1_3.*e2_3_2.*e3_3_2.*2.0+e1_3_2.*e2_1_2.*e3_3_3.*2.0-e1_3_2.*e2_1_3.*e3_3_2.*2.0-e1_3_2.*e2_3_2.*e3_1_3.*2.0+e1_3_2.*e2_3_3.*e3_1_2.*2.0+e1_3_3.*e2_1_2.*e3_3_2.*2.0+e1_3_3.*e2_3_2.*e3_1_2.*2.0+e1_1_3.*e2_3_3.*e3_3_3.*2.0+e1_3_3.*e2_1_3.*e3_3_3.*2.0+e1_3_3.*e2_3_3.*e3_1_3.*2.0;
et57 = e1_1_1.*e3_1_1.*e4_1_3.*2.0+e1_1_1.*e3_1_3.*e4_1_1.*2.0+e1_1_3.*e3_1_1.*e4_1_1.*2.0+e1_1_2.*e3_1_2.*e4_1_3.*2.0+e1_1_2.*e3_1_3.*e4_1_2.*2.0+e1_1_3.*e3_1_2.*e4_1_2.*2.0+e1_1_3.*e3_1_3.*e4_1_3.*6.0+e1_1_1.*e3_2_1.*e4_2_3.*2.0+e1_1_1.*e3_2_3.*e4_2_1.*2.0-e1_1_3.*e3_2_1.*e4_2_1.*2.0+e1_2_1.*e3_1_1.*e4_2_3.*2.0-e1_2_1.*e3_1_3.*e4_2_1.*2.0-e1_2_1.*e3_2_1.*e4_1_3.*2.0+e1_2_1.*e3_2_3.*e4_1_1.*2.0+e1_2_3.*e3_1_1.*e4_2_1.*2.0+e1_2_3.*e3_2_1.*e4_1_1.*2.0+e1_1_2.*e3_2_2.*e4_2_3.*2.0+e1_1_2.*e3_2_3.*e4_2_2.*2.0-e1_1_3.*e3_2_2.*e4_2_2.*2.0+e1_2_2.*e3_1_2.*e4_2_3.*2.0-e1_2_2.*e3_1_3.*e4_2_2.*2.0-e1_2_2.*e3_2_2.*e4_1_3.*2.0+e1_2_2.*e3_2_3.*e4_1_2.*2.0+e1_2_3.*e3_1_2.*e4_2_2.*2.0+e1_2_3.*e3_2_2.*e4_1_2.*2.0+e1_1_3.*e3_2_3.*e4_2_3.*2.0+e1_2_3.*e3_1_3.*e4_2_3.*2.0+e1_2_3.*e3_2_3.*e4_1_3.*2.0+e1_1_1.*e3_3_1.*e4_3_3.*2.0+e1_1_1.*e3_3_3.*e4_3_1.*2.0-e1_1_3.*e3_3_1.*e4_3_1.*2.0+e1_3_1.*e3_1_1.*e4_3_3.*2.0-e1_3_1.*e3_1_3.*e4_3_1.*2.0-e1_3_1.*e3_3_1.*e4_1_3.*2.0+e1_3_1.*e3_3_3.*e4_1_1.*2.0+e1_3_3.*e3_1_1.*e4_3_1.*2.0+e1_3_3.*e3_3_1.*e4_1_1.*2.0+e1_1_2.*e3_3_2.*e4_3_3.*2.0;
et58 = e1_1_2.*e3_3_3.*e4_3_2.*2.0-e1_1_3.*e3_3_2.*e4_3_2.*2.0+e1_3_2.*e3_1_2.*e4_3_3.*2.0-e1_3_2.*e3_1_3.*e4_3_2.*2.0-e1_3_2.*e3_3_2.*e4_1_3.*2.0+e1_3_2.*e3_3_3.*e4_1_2.*2.0+e1_3_3.*e3_1_2.*e4_3_2.*2.0+e1_3_3.*e3_3_2.*e4_1_2.*2.0+e1_1_3.*e3_3_3.*e4_3_3.*2.0+e1_3_3.*e3_1_3.*e4_3_3.*2.0+e1_3_3.*e3_3_3.*e4_1_3.*2.0;
et59 = e2_1_1.*e3_1_1.*e4_1_3.*2.0+e2_1_1.*e3_1_3.*e4_1_1.*2.0+e2_1_3.*e3_1_1.*e4_1_1.*2.0+e2_1_2.*e3_1_2.*e4_1_3.*2.0+e2_1_2.*e3_1_3.*e4_1_2.*2.0+e2_1_3.*e3_1_2.*e4_1_2.*2.0+e2_1_3.*e3_1_3.*e4_1_3.*6.0+e2_1_1.*e3_2_1.*e4_2_3.*2.0+e2_1_1.*e3_2_3.*e4_2_1.*2.0-e2_1_3.*e3_2_1.*e4_2_1.*2.0+e2_2_1.*e3_1_1.*e4_2_3.*2.0-e2_2_1.*e3_1_3.*e4_2_1.*2.0-e2_2_1.*e3_2_1.*e4_1_3.*2.0+e2_2_1.*e3_2_3.*e4_1_1.*2.0+e2_2_3.*e3_1_1.*e4_2_1.*2.0+e2_2_3.*e3_2_1.*e4_1_1.*2.0+e2_1_2.*e3_2_2.*e4_2_3.*2.0+e2_1_2.*e3_2_3.*e4_2_2.*2.0-e2_1_3.*e3_2_2.*e4_2_2.*2.0+e2_2_2.*e3_1_2.*e4_2_3.*2.0-e2_2_2.*e3_1_3.*e4_2_2.*2.0-e2_2_2.*e3_2_2.*e4_1_3.*2.0+e2_2_2.*e3_2_3.*e4_1_2.*2.0+e2_2_3.*e3_1_2.*e4_2_2.*2.0+e2_2_3.*e3_2_2.*e4_1_2.*2.0+e2_1_3.*e3_2_3.*e4_2_3.*2.0+e2_2_3.*e3_1_3.*e4_2_3.*2.0+e2_2_3.*e3_2_3.*e4_1_3.*2.0+e2_1_1.*e3_3_1.*e4_3_3.*2.0+e2_1_1.*e3_3_3.*e4_3_1.*2.0-e2_1_3.*e3_3_1.*e4_3_1.*2.0+e2_3_1.*e3_1_1.*e4_3_3.*2.0-e2_3_1.*e3_1_3.*e4_3_1.*2.0-e2_3_1.*e3_3_1.*e4_1_3.*2.0+e2_3_1.*e3_3_3.*e4_1_1.*2.0+e2_3_3.*e3_1_1.*e4_3_1.*2.0+e2_3_3.*e3_3_1.*e4_1_1.*2.0+e2_1_2.*e3_3_2.*e4_3_3.*2.0;
et60 = e2_1_2.*e3_3_3.*e4_3_2.*2.0-e2_1_3.*e3_3_2.*e4_3_2.*2.0+e2_3_2.*e3_1_2.*e4_3_3.*2.0-e2_3_2.*e3_1_3.*e4_3_2.*2.0-e2_3_2.*e3_3_2.*e4_1_3.*2.0+e2_3_2.*e3_3_3.*e4_1_2.*2.0+e2_3_3.*e3_1_2.*e4_3_2.*2.0+e2_3_3.*e3_3_2.*e4_1_2.*2.0+e2_1_3.*e3_3_3.*e4_3_3.*2.0+e2_3_3.*e3_1_3.*e4_3_3.*2.0+e2_3_3.*e3_3_3.*e4_1_3.*2.0;
et61 = e1_1_1.*e2_1_3.*e3_2_1.*2.0-e1_1_1.*e2_1_1.*e3_2_3.*2.0+e1_1_1.*e2_2_1.*e3_1_3.*2.0-e1_1_1.*e2_2_3.*e3_1_1.*2.0+e1_1_3.*e2_1_1.*e3_2_1.*2.0+e1_1_3.*e2_2_1.*e3_1_1.*2.0+e1_2_1.*e2_1_1.*e3_1_3.*2.0+e1_2_1.*e2_1_3.*e3_1_1.*2.0-e1_2_3.*e2_1_1.*e3_1_1.*2.0-e1_1_2.*e2_1_2.*e3_2_3.*2.0+e1_1_2.*e2_1_3.*e3_2_2.*2.0+e1_1_2.*e2_2_2.*e3_1_3.*2.0-e1_1_2.*e2_2_3.*e3_1_2.*2.0+e1_1_3.*e2_1_2.*e3_2_2.*2.0+e1_1_3.*e2_2_2.*e3_1_2.*2.0+e1_2_2.*e2_1_2.*e3_1_3.*2.0+e1_2_2.*e2_1_3.*e3_1_2.*2.0-e1_2_3.*e2_1_2.*e3_1_2.*2.0+e1_1_3.*e2_1_3.*e3_2_3.*2.0+e1_1_3.*e2_2_3.*e3_1_3.*2.0+e1_2_3.*e2_1_3.*e3_1_3.*2.0+e1_2_1.*e2_2_1.*e3_2_3.*2.0+e1_2_1.*e2_2_3.*e3_2_1.*2.0+e1_2_3.*e2_2_1.*e3_2_1.*2.0+e1_2_2.*e2_2_2.*e3_2_3.*2.0+e1_2_2.*e2_2_3.*e3_2_2.*2.0+e1_2_3.*e2_2_2.*e3_2_2.*2.0+e1_2_3.*e2_2_3.*e3_2_3.*6.0+e1_2_1.*e2_3_1.*e3_3_3.*2.0+e1_2_1.*e2_3_3.*e3_3_1.*2.0-e1_2_3.*e2_3_1.*e3_3_1.*2.0+e1_3_1.*e2_2_1.*e3_3_3.*2.0-e1_3_1.*e2_2_3.*e3_3_1.*2.0-e1_3_1.*e2_3_1.*e3_2_3.*2.0+e1_3_1.*e2_3_3.*e3_2_1.*2.0+e1_3_3.*e2_2_1.*e3_3_1.*2.0+e1_3_3.*e2_3_1.*e3_2_1.*2.0+e1_2_2.*e2_3_2.*e3_3_3.*2.0;
et62 = e1_2_2.*e2_3_3.*e3_3_2.*2.0-e1_2_3.*e2_3_2.*e3_3_2.*2.0+e1_3_2.*e2_2_2.*e3_3_3.*2.0-e1_3_2.*e2_2_3.*e3_3_2.*2.0-e1_3_2.*e2_3_2.*e3_2_3.*2.0+e1_3_2.*e2_3_3.*e3_2_2.*2.0+e1_3_3.*e2_2_2.*e3_3_2.*2.0+e1_3_3.*e2_3_2.*e3_2_2.*2.0+e1_2_3.*e2_3_3.*e3_3_3.*2.0+e1_3_3.*e2_2_3.*e3_3_3.*2.0+e1_3_3.*e2_3_3.*e3_2_3.*2.0;
et63 = e1_1_1.*e3_1_3.*e4_2_1.*2.0-e1_1_1.*e3_1_1.*e4_2_3.*2.0+e1_1_1.*e3_2_1.*e4_1_3.*2.0-e1_1_1.*e3_2_3.*e4_1_1.*2.0+e1_1_3.*e3_1_1.*e4_2_1.*2.0+e1_1_3.*e3_2_1.*e4_1_1.*2.0+e1_2_1.*e3_1_1.*e4_1_3.*2.0+e1_2_1.*e3_1_3.*e4_1_1.*2.0-e1_2_3.*e3_1_1.*e4_1_1.*2.0-e1_1_2.*e3_1_2.*e4_2_3.*2.0+e1_1_2.*e3_1_3.*e4_2_2.*2.0+e1_1_2.*e3_2_2.*e4_1_3.*2.0-e1_1_2.*e3_2_3.*e4_1_2.*2.0+e1_1_3.*e3_1_2.*e4_2_2.*2.0+e1_1_3.*e3_2_2.*e4_1_2.*2.0+e1_2_2.*e3_1_2.*e4_1_3.*2.0+e1_2_2.*e3_1_3.*e4_1_2.*2.0-e1_2_3.*e3_1_2.*e4_1_2.*2.0+e1_1_3.*e3_1_3.*e4_2_3.*2.0+e1_1_3.*e3_2_3.*e4_1_3.*2.0+e1_2_3.*e3_1_3.*e4_1_3.*2.0+e1_2_1.*e3_2_1.*e4_2_3.*2.0+e1_2_1.*e3_2_3.*e4_2_1.*2.0+e1_2_3.*e3_2_1.*e4_2_1.*2.0+e1_2_2.*e3_2_2.*e4_2_3.*2.0+e1_2_2.*e3_2_3.*e4_2_2.*2.0+e1_2_3.*e3_2_2.*e4_2_2.*2.0+e1_2_3.*e3_2_3.*e4_2_3.*6.0+e1_2_1.*e3_3_1.*e4_3_3.*2.0+e1_2_1.*e3_3_3.*e4_3_1.*2.0-e1_2_3.*e3_3_1.*e4_3_1.*2.0+e1_3_1.*e3_2_1.*e4_3_3.*2.0-e1_3_1.*e3_2_3.*e4_3_1.*2.0-e1_3_1.*e3_3_1.*e4_2_3.*2.0+e1_3_1.*e3_3_3.*e4_2_1.*2.0+e1_3_3.*e3_2_1.*e4_3_1.*2.0+e1_3_3.*e3_3_1.*e4_2_1.*2.0+e1_2_2.*e3_3_2.*e4_3_3.*2.0;
et64 = e1_2_2.*e3_3_3.*e4_3_2.*2.0-e1_2_3.*e3_3_2.*e4_3_2.*2.0+e1_3_2.*e3_2_2.*e4_3_3.*2.0-e1_3_2.*e3_2_3.*e4_3_2.*2.0-e1_3_2.*e3_3_2.*e4_2_3.*2.0+e1_3_2.*e3_3_3.*e4_2_2.*2.0+e1_3_3.*e3_2_2.*e4_3_2.*2.0+e1_3_3.*e3_3_2.*e4_2_2.*2.0+e1_2_3.*e3_3_3.*e4_3_3.*2.0+e1_3_3.*e3_2_3.*e4_3_3.*2.0+e1_3_3.*e3_3_3.*e4_2_3.*2.0;
et65 = e2_1_1.*e3_1_3.*e4_2_1.*2.0-e2_1_1.*e3_1_1.*e4_2_3.*2.0+e2_1_1.*e3_2_1.*e4_1_3.*2.0-e2_1_1.*e3_2_3.*e4_1_1.*2.0+e2_1_3.*e3_1_1.*e4_2_1.*2.0+e2_1_3.*e3_2_1.*e4_1_1.*2.0+e2_2_1.*e3_1_1.*e4_1_3.*2.0+e2_2_1.*e3_1_3.*e4_1_1.*2.0-e2_2_3.*e3_1_1.*e4_1_1.*2.0-e2_1_2.*e3_1_2.*e4_2_3.*2.0+e2_1_2.*e3_1_3.*e4_2_2.*2.0+e2_1_2.*e3_2_2.*e4_1_3.*2.0-e2_1_2.*e3_2_3.*e4_1_2.*2.0+e2_1_3.*e3_1_2.*e4_2_2.*2.0+e2_1_3.*e3_2_2.*e4_1_2.*2.0+e2_2_2.*e3_1_2.*e4_1_3.*2.0+e2_2_2.*e3_1_3.*e4_1_2.*2.0-e2_2_3.*e3_1_2.*e4_1_2.*2.0+e2_1_3.*e3_1_3.*e4_2_3.*2.0+e2_1_3.*e3_2_3.*e4_1_3.*2.0+e2_2_3.*e3_1_3.*e4_1_3.*2.0+e2_2_1.*e3_2_1.*e4_2_3.*2.0+e2_2_1.*e3_2_3.*e4_2_1.*2.0+e2_2_3.*e3_2_1.*e4_2_1.*2.0+e2_2_2.*e3_2_2.*e4_2_3.*2.0+e2_2_2.*e3_2_3.*e4_2_2.*2.0+e2_2_3.*e3_2_2.*e4_2_2.*2.0+e2_2_3.*e3_2_3.*e4_2_3.*6.0+e2_2_1.*e3_3_1.*e4_3_3.*2.0+e2_2_1.*e3_3_3.*e4_3_1.*2.0-e2_2_3.*e3_3_1.*e4_3_1.*2.0+e2_3_1.*e3_2_1.*e4_3_3.*2.0-e2_3_1.*e3_2_3.*e4_3_1.*2.0-e2_3_1.*e3_3_1.*e4_2_3.*2.0+e2_3_1.*e3_3_3.*e4_2_1.*2.0+e2_3_3.*e3_2_1.*e4_3_1.*2.0+e2_3_3.*e3_3_1.*e4_2_1.*2.0+e2_2_2.*e3_3_2.*e4_3_3.*2.0;
et66 = e2_2_2.*e3_3_3.*e4_3_2.*2.0-e2_2_3.*e3_3_2.*e4_3_2.*2.0+e2_3_2.*e3_2_2.*e4_3_3.*2.0-e2_3_2.*e3_2_3.*e4_3_2.*2.0-e2_3_2.*e3_3_2.*e4_2_3.*2.0+e2_3_2.*e3_3_3.*e4_2_2.*2.0+e2_3_3.*e3_2_2.*e4_3_2.*2.0+e2_3_3.*e3_3_2.*e4_2_2.*2.0+e2_2_3.*e3_3_3.*e4_3_3.*2.0+e2_3_3.*e3_2_3.*e4_3_3.*2.0+e2_3_3.*e3_3_3.*e4_2_3.*2.0;
et67 = e1_1_1.*e2_1_3.*e3_3_1.*2.0-e1_1_1.*e2_1_1.*e3_3_3.*2.0+e1_1_1.*e2_3_1.*e3_1_3.*2.0-e1_1_1.*e2_3_3.*e3_1_1.*2.0+e1_1_3.*e2_1_1.*e3_3_1.*2.0+e1_1_3.*e2_3_1.*e3_1_1.*2.0+e1_3_1.*e2_1_1.*e3_1_3.*2.0+e1_3_1.*e2_1_3.*e3_1_1.*2.0-e1_3_3.*e2_1_1.*e3_1_1.*2.0-e1_1_2.*e2_1_2.*e3_3_3.*2.0+e1_1_2.*e2_1_3.*e3_3_2.*2.0+e1_1_2.*e2_3_2.*e3_1_3.*2.0-e1_1_2.*e2_3_3.*e3_1_2.*2.0+e1_1_3.*e2_1_2.*e3_3_2.*2.0+e1_1_3.*e2_3_2.*e3_1_2.*2.0+e1_3_2.*e2_1_2.*e3_1_3.*2.0+e1_3_2.*e2_1_3.*e3_1_2.*2.0-e1_3_3.*e2_1_2.*e3_1_2.*2.0+e1_1_3.*e2_1_3.*e3_3_3.*2.0+e1_1_3.*e2_3_3.*e3_1_3.*2.0+e1_3_3.*e2_1_3.*e3_1_3.*2.0-e1_2_1.*e2_2_1.*e3_3_3.*2.0+e1_2_1.*e2_2_3.*e3_3_1.*2.0+e1_2_1.*e2_3_1.*e3_2_3.*2.0-e1_2_1.*e2_3_3.*e3_2_1.*2.0+e1_2_3.*e2_2_1.*e3_3_1.*2.0+e1_2_3.*e2_3_1.*e3_2_1.*2.0+e1_3_1.*e2_2_1.*e3_2_3.*2.0+e1_3_1.*e2_2_3.*e3_2_1.*2.0-e1_3_3.*e2_2_1.*e3_2_1.*2.0-e1_2_2.*e2_2_2.*e3_3_3.*2.0+e1_2_2.*e2_2_3.*e3_3_2.*2.0+e1_2_2.*e2_3_2.*e3_2_3.*2.0-e1_2_2.*e2_3_3.*e3_2_2.*2.0+e1_2_3.*e2_2_2.*e3_3_2.*2.0+e1_2_3.*e2_3_2.*e3_2_2.*2.0+e1_3_2.*e2_2_2.*e3_2_3.*2.0;
et68 = e1_3_2.*e2_2_3.*e3_2_2.*2.0-e1_3_3.*e2_2_2.*e3_2_2.*2.0+e1_2_3.*e2_2_3.*e3_3_3.*2.0+e1_2_3.*e2_3_3.*e3_2_3.*2.0+e1_3_3.*e2_2_3.*e3_2_3.*2.0+e1_3_1.*e2_3_1.*e3_3_3.*2.0+e1_3_1.*e2_3_3.*e3_3_1.*2.0+e1_3_3.*e2_3_1.*e3_3_1.*2.0+e1_3_2.*e2_3_2.*e3_3_3.*2.0+e1_3_2.*e2_3_3.*e3_3_2.*2.0+e1_3_3.*e2_3_2.*e3_3_2.*2.0+e1_3_3.*e2_3_3.*e3_3_3.*6.0;
et69 = e1_1_1.*e3_1_3.*e4_3_1.*2.0-e1_1_1.*e3_1_1.*e4_3_3.*2.0+e1_1_1.*e3_3_1.*e4_1_3.*2.0-e1_1_1.*e3_3_3.*e4_1_1.*2.0+e1_1_3.*e3_1_1.*e4_3_1.*2.0+e1_1_3.*e3_3_1.*e4_1_1.*2.0+e1_3_1.*e3_1_1.*e4_1_3.*2.0+e1_3_1.*e3_1_3.*e4_1_1.*2.0-e1_3_3.*e3_1_1.*e4_1_1.*2.0-e1_1_2.*e3_1_2.*e4_3_3.*2.0+e1_1_2.*e3_1_3.*e4_3_2.*2.0+e1_1_2.*e3_3_2.*e4_1_3.*2.0-e1_1_2.*e3_3_3.*e4_1_2.*2.0+e1_1_3.*e3_1_2.*e4_3_2.*2.0+e1_1_3.*e3_3_2.*e4_1_2.*2.0+e1_3_2.*e3_1_2.*e4_1_3.*2.0+e1_3_2.*e3_1_3.*e4_1_2.*2.0-e1_3_3.*e3_1_2.*e4_1_2.*2.0+e1_1_3.*e3_1_3.*e4_3_3.*2.0+e1_1_3.*e3_3_3.*e4_1_3.*2.0+e1_3_3.*e3_1_3.*e4_1_3.*2.0-e1_2_1.*e3_2_1.*e4_3_3.*2.0+e1_2_1.*e3_2_3.*e4_3_1.*2.0+e1_2_1.*e3_3_1.*e4_2_3.*2.0-e1_2_1.*e3_3_3.*e4_2_1.*2.0+e1_2_3.*e3_2_1.*e4_3_1.*2.0+e1_2_3.*e3_3_1.*e4_2_1.*2.0+e1_3_1.*e3_2_1.*e4_2_3.*2.0+e1_3_1.*e3_2_3.*e4_2_1.*2.0-e1_3_3.*e3_2_1.*e4_2_1.*2.0-e1_2_2.*e3_2_2.*e4_3_3.*2.0+e1_2_2.*e3_2_3.*e4_3_2.*2.0+e1_2_2.*e3_3_2.*e4_2_3.*2.0-e1_2_2.*e3_3_3.*e4_2_2.*2.0+e1_2_3.*e3_2_2.*e4_3_2.*2.0+e1_2_3.*e3_3_2.*e4_2_2.*2.0+e1_3_2.*e3_2_2.*e4_2_3.*2.0;
et70 = e1_3_2.*e3_2_3.*e4_2_2.*2.0-e1_3_3.*e3_2_2.*e4_2_2.*2.0+e1_2_3.*e3_2_3.*e4_3_3.*2.0+e1_2_3.*e3_3_3.*e4_2_3.*2.0+e1_3_3.*e3_2_3.*e4_2_3.*2.0+e1_3_1.*e3_3_1.*e4_3_3.*2.0+e1_3_1.*e3_3_3.*e4_3_1.*2.0+e1_3_3.*e3_3_1.*e4_3_1.*2.0+e1_3_2.*e3_3_2.*e4_3_3.*2.0+e1_3_2.*e3_3_3.*e4_3_2.*2.0+e1_3_3.*e3_3_2.*e4_3_2.*2.0+e1_3_3.*e3_3_3.*e4_3_3.*6.0;
et71 = e2_1_1.*e3_1_3.*e4_3_1.*2.0-e2_1_1.*e3_1_1.*e4_3_3.*2.0+e2_1_1.*e3_3_1.*e4_1_3.*2.0-e2_1_1.*e3_3_3.*e4_1_1.*2.0+e2_1_3.*e3_1_1.*e4_3_1.*2.0+e2_1_3.*e3_3_1.*e4_1_1.*2.0+e2_3_1.*e3_1_1.*e4_1_3.*2.0+e2_3_1.*e3_1_3.*e4_1_1.*2.0-e2_3_3.*e3_1_1.*e4_1_1.*2.0-e2_1_2.*e3_1_2.*e4_3_3.*2.0+e2_1_2.*e3_1_3.*e4_3_2.*2.0+e2_1_2.*e3_3_2.*e4_1_3.*2.0-e2_1_2.*e3_3_3.*e4_1_2.*2.0+e2_1_3.*e3_1_2.*e4_3_2.*2.0+e2_1_3.*e3_3_2.*e4_1_2.*2.0+e2_3_2.*e3_1_2.*e4_1_3.*2.0+e2_3_2.*e3_1_3.*e4_1_2.*2.0-e2_3_3.*e3_1_2.*e4_1_2.*2.0+e2_1_3.*e3_1_3.*e4_3_3.*2.0+e2_1_3.*e3_3_3.*e4_1_3.*2.0+e2_3_3.*e3_1_3.*e4_1_3.*2.0-e2_2_1.*e3_2_1.*e4_3_3.*2.0+e2_2_1.*e3_2_3.*e4_3_1.*2.0+e2_2_1.*e3_3_1.*e4_2_3.*2.0-e2_2_1.*e3_3_3.*e4_2_1.*2.0+e2_2_3.*e3_2_1.*e4_3_1.*2.0+e2_2_3.*e3_3_1.*e4_2_1.*2.0+e2_3_1.*e3_2_1.*e4_2_3.*2.0+e2_3_1.*e3_2_3.*e4_2_1.*2.0-e2_3_3.*e3_2_1.*e4_2_1.*2.0-e2_2_2.*e3_2_2.*e4_3_3.*2.0+e2_2_2.*e3_2_3.*e4_3_2.*2.0+e2_2_2.*e3_3_2.*e4_2_3.*2.0-e2_2_2.*e3_3_3.*e4_2_2.*2.0+e2_2_3.*e3_2_2.*e4_3_2.*2.0+e2_2_3.*e3_3_2.*e4_2_2.*2.0+e2_3_2.*e3_2_2.*e4_2_3.*2.0;
et72 = e2_3_2.*e3_2_3.*e4_2_2.*2.0-e2_3_3.*e3_2_2.*e4_2_2.*2.0+e2_2_3.*e3_2_3.*e4_3_3.*2.0+e2_2_3.*e3_3_3.*e4_2_3.*2.0+e2_3_3.*e3_2_3.*e4_2_3.*2.0+e2_3_1.*e3_3_1.*e4_3_3.*2.0+e2_3_1.*e3_3_3.*e4_3_1.*2.0+e2_3_3.*e3_3_1.*e4_3_1.*2.0+e2_3_2.*e3_3_2.*e4_3_3.*2.0+e2_3_2.*e3_3_3.*e4_3_2.*2.0+e2_3_3.*e3_3_2.*e4_3_2.*2.0+e2_3_3.*e3_3_3.*e4_3_3.*6.0;
mt71 = [e1_1_1.*e1_2_2.*e3_3_3-e1_1_1.*e1_2_3.*e3_3_2-e1_1_1.*e1_3_2.*e3_2_3+e1_1_1.*e1_3_3.*e3_2_2-e1_1_2.*e1_2_1.*e3_3_3+e1_1_2.*e1_2_3.*e3_3_1+e1_1_2.*e1_3_1.*e3_2_3-e1_1_2.*e1_3_3.*e3_2_1+e1_1_3.*e1_2_1.*e3_3_2-e1_1_3.*e1_2_2.*e3_3_1-e1_1_3.*e1_3_1.*e3_2_2+e1_1_3.*e1_3_2.*e3_2_1+e1_2_1.*e1_3_2.*e3_1_3-e1_2_1.*e1_3_3.*e3_1_2-e1_2_2.*e1_3_1.*e3_1_3+e1_2_2.*e1_3_3.*e3_1_1+e1_2_3.*e1_3_1.*e3_1_2-e1_2_3.*e1_3_2.*e3_1_1];
mt72 = [e3_1_1.*t2.*3.0+e3_1_1.*t3+e3_1_1.*t4+e3_1_1.*t5-e3_1_1.*t6-e3_1_1.*t7+e3_1_1.*t8-e3_1_1.*t9-e3_1_1.*t10+2.0.*e3_1_2.*e1_1_1.*e1_1_2+2.0.*e3_1_3.*e1_1_1.*e1_1_3+2.0.*e3_2_1.*e1_1_1.*e1_2_1+2.0.*e3_1_2.*e1_2_1.*e1_2_2+2.0.*e3_2_1.*e1_1_2.*e1_2_2+-2.0.*e3_2_2.*e1_1_1.*e1_2_2+2.0.*e3_2_2.*e1_1_2.*e1_2_1+2.0.*e3_1_3.*e1_2_1.*e1_2_3+2.0.*e3_2_1.*e1_1_3.*e1_2_3+-2.0.*e3_2_3.*e1_1_1.*e1_2_3+2.0.*e3_2_3.*e1_1_3.*e1_2_1+2.0.*e3_3_1.*e1_1_1.*e1_3_1+2.0.*e3_1_2.*e1_3_1.*e1_3_2+2.0.*e3_3_1.*e1_1_2.*e1_3_2+-2.0.*e3_3_2.*e1_1_1.*e1_3_2+2.0.*e3_3_2.*e1_1_2.*e1_3_1+2.0.*e3_1_3.*e1_3_1.*e1_3_3+2.0.*e3_3_1.*e1_1_3.*e1_3_3+-2.0.*e3_3_3.*e1_1_1.*e1_3_3+2.0.*e3_3_3.*e1_1_3.*e1_3_1];
mt73 = [e3_2_1.*t2-e3_2_1.*t3-e3_2_1.*t4+e3_2_1.*t5.*3.0+e3_2_1.*t6+e3_2_1.*t7+e3_2_1.*t8-e3_2_1.*t9-e3_2_1.*t10+2.0.*e3_1_1.*e1_1_1.*e1_2_1+2.0.*e3_1_1.*e1_1_2.*e1_2_2+2.0.*e3_1_2.*e1_1_1.*e1_2_2+-2.0.*e3_1_2.*e1_1_2.*e1_2_1+2.0.*e3_2_2.*e1_1_1.*e1_1_2+2.0.*e3_1_1.*e1_1_3.*e1_2_3+2.0.*e3_1_3.*e1_1_1.*e1_2_3+-2.0.*e3_1_3.*e1_1_3.*e1_2_1+2.0.*e3_2_3.*e1_1_1.*e1_1_3+2.0.*e3_2_2.*e1_2_1.*e1_2_2+2.0.*e3_2_3.*e1_2_1.*e1_2_3+2.0.*e3_3_1.*e1_2_1.*e1_3_1+2.0.*e3_2_2.*e1_3_1.*e1_3_2+2.0.*e3_3_1.*e1_2_2.*e1_3_2+-2.0.*e3_3_2.*e1_2_1.*e1_3_2+2.0.*e3_3_2.*e1_2_2.*e1_3_1+2.0.*e3_2_3.*e1_3_1.*e1_3_3+2.0.*e3_3_1.*e1_2_3.*e1_3_3+-2.0.*e3_3_3.*e1_2_1.*e1_3_3+2.0.*e3_3_3.*e1_2_3.*e1_3_1];
mt74 = [e3_3_1.*t2-e3_3_1.*t3-e3_3_1.*t4+e3_3_1.*t5-e3_3_1.*t6-e3_3_1.*t7+e3_3_1.*t8.*3.0+e3_3_1.*t9+e3_3_1.*t10+2.0.*e3_1_1.*e1_1_1.*e1_3_1+2.0.*e3_1_1.*e1_1_2.*e1_3_2+2.0.*e3_1_2.*e1_1_1.*e1_3_2+-2.0.*e3_1_2.*e1_1_2.*e1_3_1+2.0.*e3_3_2.*e1_1_1.*e1_1_2+2.0.*e3_1_1.*e1_1_3.*e1_3_3+2.0.*e3_1_3.*e1_1_1.*e1_3_3+-2.0.*e3_1_3.*e1_1_3.*e1_3_1+2.0.*e3_3_3.*e1_1_1.*e1_1_3+2.0.*e3_2_1.*e1_2_1.*e1_3_1+2.0.*e3_2_1.*e1_2_2.*e1_3_2+2.0.*e3_2_2.*e1_2_1.*e1_3_2+-2.0.*e3_2_2.*e1_2_2.*e1_3_1+2.0.*e3_3_2.*e1_2_1.*e1_2_2+2.0.*e3_2_1.*e1_2_3.*e1_3_3+2.0.*e3_2_3.*e1_2_1.*e1_3_3+-2.0.*e3_2_3.*e1_2_3.*e1_3_1+2.0.*e3_3_3.*e1_2_1.*e1_2_3+2.0.*e3_3_2.*e1_3_1.*e1_3_2+2.0.*e3_3_3.*e1_3_1.*e1_3_3];
mt75 = [e3_1_2.*t2+e3_1_2.*t3.*3.0+e3_1_2.*t4-e3_1_2.*t5+e3_1_2.*t6-e3_1_2.*t7-e3_1_2.*t8+e3_1_2.*t9-e3_1_2.*t10+2.0.*e3_1_1.*e1_1_1.*e1_1_2+2.0.*e3_1_3.*e1_1_2.*e1_1_3+2.0.*e3_1_1.*e1_2_1.*e1_2_2+2.0.*e3_2_1.*e1_1_1.*e1_2_2+-2.0.*e3_2_1.*e1_1_2.*e1_2_1+2.0.*e3_2_2.*e1_1_1.*e1_2_1+2.0.*e3_2_2.*e1_1_2.*e1_2_2+2.0.*e3_1_3.*e1_2_2.*e1_2_3+2.0.*e3_2_2.*e1_1_3.*e1_2_3+-2.0.*e3_2_3.*e1_1_2.*e1_2_3+2.0.*e3_2_3.*e1_1_3.*e1_2_2+2.0.*e3_1_1.*e1_3_1.*e1_3_2+2.0.*e3_3_1.*e1_1_1.*e1_3_2+-2.0.*e3_3_1.*e1_1_2.*e1_3_1+2.0.*e3_3_2.*e1_1_1.*e1_3_1+2.0.*e3_3_2.*e1_1_2.*e1_3_2+2.0.*e3_1_3.*e1_3_2.*e1_3_3+2.0.*e3_3_2.*e1_1_3.*e1_3_3+-2.0.*e3_3_3.*e1_1_2.*e1_3_3+2.0.*e3_3_3.*e1_1_3.*e1_3_2];
mt76 = [-e3_2_2.*t2+e3_2_2.*t3-e3_2_2.*t4+e3_2_2.*t5+e3_2_2.*t6.*3.0+e3_2_2.*t7-e3_2_2.*t8+e3_2_2.*t9-e3_2_2.*t10+-2.0.*e3_1_1.*e1_1_1.*e1_2_2+2.0.*e3_1_1.*e1_1_2.*e1_2_1+2.0.*e3_1_2.*e1_1_1.*e1_2_1+2.0.*e3_2_1.*e1_1_1.*e1_1_2+2.0.*e3_1_2.*e1_1_2.*e1_2_2+2.0.*e3_1_2.*e1_1_3.*e1_2_3+2.0.*e3_1_3.*e1_1_2.*e1_2_3+-2.0.*e3_1_3.*e1_1_3.*e1_2_2+2.0.*e3_2_3.*e1_1_2.*e1_1_3+2.0.*e3_2_1.*e1_2_1.*e1_2_2+2.0.*e3_2_3.*e1_2_2.*e1_2_3+2.0.*e3_2_1.*e1_3_1.*e1_3_2+2.0.*e3_3_1.*e1_2_1.*e1_3_2+-2.0.*e3_3_1.*e1_2_2.*e1_3_1+2.0.*e3_3_2.*e1_2_1.*e1_3_1+2.0.*e3_3_2.*e1_2_2.*e1_3_2+2.0.*e3_2_3.*e1_3_2.*e1_3_3+2.0.*e3_3_2.*e1_2_3.*e1_3_3+-2.0.*e3_3_3.*e1_2_2.*e1_3_3+2.0.*e3_3_3.*e1_2_3.*e1_3_2];
mt77 = [-e3_3_2.*t2+e3_3_2.*t3-e3_3_2.*t4-e3_3_2.*t5+e3_3_2.*t6-e3_3_2.*t7+e3_3_2.*t8+e3_3_2.*t9.*3.0+e3_3_2.*t10+-2.0.*e3_1_1.*e1_1_1.*e1_3_2+2.0.*e3_1_1.*e1_1_2.*e1_3_1+2.0.*e3_1_2.*e1_1_1.*e1_3_1+2.0.*e3_3_1.*e1_1_1.*e1_1_2+2.0.*e3_1_2.*e1_1_2.*e1_3_2+2.0.*e3_1_2.*e1_1_3.*e1_3_3+2.0.*e3_1_3.*e1_1_2.*e1_3_3+-2.0.*e3_1_3.*e1_1_3.*e1_3_2+2.0.*e3_3_3.*e1_1_2.*e1_1_3+-2.0.*e3_2_1.*e1_2_1.*e1_3_2+2.0.*e3_2_1.*e1_2_2.*e1_3_1+2.0.*e3_2_2.*e1_2_1.*e1_3_1+2.0.*e3_3_1.*e1_2_1.*e1_2_2+2.0.*e3_2_2.*e1_2_2.*e1_3_2+2.0.*e3_2_2.*e1_2_3.*e1_3_3+2.0.*e3_2_3.*e1_2_2.*e1_3_3+-2.0.*e3_2_3.*e1_2_3.*e1_3_2+2.0.*e3_3_3.*e1_2_2.*e1_2_3+2.0.*e3_3_1.*e1_3_1.*e1_3_2+2.0.*e3_3_3.*e1_3_2.*e1_3_3];
mt78 = [e3_1_3.*t2+e3_1_3.*t3+e3_1_3.*t4.*3.0-e3_1_3.*t5-e3_1_3.*t6+e3_1_3.*t7-e3_1_3.*t8-e3_1_3.*t9+e3_1_3.*t10+2.0.*e3_1_1.*e1_1_1.*e1_1_3+2.0.*e3_1_2.*e1_1_2.*e1_1_3+2.0.*e3_1_1.*e1_2_1.*e1_2_3+2.0.*e3_2_1.*e1_1_1.*e1_2_3+-2.0.*e3_2_1.*e1_1_3.*e1_2_1+2.0.*e3_2_3.*e1_1_1.*e1_2_1+2.0.*e3_1_2.*e1_2_2.*e1_2_3+2.0.*e3_2_2.*e1_1_2.*e1_2_3+-2.0.*e3_2_2.*e1_1_3.*e1_2_2+2.0.*e3_2_3.*e1_1_2.*e1_2_2+2.0.*e3_2_3.*e1_1_3.*e1_2_3+2.0.*e3_1_1.*e1_3_1.*e1_3_3+2.0.*e3_3_1.*e1_1_1.*e1_3_3+-2.0.*e3_3_1.*e1_1_3.*e1_3_1+2.0.*e3_3_3.*e1_1_1.*e1_3_1+2.0.*e3_1_2.*e1_3_2.*e1_3_3+2.0.*e3_3_2.*e1_1_2.*e1_3_3+-2.0.*e3_3_2.*e1_1_3.*e1_3_2+2.0.*e3_3_3.*e1_1_2.*e1_3_2+2.0.*e3_3_3.*e1_1_3.*e1_3_3];
mt79 = [-e3_2_3.*t2-e3_2_3.*t3+e3_2_3.*t4+e3_2_3.*t5+e3_2_3.*t6+e3_2_3.*t7.*3.0-e3_2_3.*t8-e3_2_3.*t9+e3_2_3.*t10+-2.0.*e3_1_1.*e1_1_1.*e1_2_3+2.0.*e3_1_1.*e1_1_3.*e1_2_1+2.0.*e3_1_3.*e1_1_1.*e1_2_1+2.0.*e3_2_1.*e1_1_1.*e1_1_3+-2.0.*e3_1_2.*e1_1_2.*e1_2_3+2.0.*e3_1_2.*e1_1_3.*e1_2_2+2.0.*e3_1_3.*e1_1_2.*e1_2_2+2.0.*e3_2_2.*e1_1_2.*e1_1_3+2.0.*e3_1_3.*e1_1_3.*e1_2_3+2.0.*e3_2_1.*e1_2_1.*e1_2_3+2.0.*e3_2_2.*e1_2_2.*e1_2_3+2.0.*e3_2_1.*e1_3_1.*e1_3_3+2.0.*e3_3_1.*e1_2_1.*e1_3_3+-2.0.*e3_3_1.*e1_2_3.*e1_3_1+2.0.*e3_3_3.*e1_2_1.*e1_3_1+2.0.*e3_2_2.*e1_3_2.*e1_3_3+2.0.*e3_3_2.*e1_2_2.*e1_3_3+-2.0.*e3_3_2.*e1_2_3.*e1_3_2+2.0.*e3_3_3.*e1_2_2.*e1_3_2+2.0.*e3_3_3.*e1_2_3.*e1_3_3];
mt80 = [-e3_3_3.*t2-e3_3_3.*t3+e3_3_3.*t4-e3_3_3.*t5-e3_3_3.*t6+e3_3_3.*t7+e3_3_3.*t8+e3_3_3.*t9+e3_3_3.*t10.*3.0+-2.0.*e3_1_1.*e1_1_1.*e1_3_3+2.0.*e3_1_1.*e1_1_3.*e1_3_1+2.0.*e3_1_3.*e1_1_1.*e1_3_1+2.0.*e3_3_1.*e1_1_1.*e1_1_3+-2.0.*e3_1_2.*e1_1_2.*e1_3_3+2.0.*e3_1_2.*e1_1_3.*e1_3_2+2.0.*e3_1_3.*e1_1_2.*e1_3_2+2.0.*e3_3_2.*e1_1_2.*e1_1_3+2.0.*e3_1_3.*e1_1_3.*e1_3_3+-2.0.*e3_2_1.*e1_2_1.*e1_3_3+2.0.*e3_2_1.*e1_2_3.*e1_3_1+2.0.*e3_2_3.*e1_2_1.*e1_3_1+2.0.*e3_3_1.*e1_2_1.*e1_2_3+-2.0.*e3_2_2.*e1_2_2.*e1_3_3+2.0.*e3_2_2.*e1_2_3.*e1_3_2+2.0.*e3_2_3.*e1_2_2.*e1_3_2+2.0.*e3_3_2.*e1_2_2.*e1_2_3+2.0.*e3_2_3.*e1_2_3.*e1_3_3+2.0.*e3_3_1.*e1_3_1.*e1_3_3+2.0.*e3_3_2.*e1_3_2.*e1_3_3];
mt81 = [e1_1_1.*e2_2_2.*e3_3_3-e1_1_1.*e2_2_3.*e3_3_2-e1_1_1.*e2_3_2.*e3_2_3+e1_1_1.*e2_3_3.*e3_2_2-e1_1_2.*e2_2_1.*e3_3_3+e1_1_2.*e2_2_3.*e3_3_1+e1_1_2.*e2_3_1.*e3_2_3-e1_1_2.*e2_3_3.*e3_2_1+e1_1_3.*e2_2_1.*e3_3_2-e1_1_3.*e2_2_2.*e3_3_1-e1_1_3.*e2_3_1.*e3_2_2+e1_1_3.*e2_3_2.*e3_2_1-e1_2_1.*e2_1_2.*e3_3_3+e1_2_1.*e2_1_3.*e3_3_2+e1_2_1.*e2_3_2.*e3_1_3-e1_2_1.*e2_3_3.*e3_1_2+e1_2_2.*e2_1_1.*e3_3_3-e1_2_2.*e2_1_3.*e3_3_1-e1_2_2.*e2_3_1.*e3_1_3+e1_2_2.*e2_3_3.*e3_1_1-e1_2_3.*e2_1_1.*e3_3_2+e1_2_3.*e2_1_2.*e3_3_1+e1_2_3.*e2_3_1.*e3_1_2-e1_2_3.*e2_3_2.*e3_1_1+e1_3_1.*e2_1_2.*e3_2_3-e1_3_1.*e2_1_3.*e3_2_2-e1_3_1.*e2_2_2.*e3_1_3+e1_3_1.*e2_2_3.*e3_1_2-e1_3_2.*e2_1_1.*e3_2_3+e1_3_2.*e2_1_3.*e3_2_1+e1_3_2.*e2_2_1.*e3_1_3-e1_3_2.*e2_2_3.*e3_1_1+e1_3_3.*e2_1_1.*e3_2_2-e1_3_3.*e2_1_2.*e3_2_1-e1_3_3.*e2_2_1.*e3_1_2+e1_3_3.*e2_2_2.*e3_1_1,et19+et20,et25+et26,et31+et32,et37+et38,et43+et44,et49+et50];
mt82 = [et55+et56,et61+et62,et67+et68,e2_1_1.*e2_2_2.*e3_3_3-e2_1_1.*e2_2_3.*e3_3_2-e2_1_1.*e2_3_2.*e3_2_3+e2_1_1.*e2_3_3.*e3_2_2-e2_1_2.*e2_2_1.*e3_3_3+e2_1_2.*e2_2_3.*e3_3_1+e2_1_2.*e2_3_1.*e3_2_3-e2_1_2.*e2_3_3.*e3_2_1+e2_1_3.*e2_2_1.*e3_3_2-e2_1_3.*e2_2_2.*e3_3_1-e2_1_3.*e2_3_1.*e3_2_2+e2_1_3.*e2_3_2.*e3_2_1+e2_2_1.*e2_3_2.*e3_1_3-e2_2_1.*e2_3_3.*e3_1_2-e2_2_2.*e2_3_1.*e3_1_3+e2_2_2.*e2_3_3.*e3_1_1+e2_2_3.*e2_3_1.*e3_1_2-e2_2_3.*e2_3_2.*e3_1_1];
mt83 = [e3_1_1.*t11.*3.0+e3_1_1.*t12+e3_1_1.*t13+e3_1_1.*t14-e3_1_1.*t15-e3_1_1.*t16+e3_1_1.*t17-e3_1_1.*t18-e3_1_1.*t19+2.0.*e3_1_2.*e2_1_1.*e2_1_2+2.0.*e3_1_3.*e2_1_1.*e2_1_3+2.0.*e3_2_1.*e2_1_1.*e2_2_1+2.0.*e3_1_2.*e2_2_1.*e2_2_2+2.0.*e3_2_1.*e2_1_2.*e2_2_2+-2.0.*e3_2_2.*e2_1_1.*e2_2_2+2.0.*e3_2_2.*e2_1_2.*e2_2_1+2.0.*e3_1_3.*e2_2_1.*e2_2_3+2.0.*e3_2_1.*e2_1_3.*e2_2_3+-2.0.*e3_2_3.*e2_1_1.*e2_2_3+2.0.*e3_2_3.*e2_1_3.*e2_2_1+2.0.*e3_3_1.*e2_1_1.*e2_3_1+2.0.*e3_1_2.*e2_3_1.*e2_3_2+2.0.*e3_3_1.*e2_1_2.*e2_3_2+-2.0.*e3_3_2.*e2_1_1.*e2_3_2+2.0.*e3_3_2.*e2_1_2.*e2_3_1+2.0.*e3_1_3.*e2_3_1.*e2_3_3+2.0.*e3_3_1.*e2_1_3.*e2_3_3+-2.0.*e3_3_3.*e2_1_1.*e2_3_3+2.0.*e3_3_3.*e2_1_3.*e2_3_1];
mt84 = [e3_2_1.*t11-e3_2_1.*t12-e3_2_1.*t13+e3_2_1.*t14.*3.0+e3_2_1.*t15+e3_2_1.*t16+e3_2_1.*t17-e3_2_1.*t18-e3_2_1.*t19+2.0.*e3_1_1.*e2_1_1.*e2_2_1+2.0.*e3_1_1.*e2_1_2.*e2_2_2+2.0.*e3_1_2.*e2_1_1.*e2_2_2+-2.0.*e3_1_2.*e2_1_2.*e2_2_1+2.0.*e3_2_2.*e2_1_1.*e2_1_2+2.0.*e3_1_1.*e2_1_3.*e2_2_3+2.0.*e3_1_3.*e2_1_1.*e2_2_3+-2.0.*e3_1_3.*e2_1_3.*e2_2_1+2.0.*e3_2_3.*e2_1_1.*e2_1_3+2.0.*e3_2_2.*e2_2_1.*e2_2_2+2.0.*e3_2_3.*e2_2_1.*e2_2_3+2.0.*e3_3_1.*e2_2_1.*e2_3_1+2.0.*e3_2_2.*e2_3_1.*e2_3_2+2.0.*e3_3_1.*e2_2_2.*e2_3_2+-2.0.*e3_3_2.*e2_2_1.*e2_3_2+2.0.*e3_3_2.*e2_2_2.*e2_3_1+2.0.*e3_2_3.*e2_3_1.*e2_3_3+2.0.*e3_3_1.*e2_2_3.*e2_3_3+-2.0.*e3_3_3.*e2_2_1.*e2_3_3+2.0.*e3_3_3.*e2_2_3.*e2_3_1];
mt85 = [e3_3_1.*t11-e3_3_1.*t12-e3_3_1.*t13+e3_3_1.*t14-e3_3_1.*t15-e3_3_1.*t16+e3_3_1.*t17.*3.0+e3_3_1.*t18+e3_3_1.*t19+2.0.*e3_1_1.*e2_1_1.*e2_3_1+2.0.*e3_1_1.*e2_1_2.*e2_3_2+2.0.*e3_1_2.*e2_1_1.*e2_3_2+-2.0.*e3_1_2.*e2_1_2.*e2_3_1+2.0.*e3_3_2.*e2_1_1.*e2_1_2+2.0.*e3_1_1.*e2_1_3.*e2_3_3+2.0.*e3_1_3.*e2_1_1.*e2_3_3+-2.0.*e3_1_3.*e2_1_3.*e2_3_1+2.0.*e3_3_3.*e2_1_1.*e2_1_3+2.0.*e3_2_1.*e2_2_1.*e2_3_1+2.0.*e3_2_1.*e2_2_2.*e2_3_2+2.0.*e3_2_2.*e2_2_1.*e2_3_2+-2.0.*e3_2_2.*e2_2_2.*e2_3_1+2.0.*e3_3_2.*e2_2_1.*e2_2_2+2.0.*e3_2_1.*e2_2_3.*e2_3_3+2.0.*e3_2_3.*e2_2_1.*e2_3_3+-2.0.*e3_2_3.*e2_2_3.*e2_3_1+2.0.*e3_3_3.*e2_2_1.*e2_2_3+2.0.*e3_3_2.*e2_3_1.*e2_3_2+2.0.*e3_3_3.*e2_3_1.*e2_3_3];
mt86 = [e3_1_2.*t11+e3_1_2.*t12.*3.0+e3_1_2.*t13-e3_1_2.*t14+e3_1_2.*t15-e3_1_2.*t16-e3_1_2.*t17+e3_1_2.*t18-e3_1_2.*t19+2.0.*e3_1_1.*e2_1_1.*e2_1_2+2.0.*e3_1_3.*e2_1_2.*e2_1_3+2.0.*e3_1_1.*e2_2_1.*e2_2_2+2.0.*e3_2_1.*e2_1_1.*e2_2_2+-2.0.*e3_2_1.*e2_1_2.*e2_2_1+2.0.*e3_2_2.*e2_1_1.*e2_2_1+2.0.*e3_2_2.*e2_1_2.*e2_2_2+2.0.*e3_1_3.*e2_2_2.*e2_2_3+2.0.*e3_2_2.*e2_1_3.*e2_2_3+-2.0.*e3_2_3.*e2_1_2.*e2_2_3+2.0.*e3_2_3.*e2_1_3.*e2_2_2+2.0.*e3_1_1.*e2_3_1.*e2_3_2+2.0.*e3_3_1.*e2_1_1.*e2_3_2+-2.0.*e3_3_1.*e2_1_2.*e2_3_1+2.0.*e3_3_2.*e2_1_1.*e2_3_1+2.0.*e3_3_2.*e2_1_2.*e2_3_2+2.0.*e3_1_3.*e2_3_2.*e2_3_3+2.0.*e3_3_2.*e2_1_3.*e2_3_3+-2.0.*e3_3_3.*e2_1_2.*e2_3_3+2.0.*e3_3_3.*e2_1_3.*e2_3_2];
mt87 = [-e3_2_2.*t11+e3_2_2.*t12-e3_2_2.*t13+e3_2_2.*t14+e3_2_2.*t15.*3.0+e3_2_2.*t16-e3_2_2.*t17+e3_2_2.*t18-e3_2_2.*t19+-2.0.*e3_1_1.*e2_1_1.*e2_2_2+2.0.*e3_1_1.*e2_1_2.*e2_2_1+2.0.*e3_1_2.*e2_1_1.*e2_2_1+2.0.*e3_2_1.*e2_1_1.*e2_1_2+2.0.*e3_1_2.*e2_1_2.*e2_2_2+2.0.*e3_1_2.*e2_1_3.*e2_2_3+2.0.*e3_1_3.*e2_1_2.*e2_2_3+-2.0.*e3_1_3.*e2_1_3.*e2_2_2+2.0.*e3_2_3.*e2_1_2.*e2_1_3+2.0.*e3_2_1.*e2_2_1.*e2_2_2+2.0.*e3_2_3.*e2_2_2.*e2_2_3+2.0.*e3_2_1.*e2_3_1.*e2_3_2+2.0.*e3_3_1.*e2_2_1.*e2_3_2+-2.0.*e3_3_1.*e2_2_2.*e2_3_1+2.0.*e3_3_2.*e2_2_1.*e2_3_1+2.0.*e3_3_2.*e2_2_2.*e2_3_2+2.0.*e3_2_3.*e2_3_2.*e2_3_3+2.0.*e3_3_2.*e2_2_3.*e2_3_3+-2.0.*e3_3_3.*e2_2_2.*e2_3_3+2.0.*e3_3_3.*e2_2_3.*e2_3_2];
mt88 = [-e3_3_2.*t11+e3_3_2.*t12-e3_3_2.*t13-e3_3_2.*t14+e3_3_2.*t15-e3_3_2.*t16+e3_3_2.*t17+e3_3_2.*t18.*3.0+e3_3_2.*t19+-2.0.*e3_1_1.*e2_1_1.*e2_3_2+2.0.*e3_1_1.*e2_1_2.*e2_3_1+2.0.*e3_1_2.*e2_1_1.*e2_3_1+2.0.*e3_3_1.*e2_1_1.*e2_1_2+2.0.*e3_1_2.*e2_1_2.*e2_3_2+2.0.*e3_1_2.*e2_1_3.*e2_3_3+2.0.*e3_1_3.*e2_1_2.*e2_3_3+-2.0.*e3_1_3.*e2_1_3.*e2_3_2+2.0.*e3_3_3.*e2_1_2.*e2_1_3+-2.0.*e3_2_1.*e2_2_1.*e2_3_2+2.0.*e3_2_1.*e2_2_2.*e2_3_1+2.0.*e3_2_2.*e2_2_1.*e2_3_1+2.0.*e3_3_1.*e2_2_1.*e2_2_2+2.0.*e3_2_2.*e2_2_2.*e2_3_2+2.0.*e3_2_2.*e2_2_3.*e2_3_3+2.0.*e3_2_3.*e2_2_2.*e2_3_3+-2.0.*e3_2_3.*e2_2_3.*e2_3_2+2.0.*e3_3_3.*e2_2_2.*e2_2_3+2.0.*e3_3_1.*e2_3_1.*e2_3_2+2.0.*e3_3_3.*e2_3_2.*e2_3_3];
mt89 = [e3_1_3.*t11+e3_1_3.*t12+e3_1_3.*t13.*3.0-e3_1_3.*t14-e3_1_3.*t15+e3_1_3.*t16-e3_1_3.*t17-e3_1_3.*t18+e3_1_3.*t19+2.0.*e3_1_1.*e2_1_1.*e2_1_3+2.0.*e3_1_2.*e2_1_2.*e2_1_3+2.0.*e3_1_1.*e2_2_1.*e2_2_3+2.0.*e3_2_1.*e2_1_1.*e2_2_3+-2.0.*e3_2_1.*e2_1_3.*e2_2_1+2.0.*e3_2_3.*e2_1_1.*e2_2_1+2.0.*e3_1_2.*e2_2_2.*e2_2_3+2.0.*e3_2_2.*e2_1_2.*e2_2_3+-2.0.*e3_2_2.*e2_1_3.*e2_2_2+2.0.*e3_2_3.*e2_1_2.*e2_2_2+2.0.*e3_2_3.*e2_1_3.*e2_2_3+2.0.*e3_1_1.*e2_3_1.*e2_3_3+2.0.*e3_3_1.*e2_1_1.*e2_3_3+-2.0.*e3_3_1.*e2_1_3.*e2_3_1+2.0.*e3_3_3.*e2_1_1.*e2_3_1+2.0.*e3_1_2.*e2_3_2.*e2_3_3+2.0.*e3_3_2.*e2_1_2.*e2_3_3+-2.0.*e3_3_2.*e2_1_3.*e2_3_2+2.0.*e3_3_3.*e2_1_2.*e2_3_2+2.0.*e3_3_3.*e2_1_3.*e2_3_3];
mt90 = [-e3_2_3.*t11-e3_2_3.*t12+e3_2_3.*t13+e3_2_3.*t14+e3_2_3.*t15+e3_2_3.*t16.*3.0-e3_2_3.*t17-e3_2_3.*t18+e3_2_3.*t19+-2.0.*e3_1_1.*e2_1_1.*e2_2_3+2.0.*e3_1_1.*e2_1_3.*e2_2_1+2.0.*e3_1_3.*e2_1_1.*e2_2_1+2.0.*e3_2_1.*e2_1_1.*e2_1_3+-2.0.*e3_1_2.*e2_1_2.*e2_2_3+2.0.*e3_1_2.*e2_1_3.*e2_2_2+2.0.*e3_1_3.*e2_1_2.*e2_2_2+2.0.*e3_2_2.*e2_1_2.*e2_1_3+2.0.*e3_1_3.*e2_1_3.*e2_2_3+2.0.*e3_2_1.*e2_2_1.*e2_2_3+2.0.*e3_2_2.*e2_2_2.*e2_2_3+2.0.*e3_2_1.*e2_3_1.*e2_3_3+2.0.*e3_3_1.*e2_2_1.*e2_3_3+-2.0.*e3_3_1.*e2_2_3.*e2_3_1+2.0.*e3_3_3.*e2_2_1.*e2_3_1+2.0.*e3_2_2.*e2_3_2.*e2_3_3+2.0.*e3_3_2.*e2_2_2.*e2_3_3+-2.0.*e3_3_2.*e2_2_3.*e2_3_2+2.0.*e3_3_3.*e2_2_2.*e2_3_2+2.0.*e3_3_3.*e2_2_3.*e2_3_3];
mt91 = [-e3_3_3.*t11-e3_3_3.*t12+e3_3_3.*t13-e3_3_3.*t14-e3_3_3.*t15+e3_3_3.*t16+e3_3_3.*t17+e3_3_3.*t18+e3_3_3.*t19.*3.0+-2.0.*e3_1_1.*e2_1_1.*e2_3_3+2.0.*e3_1_1.*e2_1_3.*e2_3_1+2.0.*e3_1_3.*e2_1_1.*e2_3_1+2.0.*e3_3_1.*e2_1_1.*e2_1_3+-2.0.*e3_1_2.*e2_1_2.*e2_3_3+2.0.*e3_1_2.*e2_1_3.*e2_3_2+2.0.*e3_1_3.*e2_1_2.*e2_3_2+2.0.*e3_3_2.*e2_1_2.*e2_1_3+2.0.*e3_1_3.*e2_1_3.*e2_3_3+-2.0.*e3_2_1.*e2_2_1.*e2_3_3+2.0.*e3_2_1.*e2_2_3.*e2_3_1+2.0.*e3_2_3.*e2_2_1.*e2_3_1+2.0.*e3_3_1.*e2_2_1.*e2_2_3+-2.0.*e3_2_2.*e2_2_2.*e2_3_3+2.0.*e3_2_2.*e2_2_3.*e2_3_2+2.0.*e3_2_3.*e2_2_2.*e2_3_2+2.0.*e3_3_2.*e2_2_2.*e2_2_3+2.0.*e3_2_3.*e2_2_3.*e2_3_3+2.0.*e3_3_1.*e2_3_1.*e2_3_3+2.0.*e3_3_2.*e2_3_2.*e2_3_3];
mt92 = [e1_1_1.*e3_2_2.*e4_3_3-e1_1_1.*e3_2_3.*e4_3_2-e1_1_1.*e3_3_2.*e4_2_3+e1_1_1.*e3_3_3.*e4_2_2-e1_1_2.*e3_2_1.*e4_3_3+e1_1_2.*e3_2_3.*e4_3_1+e1_1_2.*e3_3_1.*e4_2_3-e1_1_2.*e3_3_3.*e4_2_1+e1_1_3.*e3_2_1.*e4_3_2-e1_1_3.*e3_2_2.*e4_3_1-e1_1_3.*e3_3_1.*e4_2_2+e1_1_3.*e3_3_2.*e4_2_1-e1_2_1.*e3_1_2.*e4_3_3+e1_2_1.*e3_1_3.*e4_3_2+e1_2_1.*e3_3_2.*e4_1_3-e1_2_1.*e3_3_3.*e4_1_2+e1_2_2.*e3_1_1.*e4_3_3-e1_2_2.*e3_1_3.*e4_3_1-e1_2_2.*e3_3_1.*e4_1_3+e1_2_2.*e3_3_3.*e4_1_1-e1_2_3.*e3_1_1.*e4_3_2+e1_2_3.*e3_1_2.*e4_3_1+e1_2_3.*e3_3_1.*e4_1_2-e1_2_3.*e3_3_2.*e4_1_1+e1_3_1.*e3_1_2.*e4_2_3-e1_3_1.*e3_1_3.*e4_2_2-e1_3_1.*e3_2_2.*e4_1_3+e1_3_1.*e3_2_3.*e4_1_2-e1_3_2.*e3_1_1.*e4_2_3+e1_3_2.*e3_1_3.*e4_2_1+e1_3_2.*e3_2_1.*e4_1_3-e1_3_2.*e3_2_3.*e4_1_1+e1_3_3.*e3_1_1.*e4_2_2-e1_3_3.*e3_1_2.*e4_2_1-e1_3_3.*e3_2_1.*e4_1_2+e1_3_3.*e3_2_2.*e4_1_1,et21+et22,et27+et28,et33+et34,et39+et40,et45+et46,et51+et52];
mt93 = [et57+et58,et63+et64,et69+et70,e2_1_1.*e3_2_2.*e4_3_3-e2_1_1.*e3_2_3.*e4_3_2-e2_1_1.*e3_3_2.*e4_2_3+e2_1_1.*e3_3_3.*e4_2_2-e2_1_2.*e3_2_1.*e4_3_3+e2_1_2.*e3_2_3.*e4_3_1+e2_1_2.*e3_3_1.*e4_2_3-e2_1_2.*e3_3_3.*e4_2_1+e2_1_3.*e3_2_1.*e4_3_2-e2_1_3.*e3_2_2.*e4_3_1-e2_1_3.*e3_3_1.*e4_2_2+e2_1_3.*e3_3_2.*e4_2_1-e2_2_1.*e3_1_2.*e4_3_3+e2_2_1.*e3_1_3.*e4_3_2+e2_2_1.*e3_3_2.*e4_1_3-e2_2_1.*e3_3_3.*e4_1_2+e2_2_2.*e3_1_1.*e4_3_3-e2_2_2.*e3_1_3.*e4_3_1-e2_2_2.*e3_3_1.*e4_1_3+e2_2_2.*e3_3_3.*e4_1_1-e2_2_3.*e3_1_1.*e4_3_2+e2_2_3.*e3_1_2.*e4_3_1+e2_2_3.*e3_3_1.*e4_1_2-e2_2_3.*e3_3_2.*e4_1_1+e2_3_1.*e3_1_2.*e4_2_3-e2_3_1.*e3_1_3.*e4_2_2-e2_3_1.*e3_2_2.*e4_1_3+e2_3_1.*e3_2_3.*e4_1_2-e2_3_2.*e3_1_1.*e4_2_3+e2_3_2.*e3_1_3.*e4_2_1+e2_3_2.*e3_2_1.*e4_1_3-e2_3_2.*e3_2_3.*e4_1_1+e2_3_3.*e3_1_1.*e4_2_2-e2_3_3.*e3_1_2.*e4_2_1-e2_3_3.*e3_2_1.*e4_1_2+e2_3_3.*e3_2_2.*e4_1_1,et23+et24,et29+et30,et35+et36];
mt94 = [et41+et42,et47+et48,et53+et54,et59+et60,et65+et66,et71+et72,e3_1_1.*e4_2_2.*e4_3_3-e3_1_1.*e4_2_3.*e4_3_2-e3_1_2.*e4_2_1.*e4_3_3+e3_1_2.*e4_2_3.*e4_3_1+e3_1_3.*e4_2_1.*e4_3_2-e3_1_3.*e4_2_2.*e4_3_1-e3_2_1.*e4_1_2.*e4_3_3+e3_2_1.*e4_1_3.*e4_3_2+e3_2_2.*e4_1_1.*e4_3_3-e3_2_2.*e4_1_3.*e4_3_1-e3_2_3.*e4_1_1.*e4_3_2+e3_2_3.*e4_1_2.*e4_3_1+e3_3_1.*e4_1_2.*e4_2_3-e3_3_1.*e4_1_3.*e4_2_2-e3_3_2.*e4_1_1.*e4_2_3+e3_3_2.*e4_1_3.*e4_2_1+e3_3_3.*e4_1_1.*e4_2_2-e3_3_3.*e4_1_2.*e4_2_1];
mt95 = [e3_1_1.*t29.*3.0+e3_1_1.*t30+e3_1_1.*t31+e3_1_1.*t32-e3_1_1.*t33-e3_1_1.*t34+e3_1_1.*t35-e3_1_1.*t36-e3_1_1.*t37+2.0.*e3_1_2.*e4_1_1.*e4_1_2+2.0.*e3_1_3.*e4_1_1.*e4_1_3+2.0.*e3_2_1.*e4_1_1.*e4_2_1+2.0.*e3_1_2.*e4_2_1.*e4_2_2+2.0.*e3_2_1.*e4_1_2.*e4_2_2+-2.0.*e3_2_2.*e4_1_1.*e4_2_2+2.0.*e3_2_2.*e4_1_2.*e4_2_1+2.0.*e3_1_3.*e4_2_1.*e4_2_3+2.0.*e3_2_1.*e4_1_3.*e4_2_3+-2.0.*e3_2_3.*e4_1_1.*e4_2_3+2.0.*e3_2_3.*e4_1_3.*e4_2_1+2.0.*e3_3_1.*e4_1_1.*e4_3_1+2.0.*e3_1_2.*e4_3_1.*e4_3_2+2.0.*e3_3_1.*e4_1_2.*e4_3_2+-2.0.*e3_3_2.*e4_1_1.*e4_3_2+2.0.*e3_3_2.*e4_1_2.*e4_3_1+2.0.*e3_1_3.*e4_3_1.*e4_3_3+2.0.*e3_3_1.*e4_1_3.*e4_3_3+-2.0.*e3_3_3.*e4_1_1.*e4_3_3+2.0.*e3_3_3.*e4_1_3.*e4_3_1];
mt96 = [e3_2_1.*t29-e3_2_1.*t30-e3_2_1.*t31+e3_2_1.*t32.*3.0+e3_2_1.*t33+e3_2_1.*t34+e3_2_1.*t35-e3_2_1.*t36-e3_2_1.*t37+2.0.*e3_1_1.*e4_1_1.*e4_2_1+2.0.*e3_1_1.*e4_1_2.*e4_2_2+2.0.*e3_1_2.*e4_1_1.*e4_2_2+-2.0.*e3_1_2.*e4_1_2.*e4_2_1+2.0.*e3_2_2.*e4_1_1.*e4_1_2+2.0.*e3_1_1.*e4_1_3.*e4_2_3+2.0.*e3_1_3.*e4_1_1.*e4_2_3+-2.0.*e3_1_3.*e4_1_3.*e4_2_1+2.0.*e3_2_3.*e4_1_1.*e4_1_3+2.0.*e3_2_2.*e4_2_1.*e4_2_2+2.0.*e3_2_3.*e4_2_1.*e4_2_3+2.0.*e3_3_1.*e4_2_1.*e4_3_1+2.0.*e3_2_2.*e4_3_1.*e4_3_2+2.0.*e3_3_1.*e4_2_2.*e4_3_2+-2.0.*e3_3_2.*e4_2_1.*e4_3_2+2.0.*e3_3_2.*e4_2_2.*e4_3_1+2.0.*e3_2_3.*e4_3_1.*e4_3_3+2.0.*e3_3_1.*e4_2_3.*e4_3_3+-2.0.*e3_3_3.*e4_2_1.*e4_3_3+2.0.*e3_3_3.*e4_2_3.*e4_3_1];
mt97 = [e3_3_1.*t29-e3_3_1.*t30-e3_3_1.*t31+e3_3_1.*t32-e3_3_1.*t33-e3_3_1.*t34+e3_3_1.*t35.*3.0+e3_3_1.*t36+e3_3_1.*t37+2.0.*e3_1_1.*e4_1_1.*e4_3_1+2.0.*e3_1_1.*e4_1_2.*e4_3_2+2.0.*e3_1_2.*e4_1_1.*e4_3_2+-2.0.*e3_1_2.*e4_1_2.*e4_3_1+2.0.*e3_3_2.*e4_1_1.*e4_1_2+2.0.*e3_1_1.*e4_1_3.*e4_3_3+2.0.*e3_1_3.*e4_1_1.*e4_3_3+-2.0.*e3_1_3.*e4_1_3.*e4_3_1+2.0.*e3_3_3.*e4_1_1.*e4_1_3+2.0.*e3_2_1.*e4_2_1.*e4_3_1+2.0.*e3_2_1.*e4_2_2.*e4_3_2+2.0.*e3_2_2.*e4_2_1.*e4_3_2+-2.0.*e3_2_2.*e4_2_2.*e4_3_1+2.0.*e3_3_2.*e4_2_1.*e4_2_2+2.0.*e3_2_1.*e4_2_3.*e4_3_3+2.0.*e3_2_3.*e4_2_1.*e4_3_3+-2.0.*e3_2_3.*e4_2_3.*e4_3_1+2.0.*e3_3_3.*e4_2_1.*e4_2_3+2.0.*e3_3_2.*e4_3_1.*e4_3_2+2.0.*e3_3_3.*e4_3_1.*e4_3_3];
mt98 = [e3_1_2.*t29+e3_1_2.*t30.*3.0+e3_1_2.*t31-e3_1_2.*t32+e3_1_2.*t33-e3_1_2.*t34-e3_1_2.*t35+e3_1_2.*t36-e3_1_2.*t37+2.0.*e3_1_1.*e4_1_1.*e4_1_2+2.0.*e3_1_3.*e4_1_2.*e4_1_3+2.0.*e3_1_1.*e4_2_1.*e4_2_2+2.0.*e3_2_1.*e4_1_1.*e4_2_2+-2.0.*e3_2_1.*e4_1_2.*e4_2_1+2.0.*e3_2_2.*e4_1_1.*e4_2_1+2.0.*e3_2_2.*e4_1_2.*e4_2_2+2.0.*e3_1_3.*e4_2_2.*e4_2_3+2.0.*e3_2_2.*e4_1_3.*e4_2_3+-2.0.*e3_2_3.*e4_1_2.*e4_2_3+2.0.*e3_2_3.*e4_1_3.*e4_2_2+2.0.*e3_1_1.*e4_3_1.*e4_3_2+2.0.*e3_3_1.*e4_1_1.*e4_3_2+-2.0.*e3_3_1.*e4_1_2.*e4_3_1+2.0.*e3_3_2.*e4_1_1.*e4_3_1+2.0.*e3_3_2.*e4_1_2.*e4_3_2+2.0.*e3_1_3.*e4_3_2.*e4_3_3+2.0.*e3_3_2.*e4_1_3.*e4_3_3+-2.0.*e3_3_3.*e4_1_2.*e4_3_3+2.0.*e3_3_3.*e4_1_3.*e4_3_2];
mt99 = [-e3_2_2.*t29+e3_2_2.*t30-e3_2_2.*t31+e3_2_2.*t32+e3_2_2.*t33.*3.0+e3_2_2.*t34-e3_2_2.*t35+e3_2_2.*t36-e3_2_2.*t37+-2.0.*e3_1_1.*e4_1_1.*e4_2_2+2.0.*e3_1_1.*e4_1_2.*e4_2_1+2.0.*e3_1_2.*e4_1_1.*e4_2_1+2.0.*e3_2_1.*e4_1_1.*e4_1_2+2.0.*e3_1_2.*e4_1_2.*e4_2_2+2.0.*e3_1_2.*e4_1_3.*e4_2_3+2.0.*e3_1_3.*e4_1_2.*e4_2_3+-2.0.*e3_1_3.*e4_1_3.*e4_2_2+2.0.*e3_2_3.*e4_1_2.*e4_1_3+2.0.*e3_2_1.*e4_2_1.*e4_2_2+2.0.*e3_2_3.*e4_2_2.*e4_2_3+2.0.*e3_2_1.*e4_3_1.*e4_3_2+2.0.*e3_3_1.*e4_2_1.*e4_3_2+-2.0.*e3_3_1.*e4_2_2.*e4_3_1+2.0.*e3_3_2.*e4_2_1.*e4_3_1+2.0.*e3_3_2.*e4_2_2.*e4_3_2+2.0.*e3_2_3.*e4_3_2.*e4_3_3+2.0.*e3_3_2.*e4_2_3.*e4_3_3+-2.0.*e3_3_3.*e4_2_2.*e4_3_3+2.0.*e3_3_3.*e4_2_3.*e4_3_2];
mt100 = [-e3_3_2.*t29+e3_3_2.*t30-e3_3_2.*t31-e3_3_2.*t32+e3_3_2.*t33-e3_3_2.*t34+e3_3_2.*t35+e3_3_2.*t36.*3.0+e3_3_2.*t37+-2.0.*e3_1_1.*e4_1_1.*e4_3_2+2.0.*e3_1_1.*e4_1_2.*e4_3_1+2.0.*e3_1_2.*e4_1_1.*e4_3_1+2.0.*e3_3_1.*e4_1_1.*e4_1_2+2.0.*e3_1_2.*e4_1_2.*e4_3_2+2.0.*e3_1_2.*e4_1_3.*e4_3_3+2.0.*e3_1_3.*e4_1_2.*e4_3_3+-2.0.*e3_1_3.*e4_1_3.*e4_3_2+2.0.*e3_3_3.*e4_1_2.*e4_1_3+-2.0.*e3_2_1.*e4_2_1.*e4_3_2+2.0.*e3_2_1.*e4_2_2.*e4_3_1+2.0.*e3_2_2.*e4_2_1.*e4_3_1+2.0.*e3_3_1.*e4_2_1.*e4_2_2+2.0.*e3_2_2.*e4_2_2.*e4_3_2+2.0.*e3_2_2.*e4_2_3.*e4_3_3+2.0.*e3_2_3.*e4_2_2.*e4_3_3+-2.0.*e3_2_3.*e4_2_3.*e4_3_2+2.0.*e3_3_3.*e4_2_2.*e4_2_3+2.0.*e3_3_1.*e4_3_1.*e4_3_2+2.0.*e3_3_3.*e4_3_2.*e4_3_3];
mt101 = [e3_1_3.*t29+e3_1_3.*t30+e3_1_3.*t31.*3.0-e3_1_3.*t32-e3_1_3.*t33+e3_1_3.*t34-e3_1_3.*t35-e3_1_3.*t36+e3_1_3.*t37+2.0.*e3_1_1.*e4_1_1.*e4_1_3+2.0.*e3_1_2.*e4_1_2.*e4_1_3+2.0.*e3_1_1.*e4_2_1.*e4_2_3+2.0.*e3_2_1.*e4_1_1.*e4_2_3+-2.0.*e3_2_1.*e4_1_3.*e4_2_1+2.0.*e3_2_3.*e4_1_1.*e4_2_1+2.0.*e3_1_2.*e4_2_2.*e4_2_3+2.0.*e3_2_2.*e4_1_2.*e4_2_3+-2.0.*e3_2_2.*e4_1_3.*e4_2_2+2.0.*e3_2_3.*e4_1_2.*e4_2_2+2.0.*e3_2_3.*e4_1_3.*e4_2_3+2.0.*e3_1_1.*e4_3_1.*e4_3_3+2.0.*e3_3_1.*e4_1_1.*e4_3_3+-2.0.*e3_3_1.*e4_1_3.*e4_3_1+2.0.*e3_3_3.*e4_1_1.*e4_3_1+2.0.*e3_1_2.*e4_3_2.*e4_3_3+2.0.*e3_3_2.*e4_1_2.*e4_3_3+-2.0.*e3_3_2.*e4_1_3.*e4_3_2+2.0.*e3_3_3.*e4_1_2.*e4_3_2+2.0.*e3_3_3.*e4_1_3.*e4_3_3];
mt102 = [-e3_2_3.*t29-e3_2_3.*t30+e3_2_3.*t31+e3_2_3.*t32+e3_2_3.*t33+e3_2_3.*t34.*3.0-e3_2_3.*t35-e3_2_3.*t36+e3_2_3.*t37+-2.0.*e3_1_1.*e4_1_1.*e4_2_3+2.0.*e3_1_1.*e4_1_3.*e4_2_1+2.0.*e3_1_3.*e4_1_1.*e4_2_1+2.0.*e3_2_1.*e4_1_1.*e4_1_3+-2.0.*e3_1_2.*e4_1_2.*e4_2_3+2.0.*e3_1_2.*e4_1_3.*e4_2_2+2.0.*e3_1_3.*e4_1_2.*e4_2_2+2.0.*e3_2_2.*e4_1_2.*e4_1_3+2.0.*e3_1_3.*e4_1_3.*e4_2_3+2.0.*e3_2_1.*e4_2_1.*e4_2_3+2.0.*e3_2_2.*e4_2_2.*e4_2_3+2.0.*e3_2_1.*e4_3_1.*e4_3_3+2.0.*e3_3_1.*e4_2_1.*e4_3_3+-2.0.*e3_3_1.*e4_2_3.*e4_3_1+2.0.*e3_3_3.*e4_2_1.*e4_3_1+2.0.*e3_2_2.*e4_3_2.*e4_3_3+2.0.*e3_3_2.*e4_2_2.*e4_3_3+-2.0.*e3_3_2.*e4_2_3.*e4_3_2+2.0.*e3_3_3.*e4_2_2.*e4_3_2+2.0.*e3_3_3.*e4_2_3.*e4_3_3];
mt103 = [-e3_3_3.*t29-e3_3_3.*t30+e3_3_3.*t31-e3_3_3.*t32-e3_3_3.*t33+e3_3_3.*t34+e3_3_3.*t35+e3_3_3.*t36+e3_3_3.*t37.*3.0+-2.0.*e3_1_1.*e4_1_1.*e4_3_3+2.0.*e3_1_1.*e4_1_3.*e4_3_1+2.0.*e3_1_3.*e4_1_1.*e4_3_1+2.0.*e3_3_1.*e4_1_1.*e4_1_3+-2.0.*e3_1_2.*e4_1_2.*e4_3_3+2.0.*e3_1_2.*e4_1_3.*e4_3_2+2.0.*e3_1_3.*e4_1_2.*e4_3_2+2.0.*e3_3_2.*e4_1_2.*e4_1_3+2.0.*e3_1_3.*e4_1_3.*e4_3_3+-2.0.*e3_2_1.*e4_2_1.*e4_3_3+2.0.*e3_2_1.*e4_2_3.*e4_3_1+2.0.*e3_2_3.*e4_2_1.*e4_3_1+2.0.*e3_3_1.*e4_2_1.*e4_2_3+-2.0.*e3_2_2.*e4_2_2.*e4_3_3+2.0.*e3_2_2.*e4_2_3.*e4_3_2+2.0.*e3_2_3.*e4_2_2.*e4_3_2+2.0.*e3_3_2.*e4_2_2.*e4_2_3+2.0.*e3_2_3.*e4_2_3.*e4_3_3+2.0.*e3_3_1.*e4_3_1.*e4_3_3+2.0.*e3_3_2.*e4_3_2.*e4_3_3];
C2 = reshape([mt71,mt72,mt73,mt74,mt75,mt76,mt77,mt78,mt79,mt80,mt81,mt82,mt83,mt84,mt85,mt86,mt87,mt88,mt89,mt90,mt91,mt92,mt93,mt94,mt95,mt96,mt97,mt98,mt99,mt100,mt101,mt102,mt103],10,6);

mt104 = [e1_1_1.*e3_2_2.*e3_3_3-e1_1_1.*e3_2_3.*e3_3_2-e1_1_2.*e3_2_1.*e3_3_3+e1_1_2.*e3_2_3.*e3_3_1+e1_1_3.*e3_2_1.*e3_3_2-e1_1_3.*e3_2_2.*e3_3_1-e1_2_1.*e3_1_2.*e3_3_3+e1_2_1.*e3_1_3.*e3_3_2+e1_2_2.*e3_1_1.*e3_3_3-e1_2_2.*e3_1_3.*e3_3_1-e1_2_3.*e3_1_1.*e3_3_2+e1_2_3.*e3_1_2.*e3_3_1+e1_3_1.*e3_1_2.*e3_2_3-e1_3_1.*e3_1_3.*e3_2_2-e1_3_2.*e3_1_1.*e3_2_3+e1_3_2.*e3_1_3.*e3_2_1+e1_3_3.*e3_1_1.*e3_2_2-e1_3_3.*e3_1_2.*e3_2_1];
mt105 = [e1_1_1.*t20.*3.0+e1_1_1.*t21+e1_1_1.*t22+e1_1_1.*t23-e1_1_1.*t24-e1_1_1.*t25+e1_1_1.*t26-e1_1_1.*t27-e1_1_1.*t28+2.0.*e1_1_2.*e3_1_1.*e3_1_2+2.0.*e1_1_3.*e3_1_1.*e3_1_3+2.0.*e1_2_1.*e3_1_1.*e3_2_1+2.0.*e1_1_2.*e3_2_1.*e3_2_2+2.0.*e1_2_1.*e3_1_2.*e3_2_2+-2.0.*e1_2_2.*e3_1_1.*e3_2_2+2.0.*e1_2_2.*e3_1_2.*e3_2_1+2.0.*e1_1_3.*e3_2_1.*e3_2_3+2.0.*e1_2_1.*e3_1_3.*e3_2_3+-2.0.*e1_2_3.*e3_1_1.*e3_2_3+2.0.*e1_2_3.*e3_1_3.*e3_2_1+2.0.*e1_3_1.*e3_1_1.*e3_3_1+2.0.*e1_1_2.*e3_3_1.*e3_3_2+2.0.*e1_3_1.*e3_1_2.*e3_3_2+-2.0.*e1_3_2.*e3_1_1.*e3_3_2+2.0.*e1_3_2.*e3_1_2.*e3_3_1+2.0.*e1_1_3.*e3_3_1.*e3_3_3+2.0.*e1_3_1.*e3_1_3.*e3_3_3+-2.0.*e1_3_3.*e3_1_1.*e3_3_3+2.0.*e1_3_3.*e3_1_3.*e3_3_1];
mt106 = [e1_2_1.*t20-e1_2_1.*t21-e1_2_1.*t22+e1_2_1.*t23.*3.0+e1_2_1.*t24+e1_2_1.*t25+e1_2_1.*t26-e1_2_1.*t27-e1_2_1.*t28+2.0.*e1_1_1.*e3_1_1.*e3_2_1+2.0.*e1_1_1.*e3_1_2.*e3_2_2+2.0.*e1_1_2.*e3_1_1.*e3_2_2+-2.0.*e1_1_2.*e3_1_2.*e3_2_1+2.0.*e1_2_2.*e3_1_1.*e3_1_2+2.0.*e1_1_1.*e3_1_3.*e3_2_3+2.0.*e1_1_3.*e3_1_1.*e3_2_3+-2.0.*e1_1_3.*e3_1_3.*e3_2_1+2.0.*e1_2_3.*e3_1_1.*e3_1_3+2.0.*e1_2_2.*e3_2_1.*e3_2_2+2.0.*e1_2_3.*e3_2_1.*e3_2_3+2.0.*e1_3_1.*e3_2_1.*e3_3_1+2.0.*e1_2_2.*e3_3_1.*e3_3_2+2.0.*e1_3_1.*e3_2_2.*e3_3_2+-2.0.*e1_3_2.*e3_2_1.*e3_3_2+2.0.*e1_3_2.*e3_2_2.*e3_3_1+2.0.*e1_2_3.*e3_3_1.*e3_3_3+2.0.*e1_3_1.*e3_2_3.*e3_3_3+-2.0.*e1_3_3.*e3_2_1.*e3_3_3+2.0.*e1_3_3.*e3_2_3.*e3_3_1];
mt107 = [e1_3_1.*t20-e1_3_1.*t21-e1_3_1.*t22+e1_3_1.*t23-e1_3_1.*t24-e1_3_1.*t25+e1_3_1.*t26.*3.0+e1_3_1.*t27+e1_3_1.*t28+2.0.*e1_1_1.*e3_1_1.*e3_3_1+2.0.*e1_1_1.*e3_1_2.*e3_3_2+2.0.*e1_1_2.*e3_1_1.*e3_3_2+-2.0.*e1_1_2.*e3_1_2.*e3_3_1+2.0.*e1_3_2.*e3_1_1.*e3_1_2+2.0.*e1_1_1.*e3_1_3.*e3_3_3+2.0.*e1_1_3.*e3_1_1.*e3_3_3+-2.0.*e1_1_3.*e3_1_3.*e3_3_1+2.0.*e1_3_3.*e3_1_1.*e3_1_3+2.0.*e1_2_1.*e3_2_1.*e3_3_1+2.0.*e1_2_1.*e3_2_2.*e3_3_2+2.0.*e1_2_2.*e3_2_1.*e3_3_2+-2.0.*e1_2_2.*e3_2_2.*e3_3_1+2.0.*e1_3_2.*e3_2_1.*e3_2_2+2.0.*e1_2_1.*e3_2_3.*e3_3_3+2.0.*e1_2_3.*e3_2_1.*e3_3_3+-2.0.*e1_2_3.*e3_2_3.*e3_3_1+2.0.*e1_3_3.*e3_2_1.*e3_2_3+2.0.*e1_3_2.*e3_3_1.*e3_3_2+2.0.*e1_3_3.*e3_3_1.*e3_3_3];
mt108 = [e1_1_2.*t20+e1_1_2.*t21.*3.0+e1_1_2.*t22-e1_1_2.*t23+e1_1_2.*t24-e1_1_2.*t25-e1_1_2.*t26+e1_1_2.*t27-e1_1_2.*t28+2.0.*e1_1_1.*e3_1_1.*e3_1_2+2.0.*e1_1_3.*e3_1_2.*e3_1_3+2.0.*e1_1_1.*e3_2_1.*e3_2_2+2.0.*e1_2_1.*e3_1_1.*e3_2_2+-2.0.*e1_2_1.*e3_1_2.*e3_2_1+2.0.*e1_2_2.*e3_1_1.*e3_2_1+2.0.*e1_2_2.*e3_1_2.*e3_2_2+2.0.*e1_1_3.*e3_2_2.*e3_2_3+2.0.*e1_2_2.*e3_1_3.*e3_2_3+-2.0.*e1_2_3.*e3_1_2.*e3_2_3+2.0.*e1_2_3.*e3_1_3.*e3_2_2+2.0.*e1_1_1.*e3_3_1.*e3_3_2+2.0.*e1_3_1.*e3_1_1.*e3_3_2+-2.0.*e1_3_1.*e3_1_2.*e3_3_1+2.0.*e1_3_2.*e3_1_1.*e3_3_1+2.0.*e1_3_2.*e3_1_2.*e3_3_2+2.0.*e1_1_3.*e3_3_2.*e3_3_3+2.0.*e1_3_2.*e3_1_3.*e3_3_3+-2.0.*e1_3_3.*e3_1_2.*e3_3_3+2.0.*e1_3_3.*e3_1_3.*e3_3_2];
mt109 = [-e1_2_2.*t20+e1_2_2.*t21-e1_2_2.*t22+e1_2_2.*t23+e1_2_2.*t24.*3.0+e1_2_2.*t25-e1_2_2.*t26+e1_2_2.*t27-e1_2_2.*t28+-2.0.*e1_1_1.*e3_1_1.*e3_2_2+2.0.*e1_1_1.*e3_1_2.*e3_2_1+2.0.*e1_1_2.*e3_1_1.*e3_2_1+2.0.*e1_2_1.*e3_1_1.*e3_1_2+2.0.*e1_1_2.*e3_1_2.*e3_2_2+2.0.*e1_1_2.*e3_1_3.*e3_2_3+2.0.*e1_1_3.*e3_1_2.*e3_2_3+-2.0.*e1_1_3.*e3_1_3.*e3_2_2+2.0.*e1_2_3.*e3_1_2.*e3_1_3+2.0.*e1_2_1.*e3_2_1.*e3_2_2+2.0.*e1_2_3.*e3_2_2.*e3_2_3+2.0.*e1_2_1.*e3_3_1.*e3_3_2+2.0.*e1_3_1.*e3_2_1.*e3_3_2+-2.0.*e1_3_1.*e3_2_2.*e3_3_1+2.0.*e1_3_2.*e3_2_1.*e3_3_1+2.0.*e1_3_2.*e3_2_2.*e3_3_2+2.0.*e1_2_3.*e3_3_2.*e3_3_3+2.0.*e1_3_2.*e3_2_3.*e3_3_3+-2.0.*e1_3_3.*e3_2_2.*e3_3_3+2.0.*e1_3_3.*e3_2_3.*e3_3_2];
mt110 = [-e1_3_2.*t20+e1_3_2.*t21-e1_3_2.*t22-e1_3_2.*t23+e1_3_2.*t24-e1_3_2.*t25+e1_3_2.*t26+e1_3_2.*t27.*3.0+e1_3_2.*t28+-2.0.*e1_1_1.*e3_1_1.*e3_3_2+2.0.*e1_1_1.*e3_1_2.*e3_3_1+2.0.*e1_1_2.*e3_1_1.*e3_3_1+2.0.*e1_3_1.*e3_1_1.*e3_1_2+2.0.*e1_1_2.*e3_1_2.*e3_3_2+2.0.*e1_1_2.*e3_1_3.*e3_3_3+2.0.*e1_1_3.*e3_1_2.*e3_3_3+-2.0.*e1_1_3.*e3_1_3.*e3_3_2+2.0.*e1_3_3.*e3_1_2.*e3_1_3+-2.0.*e1_2_1.*e3_2_1.*e3_3_2+2.0.*e1_2_1.*e3_2_2.*e3_3_1+2.0.*e1_2_2.*e3_2_1.*e3_3_1+2.0.*e1_3_1.*e3_2_1.*e3_2_2+2.0.*e1_2_2.*e3_2_2.*e3_3_2+2.0.*e1_2_2.*e3_2_3.*e3_3_3+2.0.*e1_2_3.*e3_2_2.*e3_3_3+-2.0.*e1_2_3.*e3_2_3.*e3_3_2+2.0.*e1_3_3.*e3_2_2.*e3_2_3+2.0.*e1_3_1.*e3_3_1.*e3_3_2+2.0.*e1_3_3.*e3_3_2.*e3_3_3];
mt111 = [e1_1_3.*t20+e1_1_3.*t21+e1_1_3.*t22.*3.0-e1_1_3.*t23-e1_1_3.*t24+e1_1_3.*t25-e1_1_3.*t26-e1_1_3.*t27+e1_1_3.*t28+2.0.*e1_1_1.*e3_1_1.*e3_1_3+2.0.*e1_1_2.*e3_1_2.*e3_1_3+2.0.*e1_1_1.*e3_2_1.*e3_2_3+2.0.*e1_2_1.*e3_1_1.*e3_2_3+-2.0.*e1_2_1.*e3_1_3.*e3_2_1+2.0.*e1_2_3.*e3_1_1.*e3_2_1+2.0.*e1_1_2.*e3_2_2.*e3_2_3+2.0.*e1_2_2.*e3_1_2.*e3_2_3+-2.0.*e1_2_2.*e3_1_3.*e3_2_2+2.0.*e1_2_3.*e3_1_2.*e3_2_2+2.0.*e1_2_3.*e3_1_3.*e3_2_3+2.0.*e1_1_1.*e3_3_1.*e3_3_3+2.0.*e1_3_1.*e3_1_1.*e3_3_3+-2.0.*e1_3_1.*e3_1_3.*e3_3_1+2.0.*e1_3_3.*e3_1_1.*e3_3_1+2.0.*e1_1_2.*e3_3_2.*e3_3_3+2.0.*e1_3_2.*e3_1_2.*e3_3_3+-2.0.*e1_3_2.*e3_1_3.*e3_3_2+2.0.*e1_3_3.*e3_1_2.*e3_3_2+2.0.*e1_3_3.*e3_1_3.*e3_3_3];
mt112 = [-e1_2_3.*t20-e1_2_3.*t21+e1_2_3.*t22+e1_2_3.*t23+e1_2_3.*t24+e1_2_3.*t25.*3.0-e1_2_3.*t26-e1_2_3.*t27+e1_2_3.*t28+-2.0.*e1_1_1.*e3_1_1.*e3_2_3+2.0.*e1_1_1.*e3_1_3.*e3_2_1+2.0.*e1_1_3.*e3_1_1.*e3_2_1+2.0.*e1_2_1.*e3_1_1.*e3_1_3+-2.0.*e1_1_2.*e3_1_2.*e3_2_3+2.0.*e1_1_2.*e3_1_3.*e3_2_2+2.0.*e1_1_3.*e3_1_2.*e3_2_2+2.0.*e1_2_2.*e3_1_2.*e3_1_3+2.0.*e1_1_3.*e3_1_3.*e3_2_3+2.0.*e1_2_1.*e3_2_1.*e3_2_3+2.0.*e1_2_2.*e3_2_2.*e3_2_3+2.0.*e1_2_1.*e3_3_1.*e3_3_3+2.0.*e1_3_1.*e3_2_1.*e3_3_3+-2.0.*e1_3_1.*e3_2_3.*e3_3_1+2.0.*e1_3_3.*e3_2_1.*e3_3_1+2.0.*e1_2_2.*e3_3_2.*e3_3_3+2.0.*e1_3_2.*e3_2_2.*e3_3_3+-2.0.*e1_3_2.*e3_2_3.*e3_3_2+2.0.*e1_3_3.*e3_2_2.*e3_3_2+2.0.*e1_3_3.*e3_2_3.*e3_3_3];
mt113 = [-e1_3_3.*t20-e1_3_3.*t21+e1_3_3.*t22-e1_3_3.*t23-e1_3_3.*t24+e1_3_3.*t25+e1_3_3.*t26+e1_3_3.*t27+e1_3_3.*t28.*3.0+-2.0.*e1_1_1.*e3_1_1.*e3_3_3+2.0.*e1_1_1.*e3_1_3.*e3_3_1+2.0.*e1_1_3.*e3_1_1.*e3_3_1+2.0.*e1_3_1.*e3_1_1.*e3_1_3+-2.0.*e1_1_2.*e3_1_2.*e3_3_3+2.0.*e1_1_2.*e3_1_3.*e3_3_2+2.0.*e1_1_3.*e3_1_2.*e3_3_2+2.0.*e1_3_2.*e3_1_2.*e3_1_3+2.0.*e1_1_3.*e3_1_3.*e3_3_3+-2.0.*e1_2_1.*e3_2_1.*e3_3_3+2.0.*e1_2_1.*e3_2_3.*e3_3_1+2.0.*e1_2_3.*e3_2_1.*e3_3_1+2.0.*e1_3_1.*e3_2_1.*e3_2_3+-2.0.*e1_2_2.*e3_2_2.*e3_3_3+2.0.*e1_2_2.*e3_2_3.*e3_3_2+2.0.*e1_2_3.*e3_2_2.*e3_3_2+2.0.*e1_3_2.*e3_2_2.*e3_2_3+2.0.*e1_2_3.*e3_2_3.*e3_3_3+2.0.*e1_3_1.*e3_3_1.*e3_3_3+2.0.*e1_3_2.*e3_3_2.*e3_3_3];
mt114 = [e2_1_1.*e3_2_2.*e3_3_3-e2_1_1.*e3_2_3.*e3_3_2-e2_1_2.*e3_2_1.*e3_3_3+e2_1_2.*e3_2_3.*e3_3_1+e2_1_3.*e3_2_1.*e3_3_2-e2_1_3.*e3_2_2.*e3_3_1-e2_2_1.*e3_1_2.*e3_3_3+e2_2_1.*e3_1_3.*e3_3_2+e2_2_2.*e3_1_1.*e3_3_3-e2_2_2.*e3_1_3.*e3_3_1-e2_2_3.*e3_1_1.*e3_3_2+e2_2_3.*e3_1_2.*e3_3_1+e2_3_1.*e3_1_2.*e3_2_3-e2_3_1.*e3_1_3.*e3_2_2-e2_3_2.*e3_1_1.*e3_2_3+e2_3_2.*e3_1_3.*e3_2_1+e2_3_3.*e3_1_1.*e3_2_2-e2_3_3.*e3_1_2.*e3_2_1];
mt115 = [e2_1_1.*t20.*3.0+e2_1_1.*t21+e2_1_1.*t22+e2_1_1.*t23-e2_1_1.*t24-e2_1_1.*t25+e2_1_1.*t26-e2_1_1.*t27-e2_1_1.*t28+2.0.*e2_1_2.*e3_1_1.*e3_1_2+2.0.*e2_1_3.*e3_1_1.*e3_1_3+2.0.*e2_2_1.*e3_1_1.*e3_2_1+2.0.*e2_1_2.*e3_2_1.*e3_2_2+2.0.*e2_2_1.*e3_1_2.*e3_2_2+-2.0.*e2_2_2.*e3_1_1.*e3_2_2+2.0.*e2_2_2.*e3_1_2.*e3_2_1+2.0.*e2_1_3.*e3_2_1.*e3_2_3+2.0.*e2_2_1.*e3_1_3.*e3_2_3+-2.0.*e2_2_3.*e3_1_1.*e3_2_3+2.0.*e2_2_3.*e3_1_3.*e3_2_1+2.0.*e2_3_1.*e3_1_1.*e3_3_1+2.0.*e2_1_2.*e3_3_1.*e3_3_2+2.0.*e2_3_1.*e3_1_2.*e3_3_2+-2.0.*e2_3_2.*e3_1_1.*e3_3_2+2.0.*e2_3_2.*e3_1_2.*e3_3_1+2.0.*e2_1_3.*e3_3_1.*e3_3_3+2.0.*e2_3_1.*e3_1_3.*e3_3_3+-2.0.*e2_3_3.*e3_1_1.*e3_3_3+2.0.*e2_3_3.*e3_1_3.*e3_3_1];
mt116 = [e2_2_1.*t20-e2_2_1.*t21-e2_2_1.*t22+e2_2_1.*t23.*3.0+e2_2_1.*t24+e2_2_1.*t25+e2_2_1.*t26-e2_2_1.*t27-e2_2_1.*t28+2.0.*e2_1_1.*e3_1_1.*e3_2_1+2.0.*e2_1_1.*e3_1_2.*e3_2_2+2.0.*e2_1_2.*e3_1_1.*e3_2_2+-2.0.*e2_1_2.*e3_1_2.*e3_2_1+2.0.*e2_2_2.*e3_1_1.*e3_1_2+2.0.*e2_1_1.*e3_1_3.*e3_2_3+2.0.*e2_1_3.*e3_1_1.*e3_2_3+-2.0.*e2_1_3.*e3_1_3.*e3_2_1+2.0.*e2_2_3.*e3_1_1.*e3_1_3+2.0.*e2_2_2.*e3_2_1.*e3_2_2+2.0.*e2_2_3.*e3_2_1.*e3_2_3+2.0.*e2_3_1.*e3_2_1.*e3_3_1+2.0.*e2_2_2.*e3_3_1.*e3_3_2+2.0.*e2_3_1.*e3_2_2.*e3_3_2+-2.0.*e2_3_2.*e3_2_1.*e3_3_2+2.0.*e2_3_2.*e3_2_2.*e3_3_1+2.0.*e2_2_3.*e3_3_1.*e3_3_3+2.0.*e2_3_1.*e3_2_3.*e3_3_3+-2.0.*e2_3_3.*e3_2_1.*e3_3_3+2.0.*e2_3_3.*e3_2_3.*e3_3_1];
mt117 = [e2_3_1.*t20-e2_3_1.*t21-e2_3_1.*t22+e2_3_1.*t23-e2_3_1.*t24-e2_3_1.*t25+e2_3_1.*t26.*3.0+e2_3_1.*t27+e2_3_1.*t28+2.0.*e2_1_1.*e3_1_1.*e3_3_1+2.0.*e2_1_1.*e3_1_2.*e3_3_2+2.0.*e2_1_2.*e3_1_1.*e3_3_2+-2.0.*e2_1_2.*e3_1_2.*e3_3_1+2.0.*e2_3_2.*e3_1_1.*e3_1_2+2.0.*e2_1_1.*e3_1_3.*e3_3_3+2.0.*e2_1_3.*e3_1_1.*e3_3_3+-2.0.*e2_1_3.*e3_1_3.*e3_3_1+2.0.*e2_3_3.*e3_1_1.*e3_1_3+2.0.*e2_2_1.*e3_2_1.*e3_3_1+2.0.*e2_2_1.*e3_2_2.*e3_3_2+2.0.*e2_2_2.*e3_2_1.*e3_3_2+-2.0.*e2_2_2.*e3_2_2.*e3_3_1+2.0.*e2_3_2.*e3_2_1.*e3_2_2+2.0.*e2_2_1.*e3_2_3.*e3_3_3+2.0.*e2_2_3.*e3_2_1.*e3_3_3+-2.0.*e2_2_3.*e3_2_3.*e3_3_1+2.0.*e2_3_3.*e3_2_1.*e3_2_3+2.0.*e2_3_2.*e3_3_1.*e3_3_2+2.0.*e2_3_3.*e3_3_1.*e3_3_3];
mt118 = [e2_1_2.*t20+e2_1_2.*t21.*3.0+e2_1_2.*t22-e2_1_2.*t23+e2_1_2.*t24-e2_1_2.*t25-e2_1_2.*t26+e2_1_2.*t27-e2_1_2.*t28+2.0.*e2_1_1.*e3_1_1.*e3_1_2+2.0.*e2_1_3.*e3_1_2.*e3_1_3+2.0.*e2_1_1.*e3_2_1.*e3_2_2+2.0.*e2_2_1.*e3_1_1.*e3_2_2+-2.0.*e2_2_1.*e3_1_2.*e3_2_1+2.0.*e2_2_2.*e3_1_1.*e3_2_1+2.0.*e2_2_2.*e3_1_2.*e3_2_2+2.0.*e2_1_3.*e3_2_2.*e3_2_3+2.0.*e2_2_2.*e3_1_3.*e3_2_3+-2.0.*e2_2_3.*e3_1_2.*e3_2_3+2.0.*e2_2_3.*e3_1_3.*e3_2_2+2.0.*e2_1_1.*e3_3_1.*e3_3_2+2.0.*e2_3_1.*e3_1_1.*e3_3_2+-2.0.*e2_3_1.*e3_1_2.*e3_3_1+2.0.*e2_3_2.*e3_1_1.*e3_3_1+2.0.*e2_3_2.*e3_1_2.*e3_3_2+2.0.*e2_1_3.*e3_3_2.*e3_3_3+2.0.*e2_3_2.*e3_1_3.*e3_3_3+-2.0.*e2_3_3.*e3_1_2.*e3_3_3+2.0.*e2_3_3.*e3_1_3.*e3_3_2];
mt119 = [-e2_2_2.*t20+e2_2_2.*t21-e2_2_2.*t22+e2_2_2.*t23+e2_2_2.*t24.*3.0+e2_2_2.*t25-e2_2_2.*t26+e2_2_2.*t27-e2_2_2.*t28+-2.0.*e2_1_1.*e3_1_1.*e3_2_2+2.0.*e2_1_1.*e3_1_2.*e3_2_1+2.0.*e2_1_2.*e3_1_1.*e3_2_1+2.0.*e2_2_1.*e3_1_1.*e3_1_2+2.0.*e2_1_2.*e3_1_2.*e3_2_2+2.0.*e2_1_2.*e3_1_3.*e3_2_3+2.0.*e2_1_3.*e3_1_2.*e3_2_3+-2.0.*e2_1_3.*e3_1_3.*e3_2_2+2.0.*e2_2_3.*e3_1_2.*e3_1_3+2.0.*e2_2_1.*e3_2_1.*e3_2_2+2.0.*e2_2_3.*e3_2_2.*e3_2_3+2.0.*e2_2_1.*e3_3_1.*e3_3_2+2.0.*e2_3_1.*e3_2_1.*e3_3_2+-2.0.*e2_3_1.*e3_2_2.*e3_3_1+2.0.*e2_3_2.*e3_2_1.*e3_3_1+2.0.*e2_3_2.*e3_2_2.*e3_3_2+2.0.*e2_2_3.*e3_3_2.*e3_3_3+2.0.*e2_3_2.*e3_2_3.*e3_3_3+-2.0.*e2_3_3.*e3_2_2.*e3_3_3+2.0.*e2_3_3.*e3_2_3.*e3_3_2];
mt120 = [-e2_3_2.*t20+e2_3_2.*t21-e2_3_2.*t22-e2_3_2.*t23+e2_3_2.*t24-e2_3_2.*t25+e2_3_2.*t26+e2_3_2.*t27.*3.0+e2_3_2.*t28+-2.0.*e2_1_1.*e3_1_1.*e3_3_2+2.0.*e2_1_1.*e3_1_2.*e3_3_1+2.0.*e2_1_2.*e3_1_1.*e3_3_1+2.0.*e2_3_1.*e3_1_1.*e3_1_2+2.0.*e2_1_2.*e3_1_2.*e3_3_2+2.0.*e2_1_2.*e3_1_3.*e3_3_3+2.0.*e2_1_3.*e3_1_2.*e3_3_3+-2.0.*e2_1_3.*e3_1_3.*e3_3_2+2.0.*e2_3_3.*e3_1_2.*e3_1_3+-2.0.*e2_2_1.*e3_2_1.*e3_3_2+2.0.*e2_2_1.*e3_2_2.*e3_3_1+2.0.*e2_2_2.*e3_2_1.*e3_3_1+2.0.*e2_3_1.*e3_2_1.*e3_2_2+2.0.*e2_2_2.*e3_2_2.*e3_3_2+2.0.*e2_2_2.*e3_2_3.*e3_3_3+2.0.*e2_2_3.*e3_2_2.*e3_3_3+-2.0.*e2_2_3.*e3_2_3.*e3_3_2+2.0.*e2_3_3.*e3_2_2.*e3_2_3+2.0.*e2_3_1.*e3_3_1.*e3_3_2+2.0.*e2_3_3.*e3_3_2.*e3_3_3];
mt121 = [e2_1_3.*t20+e2_1_3.*t21+e2_1_3.*t22.*3.0-e2_1_3.*t23-e2_1_3.*t24+e2_1_3.*t25-e2_1_3.*t26-e2_1_3.*t27+e2_1_3.*t28+2.0.*e2_1_1.*e3_1_1.*e3_1_3+2.0.*e2_1_2.*e3_1_2.*e3_1_3+2.0.*e2_1_1.*e3_2_1.*e3_2_3+2.0.*e2_2_1.*e3_1_1.*e3_2_3+-2.0.*e2_2_1.*e3_1_3.*e3_2_1+2.0.*e2_2_3.*e3_1_1.*e3_2_1+2.0.*e2_1_2.*e3_2_2.*e3_2_3+2.0.*e2_2_2.*e3_1_2.*e3_2_3+-2.0.*e2_2_2.*e3_1_3.*e3_2_2+2.0.*e2_2_3.*e3_1_2.*e3_2_2+2.0.*e2_2_3.*e3_1_3.*e3_2_3+2.0.*e2_1_1.*e3_3_1.*e3_3_3+2.0.*e2_3_1.*e3_1_1.*e3_3_3+-2.0.*e2_3_1.*e3_1_3.*e3_3_1+2.0.*e2_3_3.*e3_1_1.*e3_3_1+2.0.*e2_1_2.*e3_3_2.*e3_3_3+2.0.*e2_3_2.*e3_1_2.*e3_3_3+-2.0.*e2_3_2.*e3_1_3.*e3_3_2+2.0.*e2_3_3.*e3_1_2.*e3_3_2+2.0.*e2_3_3.*e3_1_3.*e3_3_3];
mt122 = [-e2_2_3.*t20-e2_2_3.*t21+e2_2_3.*t22+e2_2_3.*t23+e2_2_3.*t24+e2_2_3.*t25.*3.0-e2_2_3.*t26-e2_2_3.*t27+e2_2_3.*t28+-2.0.*e2_1_1.*e3_1_1.*e3_2_3+2.0.*e2_1_1.*e3_1_3.*e3_2_1+2.0.*e2_1_3.*e3_1_1.*e3_2_1+2.0.*e2_2_1.*e3_1_1.*e3_1_3+-2.0.*e2_1_2.*e3_1_2.*e3_2_3+2.0.*e2_1_2.*e3_1_3.*e3_2_2+2.0.*e2_1_3.*e3_1_2.*e3_2_2+2.0.*e2_2_2.*e3_1_2.*e3_1_3+2.0.*e2_1_3.*e3_1_3.*e3_2_3+2.0.*e2_2_1.*e3_2_1.*e3_2_3+2.0.*e2_2_2.*e3_2_2.*e3_2_3+2.0.*e2_2_1.*e3_3_1.*e3_3_3+2.0.*e2_3_1.*e3_2_1.*e3_3_3+-2.0.*e2_3_1.*e3_2_3.*e3_3_1+2.0.*e2_3_3.*e3_2_1.*e3_3_1+2.0.*e2_2_2.*e3_3_2.*e3_3_3+2.0.*e2_3_2.*e3_2_2.*e3_3_3+-2.0.*e2_3_2.*e3_2_3.*e3_3_2+2.0.*e2_3_3.*e3_2_2.*e3_3_2+2.0.*e2_3_3.*e3_2_3.*e3_3_3];
mt123 = [-e2_3_3.*t20-e2_3_3.*t21+e2_3_3.*t22-e2_3_3.*t23-e2_3_3.*t24+e2_3_3.*t25+e2_3_3.*t26+e2_3_3.*t27+e2_3_3.*t28.*3.0+-2.0.*e2_1_1.*e3_1_1.*e3_3_3+2.0.*e2_1_1.*e3_1_3.*e3_3_1+2.0.*e2_1_3.*e3_1_1.*e3_3_1+2.0.*e2_3_1.*e3_1_1.*e3_1_3+-2.0.*e2_1_2.*e3_1_2.*e3_3_3+2.0.*e2_1_2.*e3_1_3.*e3_3_2+2.0.*e2_1_3.*e3_1_2.*e3_3_2+2.0.*e2_3_2.*e3_1_2.*e3_1_3+2.0.*e2_1_3.*e3_1_3.*e3_3_3+-2.0.*e2_2_1.*e3_2_1.*e3_3_3+2.0.*e2_2_1.*e3_2_3.*e3_3_1+2.0.*e2_2_3.*e3_2_1.*e3_3_1+2.0.*e2_3_1.*e3_2_1.*e3_2_3+-2.0.*e2_2_2.*e3_2_2.*e3_3_3+2.0.*e2_2_2.*e3_2_3.*e3_3_2+2.0.*e2_2_3.*e3_2_2.*e3_3_2+2.0.*e2_3_2.*e3_2_2.*e3_2_3+2.0.*e2_2_3.*e3_2_3.*e3_3_3+2.0.*e2_3_1.*e3_3_1.*e3_3_3+2.0.*e2_3_2.*e3_3_2.*e3_3_3];
mt124 = [e3_1_1.*e3_2_2.*e4_3_3-e3_1_1.*e3_2_3.*e4_3_2-e3_1_1.*e3_3_2.*e4_2_3+e3_1_1.*e3_3_3.*e4_2_2-e3_1_2.*e3_2_1.*e4_3_3+e3_1_2.*e3_2_3.*e4_3_1+e3_1_2.*e3_3_1.*e4_2_3-e3_1_2.*e3_3_3.*e4_2_1+e3_1_3.*e3_2_1.*e4_3_2-e3_1_3.*e3_2_2.*e4_3_1-e3_1_3.*e3_3_1.*e4_2_2+e3_1_3.*e3_3_2.*e4_2_1+e3_2_1.*e3_3_2.*e4_1_3-e3_2_1.*e3_3_3.*e4_1_2-e3_2_2.*e3_3_1.*e4_1_3+e3_2_2.*e3_3_3.*e4_1_1+e3_2_3.*e3_3_1.*e4_1_2-e3_2_3.*e3_3_2.*e4_1_1];
mt125 = [e4_1_1.*t20.*3.0+e4_1_1.*t21+e4_1_1.*t22+e4_1_1.*t23-e4_1_1.*t24-e4_1_1.*t25+e4_1_1.*t26-e4_1_1.*t27-e4_1_1.*t28+2.0.*e4_1_2.*e3_1_1.*e3_1_2+2.0.*e4_1_3.*e3_1_1.*e3_1_3+2.0.*e4_2_1.*e3_1_1.*e3_2_1+2.0.*e4_1_2.*e3_2_1.*e3_2_2+2.0.*e4_2_1.*e3_1_2.*e3_2_2+-2.0.*e4_2_2.*e3_1_1.*e3_2_2+2.0.*e4_2_2.*e3_1_2.*e3_2_1+2.0.*e4_1_3.*e3_2_1.*e3_2_3+2.0.*e4_2_1.*e3_1_3.*e3_2_3+-2.0.*e4_2_3.*e3_1_1.*e3_2_3+2.0.*e4_2_3.*e3_1_3.*e3_2_1+2.0.*e4_3_1.*e3_1_1.*e3_3_1+2.0.*e4_1_2.*e3_3_1.*e3_3_2+2.0.*e4_3_1.*e3_1_2.*e3_3_2+-2.0.*e4_3_2.*e3_1_1.*e3_3_2+2.0.*e4_3_2.*e3_1_2.*e3_3_1+2.0.*e4_1_3.*e3_3_1.*e3_3_3+2.0.*e4_3_1.*e3_1_3.*e3_3_3+-2.0.*e4_3_3.*e3_1_1.*e3_3_3+2.0.*e4_3_3.*e3_1_3.*e3_3_1];
mt126 = [e4_2_1.*t20-e4_2_1.*t21-e4_2_1.*t22+e4_2_1.*t23.*3.0+e4_2_1.*t24+e4_2_1.*t25+e4_2_1.*t26-e4_2_1.*t27-e4_2_1.*t28+2.0.*e4_1_1.*e3_1_1.*e3_2_1+2.0.*e4_1_1.*e3_1_2.*e3_2_2+2.0.*e4_1_2.*e3_1_1.*e3_2_2+-2.0.*e4_1_2.*e3_1_2.*e3_2_1+2.0.*e4_2_2.*e3_1_1.*e3_1_2+2.0.*e4_1_1.*e3_1_3.*e3_2_3+2.0.*e4_1_3.*e3_1_1.*e3_2_3+-2.0.*e4_1_3.*e3_1_3.*e3_2_1+2.0.*e4_2_3.*e3_1_1.*e3_1_3+2.0.*e4_2_2.*e3_2_1.*e3_2_2+2.0.*e4_2_3.*e3_2_1.*e3_2_3+2.0.*e4_3_1.*e3_2_1.*e3_3_1+2.0.*e4_2_2.*e3_3_1.*e3_3_2+2.0.*e4_3_1.*e3_2_2.*e3_3_2+-2.0.*e4_3_2.*e3_2_1.*e3_3_2+2.0.*e4_3_2.*e3_2_2.*e3_3_1+2.0.*e4_2_3.*e3_3_1.*e3_3_3+2.0.*e4_3_1.*e3_2_3.*e3_3_3+-2.0.*e4_3_3.*e3_2_1.*e3_3_3+2.0.*e4_3_3.*e3_2_3.*e3_3_1];
mt127 = [e4_3_1.*t20-e4_3_1.*t21-e4_3_1.*t22+e4_3_1.*t23-e4_3_1.*t24-e4_3_1.*t25+e4_3_1.*t26.*3.0+e4_3_1.*t27+e4_3_1.*t28+2.0.*e4_1_1.*e3_1_1.*e3_3_1+2.0.*e4_1_1.*e3_1_2.*e3_3_2+2.0.*e4_1_2.*e3_1_1.*e3_3_2+-2.0.*e4_1_2.*e3_1_2.*e3_3_1+2.0.*e4_3_2.*e3_1_1.*e3_1_2+2.0.*e4_1_1.*e3_1_3.*e3_3_3+2.0.*e4_1_3.*e3_1_1.*e3_3_3+-2.0.*e4_1_3.*e3_1_3.*e3_3_1+2.0.*e4_3_3.*e3_1_1.*e3_1_3+2.0.*e4_2_1.*e3_2_1.*e3_3_1+2.0.*e4_2_1.*e3_2_2.*e3_3_2+2.0.*e4_2_2.*e3_2_1.*e3_3_2+-2.0.*e4_2_2.*e3_2_2.*e3_3_1+2.0.*e4_3_2.*e3_2_1.*e3_2_2+2.0.*e4_2_1.*e3_2_3.*e3_3_3+2.0.*e4_2_3.*e3_2_1.*e3_3_3+-2.0.*e4_2_3.*e3_2_3.*e3_3_1+2.0.*e4_3_3.*e3_2_1.*e3_2_3+2.0.*e4_3_2.*e3_3_1.*e3_3_2+2.0.*e4_3_3.*e3_3_1.*e3_3_3];
mt128 = [e4_1_2.*t20+e4_1_2.*t21.*3.0+e4_1_2.*t22-e4_1_2.*t23+e4_1_2.*t24-e4_1_2.*t25-e4_1_2.*t26+e4_1_2.*t27-e4_1_2.*t28+2.0.*e4_1_1.*e3_1_1.*e3_1_2+2.0.*e4_1_3.*e3_1_2.*e3_1_3+2.0.*e4_1_1.*e3_2_1.*e3_2_2+2.0.*e4_2_1.*e3_1_1.*e3_2_2+-2.0.*e4_2_1.*e3_1_2.*e3_2_1+2.0.*e4_2_2.*e3_1_1.*e3_2_1+2.0.*e4_2_2.*e3_1_2.*e3_2_2+2.0.*e4_1_3.*e3_2_2.*e3_2_3+2.0.*e4_2_2.*e3_1_3.*e3_2_3+-2.0.*e4_2_3.*e3_1_2.*e3_2_3+2.0.*e4_2_3.*e3_1_3.*e3_2_2+2.0.*e4_1_1.*e3_3_1.*e3_3_2+2.0.*e4_3_1.*e3_1_1.*e3_3_2+-2.0.*e4_3_1.*e3_1_2.*e3_3_1+2.0.*e4_3_2.*e3_1_1.*e3_3_1+2.0.*e4_3_2.*e3_1_2.*e3_3_2+2.0.*e4_1_3.*e3_3_2.*e3_3_3+2.0.*e4_3_2.*e3_1_3.*e3_3_3+-2.0.*e4_3_3.*e3_1_2.*e3_3_3+2.0.*e4_3_3.*e3_1_3.*e3_3_2];
mt129 = [-e4_2_2.*t20+e4_2_2.*t21-e4_2_2.*t22+e4_2_2.*t23+e4_2_2.*t24.*3.0+e4_2_2.*t25-e4_2_2.*t26+e4_2_2.*t27-e4_2_2.*t28+-2.0.*e4_1_1.*e3_1_1.*e3_2_2+2.0.*e4_1_1.*e3_1_2.*e3_2_1+2.0.*e4_1_2.*e3_1_1.*e3_2_1+2.0.*e4_2_1.*e3_1_1.*e3_1_2+2.0.*e4_1_2.*e3_1_2.*e3_2_2+2.0.*e4_1_2.*e3_1_3.*e3_2_3+2.0.*e4_1_3.*e3_1_2.*e3_2_3+-2.0.*e4_1_3.*e3_1_3.*e3_2_2+2.0.*e4_2_3.*e3_1_2.*e3_1_3+2.0.*e4_2_1.*e3_2_1.*e3_2_2+2.0.*e4_2_3.*e3_2_2.*e3_2_3+2.0.*e4_2_1.*e3_3_1.*e3_3_2+2.0.*e4_3_1.*e3_2_1.*e3_3_2+-2.0.*e4_3_1.*e3_2_2.*e3_3_1+2.0.*e4_3_2.*e3_2_1.*e3_3_1+2.0.*e4_3_2.*e3_2_2.*e3_3_2+2.0.*e4_2_3.*e3_3_2.*e3_3_3+2.0.*e4_3_2.*e3_2_3.*e3_3_3+-2.0.*e4_3_3.*e3_2_2.*e3_3_3+2.0.*e4_3_3.*e3_2_3.*e3_3_2];
mt130 = [-e4_3_2.*t20+e4_3_2.*t21-e4_3_2.*t22-e4_3_2.*t23+e4_3_2.*t24-e4_3_2.*t25+e4_3_2.*t26+e4_3_2.*t27.*3.0+e4_3_2.*t28+-2.0.*e4_1_1.*e3_1_1.*e3_3_2+2.0.*e4_1_1.*e3_1_2.*e3_3_1+2.0.*e4_1_2.*e3_1_1.*e3_3_1+2.0.*e4_3_1.*e3_1_1.*e3_1_2+2.0.*e4_1_2.*e3_1_2.*e3_3_2+2.0.*e4_1_2.*e3_1_3.*e3_3_3+2.0.*e4_1_3.*e3_1_2.*e3_3_3+-2.0.*e4_1_3.*e3_1_3.*e3_3_2+2.0.*e4_3_3.*e3_1_2.*e3_1_3+-2.0.*e4_2_1.*e3_2_1.*e3_3_2+2.0.*e4_2_1.*e3_2_2.*e3_3_1+2.0.*e4_2_2.*e3_2_1.*e3_3_1+2.0.*e4_3_1.*e3_2_1.*e3_2_2+2.0.*e4_2_2.*e3_2_2.*e3_3_2+2.0.*e4_2_2.*e3_2_3.*e3_3_3+2.0.*e4_2_3.*e3_2_2.*e3_3_3+-2.0.*e4_2_3.*e3_2_3.*e3_3_2+2.0.*e4_3_3.*e3_2_2.*e3_2_3+2.0.*e4_3_1.*e3_3_1.*e3_3_2+2.0.*e4_3_3.*e3_3_2.*e3_3_3];
mt131 = [e4_1_3.*t20+e4_1_3.*t21+e4_1_3.*t22.*3.0-e4_1_3.*t23-e4_1_3.*t24+e4_1_3.*t25-e4_1_3.*t26-e4_1_3.*t27+e4_1_3.*t28+2.0.*e4_1_1.*e3_1_1.*e3_1_3+2.0.*e4_1_2.*e3_1_2.*e3_1_3+2.0.*e4_1_1.*e3_2_1.*e3_2_3+2.0.*e4_2_1.*e3_1_1.*e3_2_3+-2.0.*e4_2_1.*e3_1_3.*e3_2_1+2.0.*e4_2_3.*e3_1_1.*e3_2_1+2.0.*e4_1_2.*e3_2_2.*e3_2_3+2.0.*e4_2_2.*e3_1_2.*e3_2_3+-2.0.*e4_2_2.*e3_1_3.*e3_2_2+2.0.*e4_2_3.*e3_1_2.*e3_2_2+2.0.*e4_2_3.*e3_1_3.*e3_2_3+2.0.*e4_1_1.*e3_3_1.*e3_3_3+2.0.*e4_3_1.*e3_1_1.*e3_3_3+-2.0.*e4_3_1.*e3_1_3.*e3_3_1+2.0.*e4_3_3.*e3_1_1.*e3_3_1+2.0.*e4_1_2.*e3_3_2.*e3_3_3+2.0.*e4_3_2.*e3_1_2.*e3_3_3+-2.0.*e4_3_2.*e3_1_3.*e3_3_2+2.0.*e4_3_3.*e3_1_2.*e3_3_2+2.0.*e4_3_3.*e3_1_3.*e3_3_3];
mt132 = [-e4_2_3.*t20-e4_2_3.*t21+e4_2_3.*t22+e4_2_3.*t23+e4_2_3.*t24+e4_2_3.*t25.*3.0-e4_2_3.*t26-e4_2_3.*t27+e4_2_3.*t28+-2.0.*e4_1_1.*e3_1_1.*e3_2_3+2.0.*e4_1_1.*e3_1_3.*e3_2_1+2.0.*e4_1_3.*e3_1_1.*e3_2_1+2.0.*e4_2_1.*e3_1_1.*e3_1_3+-2.0.*e4_1_2.*e3_1_2.*e3_2_3+2.0.*e4_1_2.*e3_1_3.*e3_2_2+2.0.*e4_1_3.*e3_1_2.*e3_2_2+2.0.*e4_2_2.*e3_1_2.*e3_1_3+2.0.*e4_1_3.*e3_1_3.*e3_2_3+2.0.*e4_2_1.*e3_2_1.*e3_2_3+2.0.*e4_2_2.*e3_2_2.*e3_2_3+2.0.*e4_2_1.*e3_3_1.*e3_3_3+2.0.*e4_3_1.*e3_2_1.*e3_3_3+-2.0.*e4_3_1.*e3_2_3.*e3_3_1+2.0.*e4_3_3.*e3_2_1.*e3_3_1+2.0.*e4_2_2.*e3_3_2.*e3_3_3+2.0.*e4_3_2.*e3_2_2.*e3_3_3+-2.0.*e4_3_2.*e3_2_3.*e3_3_2+2.0.*e4_3_3.*e3_2_2.*e3_3_2+2.0.*e4_3_3.*e3_2_3.*e3_3_3];
mt133 = [-e4_3_3.*t20-e4_3_3.*t21+e4_3_3.*t22-e4_3_3.*t23-e4_3_3.*t24+e4_3_3.*t25+e4_3_3.*t26+e4_3_3.*t27+e4_3_3.*t28.*3.0+-2.0.*e4_1_1.*e3_1_1.*e3_3_3+2.0.*e4_1_1.*e3_1_3.*e3_3_1+2.0.*e4_1_3.*e3_1_1.*e3_3_1+2.0.*e4_3_1.*e3_1_1.*e3_1_3+-2.0.*e4_1_2.*e3_1_2.*e3_3_3+2.0.*e4_1_2.*e3_1_3.*e3_3_2+2.0.*e4_1_3.*e3_1_2.*e3_3_2+2.0.*e4_3_2.*e3_1_2.*e3_1_3+2.0.*e4_1_3.*e3_1_3.*e3_3_3+-2.0.*e4_2_1.*e3_2_1.*e3_3_3+2.0.*e4_2_1.*e3_2_3.*e3_3_1+2.0.*e4_2_3.*e3_2_1.*e3_3_1+2.0.*e4_3_1.*e3_2_1.*e3_2_3+-2.0.*e4_2_2.*e3_2_2.*e3_3_3+2.0.*e4_2_2.*e3_2_3.*e3_3_2+2.0.*e4_2_3.*e3_2_2.*e3_3_2+2.0.*e4_3_2.*e3_2_2.*e3_2_3+2.0.*e4_2_3.*e3_2_3.*e3_3_3+2.0.*e4_3_1.*e3_3_1.*e3_3_3+2.0.*e4_3_2.*e3_3_2.*e3_3_3];
C3 = reshape([mt104,mt105,mt106,mt107,mt108,mt109,mt110,mt111,mt112,mt113,mt114,mt115,mt116,mt117,mt118,mt119,mt120,mt121,mt122,mt123,mt124,mt125,mt126,mt127,mt128,mt129,mt130,mt131,mt132,mt133],10,3);

mt134 = [e3_1_1.*e3_2_2.*e3_3_3-e3_1_1.*e3_2_3.*e3_3_2-e3_1_2.*e3_2_1.*e3_3_3+e3_1_2.*e3_2_3.*e3_3_1+e3_1_3.*e3_2_1.*e3_3_2-e3_1_3.*e3_2_2.*e3_3_1;e3_1_1.*t21+e3_1_1.*t22+e3_1_1.*t23-e3_1_1.*t24-e3_1_1.*t25+e3_1_1.*t26-e3_1_1.*t27-e3_1_1.*t28+e3_1_1.^3+e3_1_2.*e3_2_1.*e3_2_2.*2.0+e3_1_3.*e3_2_1.*e3_2_3.*2.0+e3_1_2.*e3_3_1.*e3_3_2.*2.0+e3_1_3.*e3_3_1.*e3_3_3.*2.0;e3_2_1.*t20-e3_2_1.*t21-e3_2_1.*t22+e3_2_1.*t24+e3_2_1.*t25+e3_2_1.*t26-e3_2_1.*t27-e3_2_1.*t28+e3_2_1.^3+e3_1_1.*e3_1_2.*e3_2_2.*2.0+e3_1_1.*e3_1_3.*e3_2_3.*2.0+e3_2_2.*e3_3_1.*e3_3_2.*2.0+e3_2_3.*e3_3_1.*e3_3_3.*2.0;e3_3_1.*t20-e3_3_1.*t21-e3_3_1.*t22+e3_3_1.*t23-e3_3_1.*t24-e3_3_1.*t25+e3_3_1.*t27+e3_3_1.*t28+e3_3_1.^3+e3_1_1.*e3_1_2.*e3_3_2.*2.0+e3_1_1.*e3_1_3.*e3_3_3.*2.0+e3_2_1.*e3_2_2.*e3_3_2.*2.0+e3_2_1.*e3_2_3.*e3_3_3.*2.0];
mt135 = [e3_1_2.*t20+e3_1_2.*t22-e3_1_2.*t23+e3_1_2.*t24-e3_1_2.*t25-e3_1_2.*t26+e3_1_2.*t27-e3_1_2.*t28+e3_1_2.^3+e3_1_1.*e3_2_1.*e3_2_2.*2.0+e3_1_3.*e3_2_2.*e3_2_3.*2.0+e3_1_1.*e3_3_1.*e3_3_2.*2.0+e3_1_3.*e3_3_2.*e3_3_3.*2.0;-e3_2_2.*t20+e3_2_2.*t21-e3_2_2.*t22+e3_2_2.*t23+e3_2_2.*t25-e3_2_2.*t26+e3_2_2.*t27-e3_2_2.*t28+e3_2_2.^3+e3_1_1.*e3_1_2.*e3_2_1.*2.0+e3_1_2.*e3_1_3.*e3_2_3.*2.0+e3_2_1.*e3_3_1.*e3_3_2.*2.0+e3_2_3.*e3_3_2.*e3_3_3.*2.0;-e3_3_2.*t20+e3_3_2.*t21-e3_3_2.*t22-e3_3_2.*t23+e3_3_2.*t24-e3_3_2.*t25+e3_3_2.*t26+e3_3_2.*t28+e3_3_2.^3+e3_1_1.*e3_1_2.*e3_3_1.*2.0+e3_1_2.*e3_1_3.*e3_3_3.*2.0+e3_2_1.*e3_2_2.*e3_3_1.*2.0+e3_2_2.*e3_2_3.*e3_3_3.*2.0];
mt136 = [e3_1_3.*t20+e3_1_3.*t21-e3_1_3.*t23-e3_1_3.*t24+e3_1_3.*t25-e3_1_3.*t26-e3_1_3.*t27+e3_1_3.*t28+e3_1_3.^3+e3_1_1.*e3_2_1.*e3_2_3.*2.0+e3_1_2.*e3_2_2.*e3_2_3.*2.0+e3_1_1.*e3_3_1.*e3_3_3.*2.0+e3_1_2.*e3_3_2.*e3_3_3.*2.0;-e3_2_3.*t20-e3_2_3.*t21+e3_2_3.*t22+e3_2_3.*t23+e3_2_3.*t24-e3_2_3.*t26-e3_2_3.*t27+e3_2_3.*t28+e3_2_3.^3+e3_1_1.*e3_1_3.*e3_2_1.*2.0+e3_1_2.*e3_1_3.*e3_2_2.*2.0+e3_2_1.*e3_3_1.*e3_3_3.*2.0+e3_2_2.*e3_3_2.*e3_3_3.*2.0;-e3_3_3.*t20-e3_3_3.*t21+e3_3_3.*t22-e3_3_3.*t23-e3_3_3.*t24+e3_3_3.*t25+e3_3_3.*t26+e3_3_3.*t27+e3_3_3.^3+e3_1_1.*e3_1_3.*e3_3_1.*2.0+e3_1_2.*e3_1_3.*e3_3_2.*2.0+e3_2_1.*e3_2_3.*e3_3_1.*2.0+e3_2_2.*e3_2_3.*e3_3_2.*2.0];
C4 = [mt134;mt135;mt136];

C2 = [zeros(10,4, 'like', C2), C2];
C3 = [zeros(10,7, 'like', C3), C3];
C4 = [zeros(10,9, 'like', C4), C4];
end

%--------------------------------------------------------------------------
% Polynomial eigenvalues for 4 matrices.
function Es = polyeig4(A0, A1, A2, A3, E1, E2, E3, E4)

n = size(A0, 1);
p = 3;
np = n*p;
A = eye(np, 'like', A0);
A(1:n, 1:n) = A0;
nB = size(A,1);
B = zeros(nB, 'like', A);
B((n+1):(nB+1):end) = 1;
B(1:n, 1:end) = -[A1, A2, A3];

[X,z] = eig(A,B,'vector');
Em = zeros([3, 3, numel(z)], 'like', A0);
V = zeros(n, p, 'like', A0);
k = 0;
for j = 1:np
    % Skip infs and complex values of z, because they do not correspond to
    % valid essential matrices. In codegen the eigenvalues are always
    % complex. Use a threshold on the imaginary part to determine which
    % ones to use.
    if abs(real(z(j))) < 1e15 && abs(imag(z(j))) < 1e-8
        e = real(z(j));
        V(:) = real(X(:, j));
        e2 = e*e;
        e3 = e2*e;
        R = (A0 + e*A1 + e2*A2 + e3 * A3) * V;
        res = sum(abs(R)) ./ sum(abs(V));  % Normalized residuals.
        [~,ind] = min(res);
        v = V(:,ind);
        v = v([8,9,10])/norm(v,2);  % Eigenvector with unit 2-norm.

        % Compute the essential matrix
        E = v(1)*E1 + v(2)*E2 + (v(3)*e)*E3 + v(3)*E4;

        % Check if it is valid
        if all(isfinite(E(:))) && rank(E) >= 2
            k = k + 1;
            Em(:,:,k) = E;
        end
    end
end

% Convert to cell array, because this is what msac expects.
if isempty(coder.target)
    Es = num2cell(Em(:,:,1:k), [1 2]);
else
    Es = repmat({zeros(3, 'like', Em)}, [1, k]);
    for i = 1:k
        Es{i} = Em(:,:,i);
    end
end
end