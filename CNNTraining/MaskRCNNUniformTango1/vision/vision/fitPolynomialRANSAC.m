function [params, inliers] = fitPolynomialRANSAC(varargin) %#codegen
%fitPolynomialRANSAC Fit polynomial to [x,y] data using RANSAC.
%   P = fitPolynomialRANSAC(xyPoints, N, maxDistance) uses the M-estimator
%   SAmple Consensus (MSAC), a version of RANSAC, to find the coefficients
%   of a polynomial P(X) of degree N that best fits the data Y, while
%   rejecting outlier points. If a fit cannot be found, returned P is empty.
%   xyPoints is an M-by-2 matrix of [x,y] point coordinates. maxDistance is
%   maximum allowed distance along the Y-axis from an inlier point to
%   the estimated polynomial fit. P is a row vector of length N+1 containing
%   the polynomial coefficients in descending powers,
%   P(1)*X^N + P(2)*X^(N-1) +...+ P(N)*X + P(N+1).
%
%   [P, inlierIdx] = fitPolynomialRANSAC(...) additionally returns an
%   M-by-1 logical array specifying which data points are inliers.
%
%   [...] = fitPolynomialRANSAC(..., Name, Value) specifies additional
%   name-value pair arguments described below:
%
%   'MaxNumTrials'          A positive integer scalar specifying the
%                           maximum number of random trials for finding the
%                           inliers. Increasing this value will improve the
%                           robustness of the output at the expense of
%                           additional computation.
%
%                           Default: 1000
%
%   'Confidence'            A numeric scalar, C, 0 < C < 100, specifying
%                           the desired confidence (in percentage) for
%                           finding the maximum number of inliers.
%                           Increasing this value will improve the
%                           robustness of the output at the expense of
%                           additional computation.
%
%                           Default: 99
%
%   'ValidatePolynomialFcn' Handle to a function, which returns true if the
%                           polynomial is accepted as valid and false
%                           otherwise. This function can be used to reject
%                           some of the polynomial fits. The function must
%                           be of the form,
%
%                               isValid = validatePolynomialFcn(P)
%
%                           Default: all found polynomials are assumed to
%                                    be valid
%
%   'MaxSamplingAttempts'   Positive integer scalar specifying the maximum
%                           number of attempts to find a sample, which
%                           yields a valid polynomial.
%
%                           Default: 100
%
%   Class Support
%   -------------
%   xyPoints must be numeric. P is double if xyPoints are double, otherwise
%   it's single. inlierIdx is logical.
%
%   Example: Fit a parabola to noisy data
%   -------------------------------------
%   % Construct a parabola
%   x = (-10:0.1:10)';
%   y = (36-x.^2)/9;
%
%   % Add noise and a few outliers to the original data
%   y = y + rand(size(y));
%   y([50,150,99,199]) = [y(50)+12, y(150)-12, y(99)+33, y(199)-23];
%
%   % Use RANSAC to recover the curve parameters
%   N = 2;             % second degree polynomial
%   maxDistance = 0.6; % maximum allowed distance for a point to be inlier
%
%   [P, inlierIdx] = fitPolynomialRANSAC([x,y], N, maxDistance);
%
%   % Plot the noisy curve with outliers
%   figure
%   plot(x(inlierIdx), y(inlierIdx), '+', x(~inlierIdx), y(~inlierIdx), 'ro');
%
%   % Display the curve that we found using RANSAC
%   yRecoveredCurve = polyval(P,x);
%
%   hold on
%   plot(x,yRecoveredCurve)
%   legend('inliers', 'outliers')
%
% See also ransac, polyfit, polyval

% Copyright 2016-2019 MathWorks, Inc.

if isempty(coder.target) && nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end

[xyPoints, N, ransacParams, validatePolynomialFcn] = parseInputs(varargin{:});

% Verify that we have sufficient number of input points, otherwise we can't
% proceed with fitting
nPts = size(xyPoints, 1);

if nPts < N + 1
    
    params = zeros(0,N+1,'like',xyPoints);
    inliers = false(size(xyPoints,1), 1);
else
    
    % Set up function handles to the core fitting routines
    ransacFuncs.checkFunc = validatePolynomialFcn;
    ransacFuncs.fitFunc   = @(pts)vision.internal.leastSquaresPolynomialFit(pts,N);
    ransacFuncs.evalFunc  = @evaluateFit;
    
    [isFound, params, inliers] = ...
        vision.internal.ransac.msac(xyPoints, ransacParams, ransacFuncs);
    
    if ~isFound % Fitting failed return empty outputs
        params  = zeros(0,N+1,'like',xyPoints);
        inliers = false(size(xyPoints,1), 1);
    end
end



%==========================================================================
    function dis = evaluateFit(P, pts)
        
        x=pts(:,1);
        y=pts(:,2);
        
        f = polyval(P,x);
        dis = abs(y-f);
    end

end

%==========================================================================
function isGood = validatePolynomialFit(~)

isGood = true;

end

%==========================================================================
% Parse and check inputs
%==========================================================================
function [xyPoints, N, ransacParams, validatePolynomialFcn] = parseInputs(varargin)

narginchk(3,Inf);

if isa(varargin{1} , 'double')
    xyPoints = varargin{1};
else
    xyPoints = single(varargin{1});
end

N = varargin{2};
maxDistance = varargin{3};

% Set defaults
defaults = struct(...
    'MaxNumTrials', 1000,...
    'Confidence', 99, ...
    'ValidatePolynomialFcn', @validatePolynomialFit, ....
    'MaxSamplingAttempts', 100);

if isempty(coder.target)
    % Instantiate an input parser
    parser = inputParser;
    parser.FunctionName = mfilename;
    
    % Specify the optional parameters
    parser.addParameter('MaxNumTrials'         , defaults.MaxNumTrials);
    parser.addParameter('Confidence'           , defaults.Confidence);
    parser.addParameter('ValidatePolynomialFcn', defaults.ValidatePolynomialFcn);
    parser.addParameter('MaxSamplingAttempts'  , defaults.MaxSamplingAttempts);
    
    % Parse and check optional parameters
    parser.parse(varargin{4:end});
    r = parser.Results;
    
    maxNumTrials = r.MaxNumTrials;
    confidence   = r.Confidence;
    
    validatePolynomialFcn  = r.ValidatePolynomialFcn;
    maxSamplingAttempts    = r.MaxSamplingAttempts;
    
else
    
    parms = struct( ...
        'MaxNumTrials', uint32(0), ...
        'Confidence',  uint32(0), ...
        'ValidatePolynomialFcn', uint32(0), ...
        'MaxSamplingAttempts',  uint32(0));
    
    poptions = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand', true, ...
        'PartialMatching', false);
    
    pstruct = coder.internal.parseParameterInputs(parms, poptions, varargin{4:end});
    
    maxNumTrials = coder.internal.getParameterValue(pstruct.MaxNumTrials, defaults.MaxNumTrials, varargin{4:end});
    confidence = coder.internal.getParameterValue(pstruct.Confidence, defaults.Confidence, varargin{4:end});
    validatePolynomialFcn = coder.internal.getParameterValue(pstruct.ValidatePolynomialFcn, defaults.ValidatePolynomialFcn, ...
        varargin{4:end});
    
    maxSamplingAttempts =  coder.internal.getParameterValue(pstruct.MaxSamplingAttempts, defaults.MaxSamplingAttempts,...
        varargin{4:end});
    
end

% Check required parameters
checkPointsAndN(xyPoints, N);

% Check optional parameters
checkMaxNumTrials(maxNumTrials);
checkConfidence(confidence);
checkMaxDistance(maxDistance);
checkValidatePolynomialFcn(validatePolynomialFcn);
checkMaxSamplingAttempts(maxSamplingAttempts);

classToUse = getClassToUse(xyPoints);

ransacParams.maxNumTrials  = int32(maxNumTrials);
ransacParams.confidence    = cast(confidence,  classToUse);
ransacParams.maxDistance   = cast(maxDistance, classToUse);
ransacParams.sampleSize    = cast(N+1, classToUse);
ransacParams.maxSkipTrials = maxSamplingAttempts;

ransacParams.recomputeModelFromInliers = false;

end

%==========================================================================
function checkPointsAndN(xyPoints, N)

validateattributes(N, {'numeric'}, {'real', 'scalar', '>=', 1, 'finite'}, ...
    mfilename);

validateattributes(xyPoints, {'numeric'}, {'2d', 'nonsparse', 'real', ...
    'size', [NaN 2]}, mfilename);

end

%==========================================================================
function checkMaxNumTrials(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'integer', 'positive', 'finite'},...
    mfilename, 'MaxNumTrials');
end

%==========================================================================
function checkConfidence(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'positive', 'finite', '<', 100},...
    mfilename, 'Confidence');
end

%==========================================================================
function checkMaxDistance(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'positive', 'finite'},...
    mfilename, 'MaxDistance');
end

%==========================================================================
function checkValidatePolynomialFcn(value)
validateattributes(value, {'function_handle'}, ...
    {'scalar'}, mfilename, 'ValidatePolynomialFcn');
end

%==========================================================================
function checkMaxSamplingAttempts(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'integer', 'positive'}, mfilename, ...
    'MaxSamplingAttempts');
end

%==========================================================================
function c = getClassToUse(xyPoints)
if isa(xyPoints, 'double')
    c = 'double';
else
    c = 'single';
end

end
