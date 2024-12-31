function [model, inlierIndices, outlierIndices, meanError] = pcfitsphere(varargin)

% Copyright 2015-2023 The MathWorks, Inc.

%#codegen

narginchk(2, 8);

[ptCloud, ransacParams, sampleIndices] = ...
    vision.internal.ransac.validateAndParseRansacInputs(mfilename, false,...
    varargin{:});

% Use four points to fit a sphere
sampleSize = 4;

% Initialization
[statusCode, status, pc, validPtCloudIndices] = ...
    vision.internal.ransac.initializeRansacModel(ptCloud, sampleIndices, ...
    sampleSize);

ransacParams.sampleSize = sampleSize;

modelParams = zeros(1, 4, 'like', pc.Location);
distances = cast([], 'like', pc.Location);

% Compute the geometric model parameter with MSAC
if status == statusCode.NoError
    ransacFuncs.fitFunc = @fitSphere;
    ransacFuncs.evalFunc = @evalSphere;
    ransacFuncs.checkFunc = @checkSphere;
    
    [isFound, modelParams] = vision.internal.ransac.msac(pc.Location, ...
        ransacParams, ransacFuncs);
    
    if ~isFound
        status = statusCode.NotEnoughInliers;
    end
end

if(isempty(modelParams))
    modelParams = zeros(1, 4, 'like', pc.Location);
end

% Construct the plane object
model = sphereModel(modelParams);

% Report runtime error
vision.internal.ransac.checkRansacRuntimeStatus(statusCode, status);

% Extract inliers
needInlierIndices = (nargout > 1);
needOutlierIndices = (nargout > 2);
needMeanError = (nargout > 3);
if needInlierIndices
    if status == statusCode.NoError
        % Re-evaluate the best model
        if ~isempty(sampleIndices)
            [pc, validPtCloudIndices] = removeInvalidPoints(ptCloud);
        end
        distances = evalSphere(modelParams, pc.Location);
        inlierIndices = ...
            validPtCloudIndices(distances < ransacParams.maxDistance);
    else
        inlierIndices = [];
    end
end
% Extract outliers
if needOutlierIndices
    if status == statusCode.NoError
        flag = true(ptCloud.Count, 1);
        flag(inlierIndices) = false;
        outlierIndices = find(flag);
    else
        outlierIndices = [];
    end
end
% Report MeanError
if needMeanError
    if status == statusCode.NoError
        meanError = mean(distances(distances < ransacParams.maxDistance));
    else
        meanError = cast([], 'like', pc.Location);
    end
end

%==========================================================================
% Sphere equation: (x-a)^2 + (y-b)^2 + (z-c)^2 = d^2;
%==========================================================================
function model = fitSphere(points)
X = [points, ones(size(points,1),1)];
m11 = det(X);
if abs(m11)<=eps(class(points))
    model = cast([], 'like', points);
    return;
end

X(:,1) = points(:,1).^2+points(:,2).^2+points(:,3).^2;
m12 = det(X);

X(:,2) = X(:,1);
X(:,1) = points(:,1);
m13 = det(X);

X(:,3) = X(:,2);
X(:,2) = points(:,2);
m14 = det(X);

X(:,1) = X(:,3);
X(:,2:4) = points;
m15 = det(X);

a =  0.5*m12/m11;
b =  0.5*m13/m11;
c =  0.5*m14/m11;
d = sqrt(a^2+b^2+c^2-m15/m11);
model = cast([a, b, c, d], 'like', points);

%==========================================================================
% Calculate the distance from the point to the sphere.
% D = abs(sqrt((a-x)^2 + (b-y)^2 + (c-z)^2)-d)
%==========================================================================
function dis = evalSphere(model, points)
dis = abs(sqrt((points(:,1)-model(1)).^2 + (points(:,2)-model(2)).^2 + ...
    (points(:,3)-model(3)).^2) - model(4));

%==========================================================================
% Validate the sphere coefficients
%==========================================================================
function isValid = checkSphere(model)
isValid = (numel(model) == 4 & all(isfinite(model(:))));
