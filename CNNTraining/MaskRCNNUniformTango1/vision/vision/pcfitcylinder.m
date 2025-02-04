function [model, inlierIndices, outlierIndices, meanError] = pcfitcylinder(varargin)

% Copyright 2015-2023 The MathWorks, Inc.

%#codegen

narginchk(2, 10);

[ptCloud, ransacParams, sampleIndices, referenceVector, maxAngularDistance] = ...
    vision.internal.ransac.validateAndParseRansacInputs(mfilename, true, varargin{:});

% Use two points with normal to fit a cylinder
sampleSize = 2;
ransacParams.sampleSize = sampleSize;

% For Simulation and ML Coder.
if ~isGPUTarget()

    % Compute normal property if it is not available
    if isempty(ptCloud.Normal)
        % Use 6 neighboring points to estimate a normal vector
        ptCloud.Normal = surfaceNormalImpl(ptCloud, 6);
    end

    % Initialization
    [statusCode, status, pc, validPtCloudIndices] = ...
        vision.internal.ransac.initializeRansacModel(ptCloud, sampleIndices, sampleSize);

    modelParams = zeros(1, 7, 'like', pc.Location);
    inlierIndices = [];
    distances = cast([], 'like', pc.Location);

    % Compute the geometric model parameter with MSAC
    if status == statusCode.NoError

        % The initial model parameters are expressed as [x,y,z, dx,dy,dz, r],
        % where (x,y,z) is a point on the axis, (dx,dy,dz) specifies the
        % direction of the axis, and r is the radius.
        if isempty(referenceVector)
            ransacFuncs.fitFunc = @fitCylinder;
            ransacFuncs.evalFunc = @evalCylinder;
            ransacFuncs.checkFunc = @checkCylinder;

            [isFound, modelParams] = vision.internal.ransac.msac([pc.Location,...
                pc.Normal], ransacParams, ransacFuncs);
        else
            % In Codegen, Type mismatch: function_handle checkCylinder ~=
            % checkOrientedCylinder, and so a different variable
            % 'ransacFuncsOrientedCylinder' is used.
            ransacFuncsOrientedCylinder.fitFunc = @fitCylinder;
            ransacFuncsOrientedCylinder.evalFunc = @evalCylinder;
            % Fit the cylinder with orientation constraint
            ransacFuncsOrientedCylinder.checkFunc = @checkOrientedCylinder;

            denorm = sqrt(dot(referenceVector,referenceVector));
            normAxis = referenceVector./repmat(denorm, size(referenceVector));

            [isFound, modelParams] = vision.internal.ransac.msac(...
                [pc.Location, pc.Normal],ransacParams, ransacFuncsOrientedCylinder,...
                normAxis, maxAngularDistance);

            if isFound
                % Adjust the cylinder axis vector so that its direction matches the
                % referenceVector. This makes the absolute angular distance always
                % smaller than 90 degrees.
                a = min(1, max(-1, dot(normAxis, modelParams(4:6))));
                angle = abs(acos(a));
                if angle > pi/2
                    modelParams(4:6) = -modelParams(4:6);
                end
            end
        end

        if ~isFound
            status = statusCode.NotEnoughInliers;
        else
            % Re-evaluate the best model
            if ~isempty(sampleIndices)
                [pc, validPtCloudIndices] = removeInvalidPoints(ptCloud);
            end
            distances = evalCylinder(modelParams, pc.Location);
            inlierIndices = validPtCloudIndices(distances < ransacParams.maxDistance);

            % Convert to end-points expression
            modelParams = convertToFiniteCylinderModel(modelParams, ptCloud, inlierIndices);
        end
    end

    if(isempty(modelParams))
        modelParams = zeros(1, 7, 'like', pc.Location);
    end

    % Construct the plane object
    model = cylinderModel(modelParams);

    % Report runtime error
    vision.internal.ransac.checkRansacRuntimeStatus(statusCode, status);

    % Extract inliers
    needInlierIndices = (nargout > 1);
    needOutlierIndices = (nargout > 2);
    needMeanError = (nargout > 3);
    if needInlierIndices
        if status ~= statusCode.NoError
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
else
    % GPU specific implementation of pcfitcylinder.
    [model, inlierIndices, outlierIndices, meanError] = ...
        vision.internal.codegen.gpu.pcfitcylinder.pcfitcylinderGpuImpl...
        (ptCloud, ransacParams, sampleIndices, referenceVector,...
        maxAngularDistance);
end

%==========================================================================
% Cylinder equations:
%   axis line: v(p) = p0 + (p - p0)*k
%   point-to-axis distance: r^2 = ||(p2-p1)x(p1-p0)|| / ||p2-p1||
% The cross product of two normals determines the direction of the axis.
%==========================================================================
function model = fitCylinder(points, varargin)
p1 = points(1,1:3);
n1 = points(1,4:6);
p2 = points(2,1:3);
n2 = points(2,4:6);
w = p1 + n1 - p2;

a = sum(n1.*n1);
b = sum(n1.*n2);
c = sum(n2.*n2);
d = sum(n1.*w);
e = sum(n2.*w);
D = a * c - b * b;
if abs(D) < 1e-5
    s = cast(0, 'like', points);
    if b > c
        t = d / b;
    else
        t = e / c;
    end
else
    s = (b * e - c * d) / D;
    t = (a * e - b * d) / D;
end

% P0 is a point on the axis
p0 = p1 + n1 + s * n1;
% dp is a normalized vector for the direction of the axis
dp = p2 + t * n2 - p0;
dp = dp / norm(dp);

p1p0 = p1 - p0;
p2p1 = dp;
c = [p1p0(2)*p2p1(3) - p1p0(3)*p2p1(2), ...
    p1p0(3)*p2p1(1) - p1p0(1)*p2p1(3), ...
    p1p0(1)*p2p1(2) - p1p0(2)*p2p1(1)];
% p2p1 is a unit vector, so the denominator is not needed
r = sqrt(sum(c.*c, 2));

model = [p0, dp, r];

%==========================================================================
% Calculate the distance from the point P0 to the axis P2-P1
% D = ||(P2-P1) x (P1-P0)|| / ||P2-P1||
% The distance from P0 to the cylinder is ||D - r||
%==========================================================================
function dis = evalCylinder(model, points, varargin)
p1p0 = [points(:,1)-model(1), points(:,2)-model(2), points(:,3)-model(3)];
p2p1 = model(4:6);
c = [p1p0(:,2)*p2p1(3) - p1p0(:,3)*p2p1(2), ...
    p1p0(:,3)*p2p1(1) - p1p0(:,1)*p2p1(3), ...
    p1p0(:,1)*p2p1(2) - p1p0(:,2)*p2p1(1)];
% p2p1 is a unit vector, so the denominator is not needed
D = sum(c.*c, 2);
dis = abs(sqrt(D) - model(7));

%==========================================================================
% Validate the cylinder coefficients
%==========================================================================
function isValid = checkCylinder(model)
isValid = (numel(model) == 7 & all(isfinite(model(:))));

%==========================================================================
% Validate the cylinder coefficients with orientation constraints
%==========================================================================
function isValid = checkOrientedCylinder(model, normAxis, threshold)
isValid = checkCylinder(model);
if isValid
    a = min(1, max(-1, (normAxis(1)*model(4) + normAxis(2)*model(5) + normAxis(3)*model(6)) ));
    angle = abs(acos(a));
    angle = min(angle, pi-angle);
    isValid = (angle < threshold);
end

%==========================================================================
% Convert the point-axis format to end-points format
%
% The MSAC algorithm estimates a point on the axis and the direction of
% axis. This function projects all inliers to the axis, computes two end
% points, and use these two end-points along with the radius to describe a
% finite cylinder.
%==========================================================================
function modelParams = convertToFiniteCylinderModel(modelParams, ptCloud, inlierIndices)
% Get the direction of cylinder axis
dp = modelParams(4:6);
% Get the inlier points
points = subsetImpl(ptCloud, inlierIndices);
% Get a point on the axis
p0 = modelParams(1:3);
% Describe the axis as a line equation: p0 + k * dp, and find the
% projections of inlier points on this line
k = sum(points.*repmat(dp, numel(inlierIndices), 1),2) - p0 * dp';
% Find the two extreme points
pa = p0 + min(k) * dp;
pb = p0 + max(k) * dp;
% Set to the parameters required by cylinderModel object
modelParams(1:6) = [pa, pb];

% =========================================================================
% GPU codegen support flag
% =========================================================================
function flag = isGPUTarget()
flag = coder.gpu.internal.isGpuEnabled;