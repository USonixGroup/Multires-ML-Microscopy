function [ptCloudA, ptCloudB, options, doExtrapolate, metric] = parseICPInputs(moving, fixed, fcnName, varargin)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

validateattributes(moving, {'pointCloud'}, {'scalar'}, fcnName, 'moving');
validateattributes(fixed,  {'pointCloud'}, {'scalar'}, fcnName, 'fixed');

if ~isSimMode
    checkPtCloudsSimiliarity(moving,fixed);
end

[ptCloudA, ptCloudB, validIdxA, validIdxB] = preparePointClouds(moving, fixed);

if isSimMode
    [metric, doExtrapolate, inlierRatio, inlierDistance, maxIterations, ...
     tolerance, initTform, verbose, useDegree] = vision.internal.pc.parseICPOptionsSim(fcnName, varargin{:});

    if isempty(initTform)
        if strcmp(fcnName, 'pcregistericp')
            if strcmp(metric,'pointToPlaneWithColor')
                initTform = rigidtform3d;
            else
                t = mean(ptCloudB.Location) - mean(ptCloudA.Location);
                initTform = rigidtform3d([0 0 0], t);
            end
        else
            initTform = rigidtform3d;
        end
    end
else
    [metric, doExtrapolate, inlierRatio, inlierDistance, maxIterations, ...
     tolerance, initTform, verbose, useDegree] = vision.internal.codegen.pc.parseICPOptionsCG(ptCloudA.Location, ...
        ptCloudB.Location, varargin{:});
end

% Convert from degree to radian internally
if useDegree
    tolerance(2) = tolerance(2)*pi/180;
end

if isa(initTform, 'affine3d') || isa(initTform, 'rigid3d')
    initTformParsed = rigidtform3d(initTform.T');
else
    initTformParsed = initTform;
end

options.metric          = string(metric);
options.doExtrapolate   = logical(doExtrapolate);
options.inlierRatio     = double(inlierRatio);
options.inlierDistance  = double(inlierDistance);
options.maxIterations   = double(maxIterations);
options.tolerance       = double(tolerance);
options.initTform       = initTformParsed;
options.verbose         = logical(verbose);
options.useDegree       = logical(useDegree);

if strcmp(fcnName, 'pcregistericp') && (strcmpi(metric, "pointToPlaneWithColor") || strcmpi(metric, "planeToPlaneWithColor"))
    hasColors = ~isempty(moving.Color) && ~isempty(fixed.Color);
    coder.internal.errorIf(~hasColors, 'vision:pointcloud:pointCloudWithoutColor',metric);
end

% Get point cloud normals.
switch options.metric
    case {"pointToPlane", "pointToPlaneWithColor"}
        K = 6; % number of points used for local plane fitting.
        ptCloudB = populateNormals(fixed, K, ptCloudB, validIdxB);

    case {"planeToPlane", "planeToPlaneWithColor"}
        K = 20; % number of points used for local plane fitting.
        ptCloudA = populateNormals(moving, K, ptCloudA, validIdxA);
        ptCloudB = populateNormals(fixed , K, ptCloudB, validIdxB);
end

end

%--------------------------------------------------------------------------
function [ptCloudA, ptCloudB, validIdxA, validIdxB] = preparePointClouds(moving, fixed)
% Remove invalid points

% Unorganized M-by-3 data
[ptCloudA, validIdxA] = removeInvalidPoints(moving);
[ptCloudB, validIdxB] = removeInvalidPoints(fixed);

% At least three points are needed to determine a 3-D transformation
coder.internal.errorIf(ptCloudA.Count < 3 || ptCloudB.Count < 3, ...
    'vision:pointcloud:notEnoughPoints');
end

%--------------------------------------------------------------------------
function ptCloudOut = populateNormals(ptCloudIn, K, ptCloudOut, validPtCloudIndices)

    % Check if the point cloud already includes Normal data. Check the
    % ptCloudOut since both input point clouds can be the same point cloud
    % and it is a handle object.
    if isempty(ptCloudOut.Normal)
        ptCloudInCount = ptCloudIn.Count;

        ptCloudIn.Normal = surfaceNormalImpl(ptCloudIn, K);
        ptCloudOut.Normal = [ptCloudIn.Normal(validPtCloudIndices), ...
            ptCloudIn.Normal(validPtCloudIndices + ptCloudInCount), ...
            ptCloudIn.Normal(validPtCloudIndices + ptCloudInCount * 2)];
    end

    % Remove points if their normals are invalid
    validIndices = all(isfinite(ptCloudOut.Normal), 2);
    if nnz(validIndices) < ptCloudOut.Count
        [loc, c, nv] = subsetImpl(ptCloudOut, validIndices);
        ptCloudOut = pointCloud(loc, 'Normal', nv, 'Color', c);
        coder.internal.errorIf(ptCloudOut.Count < 3, ...
            'vision:pointcloud:notEnoughPoints');
    end
end

%--------------------------------------------------------------------------
function tf = isSimMode
tf = isempty(coder.target);
end
