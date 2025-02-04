function [tform, movingReg, rmse] = pcregisterndt(moving, fixed, gridStep, varargin)
%

% Copyright 2017-2024 The MathWorks, Inc.

%#codegen

isSimMode = isempty(coder.target);

% Parse the inputs
[ptCloudA, ptCloudB, maxStepSize, outlierRatio, maxIterations, tolerance, ...
    initTform, verbose] = parseInputs( ...
    isSimMode, moving, fixed, gridStep, varargin{:});
gridStep = double(gridStep);

% Voxelize the reference point cloud
minVoxelPoints  = 6;         % Remove voxels that contain less than 6 points
eigenValueRatio = 100;      % Flat the covariance to avoid singularity

if ~isGPUCodegen()
    [fixedVoxelMeans, fixedVoxelICov] = vision.internal.pc.voxelGridFilter(...
        ptCloudB.Location, gridStep, minVoxelPoints, eigenValueRatio);

    if isempty(fixedVoxelMeans)
        coder.internal.error('vision:pointcloud:notEnoughValidVoxels');
    end

    % Create kdtree for fast indexing
    [fixedSearchTree, pLocationHandle] = constructKdtree(isSimMode, fixedVoxelMeans);

    movingPoints = ptCloudA.Location;

    tform = vision.internal.ndt.register(movingPoints, ...
        fixedVoxelMeans, fixedVoxelICov, fixedSearchTree, ...
        initTform, gridStep, outlierRatio, maxIterations, tolerance, verbose, ...
        maxStepSize, class(moving.Location));

    if nargout >= 2
        movingReg = pctransform(moving, tform);
    end

    if nargout >= 3
        % rmse is based on point-to-point correspondence
        rmse = cast(...
            vision.internal.pc.rmse(ptCloudA, ptCloudB, tform), ...
            'like', moving.Location);
    end
    deleteKdtree(isSimMode, fixedSearchTree, pLocationHandle,  class(fixedVoxelMeans));

else
    % GPU implementation for NDT registration
    coder.inline('never');
    coder.gpu.kernelfun;

    ptCloudAPoints = ptCloudA.Location;
    ptCloudBPoints = ptCloudB.Location;

    % GPU Code implementation for voxel means and inverse voxel covariance
    % computation.
    [fixedVoxelMeans, fixedVoxelICov] = ...
        vision.internal.codegen.gpu.pcregisterndt.computeVoxelEigenCov(...
        ptCloudBPoints, gridStep, minVoxelPoints, eigenValueRatio);

    tform = vision.internal.ndt.register(ptCloudAPoints, ...
        fixedVoxelMeans, fixedVoxelICov, [], initTform, gridStep, ...
        outlierRatio, maxIterations, tolerance, verbose, maxStepSize, ...
        class(moving.Location));

    if nargout >= 2
        % GPU implementation to transform the moving point cloud points
        % based on the tform generated through GPU codegen.
        transMat = tform.Translation;
        rotMat = tform.Rotation;
        transformedPoints = vision.internal.codegen.gpu.pcregisterndt.transformPoints(...
            ptCloudAPoints, rotMat, transMat);
        movingReg = pointCloud(transformedPoints,"Color",moving.Color,...
            "Intensity",moving.Intensity);
    end

    if nargout >= 3
        % rmse is based on point-to-point correspondence
        % GPU implementation for root mean square error computation. The
        % points are transformed and the mean of the minimum euclidean
        % distance is computed.
        rmseValue = vision.internal.codegen.gpu.pcregisterndt.rmseGPUComputation(...
            ptCloudBPoints,transformedPoints);
        rmse = cast(rmseValue,'like', moving.Location);
    end
end
end
%--------------------------------------------------------------------------
function [ptCloudA, ptCloudB, maxStepSize, outlierRatio, maxIterations, ...
    tolerance, initTform, verbose] = parseInputs(...
    isSimMode, moving, fixed, gridStep, varargin)

coder.internal.prefer_const( varargin{:} );

validateattributes(moving, {'pointCloud'}, {'scalar'}, mfilename, 'moving');
validateattributes(fixed, {'pointCloud'}, {'scalar'}, mfilename, 'fixed');
validateattributes(gridStep, {'single', 'double'}, {'real','scalar', 'nonnan', 'nonsparse','positive'});

if ~isGPUCodegen()
    % A copy of the input with unorganized M-by-3 data
    if isa(moving.Location, 'double')
        ptCloudA = removeInvalidPoints(moving);
    else
        movingUnorg = removeInvalidPoints(moving);
        ptCloudA = pointCloud( double(movingUnorg.Location) );
    end

    if isa(fixed.Location, 'double')
        ptCloudB = removeInvalidPoints(fixed);
    else
        fixedUnorg = removeInvalidPoints(fixed);
        ptCloudB = pointCloud(double(fixedUnorg.Location));
    end

    % At least three points are needed to determine a 3-D transformation
    if ptCloudA.Count<3 || ptCloudB.Count<3
        coder.internal.error('vision:pointcloud:notEnoughPoints');
    end

    if isSimMode
        args = vision.internal.ndt.parseOptionsSim(varargin{:});

        maxStepSize     = args.StepSize;
        outlierRatio    = args.OutlierRatio;
        maxIterations   = args.MaxIterations;
        tolerance       = args.Tolerance;
        verbose         = args.Verbose;

        if isempty(args.InitialTransform)
            t = mean(ptCloudB.Location) - mean(ptCloudA.Location);
            initialTransform = rigidtform3d(eye(3), t);
        else
            initialTransform = args.InitialTransform;
        end
    else
        t = mean(ptCloudB.Location) - mean(ptCloudA.Location);
        [maxStepSize, outlierRatio, maxIterations, tolerance, initialTransform, verbose] = ...
            vision.internal.ndt.parseOptionsCodegen(true, t, varargin{:});
    end

    if isa(initialTransform, 'affine3d')
        % Convert to rigidtform3d and extract transformation matrix
        initRigidTform = rigidtform3d(initialTransform.T');
        initTform = double(initRigidTform.A);
    else
        initTform = double(initialTransform.T');
    end

    outlierRatio  = double(outlierRatio);
    tolerance     = [double(tolerance(1)), double(tolerance(2))*pi/180];
    maxIterations = double(maxIterations);
else
    % This parsing is specific to GPU implementation. This parsing flow is
    % similar to that of simulation/C codegen but minimizes packing and
    % unpacking of points into pointCloud class structure to avoid
    % unnecessary memory copies between CPU & GPU.
    [ptCloudAPoints, ptCloudBPoints, maxStepSize, outlierRatio, maxIterations, ...
        tolerance, initTform] = ...
        vision.internal.codegen.gpu.pcregisterndt.parserGPUImpl(...
        moving.Location, fixed.Location, gridStep, varargin{:});
    ptCloudA = pointCloud(ptCloudAPoints);
    ptCloudB = pointCloud(ptCloudBPoints);
    verbose = false;
end

end

%--------------------------------------------------------------------------
function [kdtree, pLocationHandle] = constructKdtree(isSimMode, mvals)
% Create kdtree for fast indexing

options = struct('bucketSize', 10);
if isSimMode
    kdtree = vision.internal.Kdtree();
    kdtree.index(mvals, options);
    pLocationHandle = [];
elseif coder.internal.preferMATLABHostCompiledLibraries()
    kdtree = vision.internal.buildable.kdtreeBuildable.kdtreeConstruct(class(mvals));
    if ismatrix(mvals)
        numPts = size(mvals,1);
        numDims = size(mvals,2);
    else
        numPts = size(mvals,1) * size(mvals,2);
        numDims = size(mvals,3);
    end
    pLocationHandle = vision.internal.buildable.kdtreeBuildable.kdtreeGetLocationPointer(mvals, class(mvals));
    vision.internal.buildable.kdtreeBuildable.kdtreeIndex(kdtree, class(mvals), pLocationHandle, numPts, numDims, options);
else
    kdtree = vision.internal.buildable.kdtreeBuildablePortable.kdtreeConstruct(class(mvals));
    if ismatrix(mvals)
        numPts = size(mvals,1);
        numDims = size(mvals,2);
    else
        numPts = size(mvals,1) * size(mvals,2);
        numDims = size(mvals,3);
    end
    pLocationHandle = vision.internal.buildable.kdtreeBuildablePortable.kdtreeGetLocationPointer(mvals, class(mvals));
    vision.internal.buildable.kdtreeBuildablePortable.kdtreeIndex(kdtree, class(mvals), pLocationHandle, numPts, numDims, options);
end

end

%--------------------------------------------------------------------------
function deleteKdtree(isSimMode, kdtree, pLocationHandle, pointCloudDataType)
if ~isSimMode
    if coder.internal.preferMATLABHostCompiledLibraries() % shared lib code
        vision.internal.buildable.kdtreeBuildable.kdtreeDeleteLocationPointer(...
            pLocationHandle, pointCloudDataType);
        vision.internal.buildable.kdtreeBuildable.kdtreeDelete(kdtree, ...
            pointCloudDataType);
    else % portable code
        vision.internal.buildable.kdtreeBuildablePortable.kdtreeDeleteLocationPointer(...
            pLocationHandle, pointCloudDataType);
        vision.internal.buildable.kdtreeBuildablePortable.kdtreeDelete(kdtree, ...
            pointCloudDataType);
    end
end
end

%--------------------------------------------------------------------------
% GPU Codegen flag
function flag = isGPUCodegen()
flag = coder.gpu.internal.isGpuEnabled;
end
