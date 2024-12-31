classdef pcmapndt < vision.internal.pcmap.Submappable
    
    % Copyright 2020-2024 The MathWorks, Inc.

    %#codegen

    properties (SetAccess = private)
        SelectedSubmap
        VoxelSize
        XLimits
        YLimits
        ZLimits
        VoxelMean
        VoxelCovariance
        VoxelNumPoints
    end

    properties (Access = {?matlab.unittest.TestCase})
        %VoxelInverseCovariance
        %   3-by-3-by-M matrix of voxel inverse covariances, where M is the
        %   number of voxels.
        VoxelInverseCovariance

        %SubmapVoxelIndices
        %   M-by-1 logical vector of indices.
        SubmapVoxelIndices

        %SubmapKdtree
        %   Kdtree indexed with voxel means from selected submap.
        SubmapKdtree

        %SubmapMean
        %   Voxel means for all voxels belonging to the selected submap.
        SubmapMean

        %SubmapInverseCovariance
        %   Voxel inverse covariance for all voxels belonging to the
        %   selected submap.
        SubmapInverseCovariance
    end

    properties (Constant, Access = private)
        Version = 1.0;
    end

    properties (Access = private, Transient)
        IsSimulation

        SubmapMeanHandleCodegen

        % GPU Target check flag.
        IsGPUTarget
    end

    methods
        %------------------------------------------------------------------
        function ndtMap = pcmapndt(ptCloudMap, voxelSize)

            narginchk(2,2);

            validateattributes(ptCloudMap, {'pointCloud'}, {'scalar'}, ...
                'pcmapndt', 'ptCloudMap');

            validateattributes(voxelSize, {'single', 'double'}, ...
                {'scalar', 'positive', 'real', 'finite'}, 'pcmapndt', ...
                'voxelSize');

            ndtMap.IsSimulation = isempty(coder.target);

            % Store whether GPU is enabled.
            ndtMap.IsGPUTarget = coder.gpu.internal.isGpuEnabled;

            ndtMap.VoxelSize = double(voxelSize);

            % If Simulation or ML Coder
            if ~ndtMap.IsGPUTarget
                % Voxelize point cloud map
                [ndtMap.VoxelMean, ndtMap.VoxelCovariance, ...
                    ndtMap.VoxelInverseCovariance, ndtMap.VoxelNumPoints] = ...
                    voxelize(ndtMap, ptCloudMap);

                lo = double(min(ndtMap.VoxelMean, [], 1));
                hi = double(max(ndtMap.VoxelMean, [], 1));
                ndtMap.XLimits = [lo(1), hi(1)];
                ndtMap.YLimits = [lo(2), hi(2)];
                ndtMap.ZLimits = [lo(3), hi(3)];

                ndtMap.SelectedSubmap = [lo(1) hi(1) lo(2) hi(2) lo(3) hi(3)];

                ndtMap.SubmapVoxelIndices = true(size(ndtMap.VoxelNumPoints));

                % Construct kdtree
                if ndtMap.IsSimulation
                    ndtMap.SubmapKdtree = vision.internal.Kdtree();
                else
                    ndtMap.SubmapKdtree = iConstructKdtreeCodegen();
                end

                % If GPU Coder is enabled.
            else

                % GPU specific call to pcmapndt object creation.
                ndtMap = pcmapndtGpuImpl(ndtMap, ptCloudMap);
            end

            subMapMean               = cast(zeros(0, 3), 'like', ndtMap.VoxelMean);
            coder.varsize('subMapMean', [Inf 3], [1 0]);
            ndtMap.SubmapMean        = subMapMean;
            submapInverseCovariance  = cast(zeros(3, 3, 0), 'like', ndtMap.VoxelInverseCovariance);
            coder.varsize('submapInverseCovariance', [3 3 Inf], [0 0 1]);
            ndtMap.SubmapInverseCovariance = submapInverseCovariance;

        end

        %------------------------------------------------------------------
        function ndtMap = selectSubmap(ndtMap, varargin)

            roi = vision.internal.pcmap.parseSelectSubmapInputs(...
                ndtMap.XLimits, ndtMap.YLimits, ndtMap.ZLimits, varargin{:});
            ndtMap.SelectedSubmap = roi;

            % Select voxels
            voxelMeans = ndtMap.VoxelMean;

            % If Simulation or ML Coder
            if ~ndtMap.IsGPUTarget

                ndtMap.SubmapVoxelIndices = ...
                    voxelMeans(:, 1) >= roi(1) & voxelMeans(:, 1) <= roi(2) & ...
                    voxelMeans(:, 2) >= roi(3) & voxelMeans(:, 2) <= roi(4) & ...
                    voxelMeans(:, 3) >= roi(5) & voxelMeans(:, 3) <= roi(6);

                if ~any(ndtMap.SubmapVoxelIndices)
                    coder.internal.error('vision:pointcloud:submapHasNoVoxels')
                end

                ndtMap.SubmapMean               = voxelMeans(ndtMap.SubmapVoxelIndices, :);
                ndtMap.SubmapInverseCovariance  = ndtMap.VoxelInverseCovariance(:, :, ndtMap.SubmapVoxelIndices);

                ndtMap = updateSubmapTree(ndtMap);

                % If GPU Coder is enabled.
            else
                % GPU specific implementation of selectSubmap.
                ndtMap = selectSubmapGpuImpl(ndtMap, roi);
            end
        end

        %------------------------------------------------------------------
        function [isInside, distToEdge] = isInsideSubmap(ndtMap, pos)
            [isInside, distToEdge] = isInsideSubmap@vision.internal.pcmap.Submappable(ndtMap, pos);
        end

        %------------------------------------------------------------------
        function currPose = findPose(ndtMap, ptCloud, initPose, varargin)

            validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, 'findPose', 'ptCloud');
            validateattributes(initPose, {'rigidtform3d', 'rigid3d'}, {'scalar'}, 'findPose', 'initPose');

            % Set input type flag
            isRigid3d = isa(initPose, 'rigid3d');

            initPose = double(initPose.T');

            if ndtMap.IsSimulation
                args = vision.internal.ndt.parseOptionsSim(varargin{:});

                stepSize      = args.StepSize;
                outlierRatio  = double(args.OutlierRatio);
                tolerance     = [double(args.Tolerance(1)), double(args.Tolerance(2)*pi/180)];
                maxIterations = double(args.MaxIterations);
                verbose       = args.Verbose;
            else
                [stepSize, outlierRatio, maxIterations, tolerance, ~, verbose] = ...
                    vision.internal.ndt.parseOptionsCodegen(false, [], varargin{:});

                outlierRatio  = double(outlierRatio);
                tolerance     = [double(tolerance(1)), double(tolerance(2)*pi/180)];
                maxIterations = double(maxIterations);
            end

            % If Simulation and ML Coder.
            if ~ndtMap.IsGPUTarget
                movingPoints = double(extractValidPoints(ptCloud));

                if size(movingPoints, 1) < 3
                    coder.internal.error('vision:pointcloud:notEnoughPoints');
                end

                % If GPU Coder is enabled.
            else
                locationInp = double(ptCloud.Location);
                % Check if the location matrix is M X 3 (unorganized
                % pointCloud) or M X N X 3 (organized pointCloud).
                isOrganized = ~ismatrix(locationInp);

                % if unorganized pointCloud, pass 0 to subsetImpl and if
                % organized pointCloud, pass 1 to subsetImpl.
                indices = vision.internal.codegen.gpu.PointCloudImpl.extractValidPoints(locationInp);
                [movingPoints, ~, ~, ~, ~] = ...
                    vision.internal.codegen.gpu.PointCloudImpl.subsetImpl(locationInp, [],...
                    [],[],[],indices,isOrganized,'selected');

                if size(movingPoints, 1) < 3
                    T = coder.internal.nan(4,4,'like', ptCloud.Location);
                    if isRigid3d
                        currPose = rigid3d(coder.ignoreConst(T));
                    else
                        currPose = rigidtform3d(coder.ignoreConst(T));
                    end
                    return;
                end
            end

            % If submap cache is empty, use selectSubmap to create it
            if isempty(ndtMap.SubmapMean) || isempty(ndtMap.SubmapInverseCovariance)
                fullMapROI = [ndtMap.XLimits, ndtMap.YLimits, ndtMap.ZLimits];
                ndtMap = selectSubmap(ndtMap, fullMapROI);
            end

            fixedVoxelMeans = ndtMap.SubmapMean;
            fixedVoxelICov  = ndtMap.SubmapInverseCovariance;
            fixedSearchTree = ndtMap.SubmapKdtree;

            tformType = class(ptCloud.Location);

            % This function already supports GPU codegen.
            outPose = vision.internal.ndt.register(movingPoints, ...
                fixedVoxelMeans, fixedVoxelICov, fixedSearchTree, initPose, ...
                ndtMap.VoxelSize, outlierRatio, maxIterations, ...
                tolerance, verbose, stepSize, tformType);

            % The registration outputs a rigidtform3d object, so if
            % initPose was a rigid3d object, it is converted back into a
            % rigid3d object.
            if isRigid3d
                currPose = rigid3d(outPose.T);
            else
                currPose = outPose;
            end
        end

        %------------------------------------------------------------------
        function varargout = show(ndtMap, varargin)

            [spatialExtent, extraArgs] = vision.internal.pcmap.parseShowInputs(varargin{:});

            downsamplePercent = 1;
            mapPoints = generateSamples(ndtMap, spatialExtent, downsamplePercent);

            [varargout{1:nargout}] = pcshow(mapPoints, extraArgs{:});
        end
    end

    methods (Hidden)
        %------------------------------------------------------------------
        function that = saveobj(this)

            that.SelectedSubmap         = this.SelectedSubmap;
            that.XLimits                = this.XLimits;
            that.YLimits                = this.YLimits;
            that.ZLimits                = this.ZLimits;
            that.VoxelSize              = this.VoxelSize;
            that.VoxelMean              = this.VoxelMean;
            that.VoxelCovariance        = this.VoxelCovariance;
            that.VoxelInverseCovariance = this.VoxelInverseCovariance;
            that.VoxelNumPoints         = this.VoxelNumPoints;

            that.Version                = this.Version;
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        function this = loadobj(that)

            % Construct a simple pcmapndt object and then overwrite its
            % properties. pcmapndt() is not supported because map data is
            % needed for all its methods
            this = pcmapndt(pointCloud(zeros(10,3)), 1);

            this.SelectedSubmap          = that.SelectedSubmap;
            this.XLimits                 = that.XLimits;
            this.YLimits                 = that.YLimits;
            this.ZLimits                 = that.ZLimits;
            this.VoxelSize               = that.VoxelSize;
            this.VoxelMean               = that.VoxelMean;
            this.VoxelCovariance         = that.VoxelCovariance;
            this.VoxelInverseCovariance  = that.VoxelInverseCovariance;
            this.VoxelNumPoints          = that.VoxelNumPoints;
            this.SubmapKdtree            = vision.internal.Kdtree();

            % Get submap properties using selectSubmap
            this = selectSubmap(this, this.SelectedSubmap);

        end

        %------------------------------------------------------------------
        function props = matlabCodegenNontunableProperties(~)

            props = {'IsSimulation','IsGPUTarget'};
        end
    end

    methods (Access = private)
        %------------------------------------------------------------------
        function [voxelMeans, voxelCovs, voxelICovs, voxelNumPoints] = voxelize(ndtMap, ptCloudMap)

            % Use double-precision arithmetic for voxelizing and NDT
            % optimization.
            xyzPoints = double(extractValidPoints(ptCloudMap));
            voxelSize = ndtMap.VoxelSize;

            if isempty(xyzPoints)
                coder.internal.error('vision:pointcloud:allInvalidPoints', 'pcmapndt');
            end

            minVoxelPoints = 6;
            eigRatio       = 100;
            rangeLimits    = double([ptCloudMap.XLimits, ptCloudMap.YLimits, ptCloudMap.ZLimits]);

            [voxelMeans, voxelICovs, voxelCovs, voxelNumPoints] = ...
                vision.internal.pc.voxelGridFilter(xyzPoints, voxelSize, ...
                minVoxelPoints, eigRatio, rangeLimits);

            if isempty(voxelMeans)
                coder.internal.error('vision:pointcloud:mapHasNoVoxels');
            end
        end

        %------------------------------------------------------------------
        function [voxelMeans, voxelCovs, voxelICovs, voxelNumPoints] = voxelizeGpuImpl(ndtMap, ptCloudMap)
            % voxelizeGpuImpl GPU specific implementation of voxelize function
            % used in pcmapndt object creation.
            %   voxelmeans = voxelizeGpuImpl(ndtMap, ptCloudMap) computes
            %   voxelMeans using computeVoxelEigenCov, a function from
            %   pcregisterndt package. This functions extract vaalid points
            %   from ptCloudMap object and passes the valid points to
            %   computeVoxelEigenCov.
            %
            %   [..., voxelCovs, voxelICovs, voxelNumPoints] =
            %   voxelizeGpuImpl(...) additionally returns voxel covariance,
            %   voxel inverse covariance and number of points in each voxel.
            %
            %   Input arguments:
            %   ptCloudMap:     pointCloud object used in extracting valid
            %                   points.
            %
            %   voxelSize:      size of voxel.
            %   Output Arguments:
            %   voxelMeans:     mean of pointCloud's Location attribute per
            %                   voxel.
            %
            %   voxelCovs:      mean of Covariance per voxel.
            %
            %   voxelICovs:     mean of Inverse Covariance per voxel.
            %
            %   voxelNumPoints: number of points per voxel.
            %
            %   Class Support
            %   -------------
            %   ptCloudMap must be a pointCloud object.

            coder.inline('never');

            % Use double-precision arithmetic for extractValidPoints,
            % voxelizing and NDT optimization. Due to minor differences in
            % voxelCov, there are huge difference in voxelInvCov. This can
            % be minimized by using 'double' precision.
            locationInp = double(ptCloudMap.Location);

            % Check if the location matrix is M X 3 (unorganized pointCloud) or
            % M X N X 3 (organized pointCloud).
            isOrganized = ~ismatrix(locationInp);

            % if unorganized pointCloud, pass 0 to subsetImpl and if organized
            % pointCloud, pass 1 to subsetImpl. This will help in
            % pre-allocating memory to output variable inside subsetImpl
            % (for optimized CUDA code generation).
            indices = vision.internal.codegen.gpu.PointCloudImpl.extractValidPoints(locationInp);
            [xyzPoints, ~, ~, ~, ~] = ...
                vision.internal.codegen.gpu.PointCloudImpl.subsetImpl(locationInp, [],...
                [],[],[],indices,isOrganized,'selected');

            voxelSize = ndtMap.VoxelSize;

            % if xyzPoints is empty.
            if isempty(xyzPoints)
                voxelMeans = zeros(0,3);
                voxelCovs = zeros(3,3,0);
                voxelICovs = zeros(3,3,0);
                voxelNumPoints = uint32(0);
                return;
            end

            minVoxelPoints = 6;
            eigRatio       = 100;

            % GPU specific call for getting voxelMeans, voxelCovariance,
            % voxelInverseCovariance and voxelNumPoints.
            [voxelMeans, voxelICovs, voxelCovs, voxelNumPoints] = ...
                vision.internal.codegen.gpu.pcregisterndt.computeVoxelEigenCov(...
                xyzPoints, voxelSize, minVoxelPoints, eigRatio);

            % if voxelMeans is empty.
            if isempty(voxelMeans)
                voxelMeans = zeros(0,3);
                voxelCovs = zeros(3,3,0);
                voxelICovs = zeros(3,3,0);
                voxelNumPoints = uint32(0);
                return;
            end
        end

        %------------------------------------------------------------------
        function ndtMap = pcmapndtGpuImpl(ndtMap, ptCloudMap)
            % pcmapndtGpuImpl GPU specific implementation of pcmapndt
            % constructor.
            %   ndtMap = pcmapndtGpuImpl(ndtMap, ptCloudMap) computes and
            %   stores properties of ndtMap object like : XLimits, YLimits,
            %   ZLimits, SelectedSubmap etc. This function calls
            %   voxelizeGpuImpl for computing VoxelMean, VoxelCovariance,
            %   VoxelInverseCovariance and VoxelNumPoints properties of ndtMap
            %   object.
            %
            %   Input arguments:
            %   ndtMap:     pcmapndt object.
            %
            %   ptCloudMap: pointCloud object used in extracting valid points.
            %
            %   Ouptut argument:
            %   ndtMap:     pcmapndt object with updated properties.
            %
            %   Class Support
            %   -------------
            %   ptCloudMap must be a pointCloud object.
            %   ndtMap must be a pcmapndt object.

            % Voxelize point cloud map
            [ndtMap.VoxelMean, ndtMap.VoxelCovariance, ...
                ndtMap.VoxelInverseCovariance, ndtMap.VoxelNumPoints] = ...
                voxelizeGpuImpl(ndtMap, ptCloudMap);

            if ~ndtMap.VoxelNumPoints
                ndtMap.XLimits = [];
                ndtMap.YLimits = [];
                ndtMap.ZLimits = [];
                ndtMap.SelectedSubmap = [];
                ndtMap.SubmapVoxelIndices = cast([], 'like', false);
                ndtMap.SubmapKdtree = [];
                return;
            end

            % Calling min and max reductions for respective vectors.
            tmpMat = ndtMap.VoxelMean(:, 1);
            xlims = double(gpucoder.reduce(tmpMat, {@computeMin,@computeMax}));
            tmpMat = ndtMap.VoxelMean(:, 2);
            ylims = double(gpucoder.reduce(tmpMat, {@computeMin,@computeMax}));
            tmpMat = ndtMap.VoxelMean(:, 3);
            zlims = double(gpucoder.reduce(tmpMat, {@computeMin,@computeMax}));

            % Updating XLimits, YLimits and ZLimits properties.
            ndtMap.XLimits = [xlims(1), xlims(2)];
            ndtMap.YLimits = [ylims(1), ylims(2)];
            ndtMap.ZLimits = [zlims(1), zlims(2)];

            % Updating SelectedSubmap and SubmapVoxelIndices
            % properties.
            ndtMap.SelectedSubmap = [xlims(1) xlims(2) ylims(1) ylims(2) zlims(1) zlims(2)];
            ndtMap.SubmapVoxelIndices = true(size(ndtMap.VoxelNumPoints));

            % The tree based search is not used for GPU implementation.
            % So the tree object creation is skipped.
            ndtMap.SubmapKdtree = [];
        end

        %------------------------------------------------------------------
        function ndtMap = selectSubmapGpuImpl(ndtMap, roi)
            % selectSubmapGpuImpl GPU specific implementation of selectSubmap,
            % an object function of pcmapndt.
            %   ndtMap = selectSubmapGpuImpl(ndtMap, roi) select a submap from
            %   ndtMap using roi. After selecting submap this function updates
            %   pcmapndt properties: submapMean, SubmapInverseCovariance and
            %   submapVoxelIndices and returns pcmapndt object.
            %
            %   Input arguments:
            %   ndtMap: pcmapndt object.
            %
            %   roi:    region of interest for selecting submap.
            %
            %   Ouptut argument:
            %   ndtMap: pcmapndt object with updated properties.
            %
            %   Class Support
            %   -------------
            %   ptCloudMap must be a pointCloud object.

            voxelMeans = ndtMap.VoxelMean;
            % Allocating memory to subVoxelInd.
            subVoxelInd = coder.nullcopy(zeros(size(voxelMeans, 1), 1, 'like', ndtMap.SubmapVoxelIndices));

            % Finding the voxels in roi.
            coder.gpu.kernel;
            for iter = 1:size(voxelMeans, 1)
                checkCond = voxelMeans(iter, 1) >= roi(1) & voxelMeans(iter, 1) <= roi(2) & ...
                    voxelMeans(iter, 2) >= roi(3) & voxelMeans(iter, 2) <= roi(4) & ...
                    voxelMeans(iter, 3) >= roi(5) & voxelMeans(iter, 3) <= roi(6);

                subVoxelInd(iter) = checkCond;
            end

            if ~any(subVoxelInd)
                ndtMap.SubmapMean               = cast(zeros(0,3), 'like', voxelMeans);
                ndtMap.SubmapInverseCovariance  = cast(zeros(3,3,0), 'like', ndtMap.VoxelInverseCovariance);
                return;
            end

            % Updating the SubmapMean, SubmapInverseCovariance and
            % SubmapVoxelIndices properties.
            ndtMap.SubmapMean               = voxelMeans(subVoxelInd, :);
            ndtMap.SubmapInverseCovariance  = ndtMap.VoxelInverseCovariance(:, :, subVoxelInd);
            ndtMap.SubmapVoxelIndices       = subVoxelInd;
        end

        %------------------------------------------------------------------
        function ndtMap = updateSubmapTree(ndtMap)

            options = struct('bucketSize', 10);
            if ndtMap.IsSimulation
                % Index kdtree with submap means
                index(ndtMap.SubmapKdtree, ndtMap.SubmapMean, options);
            else
                % Index kdtree with submap means
                [ndtMap.SubmapKdtree, ndtMap.SubmapMeanHandleCodegen] = ...
                    iConfigureKdtreeCodegen(ndtMap.SubmapKdtree, ...
                    ndtMap.SubmapMean, options);
            end
        end

        %------------------------------------------------------------------
        function mapPoints = generateSamples(ndtMap, spatialExtent, downsamplePercent)

            % Fix random state to generate repeatable samples
            prevState = rng(0, 'simdTwister');
            restoreState = onCleanup(@()rng(prevState));

            if spatialExtent == "map"
                voxelMeans = ndtMap.VoxelMean;
                voxelCov   = ndtMap.VoxelCovariance;
                numPoints  = ndtMap.VoxelNumPoints;
            else
                voxelMeans = ndtMap.VoxelMean(ndtMap.SubmapVoxelIndices, :);
                voxelCov   = ndtMap.VoxelCovariance(:, :, ndtMap.SubmapVoxelIndices);
                numPoints  = ndtMap.VoxelNumPoints(ndtMap.SubmapVoxelIndices);
            end

            cumPoints      = cumsum(ceil(downsamplePercent * double(numPoints)));
            totalNumPoints = cumPoints(end);
            numVoxels      = size(voxelMeans, 1);

            % Generate points from standard normal distribution
            mapPoints = randn(totalNumPoints, 3, 'like', voxelMeans);

            invalidPoints = [];

            start = 1;
            for v = 1:numVoxels
                stop = cumPoints(v);

                [R, factorizationFailed] = chol(voxelCov(:,:,v));
                
                if ~factorizationFailed
                    mapPoints(start:stop, :) = mapPoints(start:stop, :)*R + voxelMeans(v, :);
                else
                    invalidPoints = [invalidPoints start:stop]; %#ok<AGROW>
                end

                start = stop;
            end
            % Remove points that are in voxels in which factorization of
            % the covariance matrix failed.
            mapPoints(invalidPoints, :) = [];
        end
    end
end

%--------------------------------------------------------------------------
% Private Functions
%--------------------------------------------------------------------------
function submapKdtree = iConstructKdtreeCodegen()

pointsType = 'double';
if coder.internal.preferMATLABHostCompiledLibraries()
    submapKdtree = vision.internal.buildable.kdtreeBuildable.kdtreeConstruct(...
        pointsType);
else % Portable Code
    submapKdtree = vision.internal.buildable.kdtreeBuildablePortable.kdtreeConstruct(...
    pointsType);
end
end

%--------------------------------------------------------------------------
function [submapKdtree, submapMeanHandleCodegen] = iConfigureKdtreeCodegen(...
    submapKdtree, submapMean, options)

if ismatrix(submapMean)
    numPts  = size(submapMean,1);
    numDims = size(submapMean,2);
else
    numPts  = size(submapMean,1) * size(submapMean,2);
    numDims = size(submapMean,3);
end

if coder.internal.preferMATLABHostCompiledLibraries()

    % Index tree with submap means
    submapMeanHandleCodegen = vision.internal.buildable.kdtreeBuildable.kdtreeGetLocationPointer(...
        submapMean, class(submapMean));
    vision.internal.buildable.kdtreeBuildable.kdtreeIndex(submapKdtree, ...
        class(submapMean), submapMeanHandleCodegen, numPts, numDims, options);
else

    % Index tree with submap means
    submapMeanHandleCodegen = vision.internal.buildable.kdtreeBuildablePortable.kdtreeGetLocationPointer(...
        submapMean, class(submapMean));
    vision.internal.buildable.kdtreeBuildablePortable.kdtreeIndex(submapKdtree, ...
        class(submapMean), submapMeanHandleCodegen, numPts, numDims, options);
end
end

%------------------------------------------------------------------
function out = computeMin(a, b)
out = min(a, b);
end

%------------------------------------------------------------------
function out = computeMax(a, b)
out = max(a, b);
end
