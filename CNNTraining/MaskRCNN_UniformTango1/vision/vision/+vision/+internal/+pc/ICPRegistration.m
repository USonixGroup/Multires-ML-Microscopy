%vision.internal.pc.ICPRegistration Implementation class for icp registration

% Copyright 2022-2024 The MathWorks, Inc.

%#codegen
classdef ICPRegistration < handle  

    properties
        Moving % movingPtCloud
        Fixed % fixedPtCloud
    end

    % Hyper parameters
    properties(Access = private) 
        Metric
        UseAllMatches
        DoExtrapolate
        InlierRatio
        InlierDistance
        MaxIterations
        Tolerance
        InitialTransform
        Verbose
    end

    % Internal variables
    properties(Access = private)
        UseInlierRatio
        RegistrationParams
    end

    %----------------------------------------------------------------------
    % Public APIs
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = ICPRegistration(unorgMoving, unorgFixed, options)
            this.Moving          = unorgMoving;
            this.Fixed          = unorgFixed;
            this.Metric            = options.metric;
            this.DoExtrapolate     = options.doExtrapolate;
            this.MaxIterations     = options.maxIterations;
            this.Tolerance         = options.tolerance;
            this.InitialTransform  = options.initTform;
            this.Verbose           = options.verbose;
            this.InlierDistance    = options.inlierDistance;
            if isempty(this.InlierDistance)
                this.UseInlierRatio = true;
                this.InlierRatio = options.inlierRatio;
                this.UseAllMatches = options.inlierRatio == 1;
            else % Use InlierDistance
                this.UseInlierRatio = false;
                this.UseAllMatches = false;
                this.InlierRatio = 1;
            end
        end

        %------------------------------------------------------------------
        function initializeRegistration(this)

            params.RelativeTranslation = this.Tolerance(1);
            params.RelativeRotation = this.Tolerance(2)*180/pi;
            params.MaxIteration = this.MaxIterations;
            params.InitialTransform = this.InitialTransform.T;
            params.MaxInlierDistance = this.InlierDistance;
            params.InlierRatio = this.InlierRatio;
            params.UseInlierRatio = this.UseInlierRatio;
            params.Verbose = this.Verbose;
            params.Method = char(this.Metric);

            params.moving = this.Moving.Location;
            params.movingNormals = this.Moving.Normal;
            params.HasNormalsA = ~isempty(this.Moving.Normal);

            params.fixed = this.Fixed.Location;
            params.fixedNormals = this.Fixed.Normal;
            params.HasNormalsB = ~isempty(this.Fixed.Normal);
            % Initializing the params required for colored ICP algorithm
            params.UseColors = false;
            params.VoxelSizes = [-1 -1 -1];
            params.MaxIterationsVector = [0 0 0];
            
            if ismatrix(this.Moving.Location)
                params.movingColors = zeros(coder.ignoreConst(0),3,'double');
                params.fixedColors = zeros(coder.ignoreConst(0),3,'double');
            else
                params.movingColors = zeros(coder.ignoreConst(0),coder.ignoreConst(0),3,'double');
                params.fixedColors = zeros(coder.ignoreConst(0),coder.ignoreConst(0),3,'double');
            end

            if strcmpi(this.Metric,"pointToPlaneWithColor") || ...
                    strcmpi(this.Metric,"planeToPlaneWithColor")
                params.UseColors = true;
                % Getting the colored information from orgainized and ...
                % unorganized point clouds
                if ismatrix(this.Moving.Location)
                    params.movingColors = im2double(this.Moving.Color(:,1:3));
                    params.fixedColors = im2double(this.Fixed.Color(:,1:3));
                else
                    params.movingColors = im2double(this.Moving.Color(:,:,1:3));
                    params.fixedColors = im2double(this.Fixed.Color(:,:,1:3));
                end
                params.VoxelSizes = this.estimateVoxelSize();
                params.MaxIterationsVector = this.computeMaxIterationsDistribution();

                if strcmpi(this.Metric,"planeToPlaneWithColor")

                    % For the metric planeToPlaneWithColor, the color property
                    % of the point cloud is converted from RGB to LAB format,
                    % LAB format is uniform and appropriate when dealing with
                    % high illumination variablity. Color weight (0.032) serves
                    % as scale between location and color property. 0.032 is
                    % optimum value for the color weight, further changes to
                    % the color weight do not influence the results.
                    params.movingColors = 0.032*rgb2lab(params.movingColors);
                    params.fixedColors = 0.032*rgb2lab(params.fixedColors);
                end
            end

            this.RegistrationParams = params;           
        end

        %------------------------------------------------------------------
        function tform = registerPointClouds(this)

            if isempty(coder.target)
                tformOut = registerICP(this.RegistrationParams);
            elseif coder.internal.preferMATLABHostCompiledLibraries() 
                tformOut = vision.internal.buildable.registerICPBuildable.registerICP(this.RegistrationParams);
            else
                tformOut = vision.internal.buildable.registerICPBuildablePortable.registerICP(this.RegistrationParams);
            end
            tform = rigidtform3d(cast(tformOut', 'like', this.Moving.Location));
        end

        %------------------------------------------------------------------
        % estimateVoxelSize calculates three voxel sizes for Multi-scale
        % registration. These voxel sizes are used to downsample the point
        % clouds.
        %
        % Voxel sizes can be fine tuned based on the point cloud limits,
        % for smaller point clouds it is preferred to select smaller voxel
        % size, this voxel size estimation is generalized, expecting larger
        % point clouds.
        function voxelSizes = estimateVoxelSize(this)
            % Voxel sizes are derived by rescaling of point cloud location limits.
            movingMaxLimits = [this.Moving.XLimits(2) this.Moving.YLimits(2) this.Moving.ZLimits(2)];
            movingMinLimits = [this.Moving.XLimits(1) this.Moving.YLimits(1) this.Moving.ZLimits(1)];

            fixedMaxLimits = [this.Fixed.XLimits(2) this.Fixed.YLimits(2) this.Fixed.ZLimits(2)];
            fixedMinLimits = [this.Fixed.XLimits(1) this.Fixed.YLimits(1) this.Fixed.ZLimits(1)];

            movingPtCloudSpan = abs(norm(movingMaxLimits - movingMinLimits));
            fixedPtCloudSpan = abs(norm(fixedMaxLimits - fixedMinLimits));

            minSpan = min(movingPtCloudSpan,fixedPtCloudSpan);
            initialVoxelSize = minSpan/100;
            voxelSizeExponent = floor(log10(initialVoxelSize));

            voxelSize = double(10.^voxelSizeExponent);
            voxelSizes = [voxelSize*2, voxelSize, voxelSize/2];
        end

        %------------------------------------------------------------------
        % computeMaxIterationsDistribution computes distribution of 
        % MaxIterations to be used for the three voxel sizes in Multi-scale
        % registration. The distribution depends on the weights that are
        % based on heuristics.
        function iterations = computeMaxIterationsDistribution(this)
            % Maximum iterations is divided in the range of 5:3:2 for corresponding voxel sizes
            iterations = [this.MaxIterations*0.5, this.MaxIterations*0.3, this.MaxIterations*0.2];
        end
    end
end