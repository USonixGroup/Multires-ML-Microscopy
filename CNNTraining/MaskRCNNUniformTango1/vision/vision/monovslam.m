classdef monovslam < vision.internal.vslam.BaseVisualSLAM

% Copyright 2022-2024 The MathWorks, Inc.

%#codegen
    properties (Hidden, Access = protected)
        SlamObj

        Version = 2.0
    end

    methods (Access = public)
        function vslam = monovslam(intrinsics, IMUParameters, args)
            arguments
                intrinsics
                IMUParameters = []
                args.ScaleFactor
                args.NumLevels
                args.MaxNumPoints
                args.TrackFeatureRange
                args.SkipMaxFrames
                args.LoopClosureThreshold
                args.Verbose
                args.CustomBagOfFeatures
                args.CameraToIMUTransform
                args.NumPosesThreshold
                args.AlignmentFraction
                args.MaxAlignmentError
                args.ThreadLevel
                args.MaxReprojectionErrorPnP
                args.MinParallax
                args.MaxReprojectionErrorBA
                args.MaxNumIterationsBA
                args.MinPGOInterval
                args.MinNumMatches            
            end

            % Feature control: Do not allow the user to pass IMUParameters
            imuFC = matlab.internal.feature("MonovslamIMU");
		    coder.internal.assert(imuFC || isempty(IMUParameters), 'vision:vslam_utils:imuFC1');

            % Code generation is not supported with IMU in 24b.
            % g3280733 g3280725 g3280718 g3280711 g3280656
            coder.internal.assert(coder.target('MATLAB') || isempty(IMUParameters), 'vision:vslam_utils:imuCodegenNotSupported');

            argsCell = namedargs2cell(args);
            vslam = vslam@vision.internal.vslam.BaseVisualSLAM(intrinsics, IMUParameters, argsCell{:});

            coder.extrinsic('matlabroot');
            coder.extrinsic('fullfile');
            
            if coder.target('MATLAB')
                vslam.SlamObj = vision.internal.MonoVisualSLAM();
                vslam.SlamObj.configure(vslam.Intrinsics, ...
                    vslam.HasIMU, ...
                    vslam.IMUParameters, ...
                    vslam.GravityDirection, ...
                    vslam.ScaleFactor,...
                    vslam.NumLevels,...
                    vslam.MaxNumPoints, ...
                    vslam.TrackFeatureRange,...
                    vslam.SkipMaxFrames, ...
                    vslam.LoopClosureThreshold,...
                    vslam.Verbose, ...
                    vslam.VocabFilePath, ...
                    vslam.ThreadLevel, ...
                    vslam.CameraToIMUTransform, ...
                    vslam.MaxReprojectionErrorPnP, ...
                    vslam.MinParallaxCosine, ...
                    vslam.MaxReprojectionErrorBA, ...
                    vslam.MaxNumIterationsBA, ...
                    vslam.MinPGOInterval, ...
                    vslam.MinNumMatches);
            else
                vslam.SlamObj = vision.internal.buildable.MonoVisualSLAMBuildable( ...
                    vslam, ...
                    [vslam.VocabFilePath char(0)]);
            end

            % In Sumulation only, print log file name with a clickable link to open it
            printLogFileOnCMD(vslam);
        end
		
		function addFrame(vslam, I, imuGyro, imuAccel)
            arguments
                vslam
                I
                imuGyro = [];
                imuAccel = [];
            end

            nargoutchk(0, 0);
            
            % Convert image to grayscale uint8
            Iu8      = im2uint8(I);
            Iu8_gray = im2gray(Iu8);
            
            validateattributes(Iu8_gray, {'numeric','logical'}, ...
                {'nonsparse', 'size', vslam.Intrinsics.ImageSize}, mfilename, 'I');

            % Feature control: Do not allow the user to pass IMU data
            imuFC = matlab.internal.feature("MonovslamIMU");
            if ~imuFC && nargin>2
			    coder.internal.error('vision:vslam_utils:imuFC2'); 
            end

            if vslam.HasIMU && ~isempty(vslam.IMUParameters) % Second condition necessary to avoid generating code for estimateGravityRotationAndPosescale for monovslam w/o IMU

                camPoses  = poses(vslam);
		        numPoses = length(camPoses);

                if nargin>2

                    validateattributes(imuGyro, {'numeric'}, {'nonsparse', 'nonnan', 'finite', 'real'});
                    validateattributes(imuAccel, {'numeric'}, {'nonsparse', 'nonnan', 'finite', 'real'});

                    % Gyro and accel have to be the same size
                    coder.internal.errorIf(numel(imuGyro)~=numel(imuAccel), 'vision:vslam_utils:sameSizeIMU');

                    % Empty gyro and accel are allowed, to account for
                    % cases where IMU is unavailable temporarily
                    if (~isempty(imuGyro) || ~isempty(imuAccel))
                        validateattributes(imuGyro, {'numeric'}, {'ncols', 3});
                        validateattributes(imuAccel, {'numeric'}, {'ncols', 3});
                    end

                    % Throw warning when IMU is unavailable
                    if isempty(imuGyro) && isempty(imuAccel) && numPoses > 1
                        coder.internal.warning('vision:vslam_utils:noIMUData');
                    end

                else

                    % Throw warning when IMU is unavailable
                    if numPoses > 1
                        coder.internal.warning('vision:vslam_utils:noIMUData');
                    end

                end
                
                % Scale and gravity calibration
                if ~vslam.IsIMUAligned && numPoses > vslam.NumPosesThreshold

                    windowSize = round(vslam.NumPosesThreshold * vslam.AlignmentFraction);
                    [viewGyro, viewAccel] = imuMeasurements(vslam, windowSize);

                    if ~isempty(viewGyro) && ~isempty(viewAccel)

                        posesInWindow = camPoses(numPoses-windowSize+1:numPoses);

                        [gRot, scale] = estimateGravityRotationAndScale(vslam, posesInWindow, viewGyro, viewAccel);

                        vslam.GravityRotation = gRot';
                        vslam.IMUScale = scale;

                        if scale>0.3
				            % storeGravityRotationAndScale takes the transposed result of the rotation matrix returned by estimateGravityRotationAndPoseScale
				            storeGravityRotationAndScale(vslam.SlamObj, vslam.GravityRotation, vslam.IMUScale);
                            vslam.IsIMUAligned = true;
                        end

                    end
                end
            else % No IMU

                if nargin>2
				    coder.internal.error('vision:vslam_utils:InvalidIMUParams');
                end

            end

            addFrame(vslam.SlamObj, Iu8_gray, imuGyro, imuAccel);
        end

    end

    methods (Access = private)

        function [gyro, accel] = imuMeasurements(vslam,windowSize)

            viewIDs = getViewIDs(vslam.SlamObj);

            cellIdx = 1;

            imuSize = windowSize-1; %size of rolling window + 1 to exclude the first view

            % Initial values needed for cell array codegen
            gyro = repmat({zeros(0,3)}, imuSize, 1);
            accel = repmat({zeros(0,3)}, imuSize, 1);
			
            for i=length(viewIDs)-imuSize+1:length(viewIDs)
                % gyro and accel need to be set directly from this function
                % call not through a temporary variable for code
                % generation.
                [gyro{cellIdx}, accel{cellIdx}] = getViewIMUMeasurements(vslam.SlamObj, viewIDs(i));
                
                cellIdx=cellIdx+1;
            end
			
        end
        
    end
	
	methods (Hidden, Access = {?matlab.unittest.TestCase})

        function [gRot, scale] = estimateGravityRotationAndScale(vslam, poses, gyro, accel)

            imuParams = vslam.IMUParameters;
            camToIMU = se3(vslam.CameraToIMUTransform.A);

            opts = factorGraphSolverOptions(FunctionTolerance=1e-15, ...
                                            GradientTolerance=1e-15, ...
                                            StepTolerance=1e-15, ...
                                            TrustRegionStrategyType=0, ...
                                            MaxIterations=1000);
											
			posePriorWeight = 1e6;
			biasPriorWeight = 1e6;
    
            len = length(poses);
            poseNodeStates = zeros(len,7);
            for k = 1:len
                poseNodeStates(k,:) = [poses(k).Translation,rotm2quat(poses(k).R)];
            end
            
            f = factorGraph();
            
            % generate node ids
            scaleNodeID = generateNodeID(f,1);
            gravityNodeID = generateNodeID(f,1);
            poseVelBiasNodeIDs = generateNodeID(f, [len,3]);
            
            % create IMU factor with readings between poses
            for k = 2:len
                % create unique imu node ids
                nodeID = [poseVelBiasNodeIDs(k-1,:),poseVelBiasNodeIDs(k,:),gravityNodeID,scaleNodeID];
            
                % create imu factor with gravity and scale nodes
                fIMU = nav.algs.internal.FactorIMUGS( ...
                    nodeID, ...
                    gyro{k-1}, accel{k-1}, imuParams);
                fIMU.SensorTransform = camToIMU;
            
                % add imu factors to factor graph
                f.addFactor(fIMU);
            end
            
            % set node state
            kid = poseVelBiasNodeIDs(:,1);
            f.nodeState(kid,poseNodeStates(:,1:7));

            % fix pose nodes softly and optimize for IMU bias, scale and gravity direction
            for k = 1:length(kid)
                fp = factorPoseSE3Prior(kid(k), Information=posePriorWeight*eye(6), Measurement=poseNodeStates(k,:));
                addFactor(f, fp);
            end
            f.fixNode(kid(1));
            
            % add bias prior on first bias node
            fBiasPrior = factorIMUBiasPrior(poseVelBiasNodeIDs(1,3),Information=biasPriorWeight*eye(6));
            f.addFactor(fBiasPrior);
            
            % optimize factor graph
            f.optimize(opts);
            
            % extract computed scale
            scale = exp(f.nodeState(scaleNodeID));
            
            % extract computed gravity direction
            gDirQuat = f.nodeState(gravityNodeID);
            gRot = quat2tform([gDirQuat(4),gDirQuat(1),gDirQuat(2), gDirQuat(3)]);

        end

    end

    methods (Static, Hidden)
        function this = loadobj(that)
            % Feature control: Load v1 if feature is OFF
            if that.Version == 1.0 || ~matlab.internal.feature("MonovslamIMU")
                that.ThreadLevel = 2;

                this = monovslam(that.Intrinsics, ...
                    ScaleFactor          = that.ScaleFactor, ...
                    NumLevels            = that.NumLevels, ...
                    MaxNumPoints         = that.MaxNumPoints, ...
                    SkipMaxFrames        = that.SkipMaxFrames, ...
                    TrackFeatureRange    = that.TrackFeatureRange, ...
                    LoopClosureThreshold = that.LoopClosureThreshold, ...
                    Verbose              = that.Verbose, ...
                    ThreadLevel          = that.ThreadLevel);
			
            elseif that.Version == 2.0
					this = monovslam(that.Intrinsics, that.IMUParameters, ...
					ScaleFactor          = that.ScaleFactor, ...
					NumLevels            = that.NumLevels, ...
					MaxNumPoints         = that.MaxNumPoints, ...
					SkipMaxFrames        = that.SkipMaxFrames, ...
					TrackFeatureRange    = that.TrackFeatureRange, ...
					LoopClosureThreshold = that.LoopClosureThreshold, ...
					Verbose              = that.Verbose, ...
					ThreadLevel          = that.ThreadLevel, ...
					CameraToIMUTransform = that.CameraToIMUTransform, ...
					NumPosesThreshold    = that.NumPosesThreshold, ...
					AlignmentFraction    = that.AlignmentFraction, ...
					MaxAlignmentError    = that.MaxAlignmentError);				
            end
        end
    end
end