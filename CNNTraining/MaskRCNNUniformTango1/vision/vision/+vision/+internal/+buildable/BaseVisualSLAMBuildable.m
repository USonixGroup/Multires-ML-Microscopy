classdef (Abstract) BaseVisualSLAMBuildable < coder.ExternalDependency

% Copyright 2024 The MathWorks, Inc.

%#codegen
    properties (Constant, Access = private)
        APIHeaderFile = "basevslam_core_api.hpp"
    end

    properties (Abstract, Access = protected)
        SlamInternal
    end

    methods (Static, Abstract)
        name = getDescriptiveName(~)
    end

    methods (Abstract)
        addFrame(obj)
    end

    methods (Static)
        function isSupported = isSupportedContext(~)
            %isSupportedContext
            isSupported = true; % supports non-host target
        end

        function updateBuildInfo(buildInfo, context)
            %updateBuildInfo
            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, 'MonoVisualSLAM');
            vision.internal.buildable.portableMonoVisualSLAMBuildInfo(buildInfo, context);
        end
    end

    methods
        function hasAdded = hasNewKeyFrame(obj)
            %hasNewKeyFrame
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            hasAdded = false;
            hasAdded = coder.ceval("BaseVisualSLAM_hasNewKeyFrame", obj.SlamInternal);
        end

        function xyzPoints = getWorldPoints(obj)
            %mapPoints
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            numPoints = 0;
            numPoints = coder.ceval("BaseVisualSLAM_getNumWorldPoints", obj.SlamInternal);
            xyzPoints = zeros(numPoints, 3);
            coder.ceval("-row", "BaseVisualSLAM_getWorldPoints", obj.SlamInternal, coder.ref(xyzPoints));
        end

        function camPoses = getCameraPoses(obj)
            %getCameraPoses
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            numCameras = 0;
            numCameras = coder.ceval("BaseVisualSLAM_getNumCameraPoses", obj.SlamInternal);
            tformA = zeros(numCameras, 4, 4);
            coder.ceval("-row", "BaseVisualSLAM_getCameraPoses", obj.SlamInternal, coder.ref(tformA));
            if numCameras > 0
                camPoses = rigidtform3d(squeeze(tformA(1,:,:)));
                for i = 2:numCameras
                    camPoses = [camPoses, rigidtform3d(squeeze(tformA(i,:,:)))]; %#ok<AGROW>
                end
            else
                camPoses = repmat(rigidtform3d, 0, 0);
            end
        end

        function keyFrameIDs = getKeyFrameIndex(obj)
            %getKeyFrameIndex
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            numCameras = 0;
            numCameras = coder.ceval("BaseVisualSLAM_getNumCameraPoses", obj.SlamInternal);
            keyFrameIDs = zeros(1, numCameras, "int32");
            coder.ceval("BaseVisualSLAM_getKeyFrameIndex", obj.SlamInternal, coder.ref(keyFrameIDs));
        end

        function viewIDs = getViewIDs(obj)
            %getViewIDs
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            numCameras = 0;
            numCameras = coder.ceval("BaseVisualSLAM_getNumCameraPoses", obj.SlamInternal);
            viewIDs = zeros(1, numCameras, "int32");
            coder.ceval("BaseVisualSLAM_getViewIDs", obj.SlamInternal, coder.ref(viewIDs));
        end

        function flag = isInitialized(obj)
            %isMapInitialized
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            flag = false;
            flag = coder.ceval("BaseVisualSLAM_isInitialized", obj.SlamInternal);
        end

        function flag = isLoopRecentlyClosed(obj)
            %isLoopRecentlyClosed
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            flag = false;
            flag = coder.ceval("BaseVisualSLAM_isLoopRecentlyClosed", obj.SlamInternal);
        end

        function numPoints = getNumTrackedPoints(obj)
            %getNumTrackedPoints
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            numPoints = 0;
            numPoints = coder.ceval("BaseVisualSLAM_getNumTrackedPoints", obj.SlamInternal);
        end

        function done = isDone(obj)
            %isDone
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            done = false;
            done = coder.ceval("BaseVisualSLAM_isDone", obj.SlamInternal);
        end

        function reset(obj)
            %reset
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            coder.ceval("BaseVisualSLAM_reset", obj.SlamInternal);
        end

        function [imuG, imuA] = getViewIMUMeasurements(obj, viewId)
            %getViewIMUMeasurements
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);
            numMeasurements = 0;
            numMeasurements = coder.ceval("BaseVisualSLAM_getNumIMUMeasurements", obj.SlamInternal, int32(viewId));

            imuG = zeros(numMeasurements, 3, "double");
            imuA = zeros(numMeasurements, 3, "double");
            coder.ceval("-row", "BaseVisualSLAM_getViewIMUMeasurements", obj.SlamInternal, ...
                int32(viewId), coder.ref(imuG), coder.ref(imuA));
        end
    end

    %----------------------------------------------------------------------
    % Setup C-structs
    %----------------------------------------------------------------------
    methods (Access = protected)
        function baseParams = getBaseParamsCStruct(obj, vslamObj)
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            % Create MATLAB struct that will be represented as a BaseParams
            % struct in generated code.
            % 
            % Note that this code would compile if only the user-exposed 
            % BaseParams members were provided for the baseParams MATLAB 
            % struct (these fields are assigned values at the end of this 
            % function), and the non-exposed members were missing. However,
            % if all of the MATLAB struct fields are assigned values 
            % explicitly (using dot indexing at the end of this function) 
            % then the assignment from ceval call to
            % BaseVisualSLAM_defaultBaseParams is optimized out in 
            % generated code and the non-exposed members are not assigned 
            % proper default values.
            % 
            % To avoid this I included all of the non-user-exposed members 
            % in BaseParams in the MATLAB struct, but including just one of
            % these fields (or a dummy field) would have been sufficient.
            baseParams = struct('nRows', int32(0), ...
                                'nCols', int32(0), ...
                                'minNumWorldPoints', int32(0), ...
                                'maxReprojError', single(0), ...
                                'minNumPnPPoints', int32(0), ...
                                'minNumPointWeakFrame', int32(0), ...
                                'minNumMatches', int32(0), ...
                                'numSkippedFrames', int32(0), ...
                                'minNumMatchesNewConnection', int32(0), ...
                                'numFeatures', int32(0), ...
                                'numFeaturesInit', int32(0), ...
                                'scaleFactor', single(0), ...
                                'numLevels', int32(0), ...
                                'matchThreshold', int32(0), ...
                                'maxRatio', single(0), ...
                                'maxRatioInRadius', single(0), ...
                                'minNumMatchesLoop', int32(0), ...
                                'verbose', int32(0), ...
                                'maxRatioMapping', single(0), ...
                                'minNumMatchesMapping', int32(0), ...
                                'minSinParallax', 0, ...
                                'maxCosParallax', 0, ...
                                'minNumMatchesBA', int32(0), ...
                                'maxNumFixedViewsBA', int32(0), ...
                                'maxNumIterationsBA', int32(0), ...
                                'maxCeresRE', 0, ...
                                'minNumMatchesPGO', int32(0), ...
                                'optimizationInterval', int32(0), ...
                                'maxNumIterationsPGO', int32(0));

            % Tell MATLAB Coder to use the BaseParams struct from
            % ParameterStruct.hpp in the generated code.
            coder.cstructname(baseParams, 'BaseParams', ...
                'extern', 'HeaderFile', 'ParameterStruct.hpp');

            % Set the default BaseParams field values.
            baseParams = coder.ceval("BaseVisualSLAM_defaultBaseParams");
            
            % Set the BaseParams field values using vslamObj properties.
            baseParams.nRows = int32(vslamObj.Intrinsics.ImageSize(1));
            baseParams.nCols = int32(vslamObj.Intrinsics.ImageSize(2));
            baseParams.scaleFactor = single(vslamObj.ScaleFactor);
            baseParams.numLevels = int32(vslamObj.NumLevels);
            baseParams.numFeatures = int32(vslamObj.MaxNumPoints);
            baseParams.minNumPnPPoints = int32(vslamObj.TrackFeatureRange(1));
            baseParams.minNumPointWeakFrame = int32(vslamObj.TrackFeatureRange(2));
            baseParams.numSkippedFrames = int32(vslamObj.SkipMaxFrames);
            baseParams.minNumMatchesLoop = int32(vslamObj.LoopClosureThreshold);
            baseParams.verbose = int32(vslamObj.Verbose);

            % Parameters for internal use
            baseParams.maxReprojError = single(vslamObj.MaxReprojectionErrorPnP);
            baseParams.maxCosParallax = vslamObj.MinParallaxCosine;
            baseParams.maxCeresRE = vslamObj.MaxReprojectionErrorBA;
            baseParams.maxNumIterationsBA = int32(vslamObj.MaxNumIterationsBA);
            baseParams.optimizationInterval = int32(vslamObj.MinPGOInterval);
            baseParams.minNumMatches = int32(vslamObj.MinNumMatches);

        end

        function imuParams = getIMUParamsCStruct(obj, vslamObj)
            coder.inline('always');
            coder.cinclude(obj.APIHeaderFile);

            % Create MATLAB struct that will be represented as an IMUParams
            % struct in generated code. See note in equavalent line in 
            % getBaseParamsCStruct.
            imuParams = struct('hasIMU', false, ...
                               'sampleRate', int32(0), ... % TODO: shouldn't this be a double?
                               'gyroscopeBiasNoise', 0, ...
                               'accelerometerBiasNoise', 0, ...
                               'gyroscopeNoise', 0, ...
                               'accelerometerNoise', 0, ...
                               'gravityDirection', int32(0), ...
                               'maxNumIterationsIMUBA', int32(0), ...
                               'maxCeresREIMU', 0);

            % Tell MATLAB Coder to use the IMUParams struct from
            % ParameterStruct.hpp in the generated code.
            coder.cstructname(imuParams, 'IMUParams', ...
                'extern', 'HeaderFile', 'ParameterStruct.hpp');

            % Set the default IMUParams field values.
            imuParams = coder.ceval("BaseVisualSLAM_defaultIMUParams");

            % Set the IMUParams field values using vslamObj properties.
            imuParams.hasIMU = vslamObj.HasIMU;
            if imuParams.hasIMU && ~isempty(vslamObj.IMUParameters)
                imuParams.sampleRate = int32(vslamObj.IMUParameters.SampleRate);
                imuParams.gyroscopeBiasNoise = vslamObj.IMUParameters.GyroscopeBiasNoise(1,1);
                imuParams.accelerometerBiasNoise = vslamObj.IMUParameters.AccelerometerBiasNoise(1,1);
                imuParams.gyroscopeNoise = vslamObj.IMUParameters.GyroscopeNoise(1,1);
                imuParams.accelerometerNoise = vslamObj.IMUParameters.AccelerometerNoise(1,1);
                imuParams.gravityDirection = int32(vslamObj.GravityDirection);
            end
        end
    end
end