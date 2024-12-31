classdef BaseVisualSLAM < handle
% Copyright 2023-2024 The MathWorks, Inc.

%#codegen
    properties (SetAccess=private, GetAccess=public)
        Intrinsics

        IMUParameters

        ScaleFactor

        NumLevels

        MaxNumPoints

        SkipMaxFrames

        TrackFeatureRange

        LoopClosureThreshold

        CustomBagOfFeatures

        Verbose

        CameraToIMUTransform

        AlignmentFraction

        MaxAlignmentError

        NumPosesThreshold

        VocabFilePath
    end

    properties (Hidden, SetAccess = private)
        Viewer

        GravityDirection = 1
    end

    properties (GetAccess=public, SetAccess=protected)
        IsIMUAligned = false

        GravityRotation = []

        IMUScale = 1
    end

    properties (Hidden, Access = {?vision.internal.vslam.BaseVisualSLAM, ...
                                  ?vision.internal.buildable.BaseVisualSLAMBuildable})
        HasIMU  = false;

        ThreadLevel
        
        %MaxReprojectionErrorPnP  Maximum reprojection error for PnP solver. 
        %                         Increase the value to find more 2-D to
        %                         3-D pairs.
        MaxReprojectionErrorPnP
        
        %MinParallaxCosine        For a 3-D map point created from triangulation to be 
        %                         valid, its parallax corresponding to the two key frames
        %                         should be more than MinParallax. 
        %                         MinParallaxCosine = cosd(MinParallaxCosine)
        %                         Incarease this value to create more new map points though
        %                         they may be located very far away from the keyframes.
        MinParallaxCosine
        
        %MaxReprojectionErrorBA   Maximum reprojection error allowed for a 3-D map point 
        %                         to be valid after bundle adjustment. Increase this value to
        %                         keep more recently created map points though the map 
        %                         accuracy can be impacted.
        MaxReprojectionErrorBA
        
        %MaxNumIterationsBA       Maximum number of iteration when calling the backend solver  
        %                         for bundle adjustment. Increase this value to improve accuracy  
        %                         at slower speed.
        MaxNumIterationsBA
        
        %MinPGOInterval           Minimum number of keyframes passed when the next pose graph 
        %                         optimization is performed. Increase this value to improve speed.
        MinPGOInterval
        
        %MinNumMatches            Minimum number of matched pairs for a keyframe to be a local 
        %                         keyframe. Reduce the value when the tracking is weak and more 
        %                         local keyframes are required. These local keyframes are refined 
        %                         during bundle adjustment. Increase the value to improve accuracy, 
        %                         though this may result in reduced speed.
        MinNumMatches
    end

    properties (Hidden, Access = protected)
        IsPoseGraphOptimized = false
    end

    properties (Abstract, Hidden, Access = protected)
        SlamObj

        Version
    end

    methods (Access = public)
        function vslam = BaseVisualSLAM(intrinsics, imuParams, args)
            arguments
                intrinsics (1,1) {mustBeA(intrinsics,"cameraIntrinsics")}
                imuParams = []
                args.ScaleFactor (1,1) {mustBeReal,mustBeFinite,mustBeNonsparse,mustBeGreaterThan(args.ScaleFactor,1)}=1.2
                args.NumLevels (1,1) {mustBeInteger,mustBeFinite,mustBeNonsparse,mustBeGreaterThanOrEqual(args.NumLevels,1)}=8
                args.MaxNumPoints (1,1) {mustBeNumeric,mustBePositive,mustBeInteger,mustBeNonsparse}=1000
                args.TrackFeatureRange (1,2) {mustBeInteger,mustBePositive,mustBeNonsparse,...
                    mustBeMonotonicallyIncreasingFeatureRange}=[30 100]
                args.SkipMaxFrames (1,1) {mustBeNumeric,mustBePositive,mustBeInteger,mustBeNonsparse}=20
                args.LoopClosureThreshold (1,1) {mustBeNumeric,mustBePositive,mustBeInteger,mustBeNonsparse}=60
                args.Verbose (1,1) {mustBeInteger,mustBeInRange(args.Verbose,0,3)}=0
                args.CustomBagOfFeatures {validateCustomBagOfFeaturesArg}=[]
                args.ThreadLevel (1,1) {mustBeNumeric,mustBeNonnegative,mustBeInteger,mustBeNonsparse}=2
                args.CameraToIMUTransform (1,1) {mustBeA(args.CameraToIMUTransform,"rigidtform3d")} = rigidtform3d() 
                args.NumPosesThreshold (1,1) {mustBeInteger,mustBeFinite,mustBeNonsparse,mustBeGreaterThanOrEqual(args.NumPosesThreshold,2)}=40
                args.AlignmentFraction (1,1) {mustBeReal,mustBeFinite,mustBeNonsparse,mustBeGreaterThanOrEqual(args.AlignmentFraction,0),mustBeLessThanOrEqual(args.AlignmentFraction,1)}=0.9
                args.MaxAlignmentError (1,1) {mustBeReal,mustBeFinite,mustBeNonsparse,mustBePositive}=1
                args.MaxReprojectionErrorPnP (1,1) {mustBeReal,mustBeFinite,mustBeNonsparse,mustBePositive} = 4
                args.MinParallax             (1,1) {mustBeReal,mustBeFinite,mustBeNonsparse,mustBePositive} = 1 % in degress
                args.MaxReprojectionErrorBA  (1,1) {mustBeReal,mustBeFinite,mustBeNonsparse,mustBePositive} = 5.99
                args.MaxNumIterationsBA      (1,1) {mustBeInteger,mustBeFinite,mustBeNonsparse,mustBePositive} = 10
                args.MinPGOInterval          (1,1) {mustBeInteger,mustBeFinite,mustBeNonsparse,mustBePositive} = 20
                args.MinNumMatches           (1,1) {mustBeInteger,mustBeFinite,mustBeNonsparse,mustBePositive} = 20
            end

            if min(intrinsics.ImageSize) < 63
               coder.internal.error('vision:detectORBFeatures:invalidImageSize');
            end

            actualNumLevels = vision.internal.vslam.adjustNumScaleLevels(...
                intrinsics.ImageSize, args.NumLevels, args.ScaleFactor);
				
            if ~isempty(imuParams)
                if isa(imuParams,'factorIMUParameters')
                    vslam.HasIMU=true;

                    if imuParams.ReferenceFrame == "ENU"
                        vslam.GravityDirection = 1;
                    else
                        vslam.GravityDirection = -1;
                    end
                else
 				   coder.internal.error('vision:vslam_utils:InvalidIMUParams');
                end
            end

            % Assign inputs to vslam object properties
            vslam.Intrinsics           = intrinsics;
            vslam.IMUParameters        = imuParams;
            vslam.ScaleFactor          = args.ScaleFactor;
            vslam.NumLevels            = actualNumLevels;
            vslam.MaxNumPoints         = args.MaxNumPoints;
            vslam.SkipMaxFrames        = args.SkipMaxFrames;
            vslam.TrackFeatureRange    = args.TrackFeatureRange;
            vslam.LoopClosureThreshold = args.LoopClosureThreshold;
            vslam.Verbose              = args.Verbose;
            vslam.CustomBagOfFeatures  = args.CustomBagOfFeatures;
            vslam.CameraToIMUTransform = args.CameraToIMUTransform;
            vslam.NumPosesThreshold    = args.NumPosesThreshold;
            vslam.AlignmentFraction    = args.AlignmentFraction;
            vslam.MaxAlignmentError    = args.MaxAlignmentError;

            % Internal parameters
            vslam.ThreadLevel              = args.ThreadLevel;
            vslam.MaxReprojectionErrorPnP  = args.MaxReprojectionErrorPnP;
            vslam.MinParallaxCosine        = cosd(args.MinParallax);
            vslam.MaxReprojectionErrorBA   = args.MaxReprojectionErrorBA;
            vslam.MaxNumIterationsBA       = args.MaxNumIterationsBA;
            vslam.MinPGOInterval           = args.MinPGOInterval;
            vslam.MinNumMatches            = args.MinNumMatches;

            if ~isempty(vslam.CustomBagOfFeatures) && ~isempty(vslam.CustomBagOfFeatures.FileName)
                vslam.VocabFilePath = convertStringsToChars(vslam.CustomBagOfFeatures.FileName);
            else
                vslam.VocabFilePath = coder.internal.const(fullfile(matlabroot,'toolbox', ...
                    'vision','builtins','src','shared','vslamcore','bagOfFeatures.bin.gz'));
            end

            % Enabling verbose mode affects performance
            if coder.target('MATLAB') && vslam.Verbose ~= 0
                vslam.ThreadLevel = 0;
                disp(message("vision:vslam_utils:verboseEnabled").getString());
            end
        end

        function delete(vslam)
            % Closes all internal processes and clears all internal data
            reset(vslam);
        end

        function hasAdded = hasNewKeyFrame(vslam)
            nargoutchk(0, 1);

            hasAdded = hasNewKeyFrame(vslam.SlamObj);
        end

        function status = checkStatus(vslam)
            nargoutchk(0, 1);

            numPoints = getNumTrackedPoints(vslam.SlamObj);
            if numPoints < vslam.TrackFeatureRange(1)
                status = vision.internal.vslam.vslamStatusType.TrackingLost;
            elseif numPoints > vslam.TrackFeatureRange(2)
                status = vision.internal.vslam.vslamStatusType.FrequentKeyFrames;
            else
                status = vision.internal.vslam.vslamStatusType.TrackingSuccessful;
            end
        end

        function xyzPoints=mapPoints(vslam)
            xyzPoints = getWorldPoints(vslam.SlamObj);
        end

        function [camPoses, keyFrameIDs] = poses(vslam)
            nargoutchk(0, 2);

            camPoses = getCameraPoses(vslam.SlamObj);

            if nargout == 2
                keyFrameIDs = getKeyFrameIndex(vslam.SlamObj);
            end
        end

        function done = isDone(vslam)
            done = isDone(vslam.SlamObj);
        end

        function varargout = plot(vslam, options)
            arguments
                vslam
                options.MarkerSize (1,1) {mustBeNumeric, mustBePositive, mustBeFinite, mustBeNonsparse}=6
                options.MarkerColor="green"
                options.ColorSource (1,1) string {mustBeMember(options.ColorSource, ...
                    ["X", "Y", "Z", "MarkerColor", "x", "y", "z", "markercolor"])} = "Y"
                options.CameraSize (1,1) {mustBeNumeric, mustBePositive, mustBeFinite, mustBeNonsparse}=0.1
                options.CameraColor="red"
                options.Parent=[]
            end
            
            nargoutchk(0,1);

            markerColor = validateColor(options.MarkerColor, 'MarkerColor');

            if ~ismember(options.ColorSource, ["MarkerColor", "markercolor"])
                options.MarkerColor = [];
            else
                options.MarkerColor = markerColor;
            end

            options.CameraColor = validateColor(options.CameraColor, 'CameraColor');

            if ~isempty(options.Parent)
                uiaxesSupported = true;
                vision.internal.inputValidation.validateAxesHandle(options.Parent, uiaxesSupported);
            end

            xyzPoints = getWorldPoints(vslam.SlamObj);
            camPoses = getCameraPoses(vslam.SlamObj);

            if isempty(vslam.Viewer)
                vslam.Viewer = vision.internal.vslam.VisualSLAMViewer(xyzPoints, camPoses, options);
            else
                vslam.Viewer.updatePlot(xyzPoints, camPoses, isLoopRecentlyClosed(vslam.SlamObj), options);
            end
            
            if nargout == 1
                varargout{1} = vslam.Viewer.Axes;
            end

        end

        function reset(vslam)
            reset(vslam.SlamObj);
        end
    end

    methods (Access = protected)
        function printLogFileOnCMD(vslam)
            if coder.target('MATLAB') && vslam.Verbose > 1
                filename = vslam.SlamObj.getLogFileName();
                if ~isempty(filename)
                    disp(['<a href="matlab:edit ',filename,'">', message("vision:vslam_utils:logFileName",filename).getString(),'</a>']);
                end
            end
        end
    end

    methods (Abstract, Access = public)
        addFrame(vslam, args);
    end

    methods (Abstract, Static, Hidden)
        loadobj(that)
    end

    methods (Hidden)
        function that = saveobj(this)
            that.Intrinsics           = this.Intrinsics;
            that.IMUParameters        = this.IMUParameters;
            that.ScaleFactor          = this.ScaleFactor;
            that.NumLevels            = this.NumLevels;
            that.MaxNumPoints         = this.MaxNumPoints;
            that.SkipMaxFrames        = this.SkipMaxFrames;
            that.TrackFeatureRange    = this.TrackFeatureRange;
            that.LoopClosureThreshold = this.LoopClosureThreshold;
            that.CustomBagOfFeatures  = this.CustomBagOfFeatures;
            that.Verbose              = this.Verbose;
            that.IsPoseGraphOptimized = this.IsPoseGraphOptimized;
            that.Version              = this.Version;
            that.ThreadLevel          = this.ThreadLevel;
            that.CameraToIMUTransform = this.CameraToIMUTransform;
            that.NumPosesThreshold    = this.NumPosesThreshold;
            that.AlignmentFraction    = this.AlignmentFraction;
            that.MaxAlignmentError    = this.MaxAlignmentError;
        end

        function flag = isMapInitialized(vslam)
            flag = isInitialized(vslam.SlamObj);
        end

        function numPoints=getNumTrackedPoints(vslam)
            numPoints = getNumTrackedPoints(vslam.SlamObj);
        end

        function filename = getLogFileName(vslam)
            filename = vslam.SlamObj.getLogFileName();
        end
    end
end

function mustBeMonotonicallyIncreasingFeatureRange(x)
validateattributes(x(1), {'numeric'}, {'scalar'}, mfilename, 'TrackedFeatureRange(1)');
validateattributes(x(2), {'numeric'}, {'scalar', '>', x(1)},  mfilename, 'TrackedFeatureRange(2)');
end

function mustBeLogicalScalar(n)
validateattributes(n,{'logical'}, {'scalar'}, mfilename, 'Verbose');
end

function color = validateColor(input, varname)
if ischar(input) || isstring(input)
    % Convert color string to an RGB triplet.
    rgbTriplet = vision.internal.convertColorSpecToRGB(input, ...
        'uint8', mfilename, varname);
else
    validateattributes(input, {'uint8', 'uint16', 'single', 'double'}, ...
        {'real', 'nonsparse', 'size', [1 3]}, mfilename, varname);
    rgbTriplet = input;
end

% Convert single or double color values to uint8.
if isa(rgbTriplet, 'uint8') || isa(rgbTriplet, 'uint16')
    color = rgbTriplet;
else
    color = uint8(255*rgbTriplet);
end
end

function flag = validateCustomBagOfFeaturesArg(inp)
    flag = false;
    if ~isempty(inp)
        validateattributes(inp, {'bagOfFeaturesDBoW'}, {'scalar'}, mfilename, 'CustomBagOfFeatures');
        flag = true;
    end
end