classdef rgbdvslam < vision.internal.vslam.BaseVisualSLAM

% Copyright 2023-2024 The MathWorks, Inc.

%#codegen
    properties (SetAccess=private, GetAccess=public)
        DepthScaleFactor

        DepthRange
    end

    properties (Hidden, Access = protected)
        SlamObj

        Version = 1.0
    end

    methods (Access = public)
        function vslam = rgbdvslam(intrinsics, depthScaleFactor, rgbdArgs, baseArgs)
            arguments
                intrinsics
                depthScaleFactor (1,1) {mustBeNumeric,mustBePositive,mustBeNonsparse,mustBeFinite}=5000
                rgbdArgs.DepthRange (1, 2) {mustBeNumeric,mustBeNonnegative,mustBeNonsparse,mustBeFinite,...
                    mustBeMonotonicallyIncreasingDepth}=[0.5 5]
                baseArgs.ScaleFactor
                baseArgs.NumLevels
                baseArgs.MaxNumPoints
                baseArgs.TrackFeatureRange
                baseArgs.SkipMaxFrames
                baseArgs.LoopClosureThreshold
                baseArgs.Verbose
                baseArgs.CustomBagOfFeatures
                baseArgs.ThreadLevel
                baseArgs.MaxReprojectionErrorPnP
                baseArgs.MinParallax
                baseArgs.MaxReprojectionErrorBA
                baseArgs.MaxNumIterationsBA
                baseArgs.MinPGOInterval
                baseArgs.MinNumMatches
            end
            baseArgsCell = namedargs2cell(baseArgs);
            vslam = vslam@vision.internal.vslam.BaseVisualSLAM(intrinsics, baseArgsCell{:});

            % Assign inputs to vslam object properties
            vslam.DepthScaleFactor     = single(depthScaleFactor);
            vslam.DepthRange           = rgbdArgs.DepthRange;

            coder.extrinsic('matlabroot');
            coder.extrinsic('fullfile');

            if coder.target('MATLAB')
                vslam.SlamObj = vision.internal.RGBDVisualSLAM();
                vslam.SlamObj.configure(vslam.Intrinsics, ...
                    vslam.ScaleFactor,...
                    vslam.NumLevels,...
                    vslam.MaxNumPoints, ...
                    vslam.TrackFeatureRange,...
                    vslam.SkipMaxFrames, ...
                    vslam.DepthScaleFactor, ...
                    vslam.DepthRange, ...
                    vslam.LoopClosureThreshold,...
                    vslam.Verbose,...
                    vslam.VocabFilePath,...
                    vslam.ThreadLevel, ...
                    vslam.MaxReprojectionErrorPnP, ...
                    vslam.MinParallaxCosine, ...
                    vslam.MaxReprojectionErrorBA, ...
                    vslam.MaxNumIterationsBA, ...
                    vslam.MinPGOInterval, ...
                    vslam.MinNumMatches);
            else
                vslam.SlamObj = vision.internal.buildable.RGBDVisualSLAMBuildable( ...
                    vslam, ...
                    [vslam.VocabFilePath char(0)]);
            end

            % In Simulation only, print log file name with a clickable link to open it
            printLogFileOnCMD(vslam);
        end

        function addFrame(vslam, colorImage, depthImage)

            Iu8_color = im2uint8(colorImage);
            Iu8_gray  = im2gray(Iu8_color);

            validateattributes(Iu8_gray, {'numeric'}, ...
                {'nonsparse', 'size', vslam.Intrinsics.ImageSize}, 'rgbdvslam', 'colorImage');

            validateattributes(depthImage, {'numeric'}, ...
                {'nonsparse', 'size', vslam.Intrinsics.ImageSize}, 'rgbdvslam', 'depthImage');

            addFrame(vslam.SlamObj, Iu8_gray, single(depthImage));
        end
    end

    methods (Static, Hidden)
        function this = loadobj(that)
            this = rgbdvslam(that.Intrinsics, that.DepthScaleFactor,...
                ScaleFactor          = that.ScaleFactor, ...
                NumLevels            = that.NumLevels, ...
                MaxNumPoints         = that.MaxNumPoints, ...
                SkipMaxFrames        = that.SkipMaxFrames, ...
                DepthRange           = that.DepthRange, ...
                TrackFeatureRange    = that.TrackFeatureRange, ...
                LoopClosureThreshold = that.LoopClosureThreshold, ...
                Verbose              = that.Verbose,...
                ThreadLevel          = that.ThreadLevel);
        end
    end

    methods (Hidden)
        function that = saveobj(this)
            that = saveobj@vision.internal.vslam.BaseVisualSLAM(this);

            that.DepthScaleFactor     = this.DepthScaleFactor;
            that.DepthRange           = this.DepthRange;
        end
    end
end

function mustBeMonotonicallyIncreasingDepth(x)
validateattributes(x(1), {'numeric'}, {'scalar'}, mfilename, 'DepthRange(1)');
validateattributes(x(2), {'numeric'}, {'scalar', '>', x(1)},  mfilename, 'DepthRange(2)');
end
