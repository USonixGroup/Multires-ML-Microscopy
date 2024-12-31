classdef ForegroundDetector< matlab.System
%ForegroundDetector Detect foreground using Gaussian Mixture Models
% 
%   detector = vision.ForegroundDetector returns a foreground detector System 
%   object, detector, that computes foreground mask using Gaussian Mixture
%   Models (GMM) given a series of either grayscale or color video frames.
% 
%   detector = vision.ForegroundDetector('PropertyName', PropertyValue, ...) 
%   returns a foreground detector System object, H, with each specified 
%   property set to the specified value.
% 
%   Step method syntax:
% 
%   foregroundMask = step(detector, I) computes the foreground mask for input 
%   image I, and returns a logical mask where true represents foreground
%   pixels. Image I can be grayscale or color. This form of the step 
%   function call is allowed when AdaptLearningRate is true (default).
%
%   foregroundMask = step(detector, I, learningRate) computes the foreground 
%   mask for input image I using the LearningRate provided by the user. This
%   form of the step function call is allowed when AdaptLearningRate is 
%   false.
% 
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   ForegroundDetector methods:
% 
%   step     - See above description for use of this method
%   <a href="matlab:help matlab.System/reset   ">reset</a>    - Resets the GMM model to its initial state
%   release  - Allow property value and input characteristics changes
%   clone    - Create foreground detection object with same property values
%   isLocked - Locked status (logical)
% 
%   ForegroundDetector properties:
% 
%   AdaptLearningRate          - Enables the adapting of LearningRate as 
%                                1/(current frame number) during the
%                                training period specified by 
%                                NumTrainingFrames
%   NumTrainingFrames          - Number of initial video frames used for 
%                                training the background model
%   LearningRate               - Learning rate used for parameter updates
%   MinimumBackgroundRatio     - Threshold to determine the gaussian modes
%                                in the mixture model that constitute the
%                                background process
%   NumGaussians               - Number of distributions that make up the
%                                foreground-background mixture model
%   InitialVariance            - Initial variance to initialize all
%                                distributions that compose the
%                                foreground-background mixture model
% 
%   Example: Detect moving cars in video
%   ------------------------------------
%   reader = VideoReader('visiontraffic.avi');
%   detector = vision.ForegroundDetector();
%
%   blobAnalyzer = vision.BlobAnalysis(...
%       'CentroidOutputPort', false, 'AreaOutputPort', false, ...
%       'BoundingBoxOutputPort', true, 'MinimumBlobArea', 250);
% 
%   player = vision.DeployableVideoPlayer();
%
%   while hasFrame(reader)
%     frame  = readFrame(reader);
%     fgMask = step(detector, frame);
%     bbox   = step(blobAnalyzer, fgMask);
%
%     % draw bounding boxes around cars
%     out = insertShape(frame, 'Rectangle', bbox, 'ShapeColor', 'Yellow');
%     step(player, out); % view results in the video player
%   end
%
%   release(player);
%
% See also: VideoReader, vision.BlobAnalysis, regionprops, vision.KalmanFilter, 
%   imopen, imclose

     
%   Copyright 2010-2022 The MathWorks, Inc.

    methods
        function out=ForegroundDetector
        end

        function out=getNumInputsImpl(~) %#ok<STOUT>
        end

        function out=getNumOutputsImpl(~) %#ok<STOUT>
        end

        function out=isInactivePropertyImpl(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % load object properties
        end

        function out=releaseImpl(~) %#ok<STOUT>
        end

        function out=resetImpl(~) %#ok<STOUT>
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % save object properties
        end

        function out=setupImpl(~) %#ok<STOUT>
        end

        function out=setupTypes(~) %#ok<STOUT>
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

        function out=validateInputsImpl(~) %#ok<STOUT>
        end

    end
    properties
        %AdaptLearningRate Adapt learning rate
        %   Set this property to true to adapt the LearningRate as 1/(current 
        %   frame number) during the training period specified by 
        %   NumTrainingFrames. Set this property to false to specify learning 
        %   rate as input at each time step.
        %
        %   Default: true
        AdaptLearningRate;

        %InitialVariance Initial mixture model variance
        %   Specify the initial variance as a numeric scalar or 'Auto'.
        %   When 'Auto' is specified, the initial variance is set based on
        %   the input image data type:
        %  
        %   Image Data Type   Initial Variance
        %   ----------------  ---------------
        %   double/single     (30/255)^2
        %   uint8             30^2
        %
        %   If input is color, this value applies to all color channels.
        %
        %   Default: 'Auto'
        InitialVariance;

        %LearningRate Learning rate for parameter updates
        %   This property controls how quickly the model adapts to changing 
        %   conditions. Set this property appropriately to ensure algorithm 
        %   stability. When AdaptLearningRate is true, the LearningRate 
        %   property takes effect only after the training period specified by
        %   NumTrainingFrames is over. This property is not available when 
        %   AdaptLearningRate is false. This property is tunable.
        %
        %   Default: 0.005
        LearningRate;

        %MinimumBackgroundRatio Threshold to determine background model
        %   This property is the minimum of the apriori probabilities for 
        %   pixels to be background. If this value is too small, multimodal 
        %   backgrounds can not be handled. 
        %
        %   Default: 0.7
        MinimumBackgroundRatio;

        %NumGaussians Number of Gaussian modes
        %   Set this property to a positive integer. Typically this value is
        %   3, 4 or 5. Set this value to 3 or greater to be able to model
        %   multiple background modes.
        %
        %   Default: 5
        NumGaussians;

        %NumTrainingFrames Number of initial training frames
        %   Set this property to the number of training frames at the start of 
        %   the video sequence. This property is not available when 
        %   AdaptLearningRate is false.
        %
        %   Default: 150
        NumTrainingFrames;

    end
end
