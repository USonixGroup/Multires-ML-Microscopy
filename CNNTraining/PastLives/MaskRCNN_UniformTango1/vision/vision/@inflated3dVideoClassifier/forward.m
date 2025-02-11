function varargout = forward(this,varargin)
%forward  Compute Inflated-3D video classifier outputs for training.
%
%   dlYVideo = forward(i3d, dlXVideo) returns Inflated-3D video classifier
%   outputs for training. The input i3d is a inflated3dVideoClassifier
%   object. The input dlXVideo is a dlarray object that corresponds to the
%   video input to the classifier. The size of the input must be
%   H-by-W-by-C-by-T-by-B and the dlarray data format must be 'SSCTB'. H,
%   W, and C represent the height, width, and number of channels. T is the
%   number of frames and B is the batch size. The output dlYVideo is a
%   formatted dlarray object representing the activations of the video
%   network.
%
%   [dlYVideo,stateVideo] = forward(i3d, dlXVideo) also returns the updated
%   video network state. The state output contains information remembered
%   by the classifier between training iterations. For example, the state
%   of batch normalization operation.
%
%   [dlYVideo,dlYFlow] = forward(i3d, dlXVideo, dlXFlow) returns video
%   classifier video and optical flow outputs for training. Use this syntax
%   when OpticalFlowMethod property is 'Farneback'. The inputs dlXVideo and
%   dlXFlow are dlarray objects that correspond to the video input and
%   optical flow input, respectively. The size of the input must be
%   H-by-W-by-C-by-T-by-B and the dlarray data format must be 'SSCTB'. The
%   outputs, dlYVideo and dlYFlow, are formatted dlarray objects
%   representing the activations of the video and optical flow network,
%   respectively.
%
%   [dlYVideo,dlYFlow,stateVideo,stateFlow] = forward(i3d, dlXVideo, dlXFlow)
%   also returns the updated video network and optical flow network states.
%
%   Notes:
%   ------
%    - The number of channels and number of frames of the input arguments
%      dlXVideo must match the corresponding values in the InputSize 
%      property.
%    - The number of frames of the input arguments dlXFlow must match the
%      corresponding values in the InputSize property. The number of
%      channels must be 2.
%
%   Example: Forward video using Inflated-3D for training.
%   ------------------------------------------------------
%   % Load a video classifier pretrained on Kinetics-400.
%   i3d = inflated3dVideoClassifier();
%
%   % Specify the video file name.
%   videoFilename = "visiontraffic.avi";
%
%   % Create a VideoReader to read the video frames.
%   reader = VideoReader(videoFilename);
%
%   % Read number of frames corresponding to the network.
%   numFrames = i3d.InputSize(4);
%   videoFrames = read(reader, [1, numFrames]);
%
%   % Resize the video frames.
%   resized = imresize(videoFrames, i3d.InputSize(1:2));
%
%   % Cast the input to single.
%   resized = single(resized);
%
%   % Rescale the input between -1 and 1.
%   minValue = reshape(i3d.InputNormalizationStatistics.Video.Min,1,1,3);
%   maxValue = reshape(i3d.InputNormalizationStatistics.Video.Max,1,1,3);
%   resized = rescale(resized,-1,1,"InputMin",minValue,"InputMax",maxValue);
%
%   % Convert the input to dlarray object.
%   dlVideo = dlarray(resized, 'SSCTB');
%   activations = forward(i3d,dlVideo);
%
%   % Note that the classifier is not fine-tuned to compute the correct
%   % activations for visiontraffic.avi. You must train the classifier for
%   % optimal performance on your video data.
%
% See also inflated3dVideoClassifier/predict, slowFastVideoClassifier,
%          r2plus1dVideoClassifier, dlarray, dlnetwork.

%   Copyright 2021-2023 The MathWorks, Inc.

    narginchk(2,3);
    outNumArgs = nargout;
    switch nargin
        case 2
            nargoutchk(0,2);
            if isequal(this.OpticalFlowMethod, 'Farneback')
                error(message('inflated3d:inflated3dVideoClassifier:forwardInputMustContainFlow'));
            end

            [dlYVideo,videoState] = iForwardData(this.VideoNetwork,varargin{1});
            outargs = {dlYVideo, videoState};
        case 3
            nargoutchk(0,4);
            if isequal(this.OpticalFlowMethod, 'none')
                error(message('inflated3d:inflated3dVideoClassifier:forwardInputMustContainOnlyVideo'));
            end

            [dlYVideo,videoState] = iForwardData(this.VideoNetwork,varargin{1});
            [dlYFlow,flowState]   = iForwardData(this.OpticalFlowNetwork,varargin{2});

            outargs = {dlYVideo, dlYFlow, videoState, flowState};
    end

    if outNumArgs == 0
        % Allow MATLAB like assignment to workspace variable "ans".
        outNumArgs = 1;
    end

    [varargout{1:outNumArgs}] = outargs{1:outNumArgs};
end

%------------------------------------------------------------------
function [dlY,state] = iForwardData(network,dlX)
    iValidateInputFrames(dlX);
    % Convert Time to spatial.    
    dlX = dlarray(dlX, "SSCBS");
    [dlY,state] = network.forward(dlX);
end

%------------------------------------------------------------------
function iValidateInputFrames(dlX)
    mustBeA(dlX,"dlarray");
    if isempty(dims(dlX)) || ...
       ~isequal(string(dims(dlX)), "SSCBT")
        error(message('inflated3d:inflated3dVideoClassifier:mustBeAFormattedDlarray'));
    end

    if ~isequal(underlyingType(dlX), 'single')
        error(message('inflated3d:inflated3dVideoClassifier:mustBeASingleType'));
    end
end
