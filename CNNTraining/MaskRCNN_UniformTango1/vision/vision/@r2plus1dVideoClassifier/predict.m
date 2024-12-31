function varargout = predict(this,dlVideo)
%predict  Compute R(2+1)D video classifier predictions.
%
%   dlYVideo = predict(rd, dlXVideo) returns R(2+1)D video classifier
%   predictions. The input rd is a r2plus1dVideoClassifier object. The
%   input dlXVideo is a dlarray object that corresponds to the video input
%   to the classifier. The size of the input must be H-by-W-by-C-by-T-by-B
%   and the dlarray data format must be 'SSCTB'. H, W, and C represent the
%   height, width, and number of channels. T is the number of frames and B
%   is the batch size. The output dlYVideo is a formatted dlarray object
%   representing the predictions of the video network.
%
%   [dlYVideo,stateVideo] = predict(rd, dlXVideo) also returns the updated
%   video network state. The state output contains information remembered
%   by the classifier between training iterations. For example, the state
%   of batch normalization operation.
%
%   Notes:
%   ------
%   The number of channels and number of frames of the input arguments
%   dlXVideo must match the corresponding values in the InputSize property.
%
%   Example: Compute predictions for video using R(2+1)D.
%   -----------------------------------------------------
%   % Load a video classifier pretrained on Kinetics-400.
%   rd = r2plus1dVideoClassifier();
%
%   % Specify the video file name.
%   videoFilename = "visiontraffic.avi";
%
%   % Create a VideoReader to read the video frames.
%   reader = VideoReader(videoFilename);
%
%   % Read number of frames corresponding to the network.
%   numFrames = rd.InputSize(4);
%   videoFrames = read(reader, [1, numFrames]);
%
%   % Resize video frames for prediction.
%   resized = imresize(videoFrames, rd.InputSize(1:2));
%
%   % Cast the input to single.
%   resized = single(resized);
%
%   % Rescale the input between 0 and 1.
%   minValue = rd.InputNormalizationStatistics.Min;
%   maxValue = rd.InputNormalizationStatistics.Max;
%   minValue = reshape(minValue, 1, 1, 3);
%   maxValue = reshape(maxValue, 1, 1, 3);
%   resized = rescale(resized,0,1,"InputMin",minValue,"InputMax",maxValue);
%
%   % Normalize using mean and standard deviation.
%   mn = rd.InputNormalizationStatistics.Mean;
%   sd = rd.InputNormalizationStatistics.StandardDeviation;
%   mn = reshape(mn, 1, 1, 3);
%   sd = reshape(sd, 1, 1, 3);
%   resized = resized - mn;
%   resized = resized ./ sd;
%
%   % Convert the input to dlarray object.
%   dlVideo = dlarray(resized, 'SSCTB');
%   activations = predict(rd,dlVideo);
%
% See also r2plus1dVideoClassifier/forward, slowFastVideoClassifier,
%          inflated3dVideoClassifier, dlarray, dlnetwork.

%   Copyright 2021-2023 The MathWorks, Inc.

    arguments
        this {mustBeA(this,"r2plus1dVideoClassifier"),mustBeNonempty}
        dlVideo {mustBeA(dlVideo,["dlarray","gpuArray"]),iValidateInputFrames(dlVideo)}
    end

    % Convert Time to spatial.    
    dlVideo = dlarray(dlVideo, "SSCBS");
    [varargout{1:nargout}] = this.Network.predict(dlVideo);
end

function iValidateInputFrames(dlVideo)
    if ~isdlarray (dlVideo) || ...
       isempty(dims(dlVideo)) || ...
       ~isequal(string(dims(dlVideo)), "SSCBT")
        error(message('r2plus1d:r2plus1dVideoClassifier:mustBeAFormattedDlarray'));
    end

    if ~isequal(underlyingType(dlVideo), 'single')
        error(message('r2plus1d:r2plus1dVideoClassifier:mustBeASingleType'));
    end
end
