function varargout = predict(this,dlFast)
%predict  Compute SlowFast video classifier predictions.
%
%   dlYVideo = predict(sf, dlXVideo) returns SlowFast video classifier
%   predictions. The input sf is a slowFastVideoClassifier object. The
%   input dlXVideo is a dlarray object that corresponds to the video input
%   to the classifier. The size of the input must be H-by-W-by-C-by-T-by-B
%   and the dlarray data format must be 'SSCTB'. H, W, and C represent the
%   height, width, and number of channels. T is the number of frames and B
%   is the batch size. The output dlYVideo is a formatted dlarray object
%   representing the predictions of the video network.
%
%   [dlYVideo,stateVideo] = predict(sf, dlXVideo) also returns the updated
%   video network state. The state output contains information remembered
%   by the classifier between training iterations. For example, the state
%   of batch normalization operation.
%
%   Notes:
%   ------
%   The number of channels and number of frames of the input arguments
%   dlXVideo must match the corresponding values in the InputSize property.
%
%   Example: Compute predictions for video using SlowFast.
%   ------------------------------------------------------
%   % Load a video classifier pretrained on Kinetics-400.
%   sf = slowFastVideoClassifier();
%
%   % Specify the video file name.
%   videoFilename = "pushup.mp4";
%
%   % Create a VideoReader to read the video frames.
%   reader = VideoReader(videoFilename);
%
%   % Read number of frames corresponding to the network.
%   numFrames = sf.InputSize(4);
%   videoFrames = read(reader, [1, numFrames]);
%
%   % Resize video frames for prediction.
%   resized = imresize(videoFrames, sf.InputSize(1:2));
%
%   % Cast the input to single.
%   resized = single(resized);
%
%   % Rescale the input between 0 and 1.
%   minValue = sf.InputNormalizationStatistics.Min;
%   maxValue = sf.InputNormalizationStatistics.Max;
%   minValue = reshape(minValue, 1, 1, 3);
%   maxValue = reshape(maxValue, 1, 1, 3);
%   resized = rescale(resized,0,1,"InputMin",minValue,"InputMax",maxValue);
%
%   % Normalize using mean and standard deviation.
%   mn = sf.InputNormalizationStatistics.Mean;
%   sd = sf.InputNormalizationStatistics.StandardDeviation;
%   mn = reshape(mn, 1, 1, 3);
%   sd = reshape(sd, 1, 1, 3);
%   resized = resized - mn;
%   resized = resized ./ sd;
%
%   % Convert the input to dlarray object.
%   dlVideo = dlarray(resized, 'SSCTB');
%   activations = predict(sf,dlVideo);
%
% See also slowFastVideoClassifier/forward, r2plus1dVideoClassifier,
%          inflated3dVideoClassifier, dlarray, dlnetwork.

%   Copyright 2021-2023 The MathWorks, Inc.

    arguments
        this {mustBeA(this,"slowFastVideoClassifier"),mustBeNonempty}
        dlFast {mustBeA(dlFast,"dlarray"),iValidateInputFrames(dlFast)}
    end

    dlSlow = getSlowFromFast(this,dlFast);
    % Convert Time to spatial.
    dlSlow = dlarray(dlSlow, "SSCBS");
    dlFast = dlarray(dlFast, "SSCBS");
    [varargout{1:nargout}] = this.Network.predict(dlSlow,dlFast);
end

function iValidateInputFrames(dlFast)
    if ~isdlarray (dlFast) || ...
       isempty(dims(dlFast)) || ...
       ~isequal(string(dims(dlFast)), "SSCBT")
        error(message('slowfast:slowFastVideoClassifier:mustBeAFormattedDlarray'));
    end

    if ~isequal(underlyingType(dlFast), 'single')
        error(message('slowfast:slowFastVideoClassifier:mustBeASingleType'));
    end
end
