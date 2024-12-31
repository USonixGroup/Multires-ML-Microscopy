function this = resetSequence(this)
%resetSequence  Reset the video sequence for classification.
%
%   i3d = resetSequence(i3d) resets the video sequence for classification.
%   The input i3d is a inflated3dVideoClassifier object.
%
%   To update the video and optical flow sequences with data, use the updateSequence
%   function. To classify the updated sequences, use the classifySequence
%   function.
%
%   Example: Classify video sequences by streaming a video file.
%   ------------------------------------------------------------
%
%   % Load a pretrained Inflated-3D video classifier.
%   i3d = inflated3dVideoClassifier();
%
%   % Specify the video file name to stream video frames.
%   videoFilename = "visiontraffic.avi";
%
%   % Create a VideoReader to read video.
%   reader = VideoReader(videoFilename);
%
%   % Setup a video player.
%   player = vision.VideoPlayer;
%
%   % Specify number of frames at which to classify.
%   classifyInterval = 10;
%
%   % The number of frames to update the sequence before
%   % classification.
%   sequenceLength = i3d.InputSize(4);
%
%   numFrames = 0;
%   text = "";
%
%   while hasFrame(reader)
%       frame = readFrame(reader);
%       numFrames = numFrames + 1;
%
%       % Update the sequence with the next video frame.
%       i3d = updateSequence(i3d,frame);
%
%       % Classify the sequence only at every classifyInterval number of frames.
%       if mod(numFrames, classifyInterval) == 0 && numFrames >= sequenceLength
%            [label,score] = classifySequence(i3d);
%            text = string(label) + "; " + num2str(score, "%0.2f");
%       end
%       frame = insertText(frame,[30,30],text,'FontSize',18);
%       step(player,frame);
%   end
%
%   % Note that the classifier is not fine-tuned to compute the correct
%   % activations for visiontraffic.avi. You must train the classifier for
%   % optimal performance on your video data.
%
%   See also slowFastVideoClassifier, r2plus1dVideoClassifier,
%            inflated3dVideoClassifier/classifySequence, inflated3dVideoClassifier/predict.

%   Copyright 2021-2023 The MathWorks, Inc.

    this.NumFramesCollected = 0;
    this.PrivateVideoSequence = zeros(this.InputSize, "single");

    if isequal(this.OpticalFlowMethod, 'Farneback')
        flowInputSize = [this.InputSize(1:2), 2, this.SequenceLength];
        this.PrivateOpticalFlowSequence = zeros(flowInputSize, "single");
        this.OpticalFlowObject = opticalFlowFarneback;
    end
end
