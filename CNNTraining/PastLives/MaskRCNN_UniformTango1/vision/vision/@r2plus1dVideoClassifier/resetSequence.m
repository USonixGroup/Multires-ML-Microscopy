function this = resetSequence(this)
%resetSequence  Reset the video sequence for classification.
%
%   rd = resetSequence(rd) resets the video sequence for classification.
%   The input rd is a r2plus1dVideoClassifier object.
%
%   To update the video sequence with data, use the updateSequence
%   function. To classify the updated sequence, use the classifySequence
%   function.
%
%   Example: Classify video sequences by streaming a video file.
%   ------------------------------------------------------------
%
%   % Load a pretrained R(2+1)D video classifier.
%   rd = r2plus1dVideoClassifier();
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
%   sequenceLength = rd.InputSize(4);
%
%   numFrames = 0;
%   text = "";
%
%   while hasFrame(reader)
%       frame = readFrame(reader);
%       numFrames = numFrames + 1;
%
%       % Update the sequence with the next video frame.
%       rd = updateSequence(rd,frame);
%
%       % Classify the sequence only at every classifyInterval number of frames.
%       if mod(numFrames, classifyInterval) == 0 && numFrames >= sequenceLength
%            [label,score] = classifySequence(rd);
%            text = string(label) + "; " + num2str(score, "%0.2f");
%       end
%       frame = insertText(frame,[30,30],text,'FontSize',18);
%       step(player,frame);
%   end
%
%   See also slowFastVideoClassifier, inflated3dVideoClassifier,
%          r2plus1dVideoClassifier/classifySequence, r2plus1dVideoClassifier/predict.

%   Copyright 2021-2023 The MathWorks, Inc.

    this.NumFramesCollected = 0;
    this.PrivateVideoSequence = zeros(this.InputSize, "single");
end
