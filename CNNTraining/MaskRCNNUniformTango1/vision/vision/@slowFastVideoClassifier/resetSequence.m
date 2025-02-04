function this = resetSequence(this)
%resetSequence  Reset the video sequence for classification.
%
%   sf = resetSequence(sf) resets the video sequence for
%   classification. The input sf is a slowFastVideoClassifier object.
%
%   To update the video sequence with data, use the updateSequence
%   function. To classify the updated sequence, use the classifySequence
%   function.
%
%   Example: Classify video sequences by streaming a video file.
%   ------------------------------------------------------------
%
%   % Load a pretrained SlowFast video classifier.
%   sf = slowFastVideoClassifier();
%
%   % Specify the video file name to stream video frames.
%   videoFilename = "pushup.mp4";
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
%   sequenceLength = sf.InputSize(4);
%
%   numFrames = 0;
%   text = "";
%
%   while hasFrame(reader)
%       frame = readFrame(reader);
%       numFrames = numFrames + 1;
%
%       % Update the sequence with the next video frame.
%       sf = updateSequence(sf,frame);
%
%       % Classify the sequence only at every classifyInterval number of frames.
%       if mod(numFrames, classifyInterval) == 0 && numFrames >= sequenceLength
%            [label,score] = classifySequence(sf);
%            text = string(label) + "; " + num2str(score, "%0.2f");
%       end
%       frame = insertText(frame,[30,30],text,'FontSize',18);
%       step(player,frame);
%   end
%
%   See also r2plus1dVideoClassifier, inflated3dVideoClassifier,
%          slowFastVideoClassifier/classifySequence, slowFastVideoClassifier/predict.

%   Copyright 2021-2023 The MathWorks, Inc.

    this.NumFramesCollected = 0;
    this.PrivateVideoSequence = zeros(this.InputSize, "single");
end
