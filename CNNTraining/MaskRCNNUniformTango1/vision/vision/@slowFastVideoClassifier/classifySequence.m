function [label,score] = classifySequence(this,nvp)
%classifySequence  Classify the video sequence using the SlowFast Video Classifier.
%
%   label = classifySequence(sf) classifies the video sequence stored in
%   the VideoSequence property. The input sf is an slowFastVideoClassifier
%   object. The output label is a scalar categorical specifying the
%   classification of the video sequence by the video classifier object.
%   The label is one of the values of the Classes property of the video
%   classifier object.
%
%   [label, score] = classifySequence(sf) additionally returns the
%   classification score associated with the label. The score value is
%   between 0 and 1 and represent the confidence of the predicted class
%   label.
%
%   To update the VideoSequence property with new video frames, use
%   updateSequence before classifySequence.
%
%   [...] = classifySequence(..., 'ExecutionEnvironment', execEnv)
%   specifies the hardware resources used to run the classification. Valid
%   values are:
%
%         'auto' - Use a GPU if it is available, otherwise use the CPU.
%
%         'gpu'  - Use the GPU. To use a GPU, you must have Parallel
%                  Computing Toolbox(TM), and a supported GPU. If a
%                  suitable GPU is not available, an error message is
%                  issued.
%
%         'cpu'   - Use the CPU.
%
%         Default : 'auto'
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
%          slowFastVideoClassifier/updateSequence, slowFastVideoClassifier/predict.

%   Copyright 2021-2023 The MathWorks, Inc.

    arguments
        this {mustBeA(this,"slowFastVideoClassifier"),mustBeNonempty}
        nvp.ExecutionEnvironment = "auto"
    end

    exe = iValidateExecutionEnvironment(nvp.ExecutionEnvironment);

    sequenceLength = this.SequenceLength;
    if this.NumFramesCollected < sequenceLength 
        error(message('slowfast:slowFastVideoClassifier:classifySequenceNotEnoughFrames', string(sequenceLength)));
    end

    % Based on the execution environment, create gpuArray or gather
    % from the GPU.
    sequence = gpuOrCPUSequence(this.PrivateVideoSequence,exe);

    dlX = dlarray(sequence, "SSCTB");
    dlYPred = predict(this,dlX);
    scores = extractdata(gather(squeeze(dlYPred)));
    [score,idx] = max(scores,[],1);
    label = this.Classes(idx);
end

%------------------------------------------------------------------
function exe = iValidateExecutionEnvironment(exe)
%validateExecutionEnvironment Validate environment for classifySequence.

    exe = vision.internal.cnn.validation.checkExecutionEnvironment(exe,mfilename);
    exe = string(exe);

    if ~canUseGPU && isequal(exe, "gpu")
        error(message('slowfast:slowFastVideoClassifier:noGPUAvailable'));
    end

    if isequal(exe, "auto")
        if canUseGPU
            exe = "gpu";
        else
            exe = "cpu";
        end
    end
end
