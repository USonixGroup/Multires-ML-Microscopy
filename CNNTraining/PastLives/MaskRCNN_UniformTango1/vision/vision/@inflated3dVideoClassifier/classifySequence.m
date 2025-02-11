function [label,score] = classifySequence(this,nvp)
%classifySequence  Classify the video sequence using the Inflated-3D Video Classifier.
%
%   label = classifySequence(i3d) classifies the video and optical flow
%   sequences stored in the VideoSequence and OpticalFlowSequence
%   propertes. The input i3d is an inflated3dVideoClassifier object. The
%   output label is a scalar categorical specifying the classification of
%   the video sequence by the video classifier object. The label is one of
%   the values of the Classes property of the video classifier object.
%
%   [label, score] = classifySequence(i3d) additionally returns the
%   classification score associated with the label. The score value is
%   between 0 and 1 and represent the confidence of the predicted class
%   label.
%
%   To update the VideoSequence and OpticalFlowSequence properties with new
%   video frames, use updateSequence before classifySequence.
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
%            inflated3dVideoClassifier/updateSequence, inflated3dVideoClassifier/predict.

%   Copyright 2021-2023 The MathWorks, Inc.

    arguments
        this {mustBeA(this,"inflated3dVideoClassifier"),mustBeNonempty}
        nvp.ExecutionEnvironment = "auto"
    end

    exe = iValidateExecutionEnvironment(nvp.ExecutionEnvironment);

    sequenceLength = this.SequenceLength;
    if this.NumFramesCollected < sequenceLength 
        error(message('inflated3d:inflated3dVideoClassifier:classifySequenceNotEnoughFrames', string(sequenceLength)));
    end

    % Based on the execution environment, create gpuArray or gather
    % from the GPU.
    videoSequence = gpuOrCPUSequence(this.PrivateVideoSequence,exe);
    dlVideo = dlarray(videoSequence, "SSCTB");
    if isequal(this.OpticalFlowMethod, 'Farneback')
        flowSequence = gpuOrCPUSequence(this.PrivateOpticalFlowSequence,exe);
        dlFlow = dlarray(flowSequence, "SSCTB");

        [dlYPredVideo, dlYPredFlow] = predict(this,dlVideo,dlFlow);

        videoScores = extractdata(gather(squeeze(dlYPredVideo)));
        flowScores = extractdata(gather(squeeze(dlYPredFlow)));
        avgScore = (videoScores + flowScores)./2;

        [score,idx] = max(avgScore,[],1);
        label = this.Classes(idx);
    else
        dlYPredVideo = predict(this,dlVideo);
        scores = extractdata(gather(squeeze(dlYPredVideo)));
        [score,idx] = max(scores,[],1);
        label = this.Classes(idx);
    end

end

%------------------------------------------------------------------
function exe = iValidateExecutionEnvironment(exe)
%validateExecutionEnvironment Validate environment for classifySequence.

    exe = vision.internal.cnn.validation.checkExecutionEnvironment(exe,mfilename);
    exe = string(exe);

    if ~canUseGPU && isequal(exe, "gpu")
        error(message('inflated3d:inflated3dVideoClassifier:noGPUAvailable'));
    end

    if isequal(exe, "auto")
        if canUseGPU
            exe = "gpu";
        else
            exe = "cpu";
        end
    end
end
