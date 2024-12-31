function this = updateSequence(this,frame,nvp)
%updateSequence  Update the video and optical flow sequence for classification.
%
%   sf = updateSequence(sf, videoFrame) updates the video sequence with the
%   given videoFrame for classification. The input sf is a
%   slowFastVideoClassifier object. videoFrame must be numeric array in the
%   format H-by-W-by-C where H, W, and C represent the height, width and
%   number of channels. The number of channels must match the number of
%   channels of sf.InputSize. The output slowFastVideoClassifier object
%   contains the updated video sequence.
%
%   To classify the updated sequence, use the classifySequence function.
%
%   [...] = updateSequence(..., 'ExecutionEnvironment', execEnv)
%   specifies the hardware resources used to update the sequence. Valid
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
%   Notes:
%      The video classifier object maintains the sequence as a
%      First-In-First-Out (FIFO) queue of size specified by the number of
%      frames in the InputSize property. If the sequence is full, the frame
%      at the end of the sequence is removed and the input videoFrame is
%      added to the beginning of the sequence queue. The video frame
%      is resized to the input size, if needed.
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

    arguments
        this {mustBeA(this,"slowFastVideoClassifier"),mustBeNonempty}
        frame {mustBeNumeric, mustBeNonempty}
        nvp.ExecutionEnvironment = "auto"
    end

    iValidateFrame(frame, this.InputSize);
    exe = iValidateExecutionEnvironment(nvp.ExecutionEnvironment);

    frame = preprocess(this,frame);

    sequence = this.PrivateVideoSequence;

    sequenceLength = this.SequenceLength;
    if this.NumFramesCollected == sequenceLength
        % FIFO - Push the earliest frame out,
        % add the latest frame.
        frames = sequence(:,:,:,2:end);
        sequence(:,:,:,1:end-1) = frames;
        sequence(:,:,:,end) = frame;
    elseif this.NumFramesCollected < sequenceLength
        if isempty(sequence)
            % First time initialization of the sequence.
            this = resetSequence(this);
            sequence = this.PrivateVideoSequence;
        end
        this.NumFramesCollected = this.NumFramesCollected + 1;
        sequence(:,:,:,this.NumFramesCollected) = frame;
    end

    % Based on the execution environment, create gpuArray or gather
    % from the GPU.
    this.PrivateVideoSequence = gpuOrCPUSequence(sequence,exe);
end

%------------------------------------------------------------------
function iValidateFrame(frame, networkInputSize)

    [imSz(1),imSz(2),imSz(3)] = size(frame);

    imageChannelSize = imSz(3);
    networkChannelSize = networkInputSize(3);

    % Validate number of channels for input image and network input
    if imageChannelSize ~= networkChannelSize
        error(message('slowfast:slowFastVideoClassifier:invalidInputFrameChannelSize',imageChannelSize,networkChannelSize));
    end

    if networkChannelSize > 3 || networkChannelSize == 2
        args = {frame, 'I', 'multi-channel'};
    else
        args = {frame, 'I'};
    end
    % multi-channel or grayscale or RGB images allowed
    vision.internal.inputValidation.validateImage(args{:});
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
