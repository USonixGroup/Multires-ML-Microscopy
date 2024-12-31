function this = updateSequence(this,frame,nvp)
%updateSequence  Update the video and optical flow sequence for classification.
%
%   i3d = updateSequence(i3d, videoFrame) updates the video and optical
%   flow sequences with the given videoFrame for classification. The input
%   i3d is a inflated3dVideoClassifier object. videoFrame must be numeric
%   array in the format H-by-W-by-C where H, W, and C represent the height,
%   width and number of channels. The number of channels must match the
%   number of channels of i3d.InputSize. The output
%   inflated3dVideoClassifier object contains the updated video sequence.
%
%   To classify the updated sequence, use the classifySequence function.
%
%   [...] = updateSequence(..., 'ExecutionEnvironment', execEnv) specifies
%   the hardware resources used to update the sequence. Valid values are:
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
%   ------
%    - The video classifier object maintains the sequence as a
%      First-In-First-Out (FIFO) queue of size specified by the number of
%      frames in the InputSize property. If the sequence is full, the frame
%      at the end of the sequence is removed and the input videoFrame is
%      added to the beginning of the sequence queue. The video frame is
%      resized to the input size, if needed.
%    - When the OpticalFlowMethod is 'Farneback', the classifier
%      uses opticalFlowFarneback to calculate the optical flow values.
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

    arguments
        this {mustBeA(this,"inflated3dVideoClassifier"),mustBeNonempty}
        frame {mustBeNumeric, mustBeNonempty}
        nvp.ExecutionEnvironment = "auto"
    end

    iValidateFrame(frame, this.InputSize);
    exe = iValidateExecutionEnvironment(nvp.ExecutionEnvironment);

    % Setting this to false, does not create new optical flow object for
    % every frame. updateSequence does not create a new optical flow
    % object, but keeps using the existing optical flow object.
    useFirstFrameInData = false;

    videoFlowData = preprocess(this,frame,useFirstFrameInData);
    if isequal(this.OpticalFlowMethod, 'none')
        videoFrame = videoFlowData;
    else
        videoFrame = videoFlowData{1};
    end

    [this,videoSequence,flowSequence] = iGetSequences(this);

    videoSequence = iUpdateFIFOSequence(videoSequence,videoFrame,this.SequenceLength,this.NumFramesCollected);

    % Based on the execution environment, create gpuArray or gather
    % from the GPU.
    this.PrivateVideoSequence = gpuOrCPUSequence(videoSequence,exe);

    if isequal(this.OpticalFlowMethod, 'Farneback')
        flowFrame = videoFlowData{2};
        flowSequence = iUpdateFIFOSequence(flowSequence,flowFrame,this.SequenceLength,this.NumFramesCollected);
        this.PrivateOpticalFlowSequence = gpuOrCPUSequence(flowSequence,exe);
    end
    if this.NumFramesCollected < this.SequenceLength
        this.NumFramesCollected = this.NumFramesCollected + 1;
    end
end

function [this,videoSequence,flowSequence] = iGetSequences(this)
    videoSequence = this.PrivateVideoSequence;
    flowSequence = this.PrivateOpticalFlowSequence;
    if isempty(videoSequence)
        % First time initialization of the sequence.
        this = resetSequence(this);
        videoSequence = this.PrivateVideoSequence;
        flowSequence = this.PrivateOpticalFlowSequence;
    end
end

function sequence = iUpdateFIFOSequence(sequence,frame,sequenceLength,numFramesCollected)
    if numFramesCollected == sequenceLength
        % FIFO - Push the earliest frame out,
        % add the latest frame.
        frames = sequence(:,:,:,2:end);
        sequence(:,:,:,1:end-1) = frames;
        sequence(:,:,:,end) = frame;
    elseif numFramesCollected < sequenceLength
        sequence(:,:,:,numFramesCollected+1) = frame;
    end
end

%------------------------------------------------------------------
function iValidateFrame(frame,networkInputSize)

    [imSz(1),imSz(2),imSz(3)] = size(frame);

    imageChannelSize = imSz(3);
    networkChannelSize = networkInputSize(3);

    % Validate number of channels for input image and network input
    if imageChannelSize ~= networkChannelSize
        error(message('inflated3d:inflated3dVideoClassifier:invalidInputFrameChannelSize',imageChannelSize,networkChannelSize));
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
