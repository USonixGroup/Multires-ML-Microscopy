function [label,score] = classifyVideoFile(this, videoFilename, nvp)
%classifyVideoFile Classify a video file using the R(2+1)D video classifier.
%
%   classifyVideoFile is recommended for applications where the entire
%   video can be classified with one label. For example, to evaluate the
%   performance of the classifier on a collection of ground truth video
%   files. If the video contains sequences with different class labels, use
%   classifySequence.
%
%   label = classifyVideoFile(rd, videoFilename) predicts the class label
%   for the video file, videoFilename, using the r2plus1dVideoClassifier,
%   rd. The output label is a scalar categorical. The input videoFilename
%   is a character vector or a string scalar specifying the video file for
%   classification. The video file must be readable by a VideoReader
%   object.
%
%   [label, score] = classifyVideoFile(rd, videoFilename) additionally
%   returns the classification score associated with the label. The score
%   value is between 0 and 1 and represents the confidence of the predicted
%   class label.
%
%   [...] = classifyVideoFile(..., Name, Value) specifies additional
%   name-value pairs described below:
%
%   'NumSequences'
%
%        Specify the maximum number of sequences to sample from the video
%        file. Valid options are 'auto' or a positive scalar integer,
%        numSequences.
%
%           'auto'       - The number of sequences is twice the number of
%                          sequences that is possible from the video file
%                          based on number of frames available. This
%                          provides uniform coverage over the entire video.
%
%           numSequences - Specify the maximum number of clips to select
%                          from the video file. Increase the value to
%                          ensure a more uniform coverage of the video.
%
%        Default : 'auto'
%
%
%   'MiniBatchSize'
%
%        The mini-batch size used for processing a large video. Video
%        sequences are grouped into mini-batches and processed as a batch
%        to improve computational efficiency. Larger mini-batch sizes lead
%        to faster processing, at the cost of more memory.
%
%        Default: 4
%
%   'ExecutionEnvironment'
%
%        The hardware resources used to run the classifier.
%        Valid values are:
%
%           'auto' - Use a GPU if it is available, otherwise use the CPU.
%
%           'gpu'  - Use the GPU. To use a GPU, you must have Parallel
%                    Computing Toolbox(TM), and a supported GPU. If a
%                    suitable GPU is not available, an error message is
%                    issued.
%
%           'cpu'   - Use the CPU.
%
%        Default : 'auto'
%
%   Note:
%      classifyVideoFile selects sequences with T frames from the video
%      file using uniform random sampling to select the start time of each
%      sequence. T is the number of frames in the InputSize property. The
%      label with the maximum score in the video file is chosen as the
%      classification label for the file. When the video file has less than
%      T frames, the partial video sequence is repeated to form the
%      required size.
%
%   Example: Classify a video file.
%   -------------------------------
%
%   % Load a pretrained R(2+1)D video network.
%   rd = r2plus1dVideoClassifier();
%
%   % Specify the video file name to classify.
%   videoFilename = 'visiontraffic.avi';
%
%   % Choose 32 randomly selected video sequences to classify the video.
%   numSequences = 32;
%
%   % Classify the video using the video classifier.
%   label = classifyVideoFile(rd, videoFilename, 'NumSequences', numSequences)
%
%   See also slowFastVideoClassifier, inflated3dVideoClassifier,
%          r2plus1dVideoClassifier/classifySequence, r2plus1dVideoClassifier/predict.

%   Copyright 2021-2023 The MathWorks, Inc.

    arguments
        this          {mustBeA(this,"r2plus1dVideoClassifier"),mustBeNonempty}
        videoFilename {mustBeNonempty,iValidateVideoFilename(videoFilename)}
        nvp.NumSequences         = "auto";
        nvp.MiniBatchSize        = 4;
        nvp.ExecutionEnvironment = "auto";
    end

    nvp = iParseNVPairs(nvp);

    [reader, info] = iGetReaderInformation(videoFilename, this.SequenceLength, nvp.NumSequences);

    readFcn = @(f,u)vision.internal.cnn.readVideoSamples(f,u,reader,info);
    ds = fileDatastore(videoFilename,...
        'ReadFcn',readFcn,...
        'ReadMode','partialfile');

    ds = transform(ds,@(data)preprocess(this,data));

    mbq = minibatchqueue(ds, ...
        "MiniBatchSize", nvp.MiniBatchSize, ...
        "OutputEnvironment",nvp.ExecutionEnvironment,...
        "MiniBatchFormat", "SSCTB");
    numBatches = ceil(info.NumSequences / mbq.MiniBatchSize);
    scores = cell(numBatches, 1);
    labels = cell(numBatches, 1);
    classes = this.Classes;
    ii = 1;
    while hasdata(mbq)
        dlVideo = next(mbq);
        score = predict(this,dlVideo);
        [~,idx] = max(score);
        labels{ii} = classes(idx);
        scores{ii} = score;
        ii = ii + 1;
    end

    labels = vertcat(labels{:});

    % Classify the video by finding the maximum score of labels.
    scores = cat(5,scores{:});
    scores = extractdata(gather(scores));
    if info.NumSequences ~= 1
        scores = squeeze(scores); % scores is "CB"
        scores = max(scores,[],2); % maximum among all the batches.
    end
    [score, idx] = max(scores); % maximum among the label scores.
    label = classes(idx);
end

%------------------------------------------------------------------
function [reader, info] = iGetReaderInformation(filename, sequenceLength, userProvided)
    reader = VideoReader(filename);
    totalFrames = floor(reader.Duration * reader.FrameRate);
    totalFrames = min(totalFrames, reader.NumFrames);
    if totalFrames < sequenceLength
        % If the file contains lesser number of frames than the network,
        % use one clip.
        numSequences = 1;
    else
        if isequal(userProvided, "auto")
            numSequences = 2 * ceil(totalFrames/sequenceLength);
        else
            numSequences = userProvided;
        end
    end

    info.NumFrames    = sequenceLength;
    info.TotalFrames  = totalFrames;
    info.NumSequences = numSequences;
end

%------------------------------------------------------------------
function nvp = iParseNVPairs(nvp)
    nvp.NumSequences = iValidateNumSequences(nvp.NumSequences);
    nvp.MiniBatchSize = iValidateNumericScalar(nvp.MiniBatchSize, 'MiniBatchSize');
    nvp.ExecutionEnvironment = ...
        vision.internal.cnn.validation.checkExecutionEnvironment(nvp.ExecutionEnvironment,mfilename);
end

%------------------------------------------------------------------
function numSequences = iValidateNumSequences(numSequences)
    if ischar(numSequences)
        numSequences = string(numSequences);
    end
    if isequal(numSequences, "auto")
        return;
    end
    numSequences = iValidateNumericScalar(numSequences,'NumSequences');
end

%------------------------------------------------------------------
function value = iValidateNumericScalar(value, name)
    classes = {'numeric'};
    attrs = {'scalar','positive','integer','nonempty','nonzero',...
        'finite','nonnan','nonsparse',};
    validateattributes(value, classes, attrs, mfilename, name);
    value = double(value);
end

%------------------------------------------------------------------
function iValidateVideoFilename(videoFilename)
    classes = {'string','char'};
    attrs   = {'scalartext'};
    validateattributes(videoFilename, classes, attrs, mfilename, 'videoFilename', 1);
end
