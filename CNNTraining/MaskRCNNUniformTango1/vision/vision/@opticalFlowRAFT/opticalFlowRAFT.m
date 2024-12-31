%opticalFlowRAFT Estimate optical flow using the RAFT algorithm.
%
% References:
%     [1] Zachary Teed and Jia Deng. "RAFT: Recurrent All-Pairs Field Transforms
%     Optical Flow". Proceedings of the 16th European Conference on Computer
%     Vision, Online 2020.
%     [2] Neelay Shah, Prajnan Goswami and Huaizu Jiang. "EzFlow: A modular PyTorch
%     library for optical flow estimation using neural networks", Online 2021,
%     https://github.com/neu-vi/ezflow.

% Copyright 2021-2024 The MathWorks, Inc.

classdef opticalFlowRAFT < handle & vision.internal.EnforceScalarHandle

    % Configurations properties
    properties  (Access = private, Hidden)
        % Option to enable reusing previous frame's flow to initialize the 
        % current flow. Default is false. This improves accuracy in cases
        % where there is very little texture in the video frames.
        UsePreviousFlow = false
    end

    properties (Access = ?matlab.unittest.TestCase)
        Raft                   % Internal DL model
        Version = 1.0          % Internal versioning for saving/loading objects
        MinInputSize = [57 57] % Minimum input size based on the pre-trained model configuration.

        FirstCall = true         % Boolean flag of first call
        PreviousFrameBuffer = [] % previous frame
        
        InRows = 0     % input row size
        InCols = 0     % input column size
        InChannels = 0 % input number of channels
        InBatch = 0    % input batch size

        InputSize = []
    end

    methods
        % Constructor
        function obj = opticalFlowRAFT()

            % Add support package tripwireto ensure it is installed
            iTripwireOpticalFlowRAFT();

            % load the RAFT model
            obj.Raft = vision.internal.cnn.raft.RAFT();
        end

        % Predict method
        function opticalFlow = estimateFlow(obj, imageCurr, options)
            arguments
                obj
                imageCurr {checkImage} % [h,w,3 or 1,b]
                options.ExecutionEnvironment (1,1) string  {mustBeMember(options.ExecutionEnvironment,["auto","cpu","gpu"])} = "auto"
                options.MaxIterations (1,1) {mustBeInteger, mustBePositive, mustBeFinite, mustBeReal, mustBeNonempty} = 12
                options.Tolerance (1,1) {mustBeNumeric, mustBePositive, mustBeFinite, mustBeReal, mustBeNonempty} = 1e-5
                options.Acceleration (1,1) string  {mustBeMember(options.Acceleration,["auto","mex","none"])} = "auto"
                options.MiniBatchSize (1,1) {mustBeInteger, mustBePositive, mustBeFinite, mustBeReal, mustBeNonempty, mustBeScalarOrEmpty} = size(imageCurr,4)
            end

            % convert grayscale images to RGB
            if size(imageCurr,3) == 1
                imageCurr = cat(3,imageCurr,imageCurr,imageCurr);
            end

            if obj.FirstCall
                obj.FirstCall = false;
                % Set the first frame
                obj.PreviousFrameBuffer = imageCurr; 
                % Save input image size
                obj.InRows = size(imageCurr,1);
                obj.InCols = size(imageCurr,2);
                obj.InChannels = size(imageCurr,3);
                obj.InBatch = size(imageCurr,4);
                obj.InputSize = size(imageCurr,[1 2 3 4]);
            end

            % Check that the current image size and type matches the
            % previous image
            checkInputSize(imageCurr,obj.InputSize);
            checkInputType(imageCurr,obj.PreviousFrameBuffer);
            % If miniBatchSize is lesser than batch size, then compute 
            % optical flow in mini-batches. Else, just compute optical flow
            % for the entire batch.
            if(options.MiniBatchSize < size(imageCurr,4))
                opticalFlow = obj.estimateFlowInBatches(...
                        imageCurr, ...
                        options.MaxIterations, options.Tolerance, ...
                        options.ExecutionEnvironment,...
                        obj.UsePreviousFlow, options.Acceleration,...
                        options.MiniBatchSize);
                obj.PreviousFrameBuffer = imageCurr;
            else
            opticalFlow = obj.Raft.estimateFlow(...
                obj.PreviousFrameBuffer, imageCurr, ...
                options.MaxIterations, options.Tolerance, ...
                options.ExecutionEnvironment,...
                obj.UsePreviousFlow, options.Acceleration);
            obj.PreviousFrameBuffer = imageCurr;
            end
        end

        function opticalFlow = estimateFlowInBatches(obj, imageCurr, maxIterations, iterTolerance, executionEnvironment, ...
                usePreviousFlow, acceleration, miniBatchSize)
            % This function helps to compute optical flow when a batch is
            % split into mini-batches.
            opticalFlow = cell([ceil(obj.InBatch/miniBatchSize), 1]);
            for startIdx = 1: miniBatchSize : obj.InBatch
                    endIdx = min(startIdx + miniBatchSize -1, obj.InBatch);
                    previousFrameBufferBatch = obj.PreviousFrameBuffer(:,:,:,startIdx:endIdx);
                    imageCurrBatch = imageCurr(:,:,:,startIdx:endIdx);
                    opticalFlowBatch = obj.Raft.estimateFlow( ...
                        previousFrameBufferBatch, imageCurrBatch, ...
                        maxIterations, iterTolerance, ...
                        executionEnvironment,...
                        usePreviousFlow, acceleration);
                    opticalFlow{ceil(endIdx/miniBatchSize)} = opticalFlowBatch;
            end
            opticalFlow = vertcat(opticalFlow{:});
        end

        % Reset method
        function reset(obj)
            % reset Resets the internal state of the object
            %   reset(obj) resets the internal state of the object.
            obj.FirstCall = true;

            % set following to default, they will be set by line 102 in the next call.
            obj.PreviousFrameBuffer = []; % previous frame
            obj.InRows = 0; % input row size
            obj.InCols = 0; % input column size
            obj.InBatch = 0; % input batch size
            obj.InChannels = 0;
            obj.InputSize = [];
            % reset the initial flow
            obj.Raft.InitialFlow = [];
        end
    end

    methods(Hidden, Access = ?matlab.unittest.TestCase)
        % Save object with versioning
        function s = saveobj(this)
            s.Version = this.Version;
            s.Raft = this.Raft;
            s.UsePreviousFlow = this.UsePreviousFlow;
            s.MinInputSize = this.MinInputSize;
            s.FirstCall = this.FirstCall;
            s.PreviousFrameBuffer = this.PreviousFrameBuffer;
            s.InRows = this.InRows;
            s.InCols = this.InCols;
            s.InBatch = this.InBatch;
            s.InChannels = this.InChannels;
            s.InputSize = this.InputSize;
        end
    end
    
    methods(Static, Hidden, Access = ?matlab.unittest.TestCase)
        % Load object with versioning
        function this = loadobj(s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                
                this = opticalFlowRAFT();

                this.Version = s.Version;
                this.Raft = s.Raft;
                this.UsePreviousFlow = s.UsePreviousFlow;
                this.MinInputSize = s.MinInputSize;
                this.FirstCall = s.FirstCall;
                this.PreviousFrameBuffer = s.PreviousFrameBuffer;
                this.InRows = s.InRows;
                this.InCols = s.InCols;
                this.InBatch = s.InBatch;
                this.InChannels = s.InChannels;
                this.InputSize = s.InputSize;
            catch ME
                rethrow(ME)
            end
        end
    end

end

%--------------------------------------------------------------------------
function iTripwireOpticalFlowRAFT()
    % Check if support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsOpticalFlowRAFTInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for RAFT Optical Flow Estimation';
        basecode = 'CVRAFTOPTICALFLOW';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
    end
end

%==========================================================================
function checkImage(I)
    % Validate input image
    validateattributes(I,{'uint8', 'int16', 'double', 'single', 'logical'}, ...
        {'real','nonsparse', 'nonnan','finite'}, mfilename, 'ImageCurr', 1)
    
    if min(size(I,[1 2])) < 57
        error(message('vision:opticalFlowRAFT:inputMinSize'));
    end
    
    if ~(size(I,3)==3 || size(I,3)==1)
        % gray scale or RGB image
        error(message('vision:opticalFlowRAFT:invalidImageInput'));
    end

    if ndims(I)> 4
        % input images must be SSCB
        error(message('vision:opticalFlowRAFT:invalidImageInput'));
    end
    if (isa(I, 'single') || isa(I, 'double'))
        if(~all(I(:)>=0 & I(:)<=1))
            error(message('vision:opticalFlowRAFT:invalidRange'));
        end
    end
end

function checkInputSize(currentImage,expInputSize)
    % Validate input size
    if ~isequal(expInputSize,size(currentImage,[1 2 3 4]))
        error(message('vision:OpticalFlow:inputSizeChange'));
    end
end

function checkInputType(img,pImg)
    % Validate input data type
    if (class(img) ~= class(pImg))
        error(message('vision:OpticalFlow:inputDataTypeChange'));
    end
end
