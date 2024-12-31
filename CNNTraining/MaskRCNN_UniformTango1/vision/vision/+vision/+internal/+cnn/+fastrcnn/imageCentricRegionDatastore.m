classdef imageCentricRegionDatastore < ...
        matlab.io.Datastore & ...
        matlab.io.datastore.MiniBatchable & ...
        matlab.io.datastore.BackgroundDispatchable & ...
        matlab.io.datastore.PartitionableByIndex & ...
        matlab.io.datastore.Partitionable & ...
        matlab.io.datastore.Shuffleable
%

%   Copyright 2018-2020 The MathWorks, Inc.
    
    
    properties
        %MiniBatchSize - The MiniBatchSize used in training and prediction
        %
        %    MiniBatchSize is a numeric scalar the specifies the number of
        %    observations returned in each call to read. This property is
        %    is set during training and prediction to be equal to the
        %    'MiniBatchSize' Name/Value in trainingOptions as well as the
        %    same name value in the predict, classify, and activations
        %    methods of SeriesNetwork and DAGNetwork.
        MiniBatchSize
                
        % ROIPerImage The number of ROIs to sample per image.
        ROIPerImage (1,1) double {mustBePositive, mustBeInteger, mustBeReal} = 128                 
        
        % ClassNames A set of class names to assign to each image region
        %            for training an object classifier.
        ClassNames
        
        % BoxRegressionMean/Std Used to standardize the box regression
        %                       targets.
        BoxRegressionMean
        BoxRegressionStd
        
    end
    
    properties(Constant)
        % OutputTableVariableNames Column names for table output by read
        %                          and readByIndex.
        OutputTableVariableNames = {'Image','ROI','ClassificationResponse','RegressionResponse'};
        
        % BatchingFcn Define custom batching function for a cell of M-by-4
        % ROIs. This overrides the default batching function which attempts
        % to cat ROI along the 4-th dimension. Each struct field should
        % correspond to one of the table variable names. 
        BatchingFcn = struct(...
            'ROI', @(x)vertcat(x{:}),...
            'RegressionResponse', @iRegresssionResponseBatchFcn);
    end
    
    properties(SetAccess = protected, Dependent)
        
        %NumObservations - The total number of observations contained
        % within the datastore. This number of observations is the length
        % of one training epoch.
        NumObservations
        
    end
    
    properties (Access = private)
        % PositiveOverlapRange Specify a range of overlap ratios
        PositiveOverlapRange
        
        % NegativeOverlapRange The range of overlap ratios to use as
        % negative training samples.
        NegativeOverlapRange
        
        % RandomSelector Wrapper class for randperm.
        RandomSelector
        
        % ColorPreprocessing 'rgb2gray', 'gray2rgb'
        ColorPreprocessing
    end
    
    properties(Access = protected, Hidden, Dependent)
        % NumPositiveSamplesPerBatch  The actual number of positive samples
        %                             in the mini-batch.
        NumPositiveSamplesPerBatch
        
    end
    
    properties (Access = private, Transient)
        % CurrentImageIndex  Current index of image from which regions are being
        %                    read.
        CurrentImageIndex
    end
    
    properties (Access = protected)
        % ImageDatastore The ImageDatastore we are going to read data and
        %           responses from.
        ImageDatastore
        
        % Table A table containing the list of image files and region
        %       proposal boxes.
        Table
        
        % TrainingSamples A table containing indices to boxes to use as
        %                 positive and negative samples.
        TrainingSamples                                   
        
        % PercentageOfPositiveSamples The percentage of positive samples
        %                             in a mini-batch.
        PercentageOfPositiveSamples
        
        % BackgroundLabel The label to use for the background class. By
        %                 default this is "Background".
        BackgroundLabel
       
        % MiniBatchPadWithNegatives Whether to pad mini-batch with negative
        %                           samples or honor foreground fraction
        %                           specified by
        %                           PercentageOfPositiveSamples.     
        MiniBatchPadWithNegatives
        
        % BBoxRegressionNormalization Specify normalization factor for box
        % regression targets. Value values are 'batch' or 'num-positives'.
        % when the value is 'batch' the box regression targets are
        % normalized by the number of ROIs in the batch. Otherwise, the
        % normalization factor is the number of positive boxes.
        BBoxRegressionNormalization
    end
    
    methods
           
        function this = imageCentricRegionDatastore(trainingData, regionProposals, params)
            if nargin == 3
                % Set the default background label.
                this.BackgroundLabel = params.BackgroundLabel;
                this.ImageDatastore = imageDatastore(trainingData{:,1});
                classNames = trainingData.Properties.VariableNames(2:end);
                
                % Create class names from ground truth data. For object
                % detection add a "Background" class. The order of the class
                % labels is important because they are used to generate
                % response data for training.
                this.ClassNames = [classNames this.BackgroundLabel];
                
                this.PositiveOverlapRange = params.PositiveOverlapRange;
                this.NegativeOverlapRange = params.NegativeOverlapRange;
                
                % Select region proposal boxes to use as positive and negative
                % samples.
                tbl = [trainingData(:,2:end) regionProposals(:,1)];
                this.TrainingSamples = rowfun(...
                    @(varargin)selectTrainingSamples(this, classNames, varargin{:}), tbl,...
                    'InputVariables',1:width(tbl),...
                    'OutputFormat','table',...
                    'NumOutputs', 5, ...
                    'ExtractCellContents', true, ...
                    'OutputVariableNames',{'Positive','Negative','Labels', 'RegionProposals', 'PositiveBoxRegressionTargets'});
                
                % Check for missing positives
                missing = rowfun(@(x)isempty(x{1}) || ~any(x{1}),this.TrainingSamples,...
                    'InputVariables','Positive','OutputFormat','uniform');
                
                % Remove images that have no positive samples. There are
                % usually many more negatives than positives, which is why do
                % not remove images that do not have any negatives.
                this.TrainingSamples(missing,:) = [];
                
                if isempty(this.TrainingSamples)
                    error(message('vision:rcnn:noTrainingSamples'));
                end
                
                keep = ~missing;
                
                this.Table = [trainingData(keep,1) this.TrainingSamples(:,'RegionProposals')];
                
                % Standardize the column names
                this.Table.Properties.VariableNames{1} = 'Files';
                this.Table.Properties.VariableNames{2} = 'RegionProposalBoxes';
                
                if params.ScaleImage
                    % all the ground truth boxes and region proposal boxes have
                    % been scaled already. setup the imds to scale the image
                    % too.
                    this.ImageDatastore = imageDatastore(this.Table.Files, ...
                        'ReadFcn', @(filename)fastRCNNObjectDetector.scaleImage(filename, params.ImageScale));
                else
                    this.ImageDatastore = imageDatastore(this.Table.Files);
                end
                
                [this.BoxRegressionMean, this.BoxRegressionStd, this.TrainingSamples.PositiveBoxRegressionTargets] = ...
                    vision.internal.cnn.rpn.imageCentricRegionDatastore.calculateBoxRegressionMeanStd(params, this.TrainingSamples.PositiveBoxRegressionTargets);
                
                this.PercentageOfPositiveSamples = params.InternalOptions.FastForegroundFraction;
                this.ROIPerImage   = params.NumRegionsToSample;
                this.MiniBatchSize =  params.MiniBatchSize;
                this.RandomSelector = params.RandomSelector;
                this.MiniBatchPadWithNegatives = params.InternalOptions.MiniBatchPadWithNegatives;
                this.BBoxRegressionNormalization = params.InternalOptions.SmoothL1Normalization;
                this.CurrentImageIndex = 1;
                this.ColorPreprocessing = params.ColorPreprocessing;
                this.DispatchInBackground = params.DispatchInBackground;
                
                this.reset();
            end
        end
        
        function val = get.NumObservations(this)
            val = height(this.Table);
        end
        
        function set.MiniBatchSize(this, value)
            this.MiniBatchSize = value;
        end
        
        function tf = hasdata(this)
            tf = this.CurrentImageIndex <= this.NumObservations;
        end
        
        function [tbl, info] = readByIndex(this, indices)            
                        
            % read images from datastore            
            subds = copy(this.ImageDatastore);
            subds.Files = this.ImageDatastore.Files(indices);
            images = readall(subds);
            if ~iscell(images)
                images = {images};
            end
                        
            % TODO
            % Apply preprocessing and augmentations            
            % random sample ROI per image            
            % assign ROI to ground truth
                                                
            roiBatch = cell(numel(images),1);
            responseBatch = cell(numel(images),2);
            
            for batchIdx = 1:numel(indices)
                obsIdx = indices(batchIdx);
                
                images(batchIdx) = vision.internal.cnn.utils.convertImageToMatchNumberOfNetworkChannels(images(batchIdx), this.ColorPreprocessing);
                
                samples = this.Table.RegionProposalBoxes{obsIdx};
                
                posSamples = samples(this.TrainingSamples.Positive{obsIdx},:);
                
                % 1:1 ratio between positive and negatives.
                numPos = this.NumPositiveSamplesPerBatch;
                N = size(posSamples,1);
                numPos = min(numPos, N);
                
                pid = this.RandomSelector.randperm(N, numPos);
                posSamples = posSamples(pid, :);
                
                bb = samples(this.TrainingSamples.Negative{obsIdx},:);
                N = size(bb,1);
                
                if this.MiniBatchPadWithNegatives
                    numNeg = min(N, this.ROIPerImage - numPos);
                else
                    % honor foreground fraction
                    numNeg = min(N, floor(numPos/this.PercentageOfPositiveSamples - numPos));
                end
                
                nid = this.RandomSelector.randperm(N, numNeg);
                negSamples = bb(nid,:);
                
                % response
                labels = this.TrainingSamples.Labels{obsIdx};
                posResponse = labels(this.TrainingSamples.Positive{obsIdx});
                negResponse = labels(this.TrainingSamples.Negative{obsIdx});
                
                % training rois
                roi = [posSamples; negSamples];
                
                % CLS response
                % data in mini-batch need not be shuffled. training responses
                % are averaged over all mini-batch samples so order does not
                % matter.
                
                
                clsResponse = [posResponse(pid,:);negResponse(nid,:)];
                clsResponse = reshape(clsResponse,1,1,1,[]);
                
                % REG Response
                numNeg = size(negSamples,1);
                numPos = size(posSamples,1);
                
                posTargets = this.TrainingSamples.PositiveBoxRegressionTargets{obsIdx};
                posTargets = posTargets(:,pid);
                negTargets = zeros([4 numNeg],'like',posTargets);
                
                targets = [posTargets negTargets];
                
                % expand targets array for K-1 classes, excluding the
                % background.
                numClasses = numel(categories(labels));
                
                targets = repmat(targets, numClasses-1, 1);
                targets = reshape(targets, [1 1 (numClasses-1)*4 numNeg+numPos]);
                
                % Define instance weight
                if strcmp(this.BBoxRegressionNormalization,'batch')
                    w = 1/(numPos+numNeg);
                else
                    % only "valid" or positive boxes.
                    % define instance weight that effectively computes the average
                    % when summing weighted per sample loss. The average is
                    % computed over number of observations. The number of
                    % observations only include the number of positives because
                    % only positive samples are regressed.
                    w = 1/numPos;
                end
                
                % Create instance weights for each proposal. 
                dummifiedPosResponse = nnet.internal.cnn.util.dummify(posResponse(pid));
                dummifiedNegResponse = nnet.internal.cnn.util.dummify(negResponse(nid));
                
                % create a "class selection" array. This facilitates selecting
                % class specific targets when computing the regression loss.
                selection = cat(4, w * dummifiedPosResponse, zeros(size(dummifiedNegResponse),'like',dummifiedPosResponse));
                selection = squeeze(selection);
                
                % get the location of "background" label in selection and
                % remove it.
                bgIdx = strcmp(this.BackgroundLabel, categories(posResponse));
                selection(bgIdx,:) = [];
                
                % duplicate selection entries 4 times for tx,ty,tw,th, and
                % reshape to 4D array
                selection = repelem(selection, 4, 1);
                selection = reshape(selection, [1 1 (numClasses-1)*4 numNeg+numPos]);
                
                % pack output for training cls loss and reg loss
                responseBatch{batchIdx,1} = clsResponse;
                responseBatch{batchIdx,2} = {targets, selection};
                                           
                % ROI pooling layer expects roi format to be [x1 y1 x2 y2].
                roiX1Y1X2Y2 = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(roi);
                
                % associate batchIdx with each ROI.
                roiBatch{batchIdx} = [roiX1Y1X2Y2 repelem(batchIdx,size(roiX1Y1X2Y2,1),1)];
            end
            
            tbl = table(images, roiBatch, responseBatch(:,1), responseBatch(:,2), ...
                'VariableNames', this.OutputTableVariableNames);                
            
            info.empty = true;
            
        end
        
        function [tbl, info] = read(this)             
            if ~this.hasdata()
                error('Call reset');
            end
            
            indices = this.CurrentImageIndex:min(this.CurrentImageIndex+this.MiniBatchSize-1, this.NumObservations);
                      
            [tbl, info] = this.readByIndex(indices);
            info.MiniBatchIndices = indices;
            
            % Advance to next mini-batch.
            this.CurrentImageIndex = indices(end) + 1;
                        
        end
        
        %SHUFFLE Return a shuffled version of a datastore.
        %
        %   NEWDS = SHUFFLE(DS) returns a randomly shuffled copy of a
        %   datastore.
        %
        %   See also matlab.io.datastore.Shuffleable.
        function newds = shuffle(this)
            newds = copy(this);
            idx = newds.RandomSelector.randperm(height(newds.Table));
            newds.Table = newds.Table(idx, :);
            newds.TrainingSamples = newds.TrainingSamples(idx, :);
            newds.ImageDatastore.Files = newds.ImageDatastore.Files(idx);            
        end
        
        function reset(this)
            reset( this.ImageDatastore );
            this.CurrentImageIndex = 1;
        end                
        
        %------------------------------------------------------------------
        function val = get.NumPositiveSamplesPerBatch(this)
            val = floor( this.PercentageOfPositiveSamples * this.ROIPerImage );
        end
                
        %------------------------------------------------------------------
        function newds = partitionByIndex(this, indices)            
            newds = copy(this);
            newds.ImageDatastore.Files = newds.ImageDatastore.Files(indices);
            newds.Table = newds.Table(indices, :);
            newds.TrainingSamples = newds.TrainingSamples(indices, :);
        end
        
        %------------------------------------------------------------------
        function s = saveobj(this)
            s.ImageDatastore = this.ImageDatastore;
            s.Table = this.Table;
            s.TrainingSamples = this.TrainingSamples;
            s.MiniBatchSize = this.MiniBatchSize;
            s.ROIPerImage = this.ROIPerImage;
            s.PositiveOverlapRange = this.PositiveOverlapRange;
            s.NegativeOverlapRange = this.NegativeOverlapRange;
            s.RandomSelector = this.RandomSelector;
            s.ColorPreprocessing = this.ColorPreprocessing;
            s.ClassNames = this.ClassNames;
            s.BoxRegressionMean = this.BoxRegressionMean;
            s.BoxRegressionStd = this.BoxRegressionStd;
            s.BackgroundLabel = this.BackgroundLabel;
            s.PercentageOfPositiveSamples = this.PercentageOfPositiveSamples;
            s.MiniBatchPadWithNegatives = this.MiniBatchPadWithNegatives;
            s.BBoxRegressionNormalization = this.BBoxRegressionNormalization;                        
            s.DispatchInBackground = this.DispatchInBackground;
        end
        
        readall(~) % not implemented.
    end
    
    % Methods for Partionable
    methods(Access = protected)
        function n = maxpartitions(this)
            % Granularity of this datastore is the number of observations.
            n = this.NumObservations;
        end
    end
    
    % Methods for Partionable
    methods(Access = public)
        function subds = partition(this,N,ii)
            n = maxpartitions(this);
            % pigeonhole: N(r-1) + 1 objects into N boxes
            rMinus1 = (0:n - 1) / n;
            partitionIndices = floor(N * rMinus1) + 1;
            partitionIndices = find(partitionIndices == ii);
            subds = partitionByIndex(this,partitionIndices);
        end
    end
    
    methods(Static)
        function this = loadobj(s)
            this = vision.internal.cnn.fastrcnn.imageCentricRegionDatastore();
            this.ImageDatastore = s.ImageDatastore;
            this.Table = s.Table;
            this.TrainingSamples = s.TrainingSamples;
            this.MiniBatchSize = s.MiniBatchSize;
            this.ROIPerImage = s.ROIPerImage;
            this.PositiveOverlapRange = s.PositiveOverlapRange;
            this.NegativeOverlapRange = s.NegativeOverlapRange;
            this.RandomSelector = s.RandomSelector;
            this.ColorPreprocessing = s.ColorPreprocessing;
            this.ClassNames = s.ClassNames;
            this.BoxRegressionMean = s.BoxRegressionMean;
            this.BoxRegressionStd = s.BoxRegressionStd;
            this.BackgroundLabel = s.BackgroundLabel;
            this.PercentageOfPositiveSamples = s.PercentageOfPositiveSamples;
            this.MiniBatchPadWithNegatives = s.MiniBatchPadWithNegatives;
            this.BBoxRegressionNormalization = s.BBoxRegressionNormalization;                      
            this.DispatchInBackground = s.DispatchInBackground;
            
            this.reset();
        end
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function aCopy = copyElement(this)
            %COPYELEMENT   Create a deep copy of the datastore
            %   Create a deep copy of the datastore. We need to call
            %   copy on the datastore's property ImageDatastore, because it is
            %   a handle object. Creating a deep copy allows methods
            %   such as readall and preview, that call the copy method,
            %   to remain stateless.
            aCopy = copyElement@matlab.mixin.Copyable(this);
            aCopy.ImageDatastore = copy(this.ImageDatastore);  
            aCopy.reset();
        end               
        
        %------------------------------------------------------------------
        % Returns indices to boxes that should be used as positive training
        % samples and those that should be used as negative training
        % samples.
        %
        % Uses the strategy described in:
        %
        %    Girshick, Ross, et al. "Rich feature hierarchies for accurate
        %    object detection and semantic segmentation." Proceedings of
        %    the IEEE conference on computer vision and pattern
        %    recognition. 2014.
        %
        %    Girshick, Ross. "Fast r-cnn." Proceedings of the IEEE
        %    International Conference on Computer Vision. 2015.
        %
        %------------------------------------------------------------------
        function [positiveIndex, negativeIndex, labels, regionProposals, targets] = selectTrainingSamples(this, classNames, varargin)
            
            % cat all multi-class bounding boxes into one M-by-4 matrix.
            groundTruth = vertcat(varargin{1:numel(varargin)-1});
            
            % create list of class names corresponding to each ground truth
            % box.
            cls = cell(1, numel(classNames));
            for i = 1:numel(varargin)-1
                cls{i} = repelem(classNames(i), size(varargin{i},1),1);
            end
            cls = vertcat(cls{:});
            
            regionProposals = varargin{end};
            
            % Assign region proposals to ground truth boxes.
            matchAllGroundTruth = false;
            [assignments, positiveIndex, negativeIndex] = vision.internal.cnn.boxAssignmentUtils.assignBoxesToGroundTruthBoxes(...
                regionProposals, groundTruth, ...
                this.PositiveOverlapRange, ...
                this.NegativeOverlapRange, matchAllGroundTruth);            
            
            % Assign label to each region proposal.
            labels = vision.internal.cnn.boxAssignmentUtils.boxLabelsFromAssignment(...
                assignments, cls, positiveIndex, negativeIndex, ...
                classNames, this.BackgroundLabel);
            labels = {labels};
            
            % Create an array that maps ground truth box to positive
            % proposal box. i.e. this is the closest grouth truth box to
            % each positive region proposal.
            if isempty(groundTruth)
                targets = {[]};
            else
                G = groundTruth(assignments(positiveIndex), :);
                P = regionProposals(positiveIndex,:);
                
                % positive sample regression targets
                targets = vision.internal.rcnn.BoundingBoxRegressionModel.generateRegressionTargets(G, P);
                
                targets = {targets'}; % arrange as 4 by num_pos_samples
            end
            % return the region proposals, which may have been augmented
            % with the ground truth data.
            regionProposals = {regionProposals};
            
            positiveIndex = {positiveIndex};
            negativeIndex = {negativeIndex};
        end
        
    end

    methods(Hidden)
        function frac = progress(~)
            frac = 1;
        end
    end
end

%--------------------------------------------------------------------------
function batch = iRegresssionResponseBatchFcn(TColumn)
% function to batch a column of regression response data.

t1 = cellfun(@(x)x{1},TColumn,'UniformOutput',false);
t2 = cellfun(@(x)x{2},TColumn,'UniformOutput',false);

batch = { cat(4,t1{:}) cat(4,t2{:}) };
end
