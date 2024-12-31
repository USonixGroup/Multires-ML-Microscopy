%imageCentricRegionDatastore RPN image centric training datastore.
%
% Generates training data for RPN.

classdef imageCentricRegionDatastore <  ...
        matlab.io.Datastore & ...
        matlab.io.datastore.MiniBatchable & ...
        matlab.io.datastore.BackgroundDispatchable & ...
        matlab.io.datastore.PartitionableByIndex & ...
        matlab.io.datastore.Partitionable & ...
        matlab.io.datastore.Shuffleable

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
        
        % ROIPerImage The number of ROIs to sample per image. First element
        % is for RPN sub-network, second element is for fast r-cnn
        % sub-network.
        ROIPerImage (1,:) double {mustBePositive, mustBeInteger, mustBeReal} = 128
        
    end
    
    properties(SetAccess = protected, Dependent)
        
        %NumObservations - The total number of observations contained
        % within the datastore. This number of observations is the length
        % of one training epoch.
        NumObservations
              
    end
    
    properties(SetAccess = protected)
        
        % BackgroundLabel The label to use for the background class. By
        %                 default this is "Background".
        BackgroundLabel                      
        
        % OutputTableVariableNames Column names for table output by read
        %                          and readByIndex.
        OutputTableVariableNames = {'Image','ClassificationResponse','RegressionResponse'};
        
        % BatchingFcn Define custom batching function for a cell of M-by-4
        % ROIs. This overrides the default batching function which attempts
        % to cat ROI along the 4-th dimension. Each struct field should
        % correspond to one of the table variable names. 
        BatchingFcn = struct(...            
            'RegressionResponse', @iRegresssionResponseBatchFcn);
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
    
    
    properties(SetAccess = private)
        
        % ClassNames A set of class names to assign to each image region
        %            for training an object classifier.
        ClassNames
        
        % Normalization values for bounding box targets. 
        BoxRegressionMean
        BoxRegressionStd
        
    end
    
    properties (Access = private, Transient)
        % CurrentImageIndex  Current index of image from which regions are being
        %                    read.
        CurrentImageIndex
    end
    
    properties (Access = private)
        % Datastore The ImageDatastore we are going to read data and
        %           responses from.
        Datastore
        
        % Table A table containing the list of image files and region
        %       proposal boxes.
        Table
        
        % TrainingSamples A table containing indices to boxes to use as
        %                 positive and negative samples.
        TrainingSamples                
        
        % PercentageOfPositiveSamples The percentage of foreground samples
        %                             in a mini-batch.
        PercentageOfPositiveSamples
            
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
        
        % CategoricalLookup A categorical vector of classes used as a
        %                   categorical look-up table for populating the
        %                   RPN classification response.
        %                   
        CategoricalLookup
        
        % NumAnchors The number of anchor boxes.
        NumAnchors
    end
    
    properties(Dependent)
        RPNROIPerImage
    end
    
    methods
        
        function this = imageCentricRegionDatastore(trainingData, params)
            if nargin == 2
                this.setup(trainingData, params);
            end
        end
        
        function [data,info] = readByIndex(this, indices)
            
            % read images from datastore
            subds = copy(this.Datastore);
            subds.Files = this.Datastore.Files(indices);
            images = readall(subds);
            if ~iscell(images)
                images = {images};
            end
            
            clsResponseBatch = cell(numel(images),1);
            regResponseBatch = cell(numel(images),1);
            
            for batchIdx = 1:numel(indices)
                idx = indices(batchIdx);
                
                images(batchIdx) = vision.internal.cnn.utils.convertImageToMatchNumberOfNetworkChannels(images(batchIdx), this.ColorPreprocessing);
                
                samples = this.Table.RegionProposalBoxes{idx};
                
                posSamples = samples(this.TrainingSamples.Positive{idx},:);
                
                % 1:1 ratio between positive and negatives.
                numPos = floor(this.RPNROIPerImage / 2);
                numPos = min(numPos, size(posSamples,1));
                
                % Create target matrix MxNxK (K == 2*num_anchors)
                ids = this.TrainingSamples.AnchorIndices{idx};
                anchorIDs = this.TrainingSamples.AnchorIDs{idx};
                
                posAnchorIDs = anchorIDs(this.TrainingSamples.Positive{idx});
                negAnchorIDs = anchorIDs(this.TrainingSamples.Negative{idx});
                
                posxy = ids(this.TrainingSamples.Positive{idx}, :);
                negxy = ids(this.TrainingSamples.Negative{idx}, :);
                
                featureMapSize = this.TrainingSamples.FeatureMapSize{idx};
                
                clsResponse = 3*ones([featureMapSize(1),featureMapSize(2),this.NumAnchors],'uint8');
                
                bb = samples(this.TrainingSamples.Positive{idx},:);
                N = size(bb,1);
                pid = this.RandomSelector.randperm(N, min(N, numPos));
                posSamples = posSamples(pid, :);
                posxy = posxy(pid,:);
                posAnchorIDs = posAnchorIDs(pid);
                % pos
                for i = 1:numel(posAnchorIDs)
                    x = posxy(i,1);
                    y = posxy(i,2);
                    k = posAnchorIDs(i);
                    clsResponse(y, x, k) = 1; % for foreground class
                end
                
                bb = samples(this.TrainingSamples.Negative{idx},:);
                N = size(bb,1);
                
                if this.MiniBatchPadWithNegatives
                    numNeg = min(N, this.RPNROIPerImage - numPos);
                else
                    % honor foreground fraction
                    numNeg = min(N, floor(numPos/this.PercentageOfPositiveSamples - numPos));
                end
                
                id = this.RandomSelector.randperm(N, min(N, numNeg));
                
                negxy = negxy(id,:);
                negAnchorIDs = negAnchorIDs(id);
                % neg
                for i = 1:numel(negAnchorIDs)
                    x = negxy(i,1);
                    y = negxy(i,2);
                    k = negAnchorIDs(i);
                    clsResponse(y, x, k) = 2; % for background class
                end
                
                % reshape cls response into [H*W 2*NumAnchors]
                clsResponse = reshape(clsResponse, featureMapSize(1)*featureMapSize(2), []);
                
                negSamples = bb(id,:);
                
                % training rois
                roi = [posSamples; negSamples];
                
                % REG Response
                posTargets = this.TrainingSamples.PositiveBoxRegressionTargets{idx};
                posTargets = posTargets(:, pid);
                
                % Regression layer output is [M N 4*NumAnchors], where the 4
                % box coordinates are consequitive elements. W is a weight
                % matrix that indicates which sample should contribute to the
                % loss. W(y,x,k) is 1 if T(y,x,k) should be used.
                W = zeros(featureMapSize(1), featureMapSize(2), this.NumAnchors * 4, 'single');
                T = zeros(featureMapSize(1), featureMapSize(2), this.NumAnchors * 4, 'single');
                
                % Define instance weight
                if strcmp(this.BBoxRegressionNormalization,'batch')
                    w = 1/size(roi,1);
                else
                    % valid - treat each positive sample as an observation.
                    w = 1/max(1,4*size(posSamples,1));
                end
                
                % Put positive targets into appropriate location by anchor ID.
                % Only the positive anchors need to be included because they
                % are the only one that get regressed.
                for i = 1:numel(posAnchorIDs)
                    x = posxy(i,1);
                    y = posxy(i,2);
                    k = posAnchorIDs(i);
                    start = 4*(k-1) + 1;
                    stop = start+4-1;
                    W(y,x,start:stop) = w;
                    T(y,x,start:stop) = posTargets(:,i);
                end
                
                % Convert classification response to categorical.
                clsResponse = this.CategoricalLookup(clsResponse);
                
                clsResponseBatch{batchIdx} = clsResponse;
                regResponseBatch{batchIdx} = {T, W};
            end
            
            data = table( images, clsResponseBatch, regResponseBatch, ...
                'VariableNames',this.OutputTableVariableNames);
            info = [];
        end
        
        function [data,info] = read(this)
            if ~this.hasdata()
                error('Call reset');
            end
            
            indices = this.CurrentImageIndex:min(this.CurrentImageIndex+this.MiniBatchSize-1, this.NumObservations);
            
            data = this.readByIndex(indices);
            info.MiniBatchIndices = indices;
            
            % Iterate to start of next batch.
            this.CurrentImageIndex = indices(end) + 1;
        end
        
        function newds = shuffle(this)
            newds = this.shuffleImpl();
        end
        
        function reset(this)
            reset( this.Datastore );
            this.CurrentImageIndex = 1;
        end
        
        function tf = hasdata(this)
            tf = this.CurrentImageIndex <= this.NumObservations;
        end
        
        function val = get.NumObservations(this)
            val = height(this.Table);
        end
        
        function newds = partitionByIndex(this,indices)
            newds = copy(this);
            newds.Datastore.Files = newds.Datastore.Files(indices);
            newds.Table = newds.Table(indices, :);
            newds.TrainingSamples = newds.TrainingSamples(indices, :);
        end
        
        function s = saveobj(this)
            s.Datastore = this.Datastore;
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
            s.CategoricalLookup = this.CategoricalLookup;
            s.NumAnchors = this.NumAnchors;
            s.DispatchInBackground = this.DispatchInBackground;
            s.BatchingFcn = this.BatchingFcn;
        end
        
        function n = get.RPNROIPerImage(this)
            n = this.ROIPerImage(1);
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
            this = vision.internal.cnn.rpn.imageCentricRegionDatastore();
            this.setPropertiesDuringLoad(s);
            this.reset();
        end
    end

    methods(Static, Hidden)
        function [boxRegressionMean, boxRegressionStd, positiveBoxRegressionTargets] = calculateBoxRegressionMeanStd(params, positiveBoxRegressionTargets)
            %CALCULATEBOXREGRESSIONMEANSTD Calculate regression mean and standard
            % deviation for bounding boxes using the positiveBoxRegressionTargets.
            % If the InternalOptions parameter contains the mean and standard deviation,
            % use that instead, in which case positiveBoxRegressionTargets are not updated.

            if isempty(params.InternalOptions.BoxRegressionMeanStd)
                % Combine all targets to find the mean and standard deviation.
                allTargets = horzcat(positiveBoxRegressionTargets{:});
                meanAllTargets = mean(allTargets,2);
                stdAllTargets = std(allTargets,0,2);
                % Add an eps to avoid dividing by zero.
                stdAllTargets = stdAllTargets + eps(class(stdAllTargets));

                for i = 1:numel(positiveBoxRegressionTargets)
                    % Standardize regression targets
                    positiveBoxRegressionTargets{i} = ...
                        (positiveBoxRegressionTargets{i} - meanAllTargets)./stdAllTargets;
                end

                boxRegressionMean = meanAllTargets';
                boxRegressionStd = stdAllTargets';
            else
                boxRegressionMean = params.InternalOptions.BoxRegressionMeanStd(1,:);
                boxRegressionStd = params.InternalOptions.BoxRegressionMeanStd(2,:);
            end
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
            aCopy.Datastore = copy(this.Datastore);
            aCopy.reset();
        end
        
        %------------------------------------------------------------------
        function setPropertiesDuringLoad(this,s)
            this.Datastore = s.Datastore;
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
            this.CategoricalLookup = s.CategoricalLookup;
            this.NumAnchors = s.NumAnchors;
            this.DispatchInBackground = s.DispatchInBackground;
            this.BatchingFcn = s.BatchingFcn;
        end
        
        %------------------------------------------------------------------
        function [newds, idx] = shuffleImpl(this)
            newds = copy(this);
            idx = newds.RandomSelector.randperm(height(newds.Table));
            newds.Table = this.Table(idx, :);
            newds.TrainingSamples = newds.TrainingSamples(idx, :);
            newds.Datastore.Files = newds.Datastore.Files(idx);
        end
        
        %------------------------------------------------------------------
        function setup(this, trainingData, params)
            % Set the default background label.
            this.BackgroundLabel = params.BackgroundLabel;
            
            % Create class names from ground truth data. For object
            % detection add a "Background" class. The order of the class
            % labels is important because they are used to generate
            % response data for training.
            this.ClassNames = {'Foreground' this.BackgroundLabel};
            
            % Create categorical to represent categories learnt by RPN. The
            % 3rd value is intentially <undefined> to indicate a response
            % should be ignored because there is neither a foreground or
            % background sample at that location.
            this.CategoricalLookup = reshape(categorical([1 2 3],[1 2],this.ClassNames),[],1);
            
            this.PositiveOverlapRange = params.PositiveOverlapRange;
            this.NegativeOverlapRange = params.NegativeOverlapRange;
            
            this.RandomSelector = params.RandomSelector;
            
            this.TrainingSamples = trainingData;
            
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
            
            this.Table = [this.TrainingSamples(:,1) this.TrainingSamples(:,'RegionProposals')];
            
            % Standardize the column names
            this.Table.Properties.VariableNames{1} = 'Files';
            this.Table.Properties.VariableNames{2} = 'RegionProposalBoxes';
            
            if params.ScaleImage
                this.Datastore = imageDatastore(this.Table.Files, ...
                    'ReadFcn', @(filename)fastRCNNObjectDetector.scaleImage(filename, params.ImageScale));
            else
                this.Datastore = imageDatastore(this.Table.Files);
            end
            
            [this.BoxRegressionMean, this.BoxRegressionStd, this.TrainingSamples.PositiveBoxRegressionTargets] = ...
                vision.internal.cnn.rpn.imageCentricRegionDatastore.calculateBoxRegressionMeanStd(params, this.TrainingSamples.PositiveBoxRegressionTargets);
            
            this.BBoxRegressionNormalization = params.InternalOptions.SmoothL1Normalization;
            this.PercentageOfPositiveSamples = params.InternalOptions.RPNForegroundFraction;
            this.MiniBatchPadWithNegatives   = params.InternalOptions.MiniBatchPadWithNegatives;
            this.MiniBatchSize = params.MiniBatchSize;
            this.ROIPerImage   = params.NumRegionsToSample;
            this.ColorPreprocessing = params.ColorPreprocessing;
            this.NumAnchors = size(params.AnchorBoxes,1);
            this.DispatchInBackground = params.DispatchInBackground;
            
            this.reset();
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
