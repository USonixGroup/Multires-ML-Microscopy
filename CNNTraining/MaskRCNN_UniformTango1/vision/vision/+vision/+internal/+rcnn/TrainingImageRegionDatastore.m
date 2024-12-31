classdef TrainingImageRegionDatastore < ...
        matlab.io.Datastore &...
        matlab.io.datastore.MiniBatchable &...
        matlab.io.datastore.Shuffleable &...
        matlab.io.datastore.BackgroundDispatchable &...
        matlab.io.datastore.PartitionableByIndex
%

%   Copyright 2018-2020 The MathWorks, Inc.
    
    properties(Access = public)
        
        %MiniBatchSize - The MiniBatchSize used in training and prediction
        %
        %    MiniBatchSize is a numeric scalar the specifies the number of
        %    observations returned in each call to read. This property is
        %    is set during training and prediction to be equal to the
        %    'MiniBatchSize' Name/Value in trainingOptions as well as the
        %    same name value in the predict, classify, and activations
        %    methods of SeriesNetwork and DAGNetwork.
        MiniBatchSize
    end
    
    properties(SetAccess = protected)
        
        %NumObservations - The total number of observations contained
        % within the datastore. This number of observations is the length
        % of one training epoch.
        NumObservations
    end
    
    properties
        % PositiveOverlapRange Specify a range of overlap ratios
        PositiveOverlapRange
        
        % NegativeOverlapRange The range of overlap ratios to use as
        % negative training samples.
        NegativeOverlapRange
        
        % RandomSelector Wrapper class for randperm.
        RandomSelector
        
        % ClassNames A set of class names to assign to each image region
        %            for training an object classifier.
        ClassNames
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
        
        % CurrentIndex  Current index of image from which regions are being
        %                    read.
        CurrentIndex
        
        % PercentageOfPositiveSamples The percentage of positive samples
        %                             in a mini-batch.
        PercentageOfPositiveSamples
        
        % BackgroundLabel The label to use for the background class. By
        %                 default this is "Background".
        BackgroundLabel
        
        % RegionResizer An object used to crop and resize image regions.
        RegionResizer
    end
    
    properties(SetAccess = private)
        ShuffledIndices
        IndicatorMatrix
        PositiveIndices
    end
    
    methods(Access = public)
        
        function this = TrainingImageRegionDatastore(groundTruth, regionProposals, params)
            if nargin == 0
                return
            end
            
            % Set the default background label.
            this.BackgroundLabel = params.BackgroundLabel;
            
            classNames = groundTruth.Properties.VariableNames(2:end);
            
            % Create class names from ground truth data. For object
            % detection add a "Background" class. The order of the class
            % labels is important because they are used to generate
            % response data for training.
            this.ClassNames = [classNames this.BackgroundLabel];
            
            this.PositiveOverlapRange = params.PositiveOverlapRange;
            this.NegativeOverlapRange = params.NegativeOverlapRange;
            
            % Select region proposal boxes to use as positive and negative
            % samples.
            tbl = [groundTruth(:,2:end) regionProposals(:,1)];
            this.TrainingSamples = rowfun(...
                @(varargin)selectTrainingSamples(this, classNames, varargin{:}), tbl,...
                'InputVariables',1:width(tbl),...
                'OutputFormat','table',...
                'NumOutputs', 5, ...
                'ExtractCellContents', true, ...
                'OutputVariableNames',{'Positive','Negative','Labels', 'RegionProposals', 'NumPositives'});
            
            % Check for missing positives
            missing = rowfun(@(x)isempty(x{1}) || ~any(x{1}),this.TrainingSamples,...
                'InputVariables','Positive','OutputFormat','uniform');
            
            if any(missing)
                files = groundTruth{missing,1};
                warning(message('vision:rcnn:noPositiveSamples',...
                    sprintf('%s\n', files{:})));
            end
            
            % Check for missing negatives
            missingNeg = rowfun(@(x)isempty(x{1}) || ~any(x{1}),this.TrainingSamples,...
                'InputVariables','Negative','OutputFormat','uniform');
            
            if any(missingNeg)
                files = groundTruth{missingNeg,1};
                warning(message('vision:rcnn:noNegativeSamples',...
                    sprintf('%s\n', files{:})));
            end
            
            % Remove images that have no positive samples. There are
            % usually many more negatives than positives, which is why do
            % not remove images that do not have any negatives.
            this.TrainingSamples(missing,:) = [];
            
            if isempty(this.TrainingSamples)
                error(message('vision:rcnn:noTrainingSamples'));
            end
            
            keep = ~missing;
            
            this.Table = [groundTruth(keep,1) this.TrainingSamples.RegionProposals];
            
            % Standardize the column names
            this.Table.Properties.VariableNames{1} = 'Files';
            this.Table.Properties.VariableNames{2} = 'RegionProposalBoxes';
            
            this.ImageDatastore = imageDatastore(this.Table.Files);
            this.PercentageOfPositiveSamples = 0.25;
            this.MiniBatchSize  = params.MiniBatchSize;
                        
            this.RegionResizer  = params.RegionResizer;
            this.RandomSelector = params.RandomSelector;
            
            numPositivesPerImage = this.TrainingSamples.NumPositives;
            numPositiveSamples = sum(numPositivesPerImage);
            numNegativePerPositive = floor(1/this.PercentageOfPositiveSamples - 1);
            this.NumObservations = numPositiveSamples + numPositiveSamples*numNegativePerPositive;
            
            this.ShuffledIndices = this.RandomSelector.randperm(this.NumObservations);
            
            % Indicator matrix initialization
            this.IndicatorMatrix = zeros(numPositiveSamples, 1 + numNegativePerPositive);
            
            % Mark positive samples in indicator matrix using image IDs.
            % This enables looking up the image easily.
            
            ids = arrayfun(@(i,n)repelem(i,n,1), ...
                1:height(this.TrainingSamples), numPositivesPerImage', ...
                'UniformOutput',false);
            
            this.IndicatorMatrix(:,1) = vertcat(ids{:});
            indices = arrayfun(@(x)(1:x)',numPositivesPerImage,'UniformOutput',false);
            this.PositiveIndices = vertcat(indices{:});
            
            this.DispatchInBackground = params.DispatchInBackground;
            
            reset(this);
        end
        
        %------------------------------------------------------------------
        %HASDATA   Returns true if more data is available.
        %   Return logical scalar indicating availability of data. This
        %   method should be called before calling read. This is an
        %   abstract method and must be implemented by the subclasses.
        %   hasdata is used in conjunction with read to read all the data
        %   within the datastore. Following is an example usage:
        %
        %   ds = myDatastore(...);
        %   while hasdata(ds)
        %       [data, info] = read(ds);
        %   end
        %
        %   % reset to read from start of the data
        %   reset(ds);
        %   [data, info] = read(ds);
        %
        %   See also matlab.io.ImageDatastore, read, reset, readall, preview,
        %   progress.
        function tf = hasdata(this)
            tf = this.CurrentIndex <= this.NumObservations;
        end
        
        %------------------------------------------------------------------
        %READ   Read data and information about the extracted data.
        %   Return the data extracted from the datastore in the
        %   appropriate form for this datastore. Also return
        %   information about where the data was extracted from in
        %   the datastore. Both the outputs are required to be
        %   returned from the read method, and can be of any type.
        %   info is recommended to be a struct with information
        %   about the chunk of data read. data represents the
        %   underlying class of tall, if tall is created on top of
        %   this datastore.
        %
        %   See also matlab.io.ImageDatastore, hasdata, reset, readall, preview,
        %   progress.
        function [data, info] = read(this)
            
            if ~this.hasdata()
               error('no more data'); 
            end
            
            indices = this.CurrentIndex:min(this.CurrentIndex+this.MiniBatchSize-1, this.NumObservations);
            
            [data,info] = this.readByIndex(indices);
            
            % Advance to next mini-batch.
            this.CurrentIndex = indices(end) + 1;
        end
        
        %------------------------------------------------------------------
        %readByIndex Return observations from a MiniBatchDatastore
        %specified by index.
        %
        %   [DATA,INFO] = readByIndex(DS,INDICES) returns observations from a
        %   MiniBatchDatastore specified by INDICES. The readByIndex
        %   method should return DATA in the same form as the READ method,
        %   which is a table in MiniBatchable Datastores.
        %
        %   See also matlab.io.Datastore, matlab.io.datastore.MiniBatchable
        function [data,info] = readByIndex(this,indices)
            
            if isempty(indices)                               
                error('Should not be empty')
            end                        
                                         
            shuffledIndices = this.ShuffledIndices(indices);
            
            positiveObservationIndices = shuffledIndices(this.IndicatorMatrix(shuffledIndices) > 0);
            
            requiredNumPositiveObservations = ceil(this.PercentageOfPositiveSamples * numel(indices));
            
            if numel(positiveObservationIndices) < requiredNumPositiveObservations
                % add more positives to honor mini-batch foreground fraction.
                shortfall = requiredNumPositiveObservations - numel(positiveObservationIndices);
                numPositiveSamples = size(this.IndicatorMatrix,1);
                range = 1:numPositiveSamples;
                range(positiveObservationIndices) = [];
                
                availableSamples = numel(range);
                
                if shortfall > availableSamples
                    % Not enough samples to meet the shortage. Just fill as
                    % many as possible. The rest will be filled with
                    % negatives.
                    shortfall = availableSamples;
                end
                
                positiveObservationIndices = [positiveObservationIndices ...
                    this.RandomSelector.randperm(numel(range),shortfall)];
                
            elseif numel(positiveObservationIndices) > requiredNumPositiveObservations
                % remove positives
                overage = numel(positiveObservationIndices) - requiredNumPositiveObservations;
                
                positiveObservationIndices(end-overage+1:end) = [];
                
            end
            
            if isempty(positiveObservationIndices)
                % no positive samples - fill with all negatives. This is an
                % edge case.
                
                % find image from which to sample negatives.
                [m,~] = ind2sub(size(this.IndicatorMatrix),shuffledIndices);
                imgIndices = this.IndicatorMatrix(m,1);
                % Count number of negative observations to get from each image.
                numNegativesPerImage = accumarray(imgIndices,1,[height(this.TrainingSamples) 1]);
                
                whichImages = 1:height(this.TrainingSamples);
                whichImages = whichImages( numNegativesPerImage > 0 );                               
                
                % Read images
                subds = copy(this.ImageDatastore);
                subds.Files = this.ImageDatastore.Files(whichImages);
                cellArrayOfImages = subds.readall();
                
                [miniBatchData, miniBatchResponse] = sampleNegativeObservations(...
                    cellArrayOfImages, this.Table, this.TrainingSamples, ...
                    this.RegionResizer, this.RandomSelector, ...
                    numNegativesPerImage, whichImages);
                
            else
                
                imgIndices = this.IndicatorMatrix(positiveObservationIndices,1);
                
                % group positive observations by the images they come from.
                groupedByImage = accumarray(imgIndices,this.PositiveIndices(positiveObservationIndices),[height(this.TrainingSamples) 1],@(x){x});
                
                whichImages = 1:height(this.TrainingSamples);
                whichImages = whichImages( cellfun(@(x)~isempty(x),groupedByImage) );
                
                % Read images
                subds = copy(this.ImageDatastore);
                subds.Files = this.ImageDatastore.Files(whichImages);
                cellArrayOfImages = subds.readall();                                           
                
                [miniBatchData, miniBatchResponse] = batchReadAndCrop(...
                    cellArrayOfImages, this.Table, this.TrainingSamples, ...
                    this.RegionResizer, this.RandomSelector, ...
                    groupedByImage, whichImages, ...
                    numel(indices), this.PercentageOfPositiveSamples);
            end
            
            data = table(miniBatchData,miniBatchResponse,'VariableNames',{'Image','Response'});
            info.Empty = true;
            
           
        end
        
        %------------------------------------------------------------------
        % PARTITIONBYINDEX Return a partitioned part of the datastore
        % described by indices
        %
        % SUBDS = PARTITIONBYINDEX(DS,INDICES) partitions DS into a new
        % datastore containing the observations that correspond to INDICES
        % in the original datastore.
        %
        % Partition ShuffledIndices based on input indices. Everything else
        % gets copied into subds.
        function subds = partitionByIndex(this,indices)
            subds = copy(this);
            subds.ImageDatastore  = copy(this.ImageDatastore); % imds requires separate copy because it is a handle.           
            subds.ShuffledIndices = this.ShuffledIndices(indices); 
            subds.NumObservations = numel(indices);
            subds.reset();
        end
        
        %------------------------------------------------------------------
        %RESET   Reset to the start of the data.
        %   Reset the datastore to the state where no data has been
        %   read from it. This is an abstract method and must be
        %   implemented by the subclasses.
        %   In the provided example, the datastore is reset to point to the
        %   first file (and first partition) in the datastore.
        %
        %   See also matlab.io.ImageDatastore, read, hasdata, readall, preview,
        %   progress.
        function reset(this)
            this.CurrentIndex = 1;
        end
        
        %------------------------------------------------------------------
        %SHUFFLE Return a shuffled version of a datastore.
        %
        %   NEWDS = SHUFFLE(DS) returns a randomly shuffled copy of a
        %   datastore by shuffling the ShuffledIndices. This effectively
        %   shuffles all the observations.
        %
        %   See also matlab.io.datastore.Shuffleable.
        function newds = shuffle(this)
            newds = copy(this);
            newds.ImageDatastore = copy(this.ImageDatastore);
            newds.ShuffledIndices = newds.RandomSelector.randperm(newds.NumObservations);
        end
        
        %------------------------------------------------------------------
        function s = saveobj(this)
           s.Version = 1.0;
           s.MiniBatchSize = this.MiniBatchSize;
           s.NumObservations = this.NumObservations;
           
           s.PositiveOverlapRange = this.PositiveOverlapRange;
           s.NegativeOverlapRange = this.NegativeOverlapRange;            
           s.RandomSelector = this.RandomSelector;                      
           s.ClassNames = this.ClassNames;
           s.ImageDatastore = this.ImageDatastore;
           s.Table = this.Table;
           s.TrainingSamples = this.TrainingSamples;
           s.CurrentIndex = this.CurrentIndex;
           s.PercentageOfPositiveSamples = this.PercentageOfPositiveSamples;
           s.BackgroundLabel = this.BackgroundLabel;
           s.RegionResizer = this.RegionResizer;
           s.ShuffledIndices = this.ShuffledIndices;
           s.IndicatorMatrix = this.IndicatorMatrix;
           s.PositiveIndices = this.PositiveIndices;
        end
    end
    methods(Static)
        function this = loadobj(s)
            this = vision.internal.rcnn.TrainingImageRegionDatastore();
            this.MiniBatchSize = s.MiniBatchSize;
            this.NumObservations = s.NumObservations;
            
            this.PositiveOverlapRange = s.PositiveOverlapRange;
            this.NegativeOverlapRange = s.NegativeOverlapRange;
            this.RandomSelector = s.RandomSelector;
            this.ClassNames = s.ClassNames;
            this.ImageDatastore = s.ImageDatastore;
            this.Table = s.Table;
            this.TrainingSamples = s.TrainingSamples;
            this.CurrentIndex = s.CurrentIndex;
            this.PercentageOfPositiveSamples = s.PercentageOfPositiveSamples;
            this.BackgroundLabel = s.BackgroundLabel;
            this.RegionResizer = s.RegionResizer;
            this.ShuffledIndices = s.ShuffledIndices;
            this.IndicatorMatrix = s.IndicatorMatrix;
            this.PositiveIndices = s.PositiveIndices;
        end
        
    end
    methods(Hidden)
        %PROGRESS   Percentage of consumed data between 0.0 and 1.0.
        %   Return fraction between 0.0 and 1.0 indicating progress as a
        %   double. The provided example implementation returns the
        %   ratio of the index of the current file from DsFileSet
        %   to the number of files in DsFileSet. A simpler
        %   implementation can be used here that returns a 1.0 when all
        %   the data has been read from the datastore, and 0.0
        %   otherwise.
        %
        %   See also matlab.io.ImageDatastore, read, hasdata, reset, readall,
        %   preview.
        function frac = progress(~)
            % Stub implementation. Not used in R-CNN.
            frac = 1;
        end
    end
    
    methods(Access = protected)
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
        function [positiveIndex, negativeIndex, labels, regionProposals, numPos] = selectTrainingSamples(this, classNames, varargin)
            
            % cat all multi-class bounding boxes into one M-by-4 matrix.
            groundTruth = vertcat(varargin{1:numel(varargin)-1});
            
            % create list of class names corresponding to each ground truth
            % box.
            cls = cell(1, numel(classNames));
            for i = 1:numel(varargin)-1
                cls{i} = repelem(classNames(i), size(varargin{i},1),1);
            end
            cls = vertcat(cls{:});
            
            regionProposals = reshape(varargin{end},[],4);
            
            % Compute the Intersection-over-Union (IoU) metric between the
            % ground truth boxes and the region proposal boxes.
            if isempty(groundTruth)
                iou = zeros(0,size(regionProposals,1));
            else
                
                % The region proposal algorithm may miss objects in the
                % image. To make sure we use all available training data
                % append the groundTruth boxes to the region proposals.
                regionProposals = [regionProposals; groundTruth];
                
                iou = bboxOverlapRatio(groundTruth, regionProposals, 'union');
            end
            
            % Find bboxes that have largest IoU w/ GT.
            [v,i] = max(iou,[],1);
            
            % Select regions to use as positive training samples
            lower = this.PositiveOverlapRange(1);
            upper = this.PositiveOverlapRange(2);
            positiveIndex =  {v >= lower & v <= upper};
            
            % Select regions to use as negative training samples
            lower = this.NegativeOverlapRange(1);
            upper = this.NegativeOverlapRange(2);
            negativeIndex =  {v >= lower & v < upper};
            
            labels = cls(i,:);
            labels(negativeIndex{1},:) = {this.BackgroundLabel};
            labels = {categorical(labels, this.ClassNames)};
            
            % return the region proposals, which may have been augmented
            % with the ground truth data.
            regionProposals = {regionProposals};
            
            % return number of positives per image
            numPos = nnz(positiveIndex{1});
            
        end
        
    end
end

%--------------------------------------------------------------------------
function [miniBatchData, miniBatchResponse] = batchReadAndCrop(...
    cellArrayOfImages, tbl, trainingSamples, regionResizer, randomSelector, ...
    positiveSampleIndices, imgIndices, currentMiniBatchSize, posFraction)


% positiveSampleIndices linear indices into 1:nnz(trainingSamples.Positive)

networkImageSize = regionResizer.ImageSize;


if ~iscell(cellArrayOfImages)
    cellArrayOfImages = {cellArrayOfImages};
end

% gather samples
numImages = numel(cellArrayOfImages);
posSamples = cell(numImages,1);
negSamples = cell(numImages,1);
posResponse = cell(numImages,1);
negResponse = cell(numImages,1);
k = 1;

numNegSamples = 0;
for i = reshape(imgIndices,1,[])
    
    % find which proposals to crop and resize
   
    samples = tbl.RegionProposalBoxes{i};

    I = cellArrayOfImages{k};
    
    I = localConvertImageToMatchNumberOfNetworkImageChannels(I, networkImageSize);
    p = samples(trainingSamples.Positive{i},:);
    p = p(positiveSampleIndices{i},:);
    posSamples{k} = regionResizer.cropAndResize(I, ...
        p);
    posSamplesPerImage = numel(positiveSampleIndices{i});
    negSamplesPerImage = floor(posSamplesPerImage/posFraction - posSamplesPerImage);   
    
    % only crop out enough samples to fill current mini-batch.
    bb = samples(trainingSamples.Negative{i},:);
    labels = trainingSamples.Labels{i};
    N = size(bb,1);
    
    % sample negatives
    id = randomSelector.randperm(N, min(N, negSamplesPerImage));
    negSamples{k} = regionResizer.cropAndResize(I, bb(id,:));
    numNegSamples = numNegSamples + numel(id);
    
    % response
    posResponse{k} = labels(trainingSamples.Positive{i},1);
    posResponse{k} = posResponse{k}(positiveSampleIndices{i});
    negResponse{k} = labels(trainingSamples.Negative{i},1);
    negResponse{k} = negResponse{k}(id);
    
    k = k + 1; 
end

% In the rare case we have more positives than negatives it is possible we
% will produce a partially filled minibatch. 

% Stack ROI patches into N-by-1 column vector of cells.
batchDim = 4;
posSamples = unstack(posSamples,batchDim);
negSamples = unstack(negSamples,batchDim);

miniBatchData = [posSamples; negSamples];

posResponse = cat(1,posResponse{:});
negResponse = cat(1,negResponse{:});
miniBatchResponse = [posResponse; negResponse];

% Trim down data to fit current minibatch size. Remove from end to remove
% negatives before removing positives.
overage = (size(posSamples,1) + numNegSamples) - currentMiniBatchSize;
miniBatchData(end-overage+1:end,:) = [];
miniBatchResponse(end-overage+1:end,:) = [];
end

%--------------------------------------------------------------------------
function [miniBatchData, miniBatchResponse] = sampleNegativeObservations(...
    cellArrayOfImages, tbl, trainingSamples, regionResizer, randomSelector, ...
    negSamplesPerImage, imgIndices)


% positiveSampleIndices linear indices into 1:nnz(trainingSamples.Positive)

networkImageSize = regionResizer.ImageSize;


if ~iscell(cellArrayOfImages)
    cellArrayOfImages = {cellArrayOfImages};
end

% gather samples
numImages = numel(cellArrayOfImages);
negSamples = cell(numImages,1);
negResponse = cell(numImages,1);
k = 1;

numNegSamples = 0;
for i = reshape(imgIndices,1,[])
    
    % find which proposals to crop and resize
    
    samples = tbl.RegionProposalBoxes{i};
    
    I = cellArrayOfImages{k};
    
    I = localConvertImageToMatchNumberOfNetworkImageChannels(I, networkImageSize);
    
    % only crop out enough samples to fill current mini-batch.
    bb = samples(trainingSamples.Negative{i},:);
    labels = trainingSamples.Labels{i};
    N = size(bb,1);
    
    % sample negatives
    id = randomSelector.randperm(N, min(N, negSamplesPerImage(i)));
    negSamples{k} = regionResizer.cropAndResize(I, bb(id,:));
    numNegSamples = numNegSamples + numel(id);
    
    % response
    negResponse{k} = labels(trainingSamples.Negative{i});
    negResponse{k} = negResponse{k}(id);
    
    k = k + 1;
end

% Stack ROI patches into N-by-1 column vector of cells.
batchDim = 4;
negSamples = unstack(negSamples,batchDim);

miniBatchData = negSamples;

negResponse = cat(1,negResponse{:});
miniBatchResponse = negResponse;

end

%--------------------------------------------------------------------------
function p = unstack(samples,batchDim)
sz = cellfun(@(x)size(x,batchDim),samples);
numInBatch = sum(sz);
p = cell(numInBatch,1);
first = 1;
for i = 1:numel(samples)
    c = num2cell(samples{i},1:batchDim-1);
    c = reshape(c,[],1);
    last = first + sz(i) - 1;
    p(first:last) = c;
    first = last + 1;
end
end

%--------------------------------------------------------------------------
function I = localConvertImageToMatchNumberOfNetworkImageChannels(I, imageSize)

isNetImageRGB = numel(imageSize) == 3 && imageSize(end) == 3;
isImageRGB    = ~ismatrix(I);

if isImageRGB && ~isNetImageRGB
    I = rgb2gray(I);
    
elseif ~isImageRGB && isNetImageRGB
    I = repmat(I,1,1,3);
end
end
