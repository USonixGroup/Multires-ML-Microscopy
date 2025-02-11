% yolov2Datastore inherits from MiniBatchable,Shuffleable,BackgroundDispatchable &
% partionableByIndex classes.

% Copyright 2018-2021 The MathWorks, Inc.
classdef yolov2Datastore < matlab.io.Datastore & ...
            matlab.io.datastore.MiniBatchable & ...
            matlab.io.datastore.Shuffleable & ...
            matlab.io.datastore.BackgroundDispatchable & ...
            matlab.io.datastore.PartitionableByIndex & ...
            matlab.io.datastore.Partitionable
    
    properties
        % Minibatch size to load data in batches.
        MiniBatchSize
        
        % Class labels of the objects to be detected.
        Classes
        
        % Size of output images by yoloDatastore.
        DatastoreOutSize
        
        % An M-by-2 matrix defining the [height width] of image sizes used
        % to train the detector. During detection, an input image is
        % resized to nearest TrainingImageSize before it is processed by
        % the detection network.
        TrainingImageSize 
    end
    
    properties(Constant)
        % OutputTableVariableNames Column names for table output by read
        % and readByIndex.
        OutputTableVariableNames = {'Images','Response'};
    end
    
    properties(SetAccess=protected)
        % Total number of Observations in the Datastore.
        NumObservations
    end
    
    properties(Access = private)
        % Current index of the image in the datastore.
        CurrentImageIndex
        
        % BBoxImageDatasource object.
        BBoxDatastore
    end
    
    methods
        function this = yolov2Datastore(trainingData,params)
            % Initialize the yolov2Datastore object properties.
            if nargin == 2
                this.Classes = string(trainingData.Properties.VariableNames(2:end));               
                this.BBoxDatastore = vision.internal.cnn.bBoxImageDatasource(trainingData,...
                    'BackgroundExecution',true);
                this.MiniBatchSize = params.MiniBatchSize;
                this.BBoxDatastore.MiniBatchSize = this.MiniBatchSize;
                this.TrainingImageSize = params.TrainingImageSize;
                this.CurrentImageIndex = 1;
                this.DispatchInBackground = params.DispatchInBackground;
                this.DatastoreOutSize = params.DatastoreOutSize;
                this.NumObservations = size(this.BBoxDatastore.ImageDatastore.Files,1);
                this.reset();
            end
        end
       
        % Check if reached the end of the training data.
        function tf = hasdata(this)
            tf = this.CurrentImageIndex <= this.NumObservations;
        end
        
        function [tbl,info] = readByIndex(ds,indices)
           
            [data,info] = iTemporaryImageBoxLabelDatastoreAdaptor(ds,indices);
            
            for ii = 1:size(data,1)
                data(ii,1:2) = yolov2ObjectDetector.trainingTransform(data(ii,:),ds.DatastoreOutSize);
            end
            
            % Table of 2 columns for images and bbox coordinates.
            tbl = table(data(:,1),data(:,2),'VariableNames',ds.OutputTableVariableNames);
        end
        
        function [data, info] = read(ds)
            if ~ds.hasdata()
                error('Call reset');
            end
            indices = ds.CurrentImageIndex:min(ds.CurrentImageIndex+ds.MiniBatchSize-1, ds.NumObservations);
            [data,info] = readByIndex(ds,indices);
            ds.CurrentImageIndex = indices(end) + 1;
        end
        
        function reset(ds)
            reset(ds.BBoxDatastore);
            ds.CurrentImageIndex = 1;
            multiScaleIndex = randi([1 size(ds.TrainingImageSize,1)],1);
            ds.DatastoreOutSize = [ds.TrainingImageSize(multiScaleIndex,:),ds.DatastoreOutSize(1,3)];
        end
        
        % PartitionByIndex should be implemented for multi-gpu/ parallel
        % support.
        function newds = partitionByIndex(ds,indices)
            newds = copy(ds);
            newds.BBoxDatastore.ImageDatastore.Files = newds.BBoxDatastore.ImageDatastore.Files(indices);
            newds.BBoxDatastore.BoundingBoxData = newds.BBoxDatastore.BoundingBoxData(indices, :);
            newds.BBoxDatastore.ClassLabelData = newds.BBoxDatastore.ClassLabelData(indices);
            newds.NumObservations = size(newds.BBoxDatastore.ImageDatastore.Files,1);
        end
        
        % Shuffle datastore.
        function dsnew = shuffle(ds)
            dsnew = copy(ds);
            dsnew.BBoxDatastore = dsnew.BBoxDatastore.shuffle();
        end
        
        % Save datastore object.
        function s = saveobj(this)
            s.BBoxDatastore = this.BBoxDatastore;
            s.MiniBatchSize = this.MiniBatchSize;
            s.DatastoreOutSize = this.DatastoreOutSize;
            s.TrainingImageSize = this.TrainingImageSize;
            s.Classes = this.Classes;
            s.DispatchInBackground = this.DispatchInBackground;
            s.NumObservations = this.NumObservations;
        end
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
    
    % Load datastore object.
    methods(Hidden, Static)
        function this = loadobj(s)
            this = vision.internal.cnn.yolo.yolov2Datastore();
            this.BBoxDatastore = s.BBoxDatastore;
            this.MiniBatchSize = s.MiniBatchSize;
            this.DatastoreOutSize = s.DatastoreOutSize;
            this.TrainingImageSize = s.TrainingImageSize;
            this.Classes = s.Classes;
            this.DispatchInBackground = s.DispatchInBackground; 
            this.NumObservations = s.NumObservations;
        end
    end
    
    methods(Access = protected)
        function aCopy = copyElement(this)
            %   COPYELEMENT   Create a deep copy of the datastore
            %   Create a deep copy of the datastore. We need to call
            %   copy on the datastore's property ImageDatastore, because it is
            %   a handle object. Creating a deep copy allows methods
            %   such as readall and preview, that call the copy method,
            %   to remain stateless.
            aCopy = copyElement@matlab.mixin.Copyable(this);
            aCopy.BBoxDatastore = copy(this.BBoxDatastore);
            aCopy.reset();
        end
    end
    
    methods(Hidden = true)
        function frac = progress(ds)
            % Determine percentage of data read from datastore.
            frac = (ds.CurrentImageIndex-1)/ds.NumObservations;
        end
    end
    
    methods(Hidden, Static)
        function image = normalizeImageAndCastToSingle(image)
            image = single(rescale(image));
        end
    end
end

%--------------------------------------------------------------------------
function [data,info] = iTemporaryImageBoxLabelDatastoreAdaptor(ds,indices)
% This is to be replaced by combine(imds,blds) datastore.
[data,info] = ds.BBoxDatastore.readByIndex(indices);
data = table2cell(data);

for ii = 1:size(data,1)
    data{ii,3} = categorical(data{ii,3},ds.Classes);
    data{ii,3} = reshape(data{ii,3},[],1);
end
end
