classdef ImageRegionDispatcher < nnet.internal.cnn.DataDispatcher
    %   ImageRegionDispatcher class to dispatch image regions cropped and
    %   resized from a single image. This is used to form input batches
    %   for the CNN. The batches are made up of resized image regions.
   
    %   Copyright 2015-2021 The MathWorks, Inc.
    
    %----------------------------------------------------------------------
    properties
      % MiniBatchSize (int)   Number of images in a mini batch
        MiniBatchSize
        
        % EndOfEpoch    Strategy for how to cope with the last mini-batch
        % when the number of observations is not divisible by the number of
        % mini batches.
        %
        % Allowed values: 'truncateLast'
        EndOfEpoch
        
        Precision
    end
    
    %----------------------------------------------------------------------
    properties (SetAccess = private)

        % DataSize  (1x3 int) Size of each image to be dispatched
        DataSize
        
        % Image
        Image
        
        Boxes
        
        ResponseMetaData = nnet.internal.cnn.response.ResponseMetaData.empty()
        
        % ResponseSize   (1x3 int) Size of each response to be dispatched.
        % Not used in this dispatcher.
        ResponseSize
    end
    
    %----------------------------------------------------------------------
    properties (SetAccess = private, Dependent)
        % NumObservations (int) Number of observations in the data set
        NumObservations 
        
        % IsDone (logical)     True if there is no more data to dispatch
        IsDone              
    end
    
    %----------------------------------------------------------------------
    properties (Access = private)
                
        NumRegions
        NumMiniBatches
        
        DispatchedMiniBatches
        
        % CurrentIndex  (int)   Current index of image to be dispatched
        CurrentIndex
        CurrentImage
        
        % ExampleResponse   An example of the kind of response to
        % preallocate the response array
        ExampleResponse                
               
        NumObservationsDispatched
        
        % Object to crop and resize images.
        RegionResizer
    end
    
    methods
              
        %------------------------------------------------------------------
        function this = ImageRegionDispatcher(I, bboxes, miniBatchSize, ...
                endOfEpoch, precision, imageSize, resizer)
            % ImageDatastoreDispatcher   Constructor for array data dispatcher
            %
            % imageDatastore    - An ImageDatastore containing the images
            % miniBatchSize     - Size of a mini batch express in number of
            %                   images
            % endOfEpoch        - Strategy to choose how to cope with a
            %                   number of observation that is not divisible
            %                   by the desired number of mini batches
            %                   Allowed values:
            %                   'truncateLast' to truncate the last mini
            %                   batch
            % precision         - What precision to use for the dispatched
            %                   data                                   
                   
            this.NumRegions = size(bboxes,1);
            this.Boxes = bboxes;
                                
            this.MiniBatchSize = miniBatchSize;           
            this.EndOfEpoch = endOfEpoch;
            this.Precision = precision;                       
           
            % Set the expected image size
            this.DataSize = imageSize;              
            
            % Convert image to RGB or grayscale if required.
            isNetImageRGB = numel(this.DataSize) == 3 && this.DataSize(end) == 3;
            isImageRGB    = ~ismatrix(I);
            
            if isImageRGB && ~isNetImageRGB
                I = rgb2gray(I);
                
            elseif ~isImageRGB && isNetImageRGB
                I = repmat(I,1,1,3);
            end
            
            % Cast image to single as required by region resizer. This
            % saves multiple casts while calling the resizer.
            this.Image = single(I);
            
            this.RegionResizer = resizer;
        end                           
        
        %------------------------------------------------------------------
        function n = get.NumObservations( this )
            n = this.NumRegions;
        end
        
        %------------------------------------------------------------------
        function tf = get.IsDone(this)
            tf = this.NumRegions - this.NumObservationsDispatched == 0 ;          
        end
        
        %------------------------------------------------------------------
        function [miniBatchData, miniBatchResponse, info] = next(this)
            % next   Get the data and response for the next mini batch and
            % correspondent indices
                                             
            currentMiniBatchSize = min(this.MiniBatchSize, this.NumRegions - this.NumObservationsDispatched);
              
            miniBatchData = this.Precision.cast( zeros([this.DataSize currentMiniBatchSize], 'like', this.Image) );
            info = iGetInfo(zeros(currentMiniBatchSize,1));
            miniBatchResponse = [];
            
            i = 1;
            while i <= currentMiniBatchSize                                                            
                
                miniBatchData(:,:,:,i) = this.RegionResizer.cropAndResize(this.Image, this.Boxes(this.CurrentIndex,:));
                info.MiniBatchIdx(i,1) = this.nextIndex();
                i = i+1;
            end
            
            miniBatchData = this.Precision.cast(miniBatchData);
            
        end
        
        %------------------------------------------------------------------
        function start(this)
            % start     Set the next the mini batch to be the first mini
            % batch in the epoch
            
            this.CurrentIndex = 1;
            this.CurrentImage = 1;
            this.NumObservationsDispatched = 0;
        end
        
        %------------------------------------------------------------------
        function shuffle(~)
            % shuffle   Shuffle the data            
           assert(false)
        end
    end
    
    %----------------------------------------------------------------------
    methods (Access = private)
        function i = nextIndex( this )
            % nextIndex     Advance current index and return it
            
            this.NumObservationsDispatched = this.NumObservationsDispatched + 1;
            i = this.NumObservationsDispatched; 
            
            this.CurrentIndex = this.CurrentIndex + 1;                                                    
            
        end              
    end
end

function info = iGetInfo(miniBatchIdx)
% Get the info struct with fields 'MiniBatchIdx', 'Mask' and
% 'IsCompleteObservation'. This dispatcher does not process ragged
% sequences, so we set Mask to true and IsCompleteObservation to true.
info = struct('MiniBatchIdx', miniBatchIdx, ...
    'Mask', true, ...
    'IsCompleteObservation', true);
end