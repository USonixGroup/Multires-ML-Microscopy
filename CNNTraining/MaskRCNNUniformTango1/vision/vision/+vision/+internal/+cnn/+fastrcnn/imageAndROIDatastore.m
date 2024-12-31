classdef imageAndROIDatastore < matlab.io.Datastore & matlab.io.datastore.MiniBatchable
    % imageAndROIDatastore Datastore for single image and set of ROIs. 
    
    % Copyright 2018-2019 The MathWorks, Inc.
    properties
        Image
        ROI
        Index        
    end
    
    properties(Constant)
        ColumnVar = {'Image','ROI'};
    end
    
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
    
    methods
        
        function this = imageAndROIDatastore(I,roi)            
            this.Image = I;
            
            % Leave these as double (required by ROI pooling layer implementation) 
            % TODO: finalize what the datatype should be.
            this.ROI = roi; 
            
            % This datastore only ever has 1 observation or with a batch
            % of images, [H,W,C,B], has B observations.
            this.NumObservations = size(I,4);
            
            this.reset();
        end
        
        function tf = hasdata(this)
            tf = this.Index <= this.NumObservations;
        end
        
        function [data,info] = read(this)
            assert(hasdata(this));
            data = table({this.Image},{this.ROI},'VariableNames',this.ColumnVar);
            info.EMPTY_ON_PURPOSE = [];
            this.Index = this.NumObservations + 1;
        end
        
        function reset(this)
            this.Index = 1;
        end
    end
    
    methods(Hidden)
        function frac = progress(~)
            frac = 1;
        end
    end
end
