classdef FasterRCNNEndToEndDatastore < vision.internal.cnn.rpn.imageCentricRegionDatastore
    % End to end datastore. Add Fast R-CNN ground truth to RPN ground truth
    % targets produced by
    % vision.internal.cnn.rpn.imageCentricRegionDatastore.
    
    %   Copyright 2018-2021 The MathWorks, Inc.
    
    properties
        GroundTruth
    end
    
    properties(Dependent)
        FastClassNames
    end
    
    methods
        
        function this = FasterRCNNEndToEndDatastore(trainingData, trainingSamples, params)
            this = this@vision.internal.cnn.rpn.imageCentricRegionDatastore();
            if nargin > 0
                this.setup([trainingData trainingSamples], params);
                
                this.GroundTruth = trainingData;
                
                % Add to batching function for ground truth data
                this.BatchingFcn.GroundTruth = @iBatchGroundTruth;
            end
        end
        
        %-------------------------------------------------------------------
        function [data,info] = readByIndex(this,index)
            [data,info] = readByIndex@vision.internal.cnn.rpn.imageCentricRegionDatastore(this,index);
            
            gTruthData = table2cell(this.GroundTruth(index,2:end));
            
            T = cell(numel(index),1);
            for i = 1:numel(index)
                T{i} = {gTruthData(i,:)};
            end
            data = [data T];
            
            data.Properties.VariableNames{end} = 'GroundTruth';
        end

        function names = get.FastClassNames(this)
            names = this.GroundTruth.Properties.VariableNames(2:end);
        end
        
        %------------------------------------------------------------------
        function s = saveobj(this)
            s = saveobj@vision.internal.cnn.rpn.imageCentricRegionDatastore(this);
            s.GroundTruth = this.GroundTruth;            
        end
        
        %------------------------------------------------------------------
        function newds = partitionByIndex(this,indices)
            newds = partitionByIndex@vision.internal.cnn.rpn.imageCentricRegionDatastore(this,indices);
            newds.GroundTruth = newds.GroundTruth(indices,:); 
        end
        
        %------------------------------------------------------------------
        function newds = shuffle(this)
            [newds, idx] = this.shuffleImpl();
            newds.GroundTruth = this.GroundTruth(idx,:);
        end
    end
    
    methods(Static)
        function this = loadobj(s)
            this = vision.internal.cnn.FasterRCNNEndToEndDatastore();
            this.setPropertiesDuringLoad(s);
            this.GroundTruth = s.GroundTruth;
            this.reset();
        end

        %------------------------------------------------------------------
        function mapping = createMIMODatastoreMapping(variableNames, lgraph)
            
            externalLayers = lgraph.Layers;
            
            % Find locations of each layer in Layers array.
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RPNClassificationLayer'),...
                externalLayers);
            layerIndices.ClassificationLayerIdx = find(idx,1,'last');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'),...
                externalLayers);
            layerIndices.RegressionLayerIdx = find(idx,1,'last');
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
                externalLayers);
            layerIndices.ImageLayerIdx = find(idx,1,'first');
            
            idx = arrayfun(@(x)isa(x,'vision.internal.cnn.layer.FastRCNNOutputLayer'),...
                externalLayers);
            layerIndices.OutputLayerIdx = find(idx,1,'last');
            
            % Get layer name based on location in Layers array.
            clsName = externalLayers(layerIndices.ClassificationLayerIdx).Name;
            regName = externalLayers(layerIndices.RegressionLayerIdx).Name;
            imgName = externalLayers(layerIndices.ImageLayerIdx).Name;
            outName = externalLayers(layerIndices.OutputLayerIdx).Name;
            
            dst = {
                imgName
                clsName
                regName
                outName
                };
            
            
            % Add GroundTruth to table.
            variableNames = [variableNames 'GroundTruth'];
            
            mapping = table(variableNames',dst,...
                'VariableNames',{'Data','Destination'});
            
        end

        %------------------------------------------------------------------
        function mapping = createMIMODatastoreCellMappingForDatastoreInput(lgraph, batchingFcn, inputLayerSize)

            externalLayers = lgraph.Layers;
            % There is 1 input layer: ImageInputLayer.
            %  - the first column from the read output goes to the image input layer.

            inputMapping = {1};

            % There are 2 output layers: RPNOutputLayer, FastRCNNOutputLayer.
            %    - the second and third columns (imageSizes and bboxes) go to RPNOutputLayer.
            %    - the third and fourth columns (bboxes and labels) go to FastRCNNOutputLayer.

            outputMapping = {[2,3], [3,4]};
            % None of them are classification output layers.
            areClassificationOutputs = [false, false];

            % Find locations of each layer in Layers array.
            % RPN Output layer
            idx = arrayfun(@(x)isa(x,'vision.internal.cnn.layer.RPNOutputLayer'),...
                externalLayers);
            rpnOutputLayerIdx = find(idx,1,'last');

            % Fast R-CNN Output layer
            idx = arrayfun(@(x)isa(x,'vision.internal.cnn.layer.FastRCNNOutputLayer'),...
                externalLayers);
            fastRCNNOutputLayerIdx = find(idx,1,'last');

            [~,ordering] = sort([rpnOutputLayerIdx, fastRCNNOutputLayerIdx]);

            outputMapping = outputMapping(ordering);
            batchingFcn.OutputFunctions = batchingFcn.OutputFunctions(ordering);

            inputSizes = {inputLayerSize};
            % Set outputSizes to placeholder values to enforce 4D format.
            outputSizes = {[1,1,1], [1,1,1]};

            inputFormats = { deep.internal.PlaceholderArray([inputLayerSize NaN],'SSCB') };
            outputFormats = { deep.internal.PlaceholderArray([1 1 1 NaN],'SSCB'), deep.internal.PlaceholderArray([1 1 1 NaN],'SSCB') };

            mapping = {inputMapping, outputMapping, areClassificationOutputs, inputSizes, outputSizes, inputFormats, outputFormats};
        end

    end
end

%--------------------------------------------------------------------------
function batch = iBatchGroundTruth(TColumn)
% function to batch a column of ground truth data.
batch = TColumn(:,1) ;
end
