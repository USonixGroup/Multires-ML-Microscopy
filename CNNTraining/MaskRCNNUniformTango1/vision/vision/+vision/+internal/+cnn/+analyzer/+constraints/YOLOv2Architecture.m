classdef YOLOv2Architecture < nnet.internal.cnn.analyzer.constraints.Constraint
    % YOLOv2Architecture Basic network architecture tests for YOLOv2.
    
    % Copyright 2019-2024 The Mathworks, Inc.
    
    properties
        % Number of object classes YOLOv2 should be able to detect.
        NumClasses
    end
    
    methods(Access = protected)
        function addCustomNetworkError(test,messageID,varargin)
            test.addIssueWithId("E", "Network", [], messageID,...
                getString(message(messageID,varargin{:})));
        end
    end
    
    methods
        function this = YOLOv2Architecture(numClasses)
            this@nnet.internal.cnn.analyzer.constraints.Constraint();
            this.NumClasses = numClasses;
        end
        
        %------------------------------------------------------------------
        function testNumberOfFilters(test)
            % Verify the Number of filters of the detection layer.
            externalLayers = {test.LayerAnalyzers.ExternalLayer};
            outputIdx = cellfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2TransformLayer'),externalLayers);
            if numel(find(outputIdx))== 1
                if ~isempty(test.LayerAnalyzers(outputIdx).Inputs.Size{1,1})
                    numFilters = test.LayerAnalyzers(outputIdx).Inputs.Size{1,1}(3);
                    numAnchors = externalLayers{outputIdx}.NumAnchorBoxes;
                    numElements = 5;
                    requiredNumFilters = numAnchors*(numElements+test.NumClasses);
                    if numFilters~=requiredNumFilters
                        test.addCustomNetworkError('vision:yolo:invalidFilterSize',mat2str(requiredNumFilters),mat2str(numAnchors),mat2str(test.NumClasses))
                    end
                end
            end
        end
        
        %------------------------------------------------------------------
        function testTransformAndOutputConnected(test)
            % YOLOv2 network should have transform layer and output layer
            % connected serially.
            externalLayers = {test.LayerAnalyzers.ExternalLayer};
            outputIdx = cellfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2OutputLayer'),externalLayers);
            transformIdx = cellfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2TransformLayer'),externalLayers);
            if sum(outputIdx)==1 && sum(transformIdx)==1
                if ~strcmp(test.LayerAnalyzers(outputIdx).Inputs.Source{1,1},...
                        test.LayerAnalyzers(transformIdx).Name)
                    test.addCustomNetworkError("vision:yolo:invalidNetworkConnection");
                end
            end
        end
        
        %------------------------------------------------------------------
        function testFullyConnectedLayer(test)
            % YOLOv2 network is based on Convolution Layers and should not
            % contain any fullyConnected Layers.
            externalLayers = {test.LayerAnalyzers.ExternalLayer};
            idx = find(cellfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'),externalLayers));
            if numel(idx) ~= 0
                test.addCustomNetworkError("vision:yolo:mustNotHaveAnyFCLayer");
            end
        end
        
        %------------------------------------------------------------------
        function testYolov2OutputLayer(test)
            % Verify that YOLOv2 has only one output layer.
            outputLayersIdx = find([test.LayerAnalyzers.IsOutputLayer]);
            layers = {test.LayerAnalyzers.InternalLayer};
            if numel(outputLayersIdx) == 0
                test.addCustomNetworkError("vision:yolo:mustHaveOutputLayer");
            elseif numel(outputLayersIdx) > 1
                test.addCustomNetworkError("vision:yolo:mustHaveOnlyOneOutputLayer");
            else
                if ~isa(layers{outputLayersIdx},'nnet.internal.cnn.layer.YOLOv2OutputLayer')
                  test.addCustomNetworkError("vision:yolo:mustBeYOLOv2OutputLayer");
                end       
            end
        end
        
        %------------------------------------------------------------------
        function testYolov2TransformLayer(test)
            % Verify that YOLOv2 has only one transform layer.
            layers = {test.LayerAnalyzers.ExternalLayer};
            idx = find(cellfun(@(x)isa(x,'nnet.cnn.layer.YOLOv2TransformLayer'),layers));
            if numel(idx) == 0
                test.addCustomNetworkError("vision:yolo:mustHaveTransformLayer");
            elseif numel(idx) > 1
                test.addCustomNetworkError("vision:yolo:mustHaveOnlyOneTransformLayer");
            else
                % No error.
            end 
        end
        
        %------------------------------------------------------------------
        function testOutputGridSize(test)
            % Verify that YOLOv2 final activations size is greater than [1,1].
            layers = {test.LayerAnalyzers.InternalLayer};
            outputIdx = cellfun(@(x)isa(x,'nnet.internal.cnn.layer.YOLOv2OutputLayer'),layers);
            if (any(outputIdx))
                outputSize = test.LayerAnalyzers(outputIdx).Inputs.Size{1,1};
                if (isempty(outputSize)||any(outputSize(1:2) < 2))
                    test.addCustomNetworkError("vision:yolo:mustHaveValidFinalActivationsSize");
                end
            end
        end        
        
        %------------------------------------------------------------------
        function testImageInputLayer(test)
            % Verify that YOLOv2 has only one image input layer.
            inputLayersIdx = find([test.LayerAnalyzers.IsInputLayer]);
            layers = {test.LayerAnalyzers.InternalLayer};
            if numel(inputLayersIdx) == 0
                test.addCustomNetworkError("vision:yolo:mustHaveInputLayer");
            elseif numel(inputLayersIdx) > 1
                test.addCustomNetworkError("vision:yolo:mustHaveOnlyOneInputLayer");
            else
                if ~isa(layers{inputLayersIdx},'nnet.internal.cnn.layer.ImageInput')
                    test.addCustomNetworkError("vision:yolo:mustBeImageInputLayer");
                end
            end
        end  
    end
end