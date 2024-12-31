classdef FastRCNNArchitecture < vision.internal.cnn.analyzer.constraints.RCNNArchitecture        
  
    % Architecture  Constraint object to be used by analyzeNetwork.
    %               Detects issues related to the architecture of the
    %               network.
    
    %   Copyright 2018 The MathWorks, Inc.       
    
    methods
        
        function this = FastRCNNArchitecture(numClasses,allowMultiChannel)
            this@vision.internal.cnn.analyzer.constraints.RCNNArchitecture(numClasses,allowMultiChannel);
        end
       
        function testHasROIMaxPoolingLayer(test)
            vision.internal.cnn.analyzer.constraints.FastRCNNArchitecture.hasROIMaxPoolingLayer(test);
        end
        
        function testFastRCNNBoxRegression(test)
            idx = test.findExternalLayer('nnet.cnn.layer.RCNNBoxRegressionLayer');
            if isempty(idx) || numel(idx) ~= 1
                test.addCustomNetworkError("vision:rcnn:missingBoxRegressionLayerFast");
            end
            
            if numel(idx) == 1
                % has box regression layer. check whether it's input
                % feature map size has the correct size.
                if test.LayerAnalyzers(idx).IsLayerInputValid
                   % Layer size checking can only check whether input is
                   % divisible by four. Here we can check whether input is
                   % 4 * numClasses.
                   inputSize = test.LayerAnalyzers(idx).Inputs.Size{1};
                   if inputSize(3) ~= test.NumClasses * 4
                       inputSize(3) = test.NumClasses * 4; % expected size.
                       test.addCustomLayerError(idx,...
                           "vision:rcnn:invalidBoxRegressionLayerInputSize",...
                           iSizetoString(inputSize),test.NumClasses);
                   end
                end
            end
        end
        
        
        function testHasROIInputLayer(test)
            idx = test.findExternalLayer('nnet.cnn.layer.ROIInputLayer');
            if isempty(idx) || numel(idx) ~= 1
                test.addCustomNetworkError("vision:rcnn:missingROIInputLayer");
            end
        end
    end
    
    methods(Static)
        function [tf,idx] = hasROIMaxPoolingLayer(test)
            layers = [test.LayerAnalyzers.ExternalLayer];
            idx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer')|| ....
                isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer'),layers));
            tf = numel(idx)==1;
            if isempty(idx) || numel(idx) ~= 1
                test.addCustomNetworkError("vision:rcnn:missingROIMaxPoolingLayer");
            end
        end
                
    end
end

function str = iSizetoString(sz)
    str = join(string(sz), matlab.internal.display.getDimensionSpecifier);
end