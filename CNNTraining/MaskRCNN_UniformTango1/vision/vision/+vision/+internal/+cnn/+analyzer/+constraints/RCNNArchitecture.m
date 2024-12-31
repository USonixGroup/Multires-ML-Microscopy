classdef RCNNArchitecture < nnet.internal.cnn.analyzer.constraints.Constraint
    % RCNNArchitecture Basic network architecture tests for R-CNN. These
    % are shared by Fast and Faster R-CNN.
    
    % Copyright 2018 The MathWorks, Inc.
    
    properties
        % NumClasses Number of object classes R-CNN should be able to
        %            detect. 
        NumClasses
    end

    properties(Access = protected)
        AllowMultiChannel
    end

    methods(Access = protected)
        function addCustomNetworkError(test,messageID,varargin)
            test.addIssueWithId("E", "Network", [], messageID,...
                    getString(message(messageID,varargin{:})));
        end
        
        %------------------------------------------------------------------
        function addCustomLayerError(test,layerIdx,messageID,varargin)
            test.addIssueWithId("E", "Layer", layerIdx, messageID,...
                    getString(message(messageID,varargin{:})));
        end     
        
        %------------------------------------------------------------------
        function idx = findExternalLayer(test,class)
            layers = [test.LayerAnalyzers.ExternalLayer];
            idx = find(arrayfun(@(x)isa(x,class),layers));
        end
    end
    
    methods
        function this = RCNNArchitecture(numClasses, allowMultiChannel)
            this@nnet.internal.cnn.analyzer.constraints.Constraint();
            this.NumClasses = numClasses;
            this.AllowMultiChannel = allowMultiChannel;
        end
        
        %------------------------------------------------------------------
        function testImageInputLayer(test)
            % Test that R-CNN has only 1 image input layer.          
            isImageInput =  [test.LayerAnalyzers.IsImageInputLayer];
            if nnz(isImageInput) ~= 1
                test.addCustomNetworkError("vision:rcnn:firstLayerNotImageInputLayer");
            end       
     
            % R-CNN can only support RGB and Grayscale inputs. This is due
            % to a limitation of being un-able to read multichannel inputs
            % through the input table infrastructure.
            inputSize = test.LayerAnalyzers(isImageInput).Outputs.Size{1};
            isInput3D = numel(inputSize)==3;
            if( ~test.AllowMultiChannel && isInput3D && ~(inputSize(3)==1||inputSize(3)==3) )
                idx = find(isImageInput);
                test.addCustomLayerError(idx,"vision:rcnn:notMonoRGB"); %#ok<FNDSB>
            end
        end
            
        %------------------------------------------------------------------
        function testHasClassificationLayer(test)
            % Test R-CNN has one classification layer and that the input
            % has the correct size.
            
            idx = test.findExternalLayer('nnet.cnn.layer.ClassificationOutputLayer');
            if isempty(idx) || numel(idx) ~= 1
                test.addCustomNetworkError("vision:rcnn:missingClassificationLayer");
            end  
            
            if numel(idx) == 1
                % has layer. check whether it's input feature map size has
                % the correct size.
                if test.LayerAnalyzers(idx).IsLayerInputValid
                 
                   details = test.LayerAnalyzers(idx);
                   
                   % The input size must support the number of object
                   % classes plus 1, where the plus 1 is for the
                   % "background" class.
                   if details.InternalLayer.NumClasses ~= test.NumClasses + 1
                       expectedSize = details.Inputs.Size{1};
                       expectedSize(3) = test.NumClasses + 1;
                       test.addCustomLayerError(idx,...
                           "vision:rcnn:invalidInputClassificationLayer",...
                           iSizetoString(expectedSize),test.NumClasses);
                   end
 
                end
            end
        end

    end
    
end

%--------------------------------------------------------------------------
function str = iSizetoString(sz)
    str = join(string(sz), matlab.internal.display.getDimensionSpecifier);
end
