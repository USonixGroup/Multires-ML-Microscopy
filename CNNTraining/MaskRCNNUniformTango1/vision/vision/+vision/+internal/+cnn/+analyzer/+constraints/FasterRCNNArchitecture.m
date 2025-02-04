classdef FasterRCNNArchitecture < vision.internal.cnn.analyzer.constraints.RCNNArchitecture
    % FasterRCNNArchitecture Faster R-CNN network architecture constraints.
    %
    % Builds on constraints in RCNNArchitecture. Additionally checks that 
    %   1) network has only 1 image input layer.
    %   2) network has layers required for Faster R-CNN
    %      * 1x roiMaxPooling2dLayer, 2x rcnnBoxRegressionLayer,
    %        1x RPNSoftmaxLayer, 1x rpnClassificationLayer,
    %        1x regionProposalLayer
    %   3) Inputs sizes to layers are of expected sizes based on number of
    %      anchor boxes.
    
    % Copyright 2018 The MathWorks, Inc.

    methods
        function this = FasterRCNNArchitecture(numClasses, allowMultiChannel)
            this@vision.internal.cnn.analyzer.constraints.RCNNArchitecture(numClasses, allowMultiChannel);
        end
        
        %------------------------------------------------------------------
        function testHasOneImageInputLayer(test)
            layers = {test.LayerAnalyzers.InternalLayer};
            idx = find(cellfun(@(x)isa(x,'nnet.internal.cnn.layer.InputLayer'),layers));
            if numel(idx) ~= 1 ...
                    || ~isa(test.LayerAnalyzers(idx).ExternalLayer,'nnet.cnn.layer.ImageInputLayer')
                test.addCustomNetworkError("vision:rcnn:mustHaveOneImageInputLayer");
            end
        end
        
        %------------------------------------------------------------------
        function testArchitecture(test)
            % Test that network has all required layers and that the layers
            % have the expected input size. Also checks basic ordering of
            % layers, for example, to make sure 1 box regression layer is
            % within the sub-network after the ROI pooling layer.
            
            % Test that network has 2 box regression layers.
            boxRegIdx = test.findExternalLayer('nnet.cnn.layer.RCNNBoxRegressionLayer');
            
            hasTwoBoxRegressionLayers = numel(boxRegIdx) == 2;
            
            if ~hasTwoBoxRegressionLayers
                test.addCustomNetworkError("vision:rcnn:mustHaveTwoBoxRegLayers");
            end
            
            % Test that network has one ROI pooling layer.
            layers = [test.LayerAnalyzers.ExternalLayer];            
            [hasOneROIPooling,roiPoolIdx] = vision.internal.cnn.analyzer.constraints.FastRCNNArchitecture.hasROIMaxPoolingLayer(test);
            
            % Test that network has one region proposal layer.
            proposalIdx = test.findExternalLayer('nnet.cnn.layer.RegionProposalLayer');
            
            hasRegionProposalLayer = numel(proposalIdx) == 1;
            if ~hasRegionProposalLayer
                test.addCustomNetworkError("vision:rcnn:missingRegionProposalLayer");
            end
            
            % Test that network has one RPN softmax layer.
            sfidx = test.findExternalLayer('nnet.cnn.layer.RPNSoftmaxLayer');
            
            hasRPNSoftmax = numel(sfidx) == 1;
            if ~hasRPNSoftmax
                test.addCustomNetworkError("vision:rcnn:missingRPNSoftmaxLayer");
            end
            
            % Test that fast box regression layer has expected input size.
            % This requires knowing that we first have a box regression
            % layer in the fast r-cnn sub-network of faster r-cnn.
            hasFastBoxRegLayer = false;
            if hasTwoBoxRegressionLayers
                
                if hasOneROIPooling
                    
                    % find box regression layer in Fast R-CNN sub-network.
                    roiPoolName = layers(roiPoolIdx).Name;
                    fastBoxRegIdx = fasterRCNNObjectDetector.findFastBoxRegressionLayer(...
                        test.NetworkAnalyzer.LayerGraph, roiPoolName);
                    hasFastBoxRegLayer = numel(fastBoxRegIdx) == 1;
                    
                    if hasFastBoxRegLayer
                        if test.LayerAnalyzers(fastBoxRegIdx).IsLayerInputValid
                            % Layer size checking can only check whether
                            % input is divisible by four. Here we can check
                            % whether input is 4 * numClasses.
                            inputSize = test.LayerAnalyzers(fastBoxRegIdx).Inputs.Size{1};
                            if inputSize(3) ~= test.NumClasses * 4
                                inputSize(3) = test.NumClasses * 4; % expected size.
                                test.addCustomLayerError(fastBoxRegIdx,...
                                    "vision:rcnn:invalidBoxRegressionLayerInputSize",...
                                    iSizetoString(inputSize),test.NumClasses);
                            end
                        end
                        
                    else
                        % One box regression layer must follow ROI pooling.
                        test.addCustomNetworkError("vision:rcnn:missingBoxRegAfterROIPooling");
                    end
                    
                end
                
            end
            
            % Test that RPN softmax layer has correct input size.
            if hasRPNSoftmax && hasRegionProposalLayer
                % check that input is [H W numAnchors*2].
                if test.LayerAnalyzers(sfidx).IsLayerInputValid
                    % layer checks whether input is divisible by 2. Here we
                    % check if it can support the number of anchors specified
                    % in the region proposal layer.
                    proposal = test.LayerAnalyzers(proposalIdx).ExternalLayer;
                    numAnchors = size(proposal.AnchorBoxes,1);
                    sz =  test.LayerAnalyzers(sfidx).Inputs.Size{1};
                    if sz(3) ~= 2*numAnchors
                        test.addCustomLayerError(sfidx,"vision:rcnn:incorrectInputRPNSoftmax",...
                            iSizetoString(sz),numAnchors);
                    end
                end
            end
            
            % Test that RPN Box Reg Layer has the correct input size. 
            if hasTwoBoxRegressionLayers && hasFastBoxRegLayer
                rpnBoxRegIdx = boxRegIdx(boxRegIdx ~= fastBoxRegIdx);
                
                % Check box regression layer has correct input sizes.
                if hasRegionProposalLayer
                    proposal = test.LayerAnalyzers(proposalIdx).ExternalLayer;
                    numAnchors = size(proposal.AnchorBoxes,1);
                    sz =  test.LayerAnalyzers(rpnBoxRegIdx).Inputs.Size{1};
                    if sz(3) ~= 4*numAnchors
                        test.addCustomLayerError(sfidx,"vision:rcnn:incorrectInputRPNboxReg",...
                            iSizetoString(sz),numAnchors);
                    end
                end
            end
            
        end
        
        %------------------------------------------------------------------
        function testHasRPNClassificationLayer(test)
            idx = test.findExternalLayer('nnet.cnn.layer.RPNClassificationLayer');
            
            if numel(idx) ~= 1
                test.addCustomNetworkError("vision:rcnn:missingRPNClassificationLayer");
            end
        end
        
    end
    
end

%--------------------------------------------------------------------------
function str = iSizetoString(sz)
str = join(string(sz), matlab.internal.display.getDimensionSpecifier);
end
