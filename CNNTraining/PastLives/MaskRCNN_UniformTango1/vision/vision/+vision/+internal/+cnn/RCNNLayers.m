%RCNNLayers API for creating different variants of R-CNN detection networks.
%
%   Creates R-CNN, Fast R-CNN, and Faster R-CNN variants.
%
%   lgraph = RCNNLayers.create(N, modelName, 'rcnn') returns a R-CNN
%   detection network for detecting N object classes.
%
%   lgraph = RCNNLayers.create(N, modelName, 'fast-rcnn') returns a Fast
%   R-CNN detection network for detecting N object classes.
%
%   lgraph = RCNNLayers.create(N, modelName, 'faster-rcnn', anchorBoxes)
%   returns a Faster R-CNN detection network for detecting N object
%   classes. anchorBoxes is an M-by-2 matrix defining the [height width] of
%   M anchor boxes.
%
%   modelName is a string or character vector that specifies the base
%   network model upon which the object detection network is built,
%   commonly referred to as the feature extraction or backbone network.
%   Valid values for modelName are 'alexnet', 'vgg16', 'vgg19', 'resnet18',
%   'resnet50', 'resnet101', 'inceptionv3', 'googlenet',
%   'inceptionresnetv2', or, 'mobilenetv2'.
%
%   The output is a LayerGraph object.
%
%   Notes
%   -----
%   - This function builds the specified variant of R-CNN using the
%     following steps:
%
%      1 - model -> R-CNN network
%          - Add classification layer for numClasses+1 classification.
%
%      2 - R-CNN network -> Fast R-CNN network
%          - Add ROI pooling layer
%          - ROI Input layer (only for Fast R-CNN)
%          - Add bounding box regression layers.
%
%      3 - Fast R-CNN Network to Faster R-CNN network
%          - Add region proposal network (RPN)
%
%   - Model conversion functions for pretrained networks (e.g. resnet50)
%     are written plainly using hard coded layer names and parameter values
%     to make network transformations easy to read and understand.
%
%   - The initializer is used for all weight initialization is
%     randn(sz)*0.01. This matches choice made in original paper.
%
%   References
%   ----------
%   [1] Huang, Jonathan, et al. "Speed/accuracy trade-offs for modern
%       convolutional object detectors." IEEE CVPR. 2017.
%   [2] He, Kaiming, et al. "Deep residual learning for image recognition."
%       Proceedings of the IEEE conference on computer vision and pattern
%       recognition. 2016.
%   [3] Ren, Shaoqing, et al. "Object detection networks on convolutional
%       feature maps." IEEE transactions on pattern analysis and machine
%       intelligence 39.7 (2017): 1476-1481.
%   [4] Ren, Shaoqing, et al. "Faster R-CNN: towards real-time object
%       detection with region proposal networks." IEEE transactions on
%       pattern analysis and machine intelligence 39.6 (2017): 1137-1149.

% Copyright 2018-2020 The MathWorks, Inc.

classdef RCNNLayers < handle
    
    properties(Constant)
        % List of supported pretrained networks. These names must match the
        % name of the network "loading" function.
        SupportedPretrainedNetworks = {
            'alexnet'
            'vgg16'
            'vgg19'
            'resnet18'
            'resnet50'
            'resnet101'
            'googlenet'
            'inceptionv3'
            'inceptionresnetv2'
            'squeezenet'
            'mobilenetv2'            
            }
        
        % A struct containing network information. Each field name matches
        % one of the supported pretrained networks. Update this struct when
        % adding support for new predefined networks.
        NetworkInfo = iPredefinedNetworkInfo()
        
    end
    
    properties
        LayerNames
        
        % The type of ROI pooling layer to add: 'max' or 'avg'. The 'avg'
        % pool type is in place to support previous releases where we
        % replaced the last average pooling layer with a ROI average
        % pooling layer. For new LayerGraph inputs, we only support ROI max
        % pooling layers.
        ROIPoolType = 'max'
        
        % Info struct. Contains information about the network: roi scale
        % factor, proposal layer input names, etc. additional information
        % can be added as required.
        Info
    end

    properties (Access = private)
        FeatureExtractionLayer = []
        Analysis = []
        ROIMaxPoolingLayerOption = 'auto'
        ROIOutputSize = 'auto'
    end
    
    methods
        function this = RCNNLayers(lgraph)
            if nargin == 0
                this.LayerNames = iDefaultLayerNames();
            else
                this.LayerNames = iChooseUniqueLayerNames(lgraph);
            end
        end
    end
    
    %----------------------------------------------------------------------
    methods(Static)
        function lgraph = createFasterRCNN(imageSize, numClasses, network, anchorBoxes, featureExtractionLayer, roiMaxPoolingLayerOption, roiOutputSize)

            if ischar(network) || isstring(network)
                % Transform on of the pretrained networks (e.g. resnet50)
                % into r-cnn, fast r-cnn or faster-rcnn network.
                %
                % Each network tranformation method is defined in a
                % similarly named method. For example, "resnet50"
                % transformation code is in RCNNLayers.resnet50().

                % Try to load model, issues an error if unable to load.
                try
                    lgraph = iLoadModelAsLayerGraph(network);
                catch ME
                    throwAsCaller(ME)
                end

                obj = vision.internal.cnn.RCNNLayers(lgraph);

                if isempty(featureExtractionLayer)
                    featureExtractionLayer = obj.NetworkInfo.(network).FeatureExtractionLayer;
                end
                % Replace all fully connected layers with a corresponding
                % Convolution2DLayer, except the last learnable layer.
                lgraph = obj.fc2conv2d(lgraph);
                lgraph = obj.initializeForFasterRCNN(lgraph, imageSize, featureExtractionLayer, roiMaxPoolingLayerOption, roiOutputSize);

                lgraph = obj.(network)(numClasses, lgraph, 'faster-rcnn', anchorBoxes);
            else
                % User created LayerGraph.
                isLayerGraph    = isa(network,'nnet.cnn.LayerGraph');
                assert(isLayerGraph);
                assert(~isempty(featureExtractionLayer));

                lgraph = network;

                obj = vision.internal.cnn.RCNNLayers(lgraph);

                lgraph = obj.initializeForFasterRCNN(lgraph, imageSize, featureExtractionLayer, roiMaxPoolingLayerOption, roiOutputSize);

                lgraph = fastRCNNForNonSequentialNetworks(obj, numClasses, lgraph, 'faster-rcnn', anchorBoxes, featureExtractionLayer);
            end

            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);

            % Validate created network.
            allowMultiChannel = true;
            constraints = vision.internal.cnn.RCNNLayers.constraints('faster-rcnn', numClasses, allowMultiChannel);
            analysis.applyConstraints(constraints);
            analysis.throwIssuesIfAny()

            % Initialize layer weights
            lgraph = iInitializeFCLayerInFastBoxRegressionBranch(obj,lgraph, analysis);
        end

        function [lgraph, info] = create(numClasses, network, variant, anchorBoxes, allowMultiChannel)
            narginchk(3,5);
            
            switch nargin
                case 3
                    anchorBoxes = [];
                    allowMultiChannel = false;
                case 4
                    allowMultiChannel = false;
            end                        
            
            if isempty(anchorBoxes) && (variant == "faster-rcnn" || variant == "rpn")
                assert(false, 'specify anchor boxes for faster-rcnn');
            end
            
            variant = string(variant);
            
            if ischar(network) || isstring(network)
                % Transform on of the pretrained networks (e.g. resnet50)
                % into r-cnn, fast r-cnn or faster-rcnn network.
                %
                % Each network tranformation method is defined in a
                % similarly named method. For example, "resnet50"
                % transformation code is in RCNNLayers.resnet50().
                
                % Try to load model, issues an error if unable to load.
                try
                    lgraph = iLoadModelAsLayerGraph(network);
                catch ME
                    throwAsCaller(ME)
                end
                obj = vision.internal.cnn.RCNNLayers(lgraph);
                lgraph = obj.(network)(...
                    numClasses, lgraph, variant, anchorBoxes);                                             
                
            else
                % User provided a network as a Layer arrary, LayerGraph, or
                % SeriesNetwork.
                
                isLayerArray    = isa(network,'nnet.cnn.layer.Layer');
                isSeriesNetwork = isa(network,'SeriesNetwork');
                isLayerGraph    = isa(network,'nnet.cnn.LayerGraph');
                assert(isLayerArray || isSeriesNetwork || isLayerGraph);
                
                if variant == "rcnn"
                    % R-CNN training supports Layer array, LayerGraph, or
                    % SeriesNetwork. Only SeriesNetwork inputs require
                    % transformation. Layer and LayerGraph are assumed to
                    % already be valid R-CNN networks.
                    if isLayerArray
                        obj = vision.internal.cnn.RCNNLayers();
                        lgraph = network;
                        
                    elseif isLayerGraph
                        obj = vision.internal.cnn.RCNNLayers(network);
                        lgraph = network;
                        
                    elseif isSeriesNetwork
                        % Add classification layers.
                        obj = vision.internal.cnn.RCNNLayers();
                        lgraph = obj. ...
                            addClassificationLayers(network, numClasses);
                    end
                    
                elseif variant == "fast-rcnn" || variant == "faster-rcnn" || variant == "rpn"
                    % Fast R-CNN training supports converting SeriesNetwork
                    % and Layer array into Fast R-CNN network. LayerGraph
                    % must already be a valid Fast R-CNN network.
                    
                    if isLayerArray
                        
                        % Run network analyzer to get any network related
                        % errors. This also fills in layer names so we can
                        % construct a LayerGraph from the Layer array.
                        analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
                        analysis.applyConstraints();
                        try
                            analysis.throwIssuesIfAny();
                        catch ME
                            throwAsCaller(ME);
                        end
                        
                        % Get LayerGraph from analysis.
                        lgraph = analysis.LayerGraph;
                        
                        % When the input is a Layer array for Fast R-CNN we
                        % assume it already has the classification layer
                        % for numClasses + 1 setup.
                        %
                        % Here we add the remaining items for Fast R-CNN.
                        % 1) Box regression layer.
                        % 2) ROI max/avg pooling layer. Note ROI average
                        %    pooling layer support is left in place for
                        %    backward compatibility.
                        obj = vision.internal.cnn.RCNNLayers(lgraph);
                        
                        % Determine the location and parameters for layers
                        % to add based on the input network.
                        [lastPoolLayer, roiOutputSize,...
                            featureExtractionLayer, ...
                            numFiltersLastConvLayer, ...
                            boxRegressionBranchSource, roiPoolDestination,...
                            roiPoolType, scaleFactor] =...
                            iFastRCNNConversionParameters(analysis);
                        
                        % Which type of ROI Pooling layer to add: max or
                        % avg.
                        obj.ROIPoolType = roiPoolType;
                        
                        lgraph = obj.fastOrFasterRCNNFromSequentialClassificationNetwork(...
                            numClasses, lgraph, variant, anchorBoxes, roiOutputSize, ...
                            lastPoolLayer, featureExtractionLayer, numFiltersLastConvLayer,...
                            boxRegressionBranchSource,roiPoolDestination,scaleFactor);
                        
                        
                    elseif isSeriesNetwork
                        
                        analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
                        
                        lgraph = analysis.LayerGraph;
                        
                        obj = vision.internal.cnn.RCNNLayers(lgraph);
                        
                        [lastPoolLayer, roiOutputSize,...
                            featureExtractionLayer, ...
                            numFiltersLastConvLayer, ...
                            boxRegressionBranchSource, roiPoolDestination,...
                            roiPoolType, scaleFactor] =...
                            iFastRCNNConversionParameters(analysis);
                        
                        % Which type of ROI Pooling layer to add: max or
                        % avg.
                        obj.ROIPoolType = roiPoolType;
                        
                        lgraph = obj.rcnnFromSequentialClassificationNetwork(...
                            numClasses, lgraph, variant, anchorBoxes, roiOutputSize, ...
                            lastPoolLayer, featureExtractionLayer, numFiltersLastConvLayer,...
                            boxRegressionBranchSource, roiPoolDestination,scaleFactor);
                        
                    else
                        % User created LayerGraph. This must be a valid
                        % variant of R-CNN.
                        obj = vision.internal.cnn.RCNNLayers(network);
                        lgraph = network;
                        
                    end
                    
                else
                    assert(false);
                end
                
            end
            
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
            
            % Validate created network.
            constraints = vision.internal.cnn.RCNNLayers.constraints(variant, numClasses, allowMultiChannel);
            analysis.applyConstraints(constraints);
            analysis.throwIssuesIfAny()
            
            % Initialize layer weights
            if variant == "fast-rcnn" || variant == "faster-rcnn"
                lgraph = iInitializeFCLayerInFastBoxRegressionBranch(obj,lgraph, analysis);
            end
            
            % Return info filled during network creation process.
            info = obj.Info;
            
        end
        
        %------------------------------------------------------------------
        function fast = mergeFastAndRPNLayerGraphs(fast, rpn, numClasses, info)
            
            % Remove ROI input layer
            roiInputIdx = iFindROIInputLayer(fast.Layers);
            fast = fast.removeLayers(fast.Layers(roiInputIdx).Name);
            
            % Add RPN layers
            names = {rpn.Layers.Name};
            rpnLayerNames = info.RPNLayerNames;
            for i = 1:numel(rpnLayerNames)
                idx = strcmp(rpnLayerNames{i},names);
                fast = fast.addLayers(rpn.Layers(idx));
            end
            
            % Connect RPN layers
            src = info.RPNConnections.Source;
            dst = info.RPNConnections.Destination;
            
            srcIdx = ismember(src,rpnLayerNames);
            dstIdx = ismember(dst,rpnLayerNames);
            rows = srcIdx | dstIdx;
            
            connections = info.RPNConnections(rows,:);
            
            for i = 1:height(connections)
                fast = fast.connectLayers( ...
                    connections.Source{i}, ...
                    connections.Destination{i});
            end
            
            % Add region proposal layer.
            % Add and connect region proposal layer to ROI pooling layer.
            proposalLayer = regionProposalLayer(info.AnchorBoxes, 'Name',info.ProposalName);
            fast = addLayers(fast, proposalLayer);
            
            % Connect region proposal layer.
            c = info.ProposalConnections;
            for i = 1:height(c)
                fast = connectLayers(fast, c.Source{i}, c.Destination{i});
            end
           
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(fast);
            
            % Validate created network.
            allowMultiChannel = true;
            constraints = vision.internal.cnn.RCNNLayers.constraints('faster-rcnn', numClasses, allowMultiChannel);
            analysis.applyConstraints(constraints);
            analysis.throwIssuesIfAny()
        end
        
        %------------------------------------------------------------------
        function out = constraints(variant, numClasses, allowMultiChannel)
            % Return constraints for validating a variant of R-CNN.
            
            if nargin == 2
                allowMultiChannel = false;
            end
            if variant == "rpn"
                out = nnet.internal.cnn.analyzer.constraints.Constraint.getBuiltInConstraints();
                archConstraint = arrayfun(@(x)isa(x,'nnet.internal.cnn.analyzer.constraints.Architecture'),out);
                out(archConstraint) = [];
            else
                switch string(variant)
                    case "rcnn"
                        constraint = vision.internal.cnn.analyzer.constraints.RCNNArchitecture(numClasses,allowMultiChannel);
                    case "fast-rcnn"
                        constraint = vision.internal.cnn.analyzer.constraints.FastRCNNArchitecture(numClasses,allowMultiChannel);
                    case "faster-rcnn"
                        constraint = vision.internal.cnn.analyzer.constraints.FasterRCNNArchitecture(numClasses,allowMultiChannel);
                end
                out = nnet.internal.cnn.analyzer.constraints.Constraint.getBuiltInConstraints();
                archConstraint = arrayfun(@(x)isa(x,'nnet.internal.cnn.analyzer.constraints.Architecture'),out);
                out(archConstraint) = constraint;
            end
        end
        
        function sz = minimumObjectSize(network)
            % Return the minimum object size the network can process.
            % Anything smaller and it will not map to a 1x1 element in ROI
            % pooling layer's input feature map. 
            %
            % Use this for Fast or Faster R-CNN network. 
            
            if ischar(network) || isstring(network)
                imageToFeatureScaleXY = vision.internal.cnn.RCNNLayers.NetworkInfo.(network).ROIScaleFactor;
                featureToImageScaleXY = 1./imageToFeatureScaleXY;
            else
                % User provided a network as a Layer arrary, LayerGraph, or
                % SeriesNetwork.
                
                isLayerArray    = isa(network,'nnet.cnn.layer.Layer');
                isSeriesNetwork = isa(network,'SeriesNetwork');
                isLayerGraph    = isa(network,'nnet.cnn.LayerGraph');
                assert(isLayerArray || isSeriesNetwork || isLayerGraph);
                
                analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
                
                % Find the location of the ROI pooling layer or its
                % potential location in a network.
                roiIdx = vision.internal.cnn.RCNNLayers.findPotentialLocationOfROIPoolingLayer(analysis);
                
                featureToImageScaleXY = vision.internal.cnn.RCNNLayers.featureToImageScale(analysis,roiIdx);                
            end
            
            sz = ceil(fliplr(featureToImageScaleXY));
            
        end
        
        function sz = imageLayerInfo(network)
            % Return the InputSize of the network's image input layer.
            if ischar(network) || isstring(network)
                sz = vision.internal.cnn.RCNNLayers.NetworkInfo.(network).InputSize;
            else               
                analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
                idx = iFindImageLayer(analysis);
                if sum(idx) ~= 1
                    error(message('vision:rcnn:firstLayerNotImageInputLayer'));
                end
                
                sz = analysis.LayerAnalyzers(idx).Outputs.Size{1};
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Pretrained network conversion functions.
    %----------------------------------------------------------------------
    methods(Access = private)                
        %------------------------------------------------------------------
        function lgraph = alexnet(this, numClasses, lgraph, variant, anchorBoxes)
            featureExtractionLayer = this.NetworkInfo.alexnet.FeatureExtractionLayer;

            [lgraph, lastPoolLayer, roiOutputSize, ...
                numFiltersLastConvLayer, boxRegressionBranchSource, roiPoolDestination,...
                roiScaleFactor] = fastRCNNConversionParametersWithFeatureLayer(this, lgraph, featureExtractionLayer);

            lgraph = this. ...
                rcnnFromSequentialClassificationNetwork(...
                numClasses, lgraph, variant, anchorBoxes,...
                roiOutputSize, lastPoolLayer,...
                featureExtractionLayer, numFiltersLastConvLayer,...
                boxRegressionBranchSource,roiPoolDestination,roiScaleFactor);
        end

        %------------------------------------------------------------------
        function lgraph = vgg16(this, numClasses, lgraph, variant, anchorBoxes)
            featureExtractionLayer = this.NetworkInfo.vgg16.FeatureExtractionLayer;

            [lgraph, lastPoolLayer, roiOutputSize, ...
                numFiltersLastConvLayer, boxRegressionBranchSource, roiPoolDestination,...
                roiScaleFactor] = fastRCNNConversionParametersWithFeatureLayer(this, lgraph, featureExtractionLayer);
            
            lgraph = this. ...
                rcnnFromSequentialClassificationNetwork(...
                numClasses, lgraph, variant, anchorBoxes,...
                roiOutputSize, lastPoolLayer, ...
                featureExtractionLayer, numFiltersLastConvLayer,...
                boxRegressionBranchSource,roiPoolDestination, roiScaleFactor);
        end
        
        %------------------------------------------------------------------
        function lgraph = vgg19(this, numClasses, lgraph, variant, anchorBoxes)
            featureExtractionLayer = this.NetworkInfo.vgg19.FeatureExtractionLayer;

            [lgraph, lastPoolLayer, roiOutputSize, ...
                numFiltersLastConvLayer, boxRegressionBranchSource, roiPoolDestination,...
                roiScaleFactor] = fastRCNNConversionParametersWithFeatureLayer(this, lgraph, featureExtractionLayer);
            
            lgraph = this. ...
                rcnnFromSequentialClassificationNetwork(...
                numClasses, lgraph, variant, anchorBoxes,...
                roiOutputSize, lastPoolLayer, ...
                featureExtractionLayer, numFiltersLastConvLayer,...
                boxRegressionBranchSource,roiPoolDestination, roiScaleFactor);
        end
        
        %------------------------------------------------------------------
        function lgraph = resnet50(this, numClasses, lgraph, variant, anchorBoxes)
            % Use conv_4x as feature extractor. Output size of conv4_x is 14x14. See
            % reference [1] and [2].

            featureExtractionLayer = this.NetworkInfo.resnet50.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);
        end
        
        %------------------------------------------------------------------
        function lgraph = resnet101(this, numClasses, lgraph, variant, anchorBoxes)
            % Use output of "conv_4" block as feature extractor. Output size of this
            % block is 14x14. See reference [1] and [2].

            featureExtractionLayer = this.NetworkInfo.resnet101.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);
        end
        
        %------------------------------------------------------------------
        function lgraph = googlenet(this, numClasses, lgraph, variant, anchorBoxes)
            % Use output of inception block 4d. See reference [3].

            featureExtractionLayer = this.NetworkInfo.googlenet.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);
        end

        %------------------------------------------------------------------
        function lgraph = inceptionv3(this, numClasses, lgraph, variant, anchorBoxes)
            % mixed7 is last inception block with 17x17 output. See reference [1].
            % Corresponds with mixed_6e used in TF:
            % https://github.com/tensorflow/tensorflow/blob/master/tensorflow/contrib/slim/python/slim/nets/inception_v3.py

            featureExtractionLayer = this.NetworkInfo.inceptionv3.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);
        end
        
        %------------------------------------------------------------------
        function lgraph = inceptionresnetv2(this, numClasses, lgraph, variant, anchorBoxes)
            % Mixed_6a including its associated residual layers are used as feature
            % extraction layer. See reference [1].
            %
            % Original layer names can be found here:
            %    https://github.com/keras-team/keras/blob/master/keras/applications/inception_resnet_v2.py

            % The output of are the features extracted by mixed_6a and its associated residual layers.
            featureExtractionLayer = this.NetworkInfo.inceptionresnetv2.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);
        end
        
        %------------------------------------------------------------------
        function lgraph = squeezenet(this, numClasses, lgraph, variant, anchorBoxes)
            % Replace last pooling layer with ROI pooling layer.
            % Add classification layers. Squeeze net has conv layer
            % producing classifications. Replace with equivalent conv layer
            % for detection.

            featureExtractionLayer = this.NetworkInfo.squeezenet.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);

            % Squeezenet has an average pooling layer after the last learnable conv2d layer.
            % The pool size of this layer needs to be changed if the input size to the pooling
            % has changed for reasons such as a different image size than squeezenet's default
            % image input layer size.
            lastAvgPoolingLayer = 'pool10';
            lgraph = replaceAvgPoolLayerIfNeeded(this, lgraph, lastAvgPoolingLayer);
        end

        %------------------------------------------------------------------
        function lgraph = mobilenetv2(this, numClasses, lgraph, variant, anchorBoxes)
            % Choose block_13_expand_relu as feature extraction layer. This
            % has a scale factor of 16:
            %            
            % Sandler, Mark, et al. "MobileNetV2: Inverted Residuals and
            % Linear Bottlenecks." Proceedings of the IEEE Conference on
            % Computer Vision and Pattern Recognition. 2018.

            featureExtractionLayer = this.NetworkInfo.mobilenetv2.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);
        end

        %------------------------------------------------------------------
        function lgraph = resnet18(this, numClasses, lgraph, variant, anchorBoxes)
            % Use res4b_relu as feature extractor, which as scale factor
            % 16 - similar to approach used in resnet50.

            featureExtractionLayer = this.NetworkInfo.resnet18.FeatureExtractionLayer;
            lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer);
        end
    end
    
    %----------------------------------------------------------------------
    % Generic network conversion functions.
    %----------------------------------------------------------------------
    methods(Access = private)

        %------------------------------------------------------------------
        function lgraph = fastRCNNForNonSequentialNetworks(this, numClasses, lgraph, variant, anchorBoxes, featureExtractionLayer)

            if isempty(this.Analysis)
                reinitializeAnalysis(this, lgraph);
            end

            if variant ~= "rpn"
                % Replace classification layers
                [lgraph,boxRegressionSource] = replaceClassificationLayers(this, lgraph, numClasses);
            end

            if variant == "fast-rcnn" || variant == "faster-rcnn"
                % Add regression layers
                [lgraph,fcName] = iAddBBoxRegressionBranch(this,lgraph, numClasses);
                lgraph = lgraph.connectLayers(boxRegressionSource, fcName);
            end

            if ~isempty(this.FeatureExtractionLayer)
                % Use the one provided by the user, if any.
                featureExtractionLayer = this.FeatureExtractionLayer;
            end

            if variant == "fast-rcnn" || variant == "faster-rcnn"
                [lgraph, numFiltersLastConvLayer] = insertOrReplaceROIPooling(this, lgraph, featureExtractionLayer);
            end

            if variant == "fast-rcnn"
                lgraph = iAddROIInputLayerAndConnectToROIPool(this,lgraph);
            end
            if variant == "faster-rcnn" || variant == "rpn"
                lgraph = this.addRPN(lgraph, variant, featureExtractionLayer, numFiltersLastConvLayer, anchorBoxes);
            end
        end

        %------------------------------------------------------------------
        function learnableIndex = lastLearnableLayer(this)
            % Find the last learnable layer with a fully connected layer or a conv2d layer.
            learnables = arrayfun(@(x)~isempty(x.Learnables),this.Analysis.LayerAnalyzers);
            learnableIndex = find(learnables, 1, 'last');
        end

        %------------------------------------------------------------------
        function [lgraph,boxRegressionSource] = replaceClassificationLayers(this, lgraph, numClasses)

            % Replace the last learnable layer with a fully connected layer or a conv2d layer.
            learnableIndex = lastLearnableLayer(this);
            learnableLayer = this.Analysis.ExternalLayers(learnableIndex);
            learnableCls = class(learnableLayer);
            switch learnableCls
                case 'nnet.cnn.layer.Convolution2DLayer'
                    newFCLayer = iNewConv2dLayerForRCNN(this,numClasses,learnableLayer);
                case 'nnet.cnn.layer.TransposedConvolution2DLayer'
                    newFCLayer = iNewTransposedConv2d(this,numClasses,learnableLayer);
                case 'nnet.cnn.layer.FullyConnectedLayer'
                    newFCLayer = iNewFullyConnectedLayerForRCNN(this,numClasses);
                otherwise
                    % We need to see if there are other learnable layers we want to support
                    % other than conv2d and fullyConnected layers.
                    error(message('vision:fasterRCNNLayers:invalidLearnableLayer'));
            end
            lgraph = lgraph.replaceLayer(learnableLayer.Name, newFCLayer);

            newClsLayer = iNewClassificationLayerForRCNN(this);
            newSoftmax = iNewSoftmaxLayerForRCNN(this);

            % Find box regression branch source.
            clsLayerIdx = iFindLastClassificationLayer(this.Analysis.ExternalLayers);
            softMaxLayerIdx = iFindLastSoftmaxLayer(this.Analysis.ExternalLayers);
            softMaxLayerSource = this.Analysis.LayerAnalyzers(softMaxLayerIdx).Inputs.Source{1};
            if softMaxLayerSource == learnableLayer.Name
                % If source of softmax is the learnable layer that we replaced,
                % use the learnable layer's source as the box regression branch's source.
                boxRegressionSource = this.Analysis.LayerAnalyzers(learnableIndex).Inputs.Source{1};
            else
                % There are other layers between softmax and learnable layer,
                % just use the softmax layer's source as the box regression branch's source.
                boxRegressionSource = softMaxLayerSource;
            end

            % Replace softmax and classification layers
            lgraph = lgraph.replaceLayer(this.Analysis.ExternalLayers(softMaxLayerIdx).Name, newSoftmax);
            lgraph = lgraph.replaceLayer(this.Analysis.ExternalLayers(clsLayerIdx).Name, newClsLayer);
        end

        %------------------------------------------------------------------
        function [lgraph, numFiltersLastConvLayer] = insertOrReplaceROIPooling(this, lgraph, featureExtractionLayer)
            featureLayerIdx = arrayfun(@(x) x.Name == ...
                featureExtractionLayer,this.Analysis.LayerAnalyzers);
            outLayers = this.Analysis.LayerAnalyzers(featureLayerIdx).Outputs.Destination{1};
            [lgraph, outLayers, outputSize, numFiltersLastConvLayer] = removePoolLayerIfNeeded(this, ...
                lgraph, outLayers, featureLayerIdx);

            for ii = 1:numel(outLayers)
                % Disconnect
                lgraph = lgraph.disconnectLayers(featureExtractionLayer, outLayers{ii});
            end

            % Add ROI pooling layer
            roiName = this.LayerNames.ROIPooling;
            scaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScaleWithFeatureMapSize(this.Analysis,outputSize);
            roiPooling = createROIPoolingLayer(this,outputSize,roiName,scaleFactor);
            lgraph = lgraph.addLayers(roiPooling);

            % Connect feature layer to the ROI pooling layer
            lgraph = lgraph.connectLayers(featureExtractionLayer, roiName + "/in");

            for ii = 1:numel(outLayers)
                % Connect ROI Pooling layer to disconnected layers.
                lgraph = lgraph.connectLayers(roiName, outLayers{ii});
            end
        end

        %------------------------------------------------------------------
        function lgraph = replaceAvgPoolLayerIfNeeded(this, lgraph, lastAvgPoolingLayer)
            % Replace the existing avg pooling layer with a new one, if the input size
            % and the pool size do not match.
            reinitializeAnalysis(this,lgraph);
            poolIdx = arrayfun(@(x) x.Name == lastAvgPoolingLayer,this.Analysis.LayerAnalyzers);
            inputSize = this.Analysis.LayerAnalyzers(poolIdx).Inputs.Size{1}(1:2);
            poolLayer = this.Analysis.LayerAnalyzers(poolIdx).ExternalLayer;
            if iAvgPoolLayerNeedsReplace(poolLayer, inputSize)
                newPoolLayer = iNewAvgPoolLayer(poolLayer,inputSize);
                lgraph = replaceLayer(lgraph,poolLayer.Name,newPoolLayer);
            end
        end

        %------------------------------------------------------------------
        function [lgraph, outLayers, outputSize, numFiltersLastConvLayer] = removePoolLayerIfNeeded(this, lgraph, outLayers, featureLayerIdx)
            featureLayer = this.Analysis.LayerAnalyzers(featureLayerIdx).Name;
            if numel(outLayers) ~= 1
                if isequal(this.ROIMaxPoolingLayerOption, 'replace')
                    error(message('vision:fasterRCNNLayers:nonPoolingReplaceAfterFeature',...
                        featureLayer));
                end
                 % Use the feature layer's output size and channel size.
                [outputSize, numFiltersLastConvLayer] = getROIOutputAndChannelSizes(this, featureLayerIdx);
                return;
            end
            roiIdx = arrayfun(@(x) x.Name == ...
                outLayers{1},this.Analysis.LayerAnalyzers);
            externalLayers = [this.Analysis.LayerAnalyzers.ExternalLayer];
            nextLayerIsPooling = iVerifyROIType(externalLayers(roiIdx), this.ROIMaxPoolingLayerOption, featureLayer);
            if nextLayerIsPooling && any(contains(this.ROIMaxPoolingLayerOption, {'auto', 'replace'}))
                % Before removing the pool layer, get the next layer names.
                removePoolLayer = outLayers{1};
                poolLayerIdx = arrayfun(@(x) x.Name == removePoolLayer,this.Analysis.LayerAnalyzers);
                outLayers = this.Analysis.LayerAnalyzers(poolLayerIdx).Outputs.Destination{1};

                % Use the existing pool layer's output size and channel size.
                [outputSize, numFiltersLastConvLayer] = getROIOutputAndChannelSizes(this, poolLayerIdx);

                % Remove the existing pool layer.
                lgraph = lgraph.removeLayers(removePoolLayer);
            else
                % Use the feature layer's output size and channel size.
                [outputSize, numFiltersLastConvLayer] = getROIOutputAndChannelSizes(this, featureLayerIdx);
            end
        end

        %------------------------------------------------------------------
        function lgraph = fc2conv2d(this, lgraph)
            reinitializeAnalysis(this, lgraph);
            externalLayers = [this.Analysis.LayerAnalyzers.ExternalLayer];
            idx = iFindFullyConnectedLayers(externalLayers);
            learnableIndex = lastLearnableLayer(this);
            for ii = 1:numel(idx)
                fcIdx = idx(ii);
                if learnableIndex ~= fcIdx
                    fc = this.Analysis.LayerAnalyzers(fcIdx).ExternalLayer;
                    inputSize = this.Analysis.LayerAnalyzers(fcIdx).Inputs.Size{1};
                    conv2dLayer = iFullyConnected2Conv2d(fc, inputSize(1:2), inputSize(3));
                    lgraph = replaceLayer(lgraph, fc.Name, conv2dLayer);
                end
            end
        end

        %------------------------------------------------------------------
        function [outputSize, numFiltersLastConvLayer] = getROIOutputAndChannelSizes(this, layerIdx)
            % Output size for the ROI pooling layer
            outputSize = getROIOutputSize(this, layerIdx);
            % Number of channels in the input to ROI pooling layer.
            numFiltersLastConvLayer = this.Analysis.LayerAnalyzers(layerIdx).Outputs.Size{1}(3);
        end

        function roiOutputSize = getFeatureLayerOutputSize(this, lgraph, roiOutputSize)
            if isequal(roiOutputSize, 'auto')
                analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
                featureLayerIdx = arrayfun(@(x) x.Name == ...
                    this.FeatureExtractionLayer,analysis.LayerAnalyzers);
                outLayers = analysis.LayerAnalyzers(featureLayerIdx).Outputs.Destination{1};
                if numel(outLayers) == 1
                    % If feature layer leads to just one layer, check if it
                    % is a max pooling layer.
                    roiIdx = arrayfun(@(x) x.Name == ...
                        outLayers{1},analysis.LayerAnalyzers);
                    
                    externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
                    nextLayerIsPooling = iVerifyROIType(externalLayers(roiIdx), this.ROIMaxPoolingLayerOption, this.FeatureExtractionLayer);
                    if nextLayerIsPooling && any(contains(this.ROIMaxPoolingLayerOption, {'auto', 'replace'}))
                        % The option is to replace or auto, use the output
                        % size of the existing pooling layer
                        poolLayer = outLayers{1};
                        poolLayerIdx = arrayfun(@(x) x.Name == poolLayer,analysis.LayerAnalyzers);
                        roiOutputSize = analysis.LayerAnalyzers(poolLayerIdx).Outputs.Size{1}(1:2);
                        return;
                    end
                end
                % Just use feature layer's output size as pooling layer's
                % output size.
                roiOutputSize = analysis.LayerAnalyzers(featureLayerIdx).Outputs.Size{1}(1:2);
            end
        end
        %------------------------------------------------------------------
        function outputSize = getROIOutputSize(this, layerIdx)
            if isequal(this.ROIOutputSize, 'auto')
                outputSize = this.Analysis.LayerAnalyzers(layerIdx).Outputs.Size{1}(1:2);
            else
                outputSize = this.ROIOutputSize;
            end
        end
        %------------------------------------------------------------------
        function reinitializeAnalysis(this, lgraph)
            this.Analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
        end

        %------------------------------------------------------------------
        function lgraph = initializeForFasterRCNN(this, lgraph, imageSize, featureExtractionLayer, roiMaxPoolingLayerOption, roiOutputSize)
            % Validate FeatureLayer exists in the network or not.
            iValidateFeatureLayerExistence(lgraph.Layers,featureExtractionLayer);
            this.FeatureExtractionLayer = featureExtractionLayer;
            this.ROIMaxPoolingLayerOption = roiMaxPoolingLayerOption;
            this.ROIOutputSize = getFeatureLayerOutputSize(this, lgraph, roiOutputSize);
            % Replace input size of image input layer.
            [lgraph,this.Analysis] = iReplaceImageInputLayer(lgraph,imageSize);
            if isempty(this.Analysis)
                reinitializeAnalysis(this, lgraph);
            end            
        end

        %------------------------------------------------------------------
        function lgraph = fastOrFasterRCNNFromSequentialClassificationNetwork(...
                this, numClasses, lgraph, variant, anchorBoxes, roiOutputSize, ...
                lastPoolLayer, featureExtractionLayerName, numFiltersLastConvLayer,...
                regressionBranchSource, roiPoolDestination,scaleFactor)
            
            if variant ~= "rpn"
                if variant == "fast-rcnn" || variant == "faster-rcnn"
                    % Replace last max pooling layer with ROI max pooling layer.
                    poolLayerName = lastPoolLayer;
                    roiPoolName = this.LayerNames.ROIPooling;
                    roiPoolingLayer = createROIPoolingLayer(this,roiOutputSize, roiPoolName, scaleFactor);
                    lgraph = lgraph.removeLayers(poolLayerName);
                    lgraph = lgraph.addLayers(roiPoolingLayer);
                    lgraph = lgraph.connectLayers(featureExtractionLayerName,roiPoolName + "/in");
                    
                    if strcmp(lastPoolLayer,regressionBranchSource)
                        % The last pooling layer was removed so it cannot be a
                        % source any longer. Update the name to the ROI pooling
                        % layer name.
                        regressionBranchSource = roiPoolName;
                    end
                    
                    % Add regression layers
                    [lgraph, fcName] = iAddBBoxRegressionBranch(this, lgraph, numClasses);
                    lgraph = lgraph.connectLayers(regressionBranchSource, fcName);
                    
                    lgraph = lgraph.connectLayers(roiPoolName,roiPoolDestination);
                end
                
                if variant == "fast-rcnn"
                    lgraph = iAddROIInputLayerAndConnectToROIPool(this,lgraph);
                end
            end
            
            if variant == "faster-rcnn" || variant == "rpn"
                
                lgraph = this.addRPN(lgraph, variant, featureExtractionLayerName, numFiltersLastConvLayer, anchorBoxes);
                
            end
        end

        %------------------------------------------------------------------
        function lgraph = rcnnFromSequentialClassificationNetwork(this, ...
                numClasses, lgraph, variant, anchorBoxes, roiOutputSize, ...
                lastPoolLayer, featureExtractionLayerName, numFiltersLastConvLayer,...
                boxRegressionBranchSource,roiPoolDestination,roiScaleFactor)
            % Used by alexnet, vgg16, and vgg19, which all have same names
            % for last pooling layer and FC layers.
            %
            % numFilterLastConvLayer is used to create the 3x3 conv layer
            % for RPN.
            
            if variant ~= "rpn"
                % Add classification layers
                lgraph = this.addClassificationLayers(lgraph, numClasses);
            end
            
            % Add fast-rcnn or faster-rcnn layers.
            lgraph = this.fastOrFasterRCNNFromSequentialClassificationNetwork(...
                numClasses, lgraph, variant, anchorBoxes, roiOutputSize, ...
                lastPoolLayer, featureExtractionLayerName, numFiltersLastConvLayer,...
                boxRegressionBranchSource,roiPoolDestination,roiScaleFactor);
        end
        
        %------------------------------------------------------------------
        function lgraph = addRPN(this, lgraph, variant, featureExtractionLayerName, ...
                numFiltersLastConvLayer, anchorBoxes)
            
            % Record layer names in feature extraction network - used
            % to freeze/unfreeze layers during alternate training
            % rounds of Faster R-CNN.
            this.Info.FeatureExtractionLayers = vision.internal.cnn.RCNNLayers.featureExtractionLayerNames(lgraph,featureExtractionLayerName);
            
            if variant == "rpn"
                % remove all layers after feature extraction layer
                dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);
                
                % Find the feature extraction node.
                id = findnode(dg,char(featureExtractionLayerName));
                assert(~isempty(id));
                
                % Search for all nodes starting from the feature extraction
                % layer.
                ids = dfsearch(dg,id);
                names = dg.Nodes.Name(ids,:);
                
                lgraph = removeLayers(lgraph, names(2:end)); % exclude feature extraction layer which is first name.                
                
                this.Info.FeatureExtractionLayers = string({lgraph.Layers.Name});
            end
            
            rpnLayers = [
                convolution2dLayer(3, numFiltersLastConvLayer ,'padding',[1 1],'Name',this.LayerNames.rpnConv3x3,'WeightsInitializer',iNormalInitializer())
                reluLayer('Name',this.LayerNames.rpnRelu)
                ];
            
            numAnchors = size(anchorBoxes,1);
            
            rpnClsLayers = [
                convolution2dLayer(1, numAnchors*2,'Name', this.LayerNames.rpnConvCls,'WeightsInitializer',iNormalInitializer())
                rpnSoftmaxLayer('Name', this.LayerNames.rpnSoftmax)
                rpnClassificationLayer('Name',this.LayerNames.rpnCls)
                ];
            
            rpnRegLayers = [
                convolution2dLayer(1, numAnchors*4, 'Name', this.LayerNames.rpnConvReg,'WeightsInitializer',iNormalInitializer())
                rcnnBoxRegressionLayer('Name', this.LayerNames.rpnBoxReg);
                ];
            
            % Add RPN specific layers.
            lgraph = addLayers(lgraph, rpnLayers);
            lgraph = addLayers(lgraph, rpnRegLayers);
            lgraph = addLayers(lgraph, rpnClsLayers);
            
            rpnLayerNames=[{rpnLayers.Name} {rpnRegLayers.Name} {rpnClsLayers.Name}];
            
            % Connect RPN layers together
            lgraph = connectLayers(lgraph, this.LayerNames.rpnRelu, this.LayerNames.rpnConvCls);
            lgraph = connectLayers(lgraph, this.LayerNames.rpnRelu, this.LayerNames.rpnConvReg);
            
            % Connect to RPN to main network.
            lgraph = connectLayers(lgraph, featureExtractionLayerName, this.LayerNames.rpnConv3x3);
            
            if variant == "faster-rcnn"
                % Add and connect region proposal layer to ROI pooling layer.
                proposalLayer = regionProposalLayer(anchorBoxes, 'Name',this.LayerNames.rpnProposal);
                roiPoolName = this.LayerNames.ROIPooling;
                lgraph = addLayers(lgraph, proposalLayer);
                lgraph = connectLayers(lgraph, this.LayerNames.rpnConvCls, this.LayerNames.rpnProposal + "/scores");
                lgraph = connectLayers(lgraph, this.LayerNames.rpnConvReg, this.LayerNames.rpnProposal + "/boxDeltas");
                lgraph = connectLayers(lgraph, this.LayerNames.rpnProposal, roiPoolName + "/roi");
            end
            
            connections(1).Source = this.LayerNames.rpnConvCls;
            connections(1).Destination = this.LayerNames.rpnProposal + "/scores";
            connections(2).Source = this.LayerNames.rpnConvReg;
            connections(2).Destination = this.LayerNames.rpnProposal + "/boxDeltas";
            connections(3).Source = this.LayerNames.rpnProposal;
            connections(3).Destination = this.LayerNames.ROIPooling + "/roi";
            
            % Fill in proposal layer connections
            this.Info.ProposalConnections = struct2table(connections);
            this.Info.ProposalName = this.LayerNames.rpnProposal;
            
            % Fill Info struct with proposal layer input names.
            this.Info.ProposalInputLayers.ClassificationLayerName = char(this.LayerNames.rpnConvCls);
            this.Info.ProposalInputLayers.RegressionLayerName = char(this.LayerNames.rpnConvReg);
            
            % Record the layer graph connections;
            this.Info.ScaleFactor = []; % filled outside of this function.
            this.Info.AnchorBoxes = anchorBoxes;
            this.Info.RPNConnections = lgraph.Connections;
            this.Info.RPNLayerNames = rpnLayerNames;           
        end
        
        %------------------------------------------------------------------
        function [layerGraphOrLayers] = addClassificationLayers(this, network, numClasses)
            % Modify network layers by changing last layers to support
            % classification of numClasses + 1 classes (plus 1 for
            % background class).
            %
            % This function removes the last fully connected layer and
            % classification layer. These are replaced with new versions.
            %
            % R-CNN, Fast R-CNN, and Faster R-CNN all require this
            % transformation.
            
            newFCLayer = iNewFullyConnectedLayerForRCNN(this,numClasses);
            
            % boost learning rate of the new FC layer
            newFCLayer.WeightLearnRateFactor = 20;
            newFCLayer.WeightL2Factor = 1;
            newFCLayer.BiasLearnRateFactor = 10;
            newFCLayer.BiasL2Factor = 1;
            
            clsLayer = iNewClassificationLayerForRCNN(this);
            newSoftmax = iNewSoftmaxLayerForRCNN(this);
            
            if isa(network,'SeriesNetwork')
                lastFCLayerIndex = find( ...
                    arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'), network.Layers), ...
                    1, 'last');
                
                if isempty(lastFCLayerIndex)
                    error(message('vision:rcnn:dagUnableToFindLastFC',numClasses + 1));
                end
                
                % Replace last FC layer
                layerGraphOrLayers = network.Layers;
                
                layerGraphOrLayers(lastFCLayerIndex) = newFCLayer;
                
                % replace softmax and classification layers
                layerGraphOrLayers(end-1) = newSoftmax;
                layerGraphOrLayers(end)   = clsLayer;
                
                % NB: Return these as Layer array because the layers may
                % not have names. We are unable to create a LayerGraph when
                % layers have undefined names.
            else
                if isa(network, 'nnet.cnn.LayerGraph')
                    % Maybe a LayerGraph when we construct networks by
                    % name.
                    layerGraphOrLayers = network;
                else
                    assert(false,'expected LayerGraph')
                end
                
                % define new classification layer to reset ClassNames from
                % previous training.
                clsLayerIndex = find( ...
                    arrayfun(@(x)...
                    isa(x,'nnet.cnn.layer.ClassificationOutputLayer'), layerGraphOrLayers.Layers), ...
                    1, 'last');
                
                % No need to throw error if classification layer was found.
                % Error checking of the network in trainRCNNObjectDetector
                % ensures the network has one classification layer. Assert
                % here instead for internal use.
                assert(~isempty(clsLayerIndex));
                
                clsLayerName = layerGraphOrLayers.Layers(clsLayerIndex).Name;
                
                % replace classification layer
                layerGraphOrLayers = layerGraphOrLayers.replaceLayer(clsLayerName, clsLayer);
                
                % Find and replace last fully connected layer.
                lastFCLayerIndex = vision.internal.cnn.utils.findLastFullyConnectedLayer(layerGraphOrLayers);
                if isempty(lastFCLayerIndex)
                    error(message('vision:rcnn:dagUnableToFindLastFC',numClasses + 1));
                end
                
                % Find source and destination layers for FC layer (src -> FC -> dest).
                fcLayerName = layerGraphOrLayers.Layers(lastFCLayerIndex).Name;
                layerGraphOrLayers = layerGraphOrLayers.replaceLayer(fcLayerName, newFCLayer);
                
            end
        end
        
        %------------------------------------------------------------------
        function layer = createROIPoolingLayer(this,outputSize,name,roiScaleFactor)
            
            name = char(name);
            if this.ROIPoolType == "max"
                internalLayer = nnet.internal.cnn.layer.ROIMaxPooling2DLayer(...
                    name, outputSize, roiScaleFactor);
                
                % Pass the internal layer to a function to construct a user visible layer.
                layer = nnet.cnn.layer.ROIMaxPooling2DLayer(internalLayer);
            else
                layer = vision.cnn.layer.ROIAveragePooling2DLayer(...
                    vision.internal.cnn.layer.ROIAveragePooling2DLayer(name,outputSize,roiScaleFactor));
            end
        end

        %--------------------------------------------------------------------------
        function [lgraph, lastPoolName, outputSize, ...
                numFiltersLastConvLayer, regressionBranchSource, roiPoolDestination,...
                scaleFactor] = ...
                fastRCNNConversionParametersWithFeatureLayer(this, lgraph, featureExtractionLayer)

            if ~isempty(this.FeatureExtractionLayer)
                featureExtractionLayer = this.FeatureExtractionLayer;
            end
            if isempty(this.Analysis)
                reinitializeAnalysis(this, lgraph);
            end
            % The feature extraction layer. These are the features the ROI pooling
            % layer pools.
            featureLayerIdx = arrayfun(@(x) x.Name == ...
                featureExtractionLayer,this.Analysis.LayerAnalyzers);

            roiName = this.Analysis.LayerAnalyzers(featureLayerIdx).Outputs.Destination{1};

            roiIdx = find(arrayfun(@(x) x.Name == ...
                roiName,this.Analysis.LayerAnalyzers));

            externalLayers = [this.Analysis.LayerAnalyzers.ExternalLayer];
            isPooling = iVerifyROIType(externalLayers(roiIdx), this.ROIMaxPoolingLayerOption, featureExtractionLayer);
            if ~isPooling
                outLayers = this.Analysis.LayerAnalyzers(featureLayerIdx).Outputs.Destination{1};
                for ii = 1:numel(outLayers)
                    % Disconnect
                    lgraph = lgraph.disconnectLayers(featureExtractionLayer, outLayers{ii});
                end

                % Add ROI pooling layer
                outputSize = getROIOutputSize(this, featureLayerIdx);
                % Number of channels in the input to ROI pooling layer.
                numFiltersLastConvLayer = this.Analysis.LayerAnalyzers(featureLayerIdx).Outputs.Size{1}(3);

                roiName = this.LayerNames.ROIPooling;
                scaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScaleWithFeatureMapSize(this.Analysis,outputSize);
                roiPooling = createROIPoolingLayer(this,outputSize,roiName,scaleFactor);
                lgraph = lgraph.addLayers(roiPooling);

                lgraph = lgraph.connectLayers(featureExtractionLayer, roiPoolName + "/in");

                for ii = 1:numel(outLayers)
                    % Connect ROI Pooling layer to disconnected layers.
                    lgraph = lgraph.connectLayers(roiPoolName, outLayers{ii});
                end
                lastPoolName = char(roiName);

                roiPoolDestination = outLayers;
            else
                % Get size of output size of the max pooling layer. The ROI pooling
                % layer's GridSize must be set to this value.
                outputSize = getROIOutputSize(this, roiIdx);

                lastPoolName = char(roiName);

                % Number of channels in the input to ROI pooling layer.
                numFiltersLastConvLayer = this.Analysis.LayerAnalyzers(roiIdx).Inputs.Size{1}(3);

                % Destination of ROI pool output.
                roiPoolDestination = this.Analysis.LayerAnalyzers(roiIdx).Outputs.Destination{1};

                % Compute ROI pool scale factor.
                scaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScale(this.Analysis,roiIdx);
            end

            % Find source of last FC layer attached to classificaiton layer. This is
            % also the source of the box regression branch.
            lastFCLayerIndex = vision.internal.cnn.utils.findLastFullyConnectedLayer(this.Analysis.LayerGraph);
            assert(~isempty(lastFCLayerIndex));
            regressionBranchSource = this.Analysis.LayerAnalyzers(lastFCLayerIndex).Inputs.Source{1};
        end

    end
    
    methods(Static)
        function [lgraph, roiScaleFactor] = updateROIPoolingLayerScaleFactor(network,imageSize)
            
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
            
            if nargin == 1
                roiScaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScale(analysis);
            else
                roiScaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScaleGivenImageSize(analysis,imageSize);
            end                     
            
            lgraph = analysis.LayerGraph;
            
            % Find ROI pooling layer.
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            roiIdx = vision.internal.cnn.RCNNLayers.findROIPoolingLayer(externalLayers);
            oldROIPool = externalLayers(roiIdx);
            
            % Create new one.
            obj = vision.internal.cnn.RCNNLayers();
            obj.ROIPoolType = iROIType(externalLayers(roiIdx));                             
            
            newROIPool = obj.createROIPoolingLayer(...
                oldROIPool.OutputSize, ...
                oldROIPool.Name,...
                roiScaleFactor);
            
            lgraph = lgraph.replaceLayer(oldROIPool.Name, newROIPool);
        end                
        
        %------------------------------------------------------------------
        function network = updateROIPoolingLayerScaleFactorValue(network, scaleFactor)            
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
            lgraph = analysis.LayerGraph;
            
            % Find ROI pooling layer.
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            roiIdx = vision.internal.cnn.RCNNLayers.findROIPoolingLayer(externalLayers);
            oldROIPool = externalLayers(roiIdx);
            
            % Create new one.
            obj = vision.internal.cnn.RCNNLayers();
            obj.ROIPoolType = iROIType(externalLayers(roiIdx));          
            
            newROIPool = obj.createROIPoolingLayer(...
                oldROIPool.OutputSize, ...
                oldROIPool.Name,...
                scaleFactor);
            
            lgraph = lgraph.replaceLayer(oldROIPool.Name, newROIPool);      
            
            network = vision.internal.cnn.createDAGNetwork(lgraph);
        end
        
        %--------------------------------------------------------------------------
        function scaleFactor = imageToFeatureScale(analysis,roiIdx)
            % Return scaleFactor, [sx sy], to scale from image to feature space.
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            if nargin == 1
                roiIdx = vision.internal.cnn.RCNNLayers.findROIPoolingLayer(externalLayers);
            end
            featureMapInputSize = analysis.LayerAnalyzers(roiIdx).Inputs.Size{1};
            scaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScaleWithFeatureMapSize(...
                analysis,featureMapInputSize);
        end

        %--------------------------------------------------------------------------
        function scaleFactor = imageToFeatureScaleWithFeatureMapSize(analysis,featureMapInputSize)
            imgIdx = iFindImageLayer(analysis);
            inputSize = analysis.LayerAnalyzers(imgIdx).Outputs.Size{1};
            scaleFactor = fliplr( featureMapInputSize(1:2)./inputSize(1:2) );
        end
        
        %------------------------------------------------------------------
        function scaleFactor = featureToImageScale(analysis,varargin)
            % Return scaleFactor, [sx sy], to scale from feature to image space.
            scaleFactor = 1./vision.internal.cnn.RCNNLayers.imageToFeatureScale(analysis,varargin{:});
        end
        
        %--------------------------------------------------------------
        function idx = findROIPoolingLayer(externalLayers)
            idx = find(...
                arrayfun( @(x)isa(x,'nnet.cnn.layer.ROIMaxPooling2DLayer')...
                || isa(x,'vision.cnn.layer.ROIAveragePooling2DLayer'), ...
                externalLayers), 1, 'last');
        end
        
        %------------------------------------------------------------------
        function scaleFactor = imageToFeatureScaleGivenImageSize(analysis, imageSize)
            
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            roiIdx = vision.internal.cnn.RCNNLayers.findROIPoolingLayer(externalLayers);
            
            % Find source of the roi pooling layer
            roiPoolSource = analysis.LayerAnalyzers(roiIdx).Inputs.Source{1};
            
            % Preserve channel dim to prevent errors in case imageSize is
            % grayscale and network assumes RGB. gray to rgb conversion is
            % handled during inference. And the scale factor only depends
            % on spatial ([H W]) dims so this is safe.
            idx = [analysis.LayerAnalyzers.IsImageInputLayer];           
            currentImageSize = analysis.LayerAnalyzers(idx).Outputs.Size{1};
            imageSize = [imageSize(1:2) currentImageSize(3:end)];
            
            sz = vision.internal.cnn.RCNNLayers.inferOutputSizesGivenImageInputSize(roiPoolSource,analysis.LayerGraph,imageSize);
            
            scaleFactor = fliplr(sz(1:2)./imageSize(1:2));
        end
        
        function sz = inferOutputSizesGivenImageInputSize(layerName, lgraph, imageSize)
            % Return the output size of the layer named layerName, give the
            % input image size is imageSize.
            
            % Find and replace the image input layer with a new image input
            % layer configured with the new image size.
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),lgraph.Layers);
            imgLayer = imageInputLayer(imageSize,'Name',lgraph.Layers(idx).Name);
            lgraph = lgraph.replaceLayer(imgLayer.Name,imgLayer);
            
            % Use analyzer to compute output sizes.
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            names = {externalLayers.Name};
            idx = strcmp(names,layerName);
            sz = analysis.LayerAnalyzers(idx).Outputs.Size{1};
        end
        
        function dg = digraph(lgraph)
            % Create a digraph from a layer graph.
            
            edgeTable = mergevars(lgraph.Connections,...
                {'Source','Destination'},'NewVariableName','EndNodes');
            
            % Strip off port names, e.g. layer/in -> layer. This produces a
            % connected graph similar to the internal layer graph. It
            % ensures we search the complete sub-network and don't end up
            % with sub-graph shards.
            edgeTable.EndNodes = regexprep(edgeTable.EndNodes,'\/\w*','');
            
            % Create digraph.
            dg = digraph(edgeTable);
        end
        
        %------------------------------------------------------------------
        function net = revertModificationForEndToEndTraining(net, info, classNames)
            % Revert modifications made for end-to-end training. Remove
            % merge and output layers. Add back the classification and
            % regressison layers stored in info.
            
            lgraph = layerGraph(net);            
            
            % -------------------------------
            % Fast RCNN Modification Reversal 
            % -------------------------------

            % Remove Fast RCNN adapted layers.
            lgraph = lgraph.removeLayers(info.MergeLayerName);
            lgraph = lgraph.removeLayers(info.OutputLayerName);
            
            % Update class names.
            clsLayer = info.ClassificationLayer;
            clsLayer.Classes = classNames;
            
            % Add back original layers.
            lgraph = lgraph.addLayers(clsLayer);
            lgraph = lgraph.addLayers(info.BoxRegressionLayer);
            
            % Connect layers.
            lgraph = lgraph.connectLayers(info.ClassificationLayerSource, info.ClassificationLayer.Name);
            lgraph = lgraph.connectLayers(info.BoxRegressionLayerSource, info.BoxRegressionLayer.Name);

            lgraph = vision.internal.cnn.RCNNLayers.revertRPNModificationForEndToEndTraining(lgraph, info);

            % -----------------------------
            % Create DAGNetwork.
            % -----------------------------
            net = vision.internal.cnn.createDAGNetwork(lgraph);
        end

        %------------------------------------------------------------------
        function lgraph = revertRPNModificationForEndToEndTraining(lgraph, info)
            % -----------------------------
            % RPN Modification Reversal 
            % -----------------------------

            if ~isfield(info, 'RPNDepthConcatenationLayerName')
                % We did not modify RPN network to accomodate proposal calculations at the output layer.
                return;
            end

            % Remove RPN adapted layers.
            lgraph = lgraph.removeLayers(info.RPNDepthConcatenationLayerName);
            lgraph = lgraph.removeLayers(info.RPNOutputLayerName);

            % Add back original layers.
            lgraph = lgraph.addLayers(info.RPNClassificationLayer);
            lgraph = lgraph.addLayers(info.RPNBoxRegressionLayer);
            lgraph = lgraph.addLayers(info.RPNSoftmaxLayer);

            % Connect layers.
            lgraph = lgraph.connectLayers(info.RPNBoxRegressionLayerSource, info.RPNBoxRegressionLayer.Name);
            lgraph = lgraph.connectLayers(info.RPNSoftmaxLayerSource, info.RPNSoftmaxLayer.Name);
            lgraph = lgraph.connectLayers(info.RPNClassificationLayerSource, info.RPNClassificationLayer.Name);
        end
        
        %------------------------------------------------------------------
        function [lgraph, info] = modifyForEndToEndTraining(...
                lgraph, classNames, boxMatcher, ...
                numRegionsToSample, foregroundFraction, isGeneralDatastore)
            % Modify network for end-to-end training.
                   
            info = vision.internal.cnn.RCNNLayers.fasterRCNNInfo(lgraph);
            roiPoolName = info.ROIPoolingLayerName;
           
            
            % Find the ROI pooling layer node.
            dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);
            roiPoolID = findnode(dg,roiPoolName);
            assert(~isempty(roiPoolID));
            
            % Search for all nodes starting from ROI pooling layer.
            ids = dfsearch(dg,roiPoolID);
            
            % Find classification and box regression layer.
            names = dg.Nodes(ids,:);
            
            clsIdx = iFindClassificationLayer(lgraph.Layers); 
            assert(~isempty(clsIdx) && numel(clsIdx)==1);
            clsName = lgraph.Layers(clsIdx).Name;
           
            
            info.ClassificationLayer = lgraph.Layers(clsIdx);
            rowIdx = find(strcmp(lgraph.Connections.Destination,clsName));
            
            info.ClassificationLayerSource = lgraph.Connections.Source{rowIdx}; %#ok<FNDSB>
            
            % Find box regression layer in fast r-cnn branch.
            regIdx = iFindBoxRegressionLayer(lgraph.Layers);
            assert(~isempty(regIdx));
            
            regNames = {lgraph.Layers(regIdx).Name};
            
            [regName,ia] = intersect(regNames, names.Name);
            assert(~isempty(ia));
            info.BoxRegressionLayer = lgraph.Layers(regIdx(ia));
            
            rowIdx = find(strcmp(lgraph.Connections.Destination,regName));
            info.BoxRegressionLayerSource = lgraph.Connections.Source{rowIdx}; %#ok<FNDSB>
            
            lgraph = lgraph.removeLayers(info.BoxRegressionLayer.Name);
            lgraph = lgraph.removeLayers(info.ClassificationLayer.Name);
            
            % add merge and output layers
            layerNames = iChooseUniqueLayerNames(lgraph);
            mergeLayer = iMergeLayer(numel(classNames), layerNames.mergeLayer);
            outputLayer = iOutputLayer(classNames, layerNames.outputLayer,...
                boxMatcher, numRegionsToSample, foregroundFraction, isGeneralDatastore);
            
            lgraph = lgraph.addLayers(mergeLayer);
            lgraph = lgraph.addLayers(outputLayer);
            
            % Connect sources to merge layer.
            lgraph = lgraph.connectLayers(info.ClassificationLayerSource, [mergeLayer.Name '/cls']);
            lgraph = lgraph.connectLayers(info.BoxRegressionLayerSource, [mergeLayer.Name '/reg']);
            lgraph = lgraph.connectLayers(info.ProposalLayerName, [mergeLayer.Name '/bboxes']);
            
            % Connect merge layer to output layer.
            lgraph = lgraph.connectLayers(mergeLayer.Name, outputLayer.Name);
            
            % Keep a momento of merge/output layer names.
            info.MergeLayerName = mergeLayer.Name;
            info.OutputLayerName = outputLayer.Name;
        end

        function [lgraph, info] = modifyRPNForProposalCalculations(lgraph, info, params)
            % -----------------------------------
            % Modify RPN layers for single output
            % -----------------------------------

            % Find RPN Box Regression Layer 
            layerNameIsClassName = false;
            [info.RPNBoxRegressionLayer, info.RPNBoxRegressionLayerSource] = ...
                iFindLayer(info.RPNBoxRegressionLayerName, lgraph, layerNameIsClassName);

            % Find RPN Softmax layer
            layerNameIsClassName = true;
            [info.RPNSoftmaxLayer, info.RPNSoftmaxLayerSource] = ...
                iFindLayer('nnet.cnn.layer.RPNSoftmaxLayer', lgraph, layerNameIsClassName);

            % Find RPN classification layer
            layerNameIsClassName = true;
            [info.RPNClassificationLayer, info.RPNClassificationLayerSource] = ...
                iFindLayer('nnet.cnn.layer.RPNClassificationLayer', lgraph, layerNameIsClassName);

            % Define the RPN output layer customized for RPN classification and regression losses.
            layerNames                 = iChooseUniqueLayerNames(lgraph);
            info.RPNProposalParameters = iSetupRPNOptions(params);
            classNames                 = {'Foreground', 'Background'};
            rpnOutputLayer             = iRPNOutputLayer(classNames, layerNames.rpnOutputLayer,...
                info.RPNProposalParameters);

            % Remove RPN box regression layer, rpn softmax layer and
            % rpn classification layer.
            lgraph  = lgraph.removeLayers(info.RPNBoxRegressionLayer.Name);
            lgraph  = lgraph.removeLayers(info.RPNSoftmaxLayer.Name);
            lgraph  = lgraph.removeLayers(info.RPNClassificationLayer.Name);

            % The inputs of the above removed layers are concatenated using
            % a depthConcatenation layer and connected to the RPN output layer.
            depth_1 = depthConcatenationLayer(2, 'Name', layerNames.classRegressionConcatenationLayer);

            lgraph  = lgraph.addLayers(depth_1);
            lgraph  = lgraph.addLayers(rpnOutputLayer);

            lgraph  = lgraph.connectLayers(info.ProposalInputLayers.RegressionLayerName, char(depth_1.Name + "/in1"));
            lgraph  = lgraph.connectLayers(info.ProposalInputLayers.ClassificationLayerName, char(depth_1.Name + "/in2"));
            lgraph  = lgraph.connectLayers(depth_1.Name, rpnOutputLayer.Name);

            info.RPNDepthConcatenationLayerName = depth_1.Name;
            info.RPNOutputLayerName             = rpnOutputLayer.Name;
        end
        
        function [frcnn, rpn, info] = splitFasterIntoFastAndRPN(lgraph)
            % Split Faster R-CNN layer graph into a Fast R-CNN and RPN layer
            % graph.
            
            info = vision.internal.cnn.RCNNLayers.fasterRCNNInfo(lgraph);
            roiPoolName = info.ROIPoolingLayerName;
            proposalName = info.ProposalLayerName;
            featureExtractionLayer = info.FeatureExtractionLayer;
            proposalClassificationInput = info.ProposalInputLayers.ClassificationLayerName;
            proposalRegressionInput = info.ProposalInputLayers.RegressionLayerName;
            
            % Find the ROI pooling layer node.
            dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);
            roiPoolID = findnode(dg,roiPoolName);
            assert(~isempty(roiPoolID));
            
            % Search for all nodes starting from ROI pooling layer.
            ids = dfsearch(dg,roiPoolID);
            
            % Remove all layers starting from ROI pooling layer. The
            % remainder is the RPN network.
            names = dg.Nodes.Name(ids,:);
            
            % Remove all layers starting with ROI Pooling layer.
            rpn = lgraph.removeLayers(names);
            
            % Remove proposal layer
            rpn = rpn.removeLayers(proposalName);
            
            % remove all RPN layers to produce fast r-cnn layer graph.
            rpnDG = vision.internal.cnn.RCNNLayers.digraph(rpn);
            rpnSourceID = findnode(rpnDG,featureExtractionLayer);
            ids = dfsearch(rpnDG,rpnSourceID);
            rpnLayerNames = rpnDG.Nodes.Name(ids(2:end),:);
            
            frcnn = lgraph.removeLayers(rpnLayerNames);
            
            % replace region proposal layer with roiInputLayer
            layerNames = iChooseUniqueLayerNames(lgraph);
            roiInput = roiInputLayer('Name',layerNames.ROIInput);
            frcnn = frcnn.replaceLayer(proposalName,roiInput);
            
            connections(1).Source = char(proposalClassificationInput);
            connections(1).Destination = char(proposalName + "/scores");
            connections(2).Source = char(proposalRegressionInput);
            connections(2).Destination = char(proposalName + "/boxDeltas");
            connections(3).Source = proposalName;
            connections(3).Destination = roiPoolName + "/roi";
            
            % Add information required for merging fast and rpn networks.
            info.ProposalConnections = struct2table(connections); 
            info.ProposalName = proposalName;         
            info.RPNLayerNames = rpnLayerNames;       
            info.RPNConnections = rpn.Connections;   
        end
        
        %--------------------------------------------------------------------------
        function roiIdx = findPotentialLocationOfROIPoolingLayer(analysis)
            % Return the exact location of ROI pooling layer or if a ROI Pooling layer
            % is not present, the last max pooling layer.
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            roiIdx = vision.internal.cnn.RCNNLayers.findROIPoolingLayer(externalLayers);
            
            if isempty( roiIdx )
                needsROIPoolingLayer = true;
            else
                needsROIPoolingLayer = false;
            end
            
            if needsROIPoolingLayer
                % Initialize layers so that the input size of all the layers is known.
                % This information is required to set the OutputSize of the ROI Pooling
                % layer. The OutputSize must match the size of the layer that follows
                % the ROI Pooling layer.
                roiIdx = iFindLastMaxPoolingLayer(externalLayers);
                
                if isempty(roiIdx)
                    error(message('vision:rcnn:noMaxPoolingLayer'));
                end
            end
            
        end

        %------------------------------------------------------------------
        function info = fasterRCNNInfo(lgraph)
            % Extraction R-CNN information from a layerGraph. This
            % layerGraph must be the full Faster R-CNN network.
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
            
            % Source of ROI pooling layer.
            externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
            idx = vision.internal.cnn.RCNNLayers.findROIPoolingLayer(externalLayers);
            roiPoolName = externalLayers(idx).Name;
            scaleFactor = externalLayers(idx).ScaleFactor;
            
            % Pull info from proposal layer.
            proposalIdx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RegionProposalLayer'),externalLayers);
            proposalName = externalLayers(proposalIdx).Name;
            anchorBoxes = externalLayers(proposalIdx).AnchorBoxes;
            
            proposalClassificationInput = analysis.LayerAnalyzers(proposalIdx).Inputs.Source{1};
            proposalRegressionInput = analysis.LayerAnalyzers(proposalIdx).Inputs.Source{2};
            
            % find RPN BoxRegression Output Layer
            proposalRegressionIdx = string({analysis.LayerAnalyzers.Name}) == char(proposalRegressionInput);
            outputNames = analysis.LayerAnalyzers(proposalRegressionIdx).Outputs.Destination{1};
            rpnBoxRegressionName = outputNames(~contains(outputNames, proposalName));
            
            % Find the feature extraction layer.
            featureExtractionLayer = analysis.LayerAnalyzers(idx).Inputs.Source{1};
            
            % Find all the layers that make up the feature extraction
            % network.
            featureExtractionLayers = vision.internal.cnn.RCNNLayers.featureExtractionLayerNames(lgraph,featureExtractionLayer);
            
            info.ROIPoolingLayerName   = roiPoolName;
            info.ProposalLayerName     = proposalName;
            info.ScaleFactor           = scaleFactor;
            info.AnchorBoxes           = anchorBoxes;           
            info.ProposalInputLayers.ClassificationLayerName = char(proposalClassificationInput);
            info.ProposalInputLayers.RegressionLayerName = char(proposalRegressionInput);
            info.RPNBoxRegressionLayerName = char(rpnBoxRegressionName);
            info.FeatureExtractionLayers = featureExtractionLayers;
            info.FeatureExtractionLayer = featureExtractionLayer;
        end
        
        %------------------------------------------------------------------
        function names = featureExtractionLayerNames(lgraph,layerName)
            % Find all layer names in feature extraction network where the
            % feature extraction layer is named layerName.
            dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);
            dg = flipedge(dg);
            id = findnode(dg,char(layerName));
            assert(~isempty(id));
            ids = dfsearch(dg,id);
            names = dg.Nodes.Name(ids,:);
        end
                    
        %------------------------------------------------------------------
        function lgraph = freezeBatchNorm(lgraph)                 
            idx = arrayfun(@(x)...
                isa(x,'nnet.cnn.layer.BatchNormalizationLayer'),...
                lgraph.Layers);                    
            bnLayers = lgraph.Layers(idx);
            for i = 1:numel(bnLayers)
                frozenBN = vision.internal.cnn.FrozenBatchNormalizationLayer.create(bnLayers(i));
                lgraph = lgraph.replaceLayer(frozenBN.Name,frozenBN);
            end
        end
        
        %------------------------------------------------------------------
        function lgraph = unfreezeBatchNorm(lgraph)            
            idx = arrayfun(@(x)...
                isa(x,'vision.internal.cnn.FrozenBatchNormalizationLayer'),...
                lgraph.Layers);            
            frozen = lgraph.Layers(idx);
            for i = 1:numel(frozen)       
                % A frozen batch norm layer keeps a memento of the batch
                % norm layer it replaced.
                lgraph = lgraph.replaceLayer(frozen(i).Name,frozen(i).BatchNormalizationLayer);              
            end
        end
        
        %------------------------------------------------------------------
        function freeze = freezeBatchNormHeuristic(miniBatchSize)
            % Freeze batch norm layers if minibatch size is less than 8.
            % This heuristic is based on results from:
            %
            % Wu, Yuxin, and Kaiming He. "Group normalization." arXiv preprint
            % arXiv:1803.08494 (2018).
            if miniBatchSize < 8
                freeze = true;
            else
                freeze = false;
            end
        end
    end
end

%--------------------------------------------------------------------------
function lgraph = iNetworkToLayerGraph(network)
switch class(network)
    case 'SeriesNetwork'
        lgraph = layerGraph(network.Layers);
    case {'DAGNetwork','nnet.cnn.layer.Layer'}
        lgraph = layerGraph(network);
    case 'nnet.cnn.LayerGraph'
        lgraph = network;
    otherwise
        error('unknown network type');
end
end

%--------------------------------------------------------------------------
function lgraph = iLoadModelAsLayerGraph(modelName)
net = feval(modelName);
lgraph = iNetworkToLayerGraph(net);
end

%--------------------------------------------------------------------------
function lgraph = iAddROIInputLayerAndConnectToROIPool(this,lgraph)
% Add ROI input layer and connect to ROI pooling layer.
inputName = this.LayerNames.ROIInput;
roiPoolName = this.LayerNames.ROIPooling;
roiInput = iROIInputLayer(inputName);
lgraph = lgraph.addLayers(roiInput);
lgraph = lgraph.connectLayers(inputName, string(roiPoolName) + '/roi');
end

%--------------------------------------------------------------------------
function [lgraph,fcName] = iAddBBoxRegressionBranch(this, lgraph,numClasses)
fcName =  this.LayerNames.rcnnFCBoxReg;
fc_reg = fullyConnectedLayer(4 * (numClasses), 'Name',fcName,...
    'WeightsInitializer',iNormalInitializer(),...
    'WeightLearnRateFactor', 1, ...
    'BiasLearnRateFactor', 2, ...
    'WeightL2Factor', 1,...
    'BiasL2Factor', 0);

name = this.LayerNames.rcnnBoxReg;
boxRegressionLayer = rcnnBoxRegressionLayer('Name', name);

regLayers = [
    fc_reg
    boxRegressionLayer];

lgraph = lgraph.addLayers(regLayers);
end

%--------------------------------------------------------------------------
function lgraph = iInitializeFCLayerInFastBoxRegressionBranch(this, lgraph, analysis)
layers = analysis.ExternalLayers;
idx = string({layers.Name}) == this.LayerNames.rcnnFCBoxReg;
assert(sum(idx)==1);

fcReg = layers(idx);

fcLayerInputSize = fcReg.InputSize;

% Initialize fc_reg weights (std = 0.001, bias = 0)
w = 0.001 * randn(fcReg.OutputSize, fcLayerInputSize, 'single');
fcReg.Weights = w;
fcReg.Bias = zeros([fcReg.OutputSize, 1], 'single');

% Update FC layer
name = this.LayerNames.rcnnFCBoxReg;
lgraph = lgraph.replaceLayer(name,fcReg);

end

%--------------------------------------------------------------------------
function layer = iROIInputLayer(name)
layer = roiInputLayer('Name',name);
end

%--------------------------------------------------------------------------
function newConvLayer = iNewConv2dLayerForRCNN(this, numClasses, oldConvLayer)
    name = this.LayerNames.rcnnFC;
    numFilters = numClasses + 1;
    filterSize = oldConvLayer.FilterSize;
    stride = oldConvLayer.Stride;
    paddingSize = oldConvLayer.PaddingSize;
    newConvLayer = convolution2dLayer(filterSize, numFilters,...
        'Name',name,...
        'Stride',stride,...
        'Padding', paddingSize,...
        'WeightsInitializer',iNormalInitializer());
end

%--------------------------------------------------------------------------
function newConvLayer = iNewTransposedConv2d(this, numClasses, oldConvLayer)
    name = this.LayerNames.rcnnFC;
    numFilters = numClasses + 1;
    filterSize = oldConvLayer.FilterSize;
    stride = oldConvLayer.Stride;
    cropping = oldConvLayer.Cropping;
    newConvLayer = transposedConv2dLayer(filterSize, numFilters,...
        'Name',name,...
        'Stride',stride,...
        'Cropping',cropping,...
        'WeightsInitializer',iNormalInitializer());
end

%--------------------------------------------------------------------------
function newFCLayer = iNewFullyConnectedLayerForRCNN(this, numClasses)
name = this.LayerNames.rcnnFC;
newFCLayer = fullyConnectedLayer(numClasses + 1,'Name',name,'WeightsInitializer',iNormalInitializer());
end

%--------------------------------------------------------------------------
function layer = iNewClassificationLayerForRCNN(this)
name = this.LayerNames.rcnnCls;
layer = classificationLayer('Name',name);
end

%--------------------------------------------------------------------------
function layer = iNewSoftmaxLayerForRCNN(this)
name = this.LayerNames.rcnnSoftmax;
layer = softmaxLayer('Name',name);
end

%--------------------------------------------------------------------------
function names = iDefaultLayerNames()
names.ROIInput = "roiInput";
names.ROIPooling = "roiPooling";
names.rcnnFC = "rcnnFC";
names.rcnnCls = "rcnnClassification";
names.rcnnSoftmax = "rcnnSoftmax";
names.rcnnFCBoxReg = "fcBoxDeltas";
names.rcnnBoxReg = "boxDeltas";
names.rpnConvCls = "rpnConv1x1ClsScores";
names.rpnConvReg = "rpnConv1x1BoxDeltas";
names.rpnSoftmax = "rpnSoftmax";
names.rpnCls = "rpnClassification";
names.rpnBoxReg = "rpnBoxDeltas";
names.rpnProposal = "regionProposal";
names.rpnConv3x3 = "rpnConv3x3";
names.rpnRelu = "rpnRelu";
names.mergeLayer = "fastRCNNMerge";
names.outputLayer = "fastRCNNOutput";
names.rpnOutputLayer = "rpnOutputLayer";
names.classRegressionConcatenationLayer = "clsBoxRegConcatenationLayer";
end

%--------------------------------------------------------------------------
function names = iChooseUniqueLayerNames(lgraph)
% Loop over names in lgraph and choose new layer names that do not
% conflict.
names = iDefaultLayerNames();
alreadyUsedNames = string({lgraph.Layers.Name});
fnames = fieldnames(names);
for i = 1:numel(fnames)
    inuse = alreadyUsedNames == names.(fnames{i});
    if any(inuse)
        % append _1 to make unique name.
        names.(fnames{i}) = names.(fnames{i}) + "_1";
    end
end
end

%--------------------------------------------------------------------------
function [lastPoolName, outputSize, featureExtractionLayerName, ...
    numFiltersLastConvLayer, regressionBranchSource, roiPoolDestination,...
    roiType, scaleFactor] = ...
    iFastRCNNConversionParameters(analysis)

roiIdx = vision.internal.cnn.RCNNLayers.findPotentialLocationOfROIPoolingLayer(analysis);

externalLayers = [analysis.LayerAnalyzers.ExternalLayer];
roiType = iROIType(externalLayers(roiIdx));
 
% Get size of output size of the max pooling layer. The ROI pooling
% layer's GridSize must be set to this value.
outputSize = analysis.LayerAnalyzers(roiIdx).Outputs.Size{1}(1:2);

lastPoolName = char(analysis.LayerAnalyzers(roiIdx).Name);

% The feature extraction layer. These are the features the ROI pooling
% layer pools.
featureExtractionLayerName = analysis.LayerAnalyzers(roiIdx).Inputs.Source{1};

% Number of channels in the input to ROI pooling layer.
numFiltersLastConvLayer = analysis.LayerAnalyzers(roiIdx).Inputs.Size{1}(3);

% Destination of ROI pool output.
roiPoolDestination = analysis.LayerAnalyzers(roiIdx).Outputs.Destination{1};

% Find source of last FC layer attached to classificaiton layer. This is
% also the source of the box regression branch.
lastFCLayerIndex = vision.internal.cnn.utils.findLastFullyConnectedLayer(analysis.LayerGraph);
assert(~isempty(lastFCLayerIndex));
regressionBranchSource = analysis.LayerAnalyzers(lastFCLayerIndex).Inputs.Source{1};

% Compute ROI pool scale factor.
scaleFactor = vision.internal.cnn.RCNNLayers.imageToFeatureScale(analysis,roiIdx);
end

%--------------------------------------------------------------------------
function idx = iFindImageLayer(analysis)
idx = [analysis.LayerAnalyzers.IsImageInputLayer];
end

%--------------------------------------------------------------------------
function [idx] = iFindLastMaxPoolingLayer(externalLayers)
idx = find(...
    arrayfun( @(x)isa(x,'nnet.cnn.layer.MaxPooling2DLayer')...
    || isa(x,'nnet.cnn.layer.AveragePooling2DLayer'), ...
    externalLayers), 1, 'last');
end

%--------------------------------------------------------------------------
function [idx] = iFindROIInputLayer(externalLayers)
idx = find(...
    arrayfun( @(x)isa(x,'nnet.cnn.layer.ROIInputLayer'), ...
    externalLayers), 1, 'first');
end

%--------------------------------------------------------------------------
function idx = iFindFullyConnectedLayers(externalLayers)
idx = find(...
    arrayfun( @(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'), ...
    externalLayers));
end

%--------------------------------------------------------------------------
function idx = iFindClassificationLayer(externalLayers)
idx = find(...
    arrayfun( @(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'), ...
    externalLayers));
end

%--------------------------------------------------------------------------
function idx = iFindLastClassificationLayer(externalLayers)
idx = find(...
    arrayfun( @(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'), ...
    externalLayers), 1, 'last');
end

%--------------------------------------------------------------------------
function idx = iFindLastSoftmaxLayer(externalLayers)
idx = find(...
    arrayfun( @(x)isa(x,'nnet.cnn.layer.SoftmaxLayer'), ...
    externalLayers), 1, 'last');
end

%--------------------------------------------------------------------------
function idx = iFindBoxRegressionLayer(externalLayers)
idx = find(...
    arrayfun( @(x)isa(x,'nnet.cnn.layer.RCNNBoxRegressionLayer'), ...
    externalLayers));
end

%--------------------------------------------------------------------------
function roiType = iROIType(layer)
if isa(layer,'nnet.cnn.layer.MaxPooling2DLayer') || isa(layer,'nnet.cnn.layer.ROIMaxPooling2DLayer')
    roiType = "max";
elseif isa(layer,'nnet.cnn.layer.AveragePooling2DLayer') || isa(layer,'vision.cnn.layer.ROIAveragePooling2DLayer')
    roiType = "avg";
else
    assert(false,'unknown type')
end
end

%--------------------------------------------------------------------------
function isPooling = iVerifyROIType(layer, roiMaxPoolingLayerOption, featureLayer)
isMax = isa(layer,'nnet.cnn.layer.MaxPooling2DLayer') || isa(layer,'nnet.cnn.layer.ROIMaxPooling2DLayer');
isAvg = isa(layer,'nnet.cnn.layer.AveragePooling2DLayer') || isa(layer,'vision.cnn.layer.ROIAveragePooling2DLayer');
isPooling = isMax || isAvg;
if isequal(roiMaxPoolingLayerOption, 'replace') && (~isPooling)
    error(message('vision:fasterRCNNLayers:nonPoolingReplaceAfterFeature',...
                    featureLayer));
end
end

function s = iPredefinedNetworkInfo()
% ROIScaleFactor is the output size of feature extraction layer divided by
% network image input size.
s.alexnet = struct(...
    'HasAverageImage', true, ...
    'InputSize', [227 227 3], ...
    'FeatureExtractionLayer', 'relu5', ...
    'ROIScaleFactor',[13/227 13/227]);

s.vgg16 = struct(...
    'HasAverageImage', true, ...
    'InputSize', [224 224 3], ...
    'FeatureExtractionLayer', 'relu5_3', ...
    'ROIScaleFactor',[14/224 14/224]);

s.vgg19 = struct(...
    'HasAverageImage', true, ...
    'InputSize', [224 224 3], ...
    'FeatureExtractionLayer', 'relu5_4', ...
    'ROIScaleFactor',[14/224 14/224]);

s.resnet50 = struct(...
    'HasAverageImage', true, ...
    'InputSize', [224 224 3], ...
    'FeatureExtractionLayer', 'activation_40_relu', ...
    'ROIScaleFactor',[14/224 14/224]);

s.resnet101 = struct(...
    'HasAverageImage', true, ...
    'InputSize', [224 224 3], ...
    'FeatureExtractionLayer', 'res4b22_relu', ...
    'ROIScaleFactor',[14/224 14/224]);

s.googlenet = struct(...
    'HasAverageImage', true, ...
    'InputSize', [224 224 3], ...
    'FeatureExtractionLayer', 'inception_4d-output', ...
    'ROIScaleFactor',[14/224 14/224]);

s.inceptionv3 = struct(...
    'HasAverageImage', false, ...
    'InputSize', [299 299 3], ...
    'FeatureExtractionLayer', 'mixed7', ...
    'ROIScaleFactor',[17/299 17/299]);

s.inceptionresnetv2 = struct(...
    'HasAverageImage', false, ...
    'InputSize', [299 299 3], ...
    'FeatureExtractionLayer', 'block17_20_ac', ...
    'ROIScaleFactor',[17/299 17/299]);

s.squeezenet = struct(...
    'HasAverageImage', true, ...
    'InputSize', [227 227 3], ...
    'FeatureExtractionLayer', 'fire5-concat', ...
    'ROIScaleFactor',[28/227 28/227]);

s.mobilenetv2 = struct(...
    'HasAverageImage', false, ...
    'InputSize', [224 224 3], ...
    'FeatureExtractionLayer', 'block_13_expand_relu', ...
    'ROIScaleFactor',[14/224 14/224]);

s.resnet18 = struct(...
    'HasAverageImage', false, ...
    'InputSize', [224 224 3], ...
    'FeatureExtractionLayer', 'res4b_relu', ...
    'ROIScaleFactor',[14/224 14/224]);
end

%--------------------------------------------------------------------------
function externalLayer = iMergeLayer(numClasses,name)
internalLayer = vision.internal.cnn.layer.FastRCNNMergeInternalLayer(name,numClasses);
externalLayer = vision.internal.cnn.layer.FastRCNNMergeLayer(internalLayer);
end

%--------------------------------------------------------------------------
function externalLayer = iOutputLayer(classNames,name,iouRange,numRegionsToSample,foregroundFraction, isGeneralDatastore)
internalLayer = vision.internal.cnn.layer.FastRCNNOutputInternalLayer(...
    name,classNames,iouRange,numRegionsToSample,foregroundFraction, isGeneralDatastore);
externalLayer = vision.internal.cnn.layer.FastRCNNOutputLayer(internalLayer);
end

%--------------------------------------------------------------------------
function [layer, source] = iFindLayer(layerName, lgraph, layerNameIsClassName)
if layerNameIsClassName
    clsIdx = find(...
        arrayfun( @(x)isa(x,layerName), ...
        lgraph.Layers));
else
    clsIdx = find(string({lgraph.Layers.Name}) == layerName);
end
assert(~isempty(clsIdx) && numel(clsIdx)==1);
clsName = lgraph.Layers(clsIdx).Name;

layer   = lgraph.Layers(clsIdx);
rowIdx  = find(strcmp(lgraph.Connections.Destination,clsName));
source  = lgraph.Connections.Source{rowIdx}; %#ok<FNDSB>
end

%--------------------------------------------------------------------------
function externalLayer = iRPNOutputLayer(classNames,name,options)
internalLayer = vision.internal.cnn.layer.RPNOutputInternalLayer(...
                    name,classNames,options);
externalLayer = vision.internal.cnn.layer.RPNOutputLayer(internalLayer);
end

%--------------------------------------------------------------------------
function w = iNormalInitializer()
w = @(sz)randn(sz)*0.01;
end

function options = iSetupRPNOptions(params)
% Set the default background label.
options.BackgroundLabel = params.BackgroundLabel;

% Create class names from ground truth data. For object
% detection add a "Background" class. The order of the class
% labels is important because they are used to generate
% response data for training.
options.ClassNames = {'Foreground' options.BackgroundLabel};

% Create categorical to represent categories learnt by RPN. The
% 3rd value is intentially <undefined> to indicate a response
% should be ignored because there is neither a foreground or
% background sample at that location.
options.CategoricalLookup            = reshape(categorical([1 2 3],[1 2],options.ClassNames),[],1);

options.PositiveOverlapRange         = params.PositiveOverlapRange(1,:);
options.NegativeOverlapRange         = params.NegativeOverlapRange(1,:);

options.RandomSelector               = params.RandomSelector;

options.StandardizeRegressionTargets = isempty(params.InternalOptions.BoxRegressionMeanStd);

options.BBoxRegressionNormalization  = params.InternalOptions.SmoothL1Normalization;
options.PercentageOfPositiveSamples  = params.InternalOptions.RPNForegroundFraction;
options.MiniBatchPadWithNegatives    = params.InternalOptions.MiniBatchPadWithNegatives;
options.RPNROIPerImage               = params.NumRegionsToSample(1);

% BatchingFcn Define custom batching function for a cell of M-by-4
% ROIs. This overrides the default batching function which attempts
% to cat ROI along the 4-th dimension. Each struct field should
% correspond to one of the table variable names.
options.BatchingFcn       = struct('RegressionResponse', @iRegresssionResponseBatchFcn);

options.BoxRegressionMean = params.ImageInfo.BoxRegressionMean;
options.BoxRegressionStd  = params.ImageInfo.BoxRegressionStd;

options.AnchorBoxes       = params.AnchorBoxes;
options.NumAnchors        = params.NumAnchors;
options.ScaleFactor       = params.ScaleFactor;
end

%--------------------------------------------------------------------------
function batch = iRegresssionResponseBatchFcn(TColumn)
% function to batch a column of regression response data.

t1    = cellfun(@(x)x{1},TColumn,'UniformOutput',false);
t2    = cellfun(@(x)x{2},TColumn,'UniformOutput',false);

batch = { cat(4,t1{:}) cat(4,t2{:}) };
end

%-----------------------------------------------------------------------
function iValidateFeatureLayerExistence(layers,featureLayer)
% Validate featureLayer exists in the network or not.
    if ~isempty(featureLayer)
        featureLayerSizeIdx = arrayfun(@(x) x.Name == ...
                string(featureLayer),layers);
        if sum(featureLayerSizeIdx)==0
            error(message...
                    ('vision:fasterRCNNLayers:layerDoesNotExist',...
                    featureLayer));
        end
    end
end

%--------------------------------------------------------------------------
function [lgraph,analysis] = iReplaceImageInputLayer(lgraph,imageSize)
% Replace input size in image input layer.
    idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
        lgraph.Layers);
    imLayer = lgraph.Layers(idx);

    imageInput = vision.internal.cnn.utils.updateImageLayerInputSize(...
        imLayer, imageSize);
    
    lgraph = replaceLayer(lgraph,imLayer.Name,imageInput);

    % Validate that replaced layer does not cause any issue.
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
    analysis.applyConstraints();
    if ~analysis.LayerAnalyzers(2,1).IsLayerInputValid
        error(message('vision:fasterRCNNLayers:invalidLayerSize', ...
            mat2str(analysis.LayerAnalyzers(2,1).Inputs.Size{1}),...
            mat2str([analysis.LayerAnalyzers(2,1).Inputs.Size{1}(1:2),...
            analysis.LayerAnalyzers(2,1).Learnables.Size{1}(3)])));
    end
end

%--------------------------------------------------------------------------
function conv2dLayer = iFullyConnected2Conv2d(fc, filterSize, numChannels)
    name       = fc.Name;
    numFilters = fc.OutputSize;

    weightsInitializer    = fc.WeightsInitializer;
    biasInitializer       = fc.BiasInitializer;
    weightLearnRateFactor = fc.WeightLearnRateFactor;
    biasLearnRateFactor   = fc.BiasLearnRateFactor;
    weightL2Factor        = fc.WeightL2Factor;
    biasL2Factor          = fc.BiasL2Factor;

    bias = reshape(fc.Bias, [1, 1, fc.OutputSize]);
    weights = reshape(fc.Weights, [filterSize,numChannels,fc.OutputSize]);
    conv2dLayer = convolution2dLayer(filterSize, numFilters, ...
        'Name', name, ...
        'WeightsInitializer', weightsInitializer, ...
        'BiasInitializer', biasInitializer, ...
        'WeightLearnRateFactor', weightLearnRateFactor, ...
        'BiasLearnRateFactor', biasLearnRateFactor, ...
        'WeightL2Factor', weightL2Factor, ...
        'BiasL2Factor', biasL2Factor, ...
        'Weights', weights,...
        'Bias', bias);
end

%------------------------------------------------------------------
function newPoolLayer = iNewAvgPoolLayer(oldPoolLayer, inputSize)
    name = oldPoolLayer.Name;
    paddingSize = oldPoolLayer.PaddingSize;
    stride = oldPoolLayer.Stride;
    newPoolLayer = averagePooling2dLayer(inputSize(1), ...
        'Name', name, ...
        'Padding', paddingSize, ...
        'Stride', stride);
end

%------------------------------------------------------------------
function tf = iAvgPoolLayerNeedsReplace(oldPoolLayer, inputSize)
    % GlobalAveragePooling2DLayer does not have a PoolSize property
    tf = ~isa(oldPoolLayer, 'nnet.cnn.layer.GlobalAveragePooling2DLayer')...
        && ~isequal(inputSize,oldPoolLayer.PoolSize);
end
