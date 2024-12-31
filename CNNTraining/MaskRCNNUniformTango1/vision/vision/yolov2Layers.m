function lgraph = yolov2Layers(varargin)

% Copyright 2018-2023 The MathWorks, Inc.

    vision.internal.requiresNeuralToolbox(mfilename);
    [lgraph,params] = parseInputs(varargin{:});
    
    % Replace input size of image input layer.
    lgraph = iReplaceImageInputLayer(lgraph,params.ImageSize);
        
    % Remove all the layers after featureLayer.
    lgraph = iRemoveLayers(lgraph,params.FeatureLayer);
    
    % Verfiy Fully Connected layer does not exist in the network.
    iVerifyFullyConnectedExistence(lgraph);
    
    % Verfiy Global average pooling layer does not exist in the network.
    iVerifyGlobalAvgPoolExistence(lgraph);    
    
    % Choose unique names for yolo v2 specific layers.
    names = iChooseUniqueLayerNames(lgraph);
    
    % Add yolo v2 detection sub-network to featureLayer.
    lgraph = iAddDetectionSubNetwork(lgraph,params,names);
    
    % Validate application of ReorgLayer using networkAnalyzer and create
    % YOLO v2 network.
    if iReorgLayerRequested(params.ReorgLayerSource)
        
        % Calculate reorg layer stride and validate depth concatenation layer
        % inputs.
        reorgStride = iCalculateReorgStrideAndValidate(lgraph, params);      
        
        % Add reorg layer and depth concatenation layer to lgraph.
        lgraph = iAddReorgAndDepthConcat(lgraph,params,reorgStride,names);
    end
    
    % Validate created YOLO v2 layer graph using network analyzer.
    iValidateYOLOv2Network(lgraph);  
end

%--------------------------------------------------------------------------
function [lgraph,params] = parseInputs(varargin)
    % Parse and validate the input parameters.
    p = inputParser;
    p.addRequired('imageSize', @iCheckImageSize);
    p.addRequired('numClasses', @iCheckNumClasses);
    p.addRequired('anchorBoxes', @iCheckAnchorBoxes);
    p.addRequired('network', @iCheckNetwork);
    p.addRequired('featureLayer', @iCheckFeatureLayer);
    p.addParameter('reorgLayerSource', '', @iCheckRorgLayerSource);
    parse(p, varargin{:});
    userInput = p.Results;
    params.ImageSize        = double(userInput.imageSize);
    params.NumClasses       = double(userInput.numClasses);
    params.AnchorBoxes      = double(userInput.anchorBoxes);
    params.Network          = userInput.network;
    params.FeatureLayer     = char(userInput.featureLayer);
    params.ReorgLayerSource = char(userInput.reorgLayerSource);  
    
    % Validate that anchorBoxes are smaller than imageSize.
    iValidateAnchorBoxes(params);
    
    % Create layer graph from the user provided network.
    lgraph = iLayerGraph(params.Network);
    
    % Validate FeatureLayer exists in the network or not. 
    iValidateFeatureLayerExistence(lgraph,params.FeatureLayer);
    
    % Validate ReorgLayerSource exists in the network or not. 
    iValidateReorgLayerSourceExistence(lgraph,params.ReorgLayerSource);    
end

%--------------------------------------------------------------------------
function iCheckImageSize(x)
    validateattributes(x, {'numeric'}, ...
        {'vector', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
        mfilename, 'imageSize');
end
%--------------------------------------------------------------------------
function iCheckNumClasses(x)
    validateattributes(x, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
        mfilename, 'numClasses');
end

%--------------------------------------------------------------------------
function iCheckAnchorBoxes(x)
    validateattributes(x, {'numeric'}, ...
        {'2d','ncols',2,'ndims',2,'nonempty','nonsparse',...
        'real','finite','nonnan','positive','integer'},mfilename,...
        'anchorBoxes');
end

%--------------------------------------------------------------------------
function iCheckNetwork(network)
    if ~(isa(network,'nnet.cnn.LayerGraph') || isa(network,'SeriesNetwork') || isa(network,'DAGNetwork'))        
        error(message('vision:yolo:NotaValidLayerGraphSeriesNetworkDAGNetwork'));
     
    end
end

%--------------------------------------------------------------------------
function iCheckFeatureLayer(x)
    validateattributes(x, {'char', 'string'}, {'nonempty'}, mfilename, 'featureLayer');
    isCharRowVector = ischar(x) && isrow(x);
    isScalarString = (isstring(x) && isscalar(x));
    tf = isCharRowVector || isScalarString;
    if ~(tf)
        error(message('vision:yolo:MatrixCharacterOrVectorMatrixString'));
    end
        
end

%--------------------------------------------------------------------------
function iCheckRorgLayerSource(x)
    validateattributes(x, {'row', 'char', 'string'},{}, mfilename, 'reorgLayerSource');
    isCharRowVector = ischar(x) && isrow(x);
    isCharEmpty = isempty(x);
    isScalarString = (isstring(x) && isscalar(x));
    tf = isCharRowVector || isScalarString ||isCharEmpty;
    if ~(tf)
        error(message('vision:yolo:MatrixCharacterOrVectorMatrixString'));
    end
end

%--------------------------------------------------------------------------
function iValidateAnchorBoxes(params)
    imageSize = params.ImageSize;
    anchorBoxes = params.AnchorBoxes;
    for k=1:size(anchorBoxes,1)
        if any(anchorBoxes(k,:)>imageSize(:,1:2))
            error(message('vision:yolo:SmallerAnchorthanImage',...
                    mat2str(anchorBoxes(k,:)),mat2str(imageSize(:,1:2))));
        end
    end    
end
%--------------------------------------------------------------------------
function lgraph = iLayerGraph(x)
    % Get layer graph from input network.
    if isa(x,'DAGNetwork')

        lgraph = layerGraph(x);
    elseif isa(x,'SeriesNetwork')
        
        lgraph = layerGraph(x.Layers);
    else
        
        lgraph = x;
    end
end

%--------------------------------------------------------------------------
function iValidateReorgLayerSourceExistence(lgraph,reorgLayerSource)
    % Validate reorgLayerSource exists in the network or not. 
    if ~isempty(reorgLayerSource)
        analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
        reorgInputSizeIdx = arrayfun(@(x) x.Name == ...
                reorgLayerSource,analysis.LayerAnalyzers);        
        if sum(reorgInputSizeIdx)==0
            error(message...
                    ('vision:yolo:LayerDoesNotExist',...
                    reorgLayerSource));
        end
    end
end

%--------------------------------------------------------------------------
function iVerifyFullyConnectedExistence(lgraph)
% YOLOv2 network is based on Convolution Layers and should not
% contain any fullyConnected Layers.
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.FullyConnectedLayer'),...
    lgraph.Layers);    
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyFCLayer"));
end
end

%--------------------------------------------------------------------------
function iVerifyGlobalAvgPoolExistence(lgraph)
% YOLOv2 network should not contain any global average pooling layer as it
% downsamples input feature map to size of [1,1].
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.GlobalAveragePooling2DLayer')||isa(x,'nnet.cnn.layer.GlobalMaxPooling2DLayer'),...
    lgraph.Layers);    
if sum(idx) ~= 0
    error(message("vision:yolo:mustNotHaveAnyGlobalPoolingLayer"));
end
end

%--------------------------------------------------------------------------
function iValidateFeatureLayerExistence(lgraph,featureLayer)

analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
% Validate featureLayer exists in the network or not.
if ~isempty(featureLayer)
    featureLayerSizeIdx = arrayfun(@(x) x.Name == ...
        featureLayer,analysis.LayerAnalyzers);
    if sum(featureLayerSizeIdx)==0
        error(message...
            ('vision:yolo:LayerDoesNotExist',...
            featureLayer));
    end
end
end

%--------------------------------------------------------------------------
function lgraph = iReplaceImageInputLayer(lgraph,imageSize)
% Replace input size in image input layer.
    idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),...
        lgraph.Layers);
    imageInputIdx = find(idx,1,'first');
    imageInput =  vision.internal.cnn.utils.updateImageLayerInputSize(...
        lgraph.Layers(imageInputIdx), imageSize);
    lgraph = replaceLayer(lgraph,lgraph.Layers(imageInputIdx).Name,...
        imageInput);
    
    % Validate that replaced layer does not cause any issue.
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
    analysis.applyConstraints();
    if ~analysis.LayerAnalyzers(2,1).IsLayerInputValid
        error(message('vision:yolo:InvalidLayerSize', ... 
             mat2str(analysis.LayerAnalyzers(2,1).Inputs.Size{1}),...
             mat2str([analysis.LayerAnalyzers(2,1).Inputs.Size{1}(1:2),...
             analysis.LayerAnalyzers(2,1).Learnables.Size{1}(3)])));
    end
end

%--------------------------------------------------------------------------
function lgraph = iRemoveLayers(lgraph,featureLayer)
    % Remove all the layers after featureLayer.
    dg = vision.internal.cnn.RCNNLayers.digraph(lgraph); 
    
    % Find the feature extraction node.
    id = findnode(dg,char(featureLayer));
    
    % Search for all nodes starting from the feature extraction
    % layer.
    if ~(sum(id)==0)
        ids = dfsearch(dg,id);
        names = dg.Nodes.Name(ids,:);              
        lgraph = removeLayers(lgraph, names(2:end)); % exclude feature extraction layer which is first name.                
    end                
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
function names = iDefaultLayerNames()
   % Default names of yolo detection subnetwork.
   names.yolov2Conv1 = "yolov2Conv1";
   names.yolov2Batch1 = "yolov2Batch1";
   names.yolov2Relu1 = "yolov2Relu1";
   names.yolov2Conv2 = "yolov2Conv2";
   names.yolov2Batch2 = "yolov2Batch2";
   names.yolov2Relu2 = "yolov2Relu2";
   names.yolov2ClassConv = "yolov2ClassConv";
   names.yolov2OutputLayer = "yolov2OutputLayer";
   names.spaceToDepth = "spaceToDepth";
   names.reorgConcat = "reorgConcat";
   names.yolov2Transform = "yolov2Transform";
end

%--------------------------------------------------------------------------
function lgraph = iAddDetectionSubNetwork(lgraph,params,names)
   % Create yolo v2 detection subnetwork and add after featureLayer.

    % Select featureLayer output size filters as YOLO v2 detection 
    % sub-network convolution layer filters.    
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
    featureLayerIdx = arrayfun(@(x) x.Name == ...
                params.FeatureLayer,analysis.LayerAnalyzers); 
    
    % Verify that YOLOv2 featureLayer output size is greater than [1,1].        
    activationSize = analysis.LayerAnalyzers(featureLayerIdx).Inputs.Size{1,1};
    if (any(activationSize(1:2) < 2))
        error(message("vision:yolo:mustHaveValidFinalActivationsSize"));  
    end
            
    outFilters = analysis.LayerAnalyzers(featureLayerIdx).Outputs.Size{1}(3);
            
    % Create new layers specific to YOLO v2.
    yolov2Conv1 = convolution2dLayer(3,outFilters,'Name',names.yolov2Conv1,...
                                    'Padding', 'same',...
                                    'WeightsInitializer',iNormalWeightInit());
    yolov2Conv1.Bias = zeros(1,1,outFilters);
    yolov2Batch1 = batchNormalizationLayer('Name',names.yolov2Batch1);
    yolov2Relu1 = reluLayer('Name',names.yolov2Relu1);

    yolov2Conv2 = convolution2dLayer(3,outFilters,'Name',names.yolov2Conv2,...
                                    'Padding', 'same',...
                                    'WeightsInitializer',iNormalWeightInit());
    yolov2Conv2.Bias = zeros(1,1,outFilters);
    yolov2Batch2 = batchNormalizationLayer('Name',names.yolov2Batch2);
    yolov2Relu2 = reluLayer('Name',names.yolov2Relu2);

    % YOLO v2 predicts Box co-ordinate (x,y,w,h), objectness score for 
    % every anchor box.
    yolov2Predictions = 5;
    numFilters = size(params.AnchorBoxes,1)*(params.NumClasses + yolov2Predictions);
    yolov2ClassConv = convolution2dLayer(1,numFilters,'Name',...
                                        names.yolov2ClassConv,...
                                        'WeightsInitializer', iNormalWeightInit());
                                    
                                    
    numAnchors = size(params.AnchorBoxes,1);                                
    yolov2Transform = yolov2TransformLayer(numAnchors,'Name',names.yolov2Transform);

    yolov2Output = yolov2OutputLayer(params.AnchorBoxes,'Name',...
                                    names.yolov2OutputLayer);                               
    yolov2Layers = [yolov2Conv1;yolov2Batch1;yolov2Relu1;...
        yolov2Conv2;yolov2Batch2;yolov2Relu2;...
        yolov2ClassConv;yolov2Transform;yolov2Output];
    lgraph = addLayers(lgraph,yolov2Layers);
    lgraph = connectLayers(lgraph,params.FeatureLayer,names.yolov2Conv1);

end

%--------------------------------------------------------------------------
function flag = iReorgLayerRequested(reorgLayerSource)
    % Check if reorg layer is present in the network or not.
    if ~isempty(reorgLayerSource)
        flag = 1;
    else
        flag = 0;
    end
end

%--------------------------------------------------------------------------
function reorgStride = iCalculateReorgStrideAndValidate(lgraph, params)
% Calculate reorg layer stride and validate depth-concatenation layer
% inputs using network analyzer.
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
    reorgInputSizeIdx = arrayfun(@(x) x.Name == ...
         params.ReorgLayerSource,analysis.LayerAnalyzers);
    reorgOutSizeIdx = arrayfun(@(x) x.Name == ...
    params.FeatureLayer,analysis.LayerAnalyzers);        
    reorgInputSize = analysis.LayerAnalyzers...
         (reorgInputSizeIdx).Outputs.Size{1};
    reorgOutSize   = analysis.LayerAnalyzers...
         (reorgOutSizeIdx).Outputs.Size{1};
    reorgStride =  floor(reorgInputSize(1:2)./reorgOutSize(1:2));
    actual = floor(reorgInputSize(1:2)./reorgStride);
    expected = reorgOutSize(1:2);
    if ~(actual == expected)
       error(message('vision:yolo:DepthConcatInputMisMatch', ... 
             mat2str(expected), mat2str(params.ImageSize)));
    end
end

%--------------------------------------------------------------------------
function lgraph = iAddReorgAndDepthConcat(lgraph,params,reorgStride,names)
    % Add reorg and depth concatenation layer to the lgraph.
    spaceToDepth = spaceToDepthLayer(reorgStride,'Name',names.spaceToDepth);
    depthConcat = depthConcatenationLayer(2,'Name',names.reorgConcat);
    lgraph = addLayers(lgraph,depthConcat);
    lgraph = addLayers(lgraph,spaceToDepth);
    lgraph = connectLayers(lgraph,names.yolov2Relu1,strcat(names.reorgConcat,'/in1'));
    lgraph = connectLayers(lgraph,params.ReorgLayerSource,names.spaceToDepth);
    lgraph = connectLayers(lgraph,names.spaceToDepth,strcat(names.reorgConcat,'/in2'));
    lgraph = disconnectLayers(lgraph,names.yolov2Relu1,names.yolov2Conv2);
    lgraph = connectLayers(lgraph,names.reorgConcat,names.yolov2Conv2);
end

%--------------------------------------------------------------------------
function iValidateYOLOv2Network(lgraph)
    % Validate created YOLO v2 layer graph using network analyzer.
    analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
    analysis.applyConstraints();
    try
        analysis.throwIssuesIfAny();
    catch ME
        throwAsCaller(ME);
    end
end

%--------------------------------------------------------------------------
function w = iNormalWeightInit()
w = @(sz)randn(sz)*0.01;
end
