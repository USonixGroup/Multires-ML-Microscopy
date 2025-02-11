function params = ssdNetworkCreation(params)
% ssdNetworkCreation Create a SSD object detection network.
% Assign sorted anchorBoxes and detection network source. Calculate
% default anchor boxes if anchorBoxes are empty.
% If the detection networkSource is empty, extract it and assign it to
% params.

% Copyright 2021-2024 The MathWorks, Inc.

% Validate input dlnetwork and add prediction layers if detection
% network sources have been provided.
dlnet = iUpdateFirstConvChannelsAndInputLayer(params.Network,params.InputSize,params.ImgInputIdx);
dlNetAnalysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(dlnet);
numClasses = size(params.ClassNames,1);

if isempty(params.DetectionNetworkSource)
    extractedDetectionNetworkSource = params.ExtractedDetectionNetworkSource;
    [params.DetectionNetworkSource,params.DetectionNetworkSourceTable] = iSortDetectionNetworkSource( ...
        dlNetAnalysis,extractedDetectionNetworkSource);
    if ~(params.QuantizedFlag)
        if isa(dlnet,'dlnetwork')
            ssdNetwork = dlnet;
        else
            % Support backward compatibility where input to
            % trainSSDObjectDetector is lgrpah.
            ssdNetwork = dlnetwork(dlnet,Initialize=true);
        end
        ssdNetwork = iUpdateAndValidatedlnetwork(ssdNetwork,dlNetAnalysis,params,extractedDetectionNetworkSource,numClasses);
        params.Network = ssdObjectDetector.convertToDLNetwork(params,ssdNetwork,params.RemovedLayers);
    end
else
    iVerifyLayersExist(dlnet, params.DetectionNetworkSource);
    [params.DetectionNetworkSource,params.DetectionNetworkSourceTable] = iSortDetectionNetworkSource( ...
        dlNetAnalysis,params.DetectionNetworkSource);
    % Remove all layers after the last predictorBranchName.
    [dlnet,~] = iRemoveLayers(dlnet, params.DetectionNetworkSource(end));

    % Verify that dlnetwork should not have any outputLayer other then
    % predictorBranchNames after removing all layers after the last
    % predictorBranchName.
    if ~isa(dlnet,'dlnetwork')
        dlnet = dlnetwork(dlnet);
    end
    extraOutputLayers = setdiff(dlnet.OutputNames,cellstr(params.DetectionNetworkSource));
    if ~isempty(extraOutputLayers)
        error(message('vision:ssd:invalidSSDNetwork'));
    end

    % Create predictor layers.
    weightsInitializerValue = 'glorot';
    biasInitializerValue = 'zeros';

    ssdDlnet = iCreateSSDNetwork(dlnet, params.DetectionNetworkSource, ...
        weightsInitializerValue, biasInitializerValue, ...
        numClasses, ...
        params.AnchorBoxes, params.ClassNames);
    params.Network = ssdDlnet;
end
end

%------------------------------------------------------------------------------------------------------------------------------------------------------
function [sortedDetectionNetworkSource,sortedDetectionNetworkSourceTable] = iSortDetectionNetworkSource( ...
    dagNetAnalysis,detectionNetworkSource)
% Extract the feature map index and size based on given detection
% network source Names. It sort the detection network source and
% returns the sorted detection network source name and name,index and
% feature map size table.
numDetectionNetworkSource = numel(detectionNetworkSource);
detectionNetworkSourceTable = cell2table(cell(numDetectionNetworkSource,3),'VariableNames', ...
    {'DetectionNetworkSourceName','DetectionNetworkSourceIndex','FeatureMapSize'});

for idx = 1: numDetectionNetworkSource
    detectionNetworkSourceLayerIdx_Index = find(strcmp({dagNetAnalysis.ExternalLayers.Name}, detectionNetworkSource{idx}));
    detectionNetworkSourceLayerIdx_Name = dagNetAnalysis.ExternalLayers(detectionNetworkSourceLayerIdx_Index).Name;
    detectionNetworkSourceLayerIdx_FeatureMapSize = dagNetAnalysis.LayerAnalyzers(detectionNetworkSourceLayerIdx_Index).Outputs.Size{1};
    detectionNetworkSourceTable(idx,:) = cell2table({detectionNetworkSourceLayerIdx_Name,{detectionNetworkSourceLayerIdx_Index}, ...
        {detectionNetworkSourceLayerIdx_FeatureMapSize}},'VariableNames', {'DetectionNetworkSourceName','DetectionNetworkSourceIndex','FeatureMapSize'});
end
featureMapSizes = cell2mat(detectionNetworkSourceTable.FeatureMapSize(:));
featureMapSizes = sum(featureMapSizes(:,1:2),2);
[~,featureMapOrder] = sort(featureMapSizes,'descend');
sortedDetectionNetworkSourceTable = detectionNetworkSourceTable;
for j = 1:numDetectionNetworkSource
    sortedDetectionNetworkSourceTable(j,:) = detectionNetworkSourceTable(featureMapOrder(j),:);
end
sortedDetectionNetworkSource = sortedDetectionNetworkSourceTable.DetectionNetworkSourceName;
end

%--------------------------------------------------------------------------
function ssdDlnet = iCreateSSDNetwork(ssdDlnet, layersToConnect, ...
    weightsInitializerValue, biasInitializerValue, ...
    numClasses, ...
    anchorBoxes, ~)

% Create and add predictor layers, and connect to the right location
% in the layer graph.
numClassesPlusBackground = numClasses + 1;
predictorLayerStruct = iCreatePredictorLayers(...
    layersToConnect, ...
    weightsInitializerValue, biasInitializerValue, ...
    numClassesPlusBackground, ...
    anchorBoxes);
numFeaturePredictors = numel(predictorLayerStruct);

for idx = 1:numFeaturePredictors
    numBranches = size(predictorLayerStruct(idx).Layers, 1);
    for lidx = 1:numBranches
        ssdDlnet = addLayers(ssdDlnet, predictorLayerStruct(idx).Layers(lidx));
    end

    ssdDlnet = connectLayers(ssdDlnet, predictorLayerStruct(idx).ConnectTo, ...
        predictorLayerStruct(idx).Layers(1).Name);
    ssdDlnet = connectLayers(ssdDlnet, predictorLayerStruct(idx).ConnectTo, ...
        predictorLayerStruct(idx).Layers(2).Name);
end
mLayer = ssdMergeLayer(numClassesPlusBackground, numFeaturePredictors, 'Name', 'confmerge');
ssdDlnet = addLayers(ssdDlnet, mLayer);

mLayer = ssdMergeLayer(4, numFeaturePredictors, 'Name', 'locmerge');
ssdDlnet = addLayers(ssdDlnet, mLayer);

for idx = 1:numFeaturePredictors
    ssdDlnet = connectLayers(ssdDlnet, predictorLayerStruct(idx).Layers(1).Name, "confmerge/in" + num2str(idx));
    ssdDlnet = connectLayers(ssdDlnet, predictorLayerStruct(idx).Layers(2).Name, "locmerge/in" + num2str(idx));
end

% Add softmax layer to the classification head.
softmaxLayerName = "anchorBoxSoftmax";
abSoftmaxLayer = softmaxLayer('Name', softmaxLayerName);
ssdDlnet = addLayers(ssdDlnet, abSoftmaxLayer);
ssdDlnet = connectLayers(ssdDlnet,'confmerge', softmaxLayerName);
end

%--------------------------------------------------------------------------
function [inpNet,imageInputIdx] = iUpdateFirstConvChannelsAndInputLayer(inpNet,imageSize,imageInputIdx)
% This function update the channels of first conv layer if InputSize
% channel does not match with channels of first conv layer. It also update
% the imageInputLayer.

numChannel = imageSize(3);

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution2DLayer'),...
    inpNet.Layers);
convIdx = find(idx,1,'first');
if ~isempty(convIdx)
    numFirstConvLayerChannels = inpNet.Layers(convIdx,1).NumChannels;
else
    error(message('vision:ssd:networkMustHaveConvLayers'));
end

% If number of channels in imageSize is not equal to the channel count of
%  first convolutional layer. Update the channel count of first conv
%  layer and use values of properties as it is.Pyramid pooling concept
%  has been used for concatenating extra channel. Each extra channel is
%  mean of original (initial) channels of conv layer
%
%  Zhao, Hengshuang, et al. "Pyramid Scene Parsing Network." 2017 IEEE
% Conference on Computer Vision and Pattern Recognition (CVPR). IEEE, 2017.
if (~strcmp(numFirstConvLayerChannels,'auto'))
    if numFirstConvLayerChannels~=numChannel
        firstConvLayer = inpNet.Layers(convIdx,1);
        firstConvLayerWeights = firstConvLayer.Weights;
        meanChannelWeights = reshape(mean(firstConvLayerWeights,3),size(firstConvLayerWeights(:,:,1,:)));
        if numChannel>numFirstConvLayerChannels
            extraChanels = abs(numChannel-numFirstConvLayerChannels);
            extraChannelWeights = repmat(meanChannelWeights,1,1,extraChanels);
            updatedConvLayerWeights = cat(3,firstConvLayerWeights,extraChannelWeights);
        else
            updatedConvLayerWeights = repmat(meanChannelWeights,1,1,numChannel);
        end
        updatedConvLayer = convolution2dLayer(firstConvLayer.FilterSize, firstConvLayer.NumFilters, 'NumChannels', numChannel, ...
            'Stride',firstConvLayer.Stride,...
            'Padding',firstConvLayer.PaddingSize , ...
            'PaddingValue',firstConvLayer.PaddingValue,...
            'DilationFactor', firstConvLayer.DilationFactor, ...
            'Weights',updatedConvLayerWeights,...
            'Bias',firstConvLayer.Bias,...
            'WeightL2Factor',firstConvLayer.WeightL2Factor,...
            'BiasL2Factor',firstConvLayer.BiasL2Factor,...
            'WeightLearnRateFactor',firstConvLayer.WeightLearnRateFactor,...
            'BiasLearnRateFactor',firstConvLayer.BiasLearnRateFactor,...
            'Name', firstConvLayer.Name, ...
            'WeightsInitializer', firstConvLayer.WeightsInitializer, ...
            'BiasInitializer', firstConvLayer.BiasInitializer);
        inpNet = replaceLayer(inpNet,inpNet.Layers(convIdx).Name,...
            updatedConvLayer);
    end
end

if ~isempty(imageInputIdx)
    inputLayer = inpNet.Layers(imageInputIdx,1);
    if ~isequal(inputLayer.InputSize,imageSize)
        inputLayerChannel = inputLayer.InputSize(3);
        extraChannels = abs(numChannel - inputLayerChannel);
        % Update imageInputLayer properties value to incorporate,
        % detector's training imageSize.
        updatedMean = iUpdateImageTnputLayerStats(inputLayer.Mean,imageSize);
        updatedStandardDeviation = iUpdateImageTnputLayerStats( ...
            inputLayer.StandardDeviation,imageSize);
        updatedMin = iUpdateImageTnputLayerStats(inputLayer.Min,imageSize);
        updatedMax = iUpdateImageTnputLayerStats(inputLayer.Max,imageSize);

        if numChannel>inputLayerChannel
            if ~isempty(updatedMean) && numel(updatedMean)~=1
                updatedMean = iUpdateChannelProperties( ...
                    updatedMean,extraChannels);
            end
            if ~isempty(updatedStandardDeviation) && numel(updatedStandardDeviation)~=1
                updatedStandardDeviation = iUpdateChannelProperties( ...
                    updatedStandardDeviation,extraChannels);
            end
            if ~isempty(updatedMin) && numel(updatedMin)~=1
                updatedMin = iUpdateChannelProperties( ...
                    updatedMin,extraChannels);
            end
            if ~isempty(updatedMax) && numel(updatedMax)~=1
                updatedMax = iUpdateChannelProperties( ...
                    updatedMax,extraChannels);
            end
        elseif numChannel< inputLayerChannel
            if ~isempty(updatedMean)  && numel(updatedMean)~=1
                updatedMean = repmat(mean(updatedMean,3),1,1,numChannel);
            end
            if ~isempty(updatedStandardDeviation) && numel(updatedStandardDeviation)~=1
                updatedStandardDeviation = repmat( ...
                    mean(updatedStandardDeviation,3),1,1,numChannel);
            end
            if ~isempty(updatedMin)  && numel(updatedMin)~=1
                updatedMin = repmat(mean(updatedMin,3),1,1,numChannel);
            end
            if ~isempty(updatedMax) && numel(updatedMax)~=1
                updatedMax = repmat(mean(updatedMax,3),1,1,numChannel);
            end
        end
        imageInput = imageInputLayer(imageSize, ...
            'Normalization',inputLayer.Normalization,...
            'NormalizationDimension',inputLayer.NormalizationDimension,...
            'Mean',updatedMean ,...
            'StandardDeviation',updatedStandardDeviation,...
            'Min',updatedMin,...
            'Max',updatedMax,...
            'Name',inpNet.Layers(imageInputIdx).Name);

        inpNet = replaceLayer(inpNet,inpNet.Layers(imageInputIdx).Name,...
            imageInput);
    end
else
    error(message('vision:ssd:imageInputLayerRequired'));
end

end

%--------------------------------------------------------------------------
function extraChannelsOfImage = iUpdateChannelProperties(image, extraChannels)
% It add the extra channels to network's imageInputLayer to incorporate
% training Image inputSize channels.
imageStatsMean = repmat(mean(image,3),1,1,extraChannels);
extraChannelsOfImage = cat(3,image,imageStatsMean);
end
%------------------------------------------------------------------------------------------------------------------------------------------------------
function updatedImageStats = iUpdateImageTnputLayerStats(imageStats,imageSize)
% It updates the imageInputLayer's  properties like mean, standard
% deviation, min, max based on the user InputSize

% 1-by-1-by-c array per channel or empty []  or a numeric scalar.
if isequal([1 1],size(imageStats,1:2)) || isempty(imageStats) || isscalar(imageStats)
    updatedImageStats = imageStats;
else
    %  If value h-by-w-by-c array then resize according to inputSize.
    updatedImageStats = imresize(imageStats,imageSize(1:2),"nearest");
end
end
%------------------------------------------------------------------------------------------------------------------------------------------------------
function iVerifyLayersExist(lgraph, layerNames)
% Verifies that particular layer presents in layerGraph.
numLayers = numel(lgraph.Layers);
for idx = 1:numel(layerNames)
    foundLayer = false;
    for lIdx = 1:numLayers
        if strcmp(layerNames(idx), lgraph.Layers(lIdx).Name)
            foundLayer = true;
            break;
        end
    end
    if ~foundLayer
        error(message('vision:ssd:InvalidLayerName', layerNames{idx}));
    end
end
end
%--------------------------------------------------------------------------
function [lgraph,removeLayersName] = iRemoveLayers(lgraph, lastLayer)
% Remove all the layers after lastLayer.
dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);

% Find the last layer.
id = findnode(dg,char(lastLayer));

% Search for all nodes starting from the feature extraction
% layer.
if ~(sum(id)==0)
    ids = dfsearch(dg,id);
    names = dg.Nodes.Name(ids,:);
    removeLayersName = names(2:end)';
    lgraph = removeLayers(lgraph, removeLayersName');
end
end

%--------------------------------------------------------------------------
function predictorLayerStruct = iCreatePredictorLayers(...
    layersToConnect, weightsInitializerValue, biasInitializerValue, ...
    numClasses, anchorBoxes)

% Create independent predictor layers for each branch.
numPredictorBranches = numel(layersToConnect);
predictorLayerStruct = repmat(struct('ConnectTo', '', 'Layers', []), ...
    numPredictorBranches, 1);
for idx = 1:numPredictorBranches

    conv_layers = iCreateConvHead(layersToConnect{idx}, ...
        weightsInitializerValue, biasInitializerValue, ...
        numClasses, ...
        size(anchorBoxes{idx}, 1));
    predictorLayerStruct(idx).ConnectTo = layersToConnect{idx};
    predictorLayerStruct(idx).Layers = conv_layers;
end
end

%--------------------------------------------------------------------------
function p = iSamePadding(FilterSize)
p = floor(FilterSize / 2);
end
%--------------------------------------------------------------------------
function layers = iCreateConvHead(layerName, weightsInitializerValue, biasInitializerValue, numClasses, numBoxesPerGrid)

layers = [];
% Add mbox_conf.
filterSize = 3;
numFilters = numClasses * numBoxesPerGrid;

layer_mbox_conf = convolution2dLayer(filterSize, numFilters, ...
    'Padding', iSamePadding(filterSize), ...
    'Name', layerName + "_mbox_conf", ...
    'WeightsInitializer', weightsInitializerValue, ...
    'BiasInitializer', biasInitializerValue);
layers = [layers; layer_mbox_conf];

% Add mbox_loc
numBBoxElems = 4;
filterSize = 3;
numFilters = numBBoxElems * numBoxesPerGrid;

layer_mbox_loc = convolution2dLayer(filterSize, numFilters, ...
    'Padding', iSamePadding(filterSize), ...
    'Name', layerName + "_mbox_loc", ...
    'WeightsInitializer', weightsInitializerValue, ...
    'BiasInitializer', biasInitializerValue);

layers = [layers; layer_mbox_loc];
end

%------------------------------------------------------------------------------------------------------------------------------------------------------
function dlnet = iUpdateAndValidatedlnetwork(dlnet,dlnetAnalysis,params,extractedDetectionNetworkSource,numClasses)
% Verify the dlnetwork and connected prediction branches like num of
% classification and prediction layer's output. It also verifies that
% connected prediction layers have desired order if not then set the order
% of outputs of dlnetwork in such a way that Prediction layers connected
% with DetectionNetworkSource(DNS) of larger feature map should appear
% first in output Names of dlentwork
% [DNSlargerFeatureMapSize_connectedPredictionLayerOutput........DNSSmallerFeatureMapSize_connectedPredictionLayerOutput]
% = predict(detector.dlentwork)
outputsOfNetwork = dlnet.OutputNames;
AnchorBoxes = params.AnchorBoxes;
outputLayers = cell(1,numel(outputsOfNetwork));
for i = 1:numel(AnchorBoxes)
    sources = iSourceAnalyzerOfLayer(dlnetAnalysis,extractedDetectionNetworkSource{i});
    currentDNSLayerOutputs = convertStringsToChars(sources.Outputs.Destination{:});
    predictionLayersOfCurrentDNSLayer = intersect(outputsOfNetwork,currentDNSLayerOutputs);
    % Calculated the source layer analyzer of prediction layer connected
    % with ith detection network source.
    sourcesOut1 = iSourceAnalyzerOfLayer(dlnetAnalysis,predictionLayersOfCurrentDNSLayer{1});
    sourcesOut2 = iSourceAnalyzerOfLayer(dlnetAnalysis,predictionLayersOfCurrentDNSLayer{2});
    numAnchorBoxes = size(AnchorBoxes{i},1);
    numclassesIncludingBackgd = (numClasses + 1);
    numChannelsClassificationLayer = numAnchorBoxes* numclassesIncludingBackgd;
    numChannelsRegressionLayer = numAnchorBoxes *4;
    isRegClassChannelEqual = (sourcesOut1.Outputs.Size{1}(3) == numChannelsRegressionLayer && sourcesOut2.Outputs.Size{1}(3) == numChannelsRegressionLayer && numChannelsClassificationLayer == numChannelsRegressionLayer);

    if ~isRegClassChannelEqual
        % If channelSize of both prediction layers are same then we are
        % assuming that user has connected prediction layers in proper
        % order.
        if ~(sourcesOut1.Outputs.Size{1}(3) == numChannelsRegressionLayer || sourcesOut2.Outputs.Size{1}(3) == numChannelsRegressionLayer)
            % If channelSize of the any prediction branch (layer) connected
            % with ith detectionNetworkSource (DNS) is not equal to desired
            % regression layer channel the throw the error.
            error(message('vision:ssd:invalidChannelSizeForRegressionPredictionLayer',extractedDetectionNetworkSource{i}));
        elseif  (sourcesOut1.Outputs.Size{1}(3) == numChannelsClassificationLayer || sourcesOut2.Outputs.Size{1}(3) == numChannelsClassificationLayer) && ~isempty(extractedDetectionNetworkSource)
            if sourcesOut1.Outputs.Size{1}(3) == numChannelsClassificationLayer
                % If first prediction layer is classification layer.
                outputLayers{2*i-1} = predictionLayersOfCurrentDNSLayer{1};
                outputLayers{2*i} = predictionLayersOfCurrentDNSLayer{2};
            elseif sourcesOut2.Outputs.Size{1}(3) == numChannelsClassificationLayer
                % If second prediction layer is classification layer.
                outputLayers{2*i-1} = predictionLayersOfCurrentDNSLayer{2};
                outputLayers{2*i} = predictionLayersOfCurrentDNSLayer{1};
            end
        else
            % If any of the prediction layer does not have
            % classification layer then throw the error.
            if sourcesOut1.Outputs.Size{1}(3) == numChannelsRegressionLayer
                classificationLayer = sourcesOut2;
            else
                classificationLayer = sourcesOut1;
            end
            if classificationLayer.Outputs.Size{1}(3) ~= numChannelsClassificationLayer
                incorrectSize = mat2str(classificationLayer.Outputs.Size{1});
                error(message('vision:ssd:invalidChannelSizeForClassificationPredictionLayer',classificationLayer.Name, extractedDetectionNetworkSource{i},numChannelsClassificationLayer,incorrectSize));
            end
        end
    end
end
if ~isequal(convertCharsToStrings(outputLayers(:)),convertCharsToStrings(outputsOfNetwork(:))) && ~isRegClassChannelEqual
    % Set the dlnetwork output order if it is not in proper order and number of
    % channels of regression and classification layer is not equal (number
    % of classes including back ground is 4 and regression channel is also
    % 4).
    dlnet.OutputNames = outputLayers;
end
end
%------------------------------------------------------------------------------------------------------------------------------------------------------
function source = iSourceAnalyzerOfLayer(analysis,layerName)
% Extract the network analyzer source of particular layer.
layerIdx = strcmp({analysis.ExternalLayers.Name},layerName);
source = analysis.LayerAnalyzers(layerIdx);
end
