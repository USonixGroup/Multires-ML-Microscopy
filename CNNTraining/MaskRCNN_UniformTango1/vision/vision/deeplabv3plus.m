function network = deeplabv3plus(imageSize, numClasses, networkName, options)

% Copyright 2023 The MathWorks, Inc.

arguments
    imageSize {validateImageSize}
    numClasses {mustBeNumeric, mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeInteger, mustBeNonsparse, mustBeGreaterThan(numClasses,1)}
    networkName string {mustBeMember(networkName,["resnet18","resnet50","mobilenetv2","xception","inceptionresnetv2"])}
    options.DownsamplingFactor {validateDownsamplingFactor} = 16
end

vision.internal.requiresNeuralToolbox(mfilename);

% Append 3 if imageSize is 2-element vector.
if numel(imageSize) == 2
    imageSize = [imageSize 3];
end

% Use depth separable conv in ASPP and decoder sub-networks for xception
% and mobilenet v2. 
if any(strcmp(networkName,["xception" "mobilenetv2"]))
    useDepthSeparableConv = true;
else
    useDepthSeparableConv = false;
end

% Construct the network.
networkInfo = iPredefinedNetworkInfo(networkName);
network = iLoadModel(networkName);

% Verify user imageSize is at least as big as the network's image input size.
iAssertImageSizeIsCompatibileWithNetwork(imageSize,network,networkInfo.Name);

% Modify backbone network using below-indented functions.
network = iRemoveClassificationLayers(network, networkInfo);
[network,networkInfo] = iChangeInputSize(network,imageSize,networkInfo);
network = iFixMisalignment(network, networkInfo);
network = iReduceStrides(network,networkInfo,options.DownsamplingFactor);
network = iDilateNetwork(network,networkInfo,options.DownsamplingFactor);

network = iAddAspptoNetwork(network, networkInfo, options.DownsamplingFactor, 256, useDepthSeparableConv);
network = iAddDecoderToNetwork(network, networkInfo, options.DownsamplingFactor, numClasses, useDepthSeparableConv);
network = iAddSoftmaxLayers(network, networkInfo);
network = initialize(network);
end


%--------------------------------------------------------------------------
function networkInfo = iPredefinedNetworkInfo(networkName)
% FeatureExtractionLayer - last layer in network before classification
%                          layers.
%
% LowLevelFeatureLayer   - last layer whose output size is downsampled by
%                          factor of 4.
%
% LastDownsamplingLayers - last layers that downsample.
%
% SecondLastDownsamplingLayers - second to last layers that downsample.

switch networkName
    case "resnet18"
        networkInfo = struct(...
            'Name',"resnet18",...
            'FeatureExtractionLayer', "res5b_relu", ...
            'LowLevelFeatureLayer', "res2b_relu",...
            'LastDownsamplingLayers',["res5a_branch2a", "res5a_branch1"],...
            'SecondLastDownsamplingLayers',["res4a_branch2a", "res4a_branch1"]);

    case "resnet50"
        networkInfo = struct(...
            'Name',"resnet50",...
            'FeatureExtractionLayer', "activation_49_relu", ...
            'LowLevelFeatureLayer', "activation_10_relu",...
            'LastDownsamplingLayers',["res5a_branch2a", "res5a_branch1"],...
            'SecondLastDownsamplingLayers',["res4a_branch2a", "res4a_branch1"]);

    case "mobilenetv2"
        % MobileNet v2: Choose second to last output layer as feature extractor to
        % reduce cost of computation, as done in:
        %
        %   Sandler, Mark, et al. "Mobilenetv2: Inverted residuals and linear
        %   bottlenecks." 2018 IEEE/CVF Conference on Computer Vision and Pattern
        %   Recognition. IEEE, 2018.
        networkInfo = struct(...
            'Name',"mobilenetv2",...
            'FeatureExtractionLayer', "block_16_project_BN", ...
            'LowLevelFeatureLayer', "block_3_expand_relu",...
            'LastDownsamplingLayers',"block_13_depthwise",...
            'SecondLastDownsamplingLayers',"block_6_depthwise");

    case "xception"
        networkInfo = struct(...
            'Name',"xception",...
            'FeatureExtractionLayer', "block14_sepconv2_act", ...
            'LowLevelFeatureLayer', "add_1",...
            'LastDownsamplingLayers',["block13_pool","conv2d_4"],...
            'SecondLastDownsamplingLayers',["block4_pool","conv2d_3"]);

    case "inceptionresnetv2"
        networkInfo = struct(...
            'Name',"inceptionresnetv2",...
            'FeatureExtractionLayer', "conv_7b_ac", ...
            'LowLevelFeatureLayer', "activation_5",...
            'LastDownsamplingLayers',["conv2d_158","conv2d_160","conv2d_163","max_pooling2d_4"],...
            'SecondLastDownsamplingLayers',["conv2d_73","conv2d_76","max_pooling2d_3"]);
    otherwise
        % This branch should not be reached as the networkName input argument
        % validation will error first.
        assert(false, 'unsupported network');
end

networkInfo.DecoderOutputLayer = "dec_crop2";
networkInfo.AsppTail = 'catAspp';

end

%--------------------------------------------------------------------------
% Load backbone network
function network = iLoadModel(modelName)
try
    % Use imagePretrainedNetwork for all networks when ready.
    network = imagePretrainedNetwork(modelName);
catch ME
    throwAsCaller(ME)
end
end

%--------------------------------------------------------------------------
function network = iRemoveClassificationLayers(network, networkInfo)
layerList = iFindIntermediateLayers(network,networkInfo.FeatureExtractionLayer);
network = removeLayers(network,layerList);
end

%--------------------------------------------------------------------------
function [network,networkInfo] = iChangeInputSize(network, imageSize,networkInfo)
idx = arrayfun(@(x)strcmp(class(x),"nnet.cnn.layer.ImageInputLayer"),network.Layers);
originalInputLayer = network.Layers(idx);
newLayer = vision.internal.cnn.utils.updateImageLayerInputSize(originalInputLayer,imageSize);
network = replaceLayer(network,originalInputLayer.Name,newLayer);
networkInfo.InputLayerName = originalInputLayer.Name;
end

%--------------------------------------------------------------------------
function network = iFixMisalignment(network, networkInfo)
% Fix misalignment by changing downsampling layers with asymmetrical
% padding to have 'same' padding.
switch networkInfo.Name
    case "resnet18"
        % no-op
    case "resnet50"
        network = iAlignResNet50(network);
    case "mobilenetv2"
        % no-op
    case "xception"
        network = iAlignXception(network);
    case "inceptionresnetv2"
        network = iAlignInceptionResNetV2(network);
    otherwise
        % This branch should not be reached as the networkName input argument
        % validation will error first.
        assert(false, 'unsupported network');
end

% Second to last downsampling layers should also be set to have 'same'.
% These are not modified is DownsamplingFactor is 16 so we must do it here.
layers = networkInfo.SecondLastDownsamplingLayers;
for i = 1:numel(layers)
    layer = iFindLayer(network,layers(i));
    network = iUseSamePadding(network,layer);
end
end

%--------------------------------------------------------------------------
function network = iAlignResNet50(network)
oldLayer = iFindLayer(network,'max_pooling2d_1');
newLayer = iUpdateMaxPoolingLayer(oldLayer,Padding="same");
network = replaceLayer(network,oldLayer.Name,newLayer);
end

%--------------------------------------------------------------------------
function network = iAlignXception(network)
% Update block1_conv1 to use 'same' padding.
oldL = iFindLayer(network,'block1_conv1');
newLayer = iUpdateConvLayer(oldL,Padding="same");
network = replaceLayer(network,oldL.Name,newLayer);

% Update block1_conv2 to use 'same' padding.
oldL = iFindLayer(network,'block1_conv2');
newLayer = iUpdateConvLayer(oldL,Padding="same");
network = replaceLayer(network,oldL.Name,newLayer);
end

%--------------------------------------------------------------------------
function network = iAlignInceptionResNetV2(network)

convLayers = ["conv2d_1","conv2d_2","conv2d_5"];
for i = 1:numel(convLayers)
    oldL = iFindLayer(network,convLayers(i));
    newLayer = iUpdateConvLayer(oldL,Padding="same");
    network = replaceLayer(network,oldL.Name,newLayer);
end

poolLayers = ["max_pooling2d_1", "max_pooling2d_2"];
for i = 1:numel(poolLayers)
    oldL = iFindLayer(network,poolLayers(i));
    newLayer = iUpdateMaxPoolingLayer(oldL,Padding="same");
    network = replaceLayer(network,oldL.Name,newLayer);
end
end

%--------------------------------------------------------------------------
function newLayer = iUpdateMaxPoolingLayer(oldLayer, varargin)
% Extract padding value from the old layer.
padVal = iPadValFromMode(oldLayer);

p = inputParser;
p.addParameter('Stride',oldLayer.Stride);
p.addParameter('Padding',padVal);

p.parse(varargin{:});

newLayer = maxPooling2dLayer(oldLayer.PoolSize,Stride=p.Results.Stride,...
    Name=oldLayer.Name,Padding=p.Results.Padding);

end

%--------------------------------------------------------------------------
function newLayer = iUpdateAvgPoolingLayer(oldLayer, varargin)
% Extract padding value from the old layer.
padVal = iPadValFromMode(oldLayer);

p = inputParser;
p.addParameter('Stride',oldLayer.Stride);
p.addParameter('Padding',padVal);

p.parse(varargin{:});

newLayer = averagePooling2dLayer(oldLayer.PoolSize,Stride=p.Results.Stride,...
    Name=oldLayer.Name,Padding=p.Results.Padding);

end

%--------------------------------------------------------------------------
function padVal = iPadValFromMode(layer)
% Extract padding value from the old layer.
if strcmp(layer.PaddingMode,'manual')
    padVal = layer.PaddingSize;
else
    padVal = layer.PaddingMode;
end
end

%--------------------------------------------------------------------------
function newLayer = iUpdateConvLayer(oldLayer, varargin)
% Update conv layer Stride, DilationFactor, or Padding. Copy all other
% properties.

padVal = iPadValFromMode(oldLayer);

p = inputParser;
p.addParameter('Stride',oldLayer.Stride);
p.addParameter('Padding',padVal);
p.addParameter('DilationFactor',oldLayer.DilationFactor);

p.parse(varargin{:});

newLayer = convolution2dLayer(oldLayer.FilterSize,oldLayer.NumFilters,...
    Stride=p.Results.Stride,...
    DilationFactor=p.Results.DilationFactor,...
    Name=oldLayer.Name,...
    Padding=p.Results.Padding,...
    WeightLearnRateFactor=oldLayer.WeightLearnRateFactor,...
    BiasLearnRateFactor=oldLayer.BiasLearnRateFactor,...
    WeightsInitializer=oldLayer.WeightsInitializer,...
    BiasInitializer=oldLayer.BiasInitializer,...
    BiasL2Factor= oldLayer.BiasL2Factor,...
    WeightL2Factor=oldLayer.WeightL2Factor);
newLayer.Weights = oldLayer.Weights;
newLayer.Bias = oldLayer.Bias;
end

%--------------------------------------------------------------------------
function newLayer = iUpdateGroupedConvLayer(oldLayer, varargin)
% Update conv layer Stride, DilationFactor, or Padding. Copy all other
% properties.

padVal = iPadValFromMode(oldLayer);

p = inputParser;
p.addParameter('Stride',oldLayer.Stride);
p.addParameter('Padding',padVal);
p.addParameter('DilationFactor',oldLayer.DilationFactor);

p.parse(varargin{:});

newLayer = groupedConvolution2dLayer(oldLayer.FilterSize,...
    oldLayer.NumFiltersPerGroup,oldLayer.NumGroups,...
    Stride=p.Results.Stride,...
    DilationFactor=p.Results.DilationFactor,...
    Name=oldLayer.Name,...
    Padding=p.Results.Padding,...
    WeightLearnRateFactor=oldLayer.WeightLearnRateFactor,...
    BiasLearnRateFactor=oldLayer.BiasLearnRateFactor,...
    WeightsInitializer=oldLayer.WeightsInitializer,...
    BiasInitializer=oldLayer.BiasInitializer,...
    BiasL2Factor= oldLayer.BiasL2Factor,...
    WeightL2Factor=oldLayer.WeightL2Factor);
newLayer.Weights = oldLayer.Weights;
newLayer.Bias = oldLayer.Bias;
end

%--------------------------------------------------------------------------
function network = iReduceStrides(network,networkInfo,downsamplingFactor)

% DF : downsamplingFactor.
% Reduce stride of the last pooling layer for all branches for DF = 16
for ilayerName=networkInfo.LastDownsamplingLayers
    oldLayer = iFindLayer(network,ilayerName);
    network = iReduceStridesOfLayer(network,oldLayer);
end
% Reduce stride of the last 2nd pooling layer for all branches for DF = 8
if downsamplingFactor == 8
    for ilayerName=networkInfo.SecondLastDownsamplingLayers
        oldLayer = iFindLayer(network,ilayerName);
        network = iReduceStridesOfLayer(network,oldLayer);
    end
end
end

%--------------------------------------------------------------------------
function network = iUseSamePadding(network,oldL)

if class(oldL) == "nnet.cnn.layer.MaxPooling2DLayer"
    newLayer = iUpdateMaxPoolingLayer(oldL,Padding="same");
elseif class(oldL) == "nnet.cnn.layer.Convolution2DLayer"
    newLayer = iUpdateConvLayer(oldL,Padding="same");
elseif class(oldL) == "nnet.cnn.layer.AveragePooling2DLayer"
    newLayer = iUpdateAvgPoolingLayer(oldL,Padding="same");
elseif class(oldL) == "nnet.cnn.layer.GroupedConvolution2DLayer"
    newLayer = iUpdateGroupedConvLayer(oldL,Padding="same");
else
    assert(false,'unsupported layer type');
end
network = replaceLayer(network,oldL.Name,newLayer);
end

%--------------------------------------------------------------------------
function network = iReduceStridesOfLayer(network,oldL)

if oldL.Stride ~= 2
    % all networks supported by this function have Stride of 1 or 2.
    assert(false,'Expected Stride to be 1 or 2');
end

if class(oldL) == "nnet.cnn.layer.MaxPooling2DLayer"
    network = iReduceStrideToOneForMaxPooling(network,oldL);
elseif class(oldL) == "nnet.cnn.layer.Convolution2DLayer"
    network = iReduceStrideToOneForConv(network,oldL);
elseif class(oldL) == "nnet.cnn.layer.AveragePooling2DLayer"
    network = iReduceStrideToOneForAvgPool(network,oldL);
elseif class(oldL) == "nnet.cnn.layer.GroupedConvolution2DLayer"
    network = iReduceStrideToOneFoGroupedConv(network,oldL);
end

end

%--------------------------------------------------------------------------
function network = iReduceStrideToOneForMaxPooling(network,oldL)
% Written to reduce stride = 2 to stride = 1.
% Assumes downsizing filter is symmetrical.

if oldL.PoolSize ==1
    newLayer = iUpdateMaxPoolingLayer(oldL,Stride=1,Padding=0);
elseif oldL.PoolSize > 1
    newLayer = iUpdateMaxPoolingLayer(oldL,Stride=1,Padding="same");
end
network = replaceLayer(network,oldL.Name,newLayer);
end

%--------------------------------------------------------------------------
function network = iReduceStrideToOneFoGroupedConv(network,oldL)
% Assumes downsizing filter is symmetrical.
% Written to reduce stride = 2 to stride = 1.
if max(oldL.FilterSize) ==1
    newLayer = iUpdateGroupedConvLayer(oldL,Stride=1,Padding=0);
elseif max(oldL.FilterSize) > 1
    newLayer = iUpdateGroupedConvLayer(oldL,Stride=1,Padding="same");
end
newLayer.Weights = oldL.Weights;
newLayer.Bias = oldL.Bias;
network = replaceLayer(network,oldL.Name,newLayer);
end

%--------------------------------------------------------------------------
function network = iReduceStrideToOneForConv(network,oldL)
% Assumes downsizing filter is symmetrical.
% Written to reduce stride = 2 to stride = 1.
if max(oldL.FilterSize) ==1
    newLayer = iUpdateConvLayer(oldL,Stride=1,Padding=0);
elseif max(oldL.FilterSize) > 1
    newLayer = iUpdateConvLayer(oldL,Stride=1,Padding="same");
end
newLayer.Weights = oldL.Weights;
newLayer.Bias = oldL.Bias;
network = replaceLayer(network,oldL.Name,newLayer);
end

%--------------------------------------------------------------------------
function network = iReduceStrideToOneForAvgPool(network,oldL)
% Written to reduce stride = 2 to stride = 1.
% Assumes downsizing filter is symmetrical.

if oldL.PoolSize ==1
    newLayer = iUpdateAvgPoolingLayer(oldL,Stride=1,Padding=0);
elseif oldL.PoolSize > 1
    newLayer = iUpdateAvgPoolingLayer(oldL,Stride=1,Padding="same");
end
network = replaceLayer(network,oldL.Name,newLayer);
end

%--------------------------------------------------------------------------
function network = iDilateNetwork(network,networkInfo,downsamplingFactor)
% First dilation for all layers after the last pooling layer.
network = iDilateAfterPool(network,networkInfo.LastDownsamplingLayers);
% Dilate an extra time if DF = 8 for all layers after 2nd last pool.
if downsamplingFactor == 8
    network = iDilateAfterPool(network,networkInfo.SecondLastDownsamplingLayers);
end
end

%--------------------------------------------------------------------------
function network = iDilateAfterPool(network,poolLayers)
% Find all layers that come after the pooling layer.
layersAfterPool = [];
for i = poolLayers
    layersAfterPool = [layersAfterPool,iFindIntermediateLayers(network,i)]; %#ok<AGROW>
end
layersAfterPool = unique(layersAfterPool);

% Find all Convolution2DLayers and GroupedConvolution2dLayers that come
% after the removed pooling layer.
toDilate = [];
for i = layersAfterPool
    iLayer = iFindLayer(network,i);
    if strcmp(class(iLayer),"nnet.cnn.layer.Convolution2DLayer") || strcmp(class(iLayer),"nnet.cnn.layer.GroupedConvolution2dLayer")
        if max(iLayer.FilterSize) > 1
            toDilate = [toDilate ,i]; %#ok<AGROW>
        end
    end
end
% Multiply dilation of all layers in toDilate by 2.
% The network assumes that the downsampling steps that were removed had a
% stride of 2 each.
for i = toDilate
    oldL = iFindLayer(network,i);
    newLayer = iUpdateConvLayer(oldL,Stride=1,Padding="same",DilationFactor=2*oldL.DilationFactor);
    network = replaceLayer(network,i,newLayer);
end
end

%-----------------------------------------------------------------------
% Aspp module
% An ASPP module can be visualized as follows:
%
%                           |
%                 +-----+---+--+------+------------+
%                 |     |      |      |            |
%                 |     |      |      |            |
%                 v     v      v      v            v
%           +-----++ +--+---+ ++----+ +-----+ +----+-+
%           |ASPP  | |ASPP  | |ASPP | |ASPP | |ASPP  |
%           |Branch| |Branch| |Branch |Branch |Branch|
%           |D1    | |D6    | |D12  | |D18  | |GAP   |
%           +-----++ +--+---+ +--+--+ +--+--+ +---+--+
%                 |     |        |       |        |
%                 v     v        v       v        v
%           +-----+-----+--------+-------+--------+--+
%           |Depth concatenation                     |
%           +----------------------------------------+

function network = iAddAspptoNetwork(network, networkInfo, downsamplingFactor, numFilters, useDepthSeparableConv)

%Notes:
%----------------------------------------
% 1. nLayerInputChannels is needed to initialize weights despite being
% optional argument to convolution2dLayer
% 2. the asppDilationFactors in each branch are selected based on the
% network's downsampling factor. Both values are taken directly from the
% paper, selected from empirical testing.
% Twice the size of asppDilationFactors for a downsamplingFactor of 8,
% compensates for the loss of receptive field due to having been
% downsampled fewer times.
%----------------------------------------

switch downsamplingFactor
    case 16
        asppDilationFactors = [1, 6 ,12 ,18];
    case 8
        asppDilationFactors = [1, 12 ,24 ,36];
end
asppFilterSizes = [1, 3, 3, 3];

% Create concatenation layer and connect it to the tail.
tempLayer = depthConcatenationLayer(4,Name=networkInfo.AsppTail);
network = addLayers(network,tempLayer);

% Create all convolutional aspp branches.
for i = 1:numel(asppDilationFactors)
    asppConvName = "aspp_Conv_" + string(i);
    branchFilterSize = asppFilterSizes(i);
    branchDilationFactor = asppDilationFactors(i);
    
    % Reassign tempLayer to new conv layer.
    if useDepthSeparableConv
        asppConvNameStart = asppConvName + "_depthwise";
        asppConvNameEnd = asppConvName + "_pointwise";
        tempLayer = [
            groupedConvolution2dLayer(branchFilterSize, 1, 'channel-wise',...
            DilationFactor=branchDilationFactor, Padding="same",...
            Name=asppConvNameStart,...
            WeightsInitializer="glorot",BiasInitializer="zeros",...
            BiasLearnRateFactor=0,...
            WeightLearnRateFactor=10);

            convolution2dLayer(1, numFilters, ...
            Padding=0, ...
            Name=asppConvNameEnd,...
            WeightsInitializer="glorot",BiasInitializer="zeros",...
            BiasLearnRateFactor=0,...
            WeightLearnRateFactor=10);
            ];

    else
        asppConvNameStart = asppConvName;
        asppConvNameEnd = asppConvName;
        tempLayer = convolution2dLayer(branchFilterSize, numFilters,...
            DilationFactor=branchDilationFactor, Padding="same",...
            Name=asppConvName,...
            WeightsInitializer="glorot",BiasInitializer="zeros",...
            BiasLearnRateFactor=0, ...
            WeightLearnRateFactor=10);
    end

    network = addLayers(network,tempLayer);

    % Reassign tempLayer to new batch norm layer.
    asppBNName = "aspp_BatchNorm_" + string(i);
    tempLayer = batchNormalizationLayer(Name=asppBNName);
    network = addLayers(network,tempLayer);

    % Reassign tempLayer to new relu layer.
    asppReluName = "aspp_Relu_" + string(i);
    tempLayer = reluLayer(Name=asppReluName);
    network = addLayers(network,tempLayer);

    network = connectLayers(network,networkInfo.FeatureExtractionLayer,asppConvNameStart);
    network = connectLayers(network,asppConvNameEnd,asppBNName);
    network = connectLayers(network,asppBNName,asppReluName);
    network = connectLayers(network,asppReluName,strcat(networkInfo.AsppTail,"/in",string(i)));
end
end

%-----------------------------------------------------------------------
% Decoder
% The Decoder module can be visualized as follows :
%
% +----------------+  +--------------------+--------+
% |ASPP Output     |  |LowLevelFeature      Layers  |
% +----------------+  +-----------------------------+
% +----------------+  +-----------------------------+
% |Preprocess      |  |Preprocess          |        |
% |ASPP Output     |  |Intermediate Layers |        |
% |                |  |                    |        |
% |Conv - BN - Relu|  |Conv - BN - Relu    |        |
% |                |  |                    |        |
% |                |  +-----------------------------+
% |                |                       |
% +----------------+                       |
% +----------------------------+           |
% |Upsample ASPP outputs  x 4  |           |
% +----------------------------+           |
% +----------------------------+           |
% |Concatenate     |           | <---------+
% +----------------------------+
% +----------------------------+
% | Conv |BN |Relu |           |
% | Conv |BN |Relu |           |
% +----------------------------+
% +-------------------------------------------+
% |                v                          |
% |Upsample final feature maps  x 4           |
% |Predict segmentation maps                  |
% +-------------------------------------------+
% +-------------------------------------------+
% |Output Prediction                          |
% +-------------------------------------------+

function network = iAddDecoderToNetwork(network, networkInfo, downsamplingFactor, numClasses, useDepthSeparableConv)
%Notes:
%----------------------------------------
% 1. Wrapper decoder function to enable additon of different types of
% decoder modules for model extensibility
%----------------------------------------
network = iDecoderv3Plus(network, networkInfo, downsamplingFactor, numClasses, useDepthSeparableConv);
end

%--------------------------------------------------------------------------
function network = iDecoderv3Plus(network, networkInfo, downsamplingFactor, numClasses, useDepthSeparableConv)
switch downsamplingFactor
    case 16
        bilinearUpsamplingFactor = [4 4];
    case 8
        bilinearUpsamplingFactor = [2 4];
    otherwise
        assert(0,'unsupported downsampling factor');
end
% Preprocess aspp output before decoding.
tempLayerArray = [
    convolution2dLayer(1,256,Name="dec_c1",WeightLearnRateFactor=10,...
    BiasInitializer="zeros",BiasLearnRateFactor=0);
    batchNormalizationLayer(Name="dec_bn1");
    reluLayer(Name="dec_relu1");
    iBilinearUpsamplingLayer(bilinearUpsamplingFactor(1), 256,'dec_upsample1')
    crop2dLayer('centercrop',Name="dec_crop1")];

network = addLayers(network,tempLayerArray);

% Preprocess LowLevelFeatureLayer output before decoding.
tempLayerArray = [
    convolution2dLayer(1,48,BiasInitializer="zeros",BiasLearnRateFactor=0,Name="dec_c2",WeightLearnRateFactor=10);
    batchNormalizationLayer(Name="dec_bn2");
    reluLayer(Name="dec_relu2")];

network = addLayers(network,tempLayerArray);

tempLayerArray = depthConcatenationLayer(2,Name="dec_cat1");

if useDepthSeparableConv

    tempLayerArray = [
        tempLayerArray
        groupedConvolution2dLayer(3,1,'channel-wise',BiasInitializer="zeros",BiasLearnRateFactor=0,Name="dec_c3_depthwise",Padding="same",WeightLearnRateFactor=10);
        convolution2dLayer(1,256,BiasInitializer="zeros",BiasLearnRateFactor=0,Name="dec_c3_pointwise",Padding=0,WeightLearnRateFactor=10);
        batchNormalizationLayer(Name="dec_bn3");
        reluLayer(Name="dec_relu3")
        groupedConvolution2dLayer(3,1,'channel-wise',BiasInitializer="zeros",BiasLearnRateFactor=0,Name="dec_c4_depthwise",Padding="same",WeightLearnRateFactor=10)
        convolution2dLayer(1,256,BiasInitializer="zeros",BiasLearnRateFactor=0,Name="dec_c4_pointwise",Padding=0,WeightLearnRateFactor=10)
        ];
else
    tempLayerArray = [
        tempLayerArray
        convolution2dLayer(3,256,BiasInitializer="zeros",BiasLearnRateFactor=0,Name="dec_c3",Padding="same",WeightLearnRateFactor=10);
        batchNormalizationLayer(Name="dec_bn3");
        reluLayer(Name="dec_relu3")
        convolution2dLayer(3,256,BiasInitializer="zeros",BiasLearnRateFactor=0,Name="dec_c4",Padding="same",WeightLearnRateFactor=10);
        ];

end

tempLayerArray = [
    tempLayerArray
    batchNormalizationLayer(Name="dec_bn4");
    reluLayer(Name="dec_relu4")
    convolution2dLayer(1,numClasses,BiasInitializer="zeros",BiasLearnRateFactor=0,Name="scorer",WeightLearnRateFactor=10)
    iBilinearUpsamplingLayer(bilinearUpsamplingFactor(2), numClasses,"dec_upsample2")
    crop2dLayer('centercrop',Name="dec_crop2")
    ];

network = addLayers(network,tempLayerArray);
network = connectLayers(network,networkInfo.AsppTail,'dec_c1');
network = connectLayers(network,'dec_relu2','dec_cat1/in1');
network = connectLayers(network,'dec_crop1','dec_cat1/in2');
network = connectLayers(network,networkInfo.LowLevelFeatureLayer,'dec_c2');
network = connectLayers(network,'dec_relu2','dec_crop1/ref');
network = connectLayers(network,networkInfo.InputLayerName,'dec_crop2/ref');
end

%--------------------------------------------------------------------------
function network = iAddSoftmaxLayers(network, networkInfo)
network = addLayers(network,softmaxLayer(Name="softmax-out"));
network = connectLayers(network,networkInfo.DecoderOutputLayer,"softmax-out");
end

%--------------------------------------------------------------------------
function upsamplingLayer = iBilinearUpsamplingLayer(scaleFactor, numFilters, name)
% Configure a transposed convolution layer for bilinear upsampling. Weights
% are frozen to bilinear interpolation weights.
%
% Upsampling can be done only by integer values, which avoids
% checkerboard artifacts.
numChannels = numFilters;
if isscalar(scaleFactor)
    scaleFactor = [scaleFactor, scaleFactor];
end

factor = scaleFactor;
filterSize = 2*factor - mod(factor,2);
cropping = (factor-mod(factor,2))/2;

upsamplingLayer = transposedConv2dLayer(filterSize,numFilters, ...
    NumChannels=numChannels,Stride=factor,Cropping=cropping,Name=name);

upsamplingLayer.Weights = vision.internal.cnn.bilinearUpsamplingWeights(filterSize,numChannels,numFilters);
upsamplingLayer.Bias = zeros(1,1,numFilters);

% Freeze weights and bias.
upsamplingLayer.WeightLearnRateFactor = 0;
upsamplingLayer.BiasLearnRateFactor = 0;

end

%--------------------------------------------------------------------------
function layerList = iFindIntermediateLayers(network,head,tail)
% Find all layers between the head and tail layer, excluding head and
% tail layer themselves.

layerNames = string({network.Layers.Name});
headIdx = find(strcmp(head, layerNames));

if nargin == 3
    tailIdx = find(strcmp(tail, layerNames));
    layerList = layerNames(headIdx+1:tailIdx-1);
else
    layerList = layerNames(headIdx+1:end);
end

end

%--------------------------------------------------------------------------
function layer = iFindLayer(network,layerName)
% Finds the layer corresponding to the layerName in network.Layers.
idx = arrayfun(@(x) strcmp(x.Name,layerName),network.Layers);
layer = network.Layers(idx);
end 

%--------------------------------------------------------------------------
function validateImageSize(x)
% Input should be RGB MxNx3 image.
% All the networks we will support have RGB input sizes.

validateattributes(x, {'numeric'}, ...
    {'vector', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
    mfilename, 'imageSize');

n = numel(x);
if n < 2 || n > 3
    error(message('vision:semanticseg:imageSizeIncorrect'));
end

% Third dim must be 3 for supported networks because they only support RGB
% images.
if n == 3 && x(3) ~= 3
    error(message('vision:semanticseg:imageSizeThirdDimMustBeThree'));
end
end

%--------------------------------------------------------------------------
function validateDownsamplingFactor(x)
hadError = false;
try
    validateattributes(x, {'numeric'}, ...
        {'positive','scalar', 'real', 'finite', 'integer', 'nonsparse'}, ...
        mfilename, 'DownsamplingFactor');
catch
    hadError = true;
end
if hadError || ~(x == 8 || x == 16)
    error(message('vision:semanticseg:invalidDownsamplingFactor'));
end
end

%--------------------------------------------------------------------------
function iAssertImageSizeIsCompatibileWithNetwork(imageSize, network, networkName)

analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
imageLayerIdx = [analysis.LayerAnalyzers.IsInputLayer];
inputSize = network.Layers(imageLayerIdx).InputSize;

if any(imageSize(1:2) < inputSize(1:2))
    error(message('vision:semanticseg:imageSizeInvalidForNetwork',...
        mat2str(inputSize(1:2)),networkName));
end
end