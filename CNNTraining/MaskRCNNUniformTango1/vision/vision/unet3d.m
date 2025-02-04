function [network, outputSize] = unet3d(inputSize, numClasses, options)

% Copyright 2023 The MathWorks, Inc.

arguments
    inputSize {iValidateImageInput}
    numClasses {mustBeNumeric, mustBeScalarOrEmpty, mustBeNonempty, mustBeReal, mustBeFinite, mustBeInteger, mustBeNonsparse, mustBeGreaterThan(numClasses,1)}
    options.EncoderNetwork {iValidateInputEncoderNetwork} = []
    options.EncoderDepth {mustBeNumeric, mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeInteger, mustBeNonsparse, mustBePositive} = 3
    options.NumFirstEncoderFilters {mustBeNumeric, mustBeScalarOrEmpty, mustBeReal, mustBeFinite, mustBeInteger, mustBeNonsparse, mustBePositive} = 32
    options.FilterSize {iValidateFilterSize} = [3 3 3]
    options.ConvolutionPadding {mustBeNonempty, mustBeTextScalar, mustBeMember(options.ConvolutionPadding, ["same", "valid"])} = "same"
end

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

encoderDepth = options.EncoderDepth;
initialEncoderNumChannels = options.NumFirstEncoderFilters;

% Add channel dimension if only H, W, and D are given for input size.
if numel(inputSize) == 3
    % The number of image channels is 1 if only [H W D] vector is specified.
    inputSize = [inputSize 1];
end

% Set convolutional filter size.
if isscalar(options.FilterSize)
    convFilterSize = [options.FilterSize options.FilterSize options.FilterSize];
else
    convFilterSize = options.FilterSize;
end

% Set padding for convolution layers.
if strcmp(options.ConvolutionPadding, "same")
    convolutionPadding = options.ConvolutionPadding;
else
    convolutionPadding = [0 0 0 0 0 0];
end

if isempty(options.EncoderNetwork)
    % Validate the U-Net input configuration size.
    iValidateInputSize(inputSize, encoderDepth, convolutionPadding, convFilterSize);

    % Create encoder sub-network from given input parameters.
    [encoder, finalNumChannels] = iCreateEncoder(encoderDepth, ...
        convFilterSize, initialEncoderNumChannels, convolutionPadding);
else
    % encoder = pretrainedEncoderNetwork(options.EncoderNetwork, encoderDepth);
    encoder = options.EncoderNetwork;
    finalNumChannels = iFindPretrainedNumChannels(encoder);

    % Remove the image input layer if the number of channels do not match
    % with the given inputSize.
    if isa(encoder.Layers(1),"nnet.cnn.layer.Image3DInputLayer") && ...
            inputSize(3) ~= encoder.Layers(1).InputSize(3)
        encoder = removeLayers(encoder,encoder.Layers(1).Name);
        encoder = iUpdateConvChannels(encoder);
    end
end

% Create encoder-decoder bridge section of the network.
encoderDecoderBridge = iCreateEncoderDecoderBridge(convFilterSize, ...
    finalNumChannels, convolutionPadding);

% Initialize decoder sub-network parameters and create decoder sub-network.
initialDecoderNumChannels = finalNumChannels;

upConvFilterSize = 2;

% Create decoder sub-network from given input parameters.
decoder = iCreateDecoder(encoderDepth, upConvFilterSize,...
    convFilterSize, initialDecoderNumChannels, convolutionPadding);

% Create final network that will be appended at the output of the decoder.
finalNetwork = iCreateFinalNetwork();

% Find encoder skip connection outputs.
[encoder, encoderOutputNames] = iFindEncoderOutputs(encoder, inputSize);

% Gather decoder skip connection inputs.
decoderInputNames = iGatherDecoderInputs(decoder);

% Verify that encoder outputs match the number of decoder inputs for skip
% connections.
if length(decoderInputNames) ~= length(encoderOutputNames)
    error(message('vision:semanticseg:encoderDecoderMismatch', num2str(length(decoderInputNames))));
end

% Create encoder/decoder U-Net network.
network = encoderDecoderNetwork(inputSize,encoder,decoder, ...
   OutputChannels=numClasses,SkipConnections="concatenate", ...
   SkipConnectionNames=[encoderOutputNames decoderInputNames], ...
   LatentNetwork=encoderDecoderBridge,FinalNetwork=finalNetwork);

% Use network analyzer to calculate output size of the network.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
analysis.applyConstraints();
outputSize = analysis.LayerAnalyzers(end-2,1).Outputs.Size{1};
end

%--------------------------------------------------------------------------
function iValidateInputSize(inputSize, encoderDepth, ...
    convolutionPadding, convFilterSize)
% Validate the input image size. In "same" convolution settings, it should
% be divisible by 2^encoderDepth. In case of "valid" convolution settings,
% convolution layer size reduction value should be subtracted from image
% size to consider to be divisible by 2^encoderDepth.
inSize = inputSize;
sizeFactor = 2^encoderDepth;
errId = zeros([1,3]);
if strcmp(convolutionPadding, "same")
    % Convolution layer with "same" padding will not reduce feature map
    % size after convolution layer therefore encDownsamplingFactor = 0.
    encDownsamplingFactor = 0;
    for idx=1:3 %% For Height and width.
        [inputSize(idx),errId(idx)] = iValidateAndSuggestInputSize...
            (inputSize(idx), sizeFactor, encDownsamplingFactor);
    end
    errMessage = 'vision:semanticseg:imageSizeIncompatible3d';
else  %% "valid" convolution setting
    % The 3-D UNet paper imposes a constraint that input of each 2x2x2
    % max-pooling layer must be even in height, width, and depth dimensions.
    % This constraint can be satisfied in "valid" convolution settings by
    % using following things:
    % 1. Calculating the difference between the input size and the value of
    % size reduction caused by convolution layer and max-pooling layer.
    % 2. Checking that this difference is divisible by 2^encoderDepth.
    encDownsamplingFactor = (2^(encoderDepth+1)-2)*(convFilterSize-1);

    % In addition to encDownSamplingFactor, the max-pooling and convolution
    % layers further reduce the input size by a factor of
    % ((2^encoderDepth)*2*(convFilterSize-1)).
    finalEncDownsamplingFactor = (2^(encoderDepth+2)-2)*(convFilterSize-1);

    % Considering the decoder, the transposed convolution layers will
    % upsample the output. The overall network size reduction is
    % finalEncDownsamplingFactor + encDownsamplingFactor.
    encDecDownsamplingFactor = finalEncDownsamplingFactor + encDownsamplingFactor;

    % Validate the height and width of the input size.
    for idx=1:3
        [inputSize(idx),errId(idx)] = ...
            iValidateAndSuggestInputSizeForValidConv(inputSize(idx), sizeFactor,...
            encDownsamplingFactor(idx), encDecDownsamplingFactor(idx));
    end
    errMessage = 'vision:semanticseg:imageSizeIncompatibleValidConv';
end
if any(errId)
    error(message(errMessage, mat2str(inSize), mat2str(inputSize)));
end
end 

%--------------------------------------------------------------------------
function iValidateImageInput(x)
validateattributes(x, {'numeric'}, ...
    {'nonempty', 'real', 'finite', 'integer', 'positive','row'}, ...
    mfilename, 'inputSize');
isValidSize = isrow(x) && (numel(x)==3 || numel(x)==4);
if ~isValidSize
    error(message('vision:semanticseg:imageSizeIncorrect3d'));
end
end

%--------------------------------------------------------------------------
function iValidateInputEncoderNetwork(encoder)

if isempty(encoder)
    return
else
    validateattributes(encoder, {'dlnetwork'}, {'scalar'}, ...
        mfilename, 'EncoderNetwork');
end

end

%--------------------------------------------------------------------------
function iValidateFilterSize(x)
if isscalar(x)
    validateattributes(x, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
        mfilename, 'FilterSize');
else
    validateattributes(x, {'numeric'}, ...
        {'vector', 'real', 'finite', 'integer', 'nonsparse', 'positive',...
        'ncols', 3}, mfilename, 'FilterSize');
end
end

%--------------------------------------------------------------------------
function [modInputSize,errFlag] = iValidateAndSuggestInputSize(inputSize,sizeFactor,...
    encDownsamplingFactor)
% Function to validate and suggest new input size if given input size is
% not valid.
errFlag = 0;
% General constraint from paper to have feature map before each max-pooling
% layer to be even sized.
inputSizeCheck = rem(inputSize-encDownsamplingFactor, sizeFactor);
if inputSizeCheck
    errFlag = 1;
    % Input is smaller than excepted input i.e. 2^encoderDepth. For e.g.
    % 4x4x4 input with encoderDepth = 3, lead to modified input of 8x8x8.
    if any(inputSize < sizeFactor)
        modInputSize = sizeFactor;
    % Input size is not divisible perfectly, then based upon reminder
    % modify then input size.
    elseif (inputSizeCheck > (sizeFactor/2))
            modInputSize = inputSize+(sizeFactor-inputSizeCheck);
    else
          modInputSize = inputSize-inputSizeCheck;
    end
else
    modInputSize = inputSize;
end
end

%--------------------------------------------------------------------------
function [modInputSize,errFlag] = iValidateAndSuggestInputSizeForValidConv...
    (inputSize, sizeFactor, encDownsamplingFactor, encDecDownsamplingFactor)
% Function to validate and suggest new input size if given input size is
% not valid in case of "valid" convolution settings.

% Input size is smaller than excepted input i.e. encDecDownsamplingFactor.
% For e.g.16x16x16 input with encoderDepth = 1, lead to modified input of 
% 18x18x18.
encDepth = log2(sizeFactor);
minInputSize = iSuggestMinValidInputSize(encDepth, encDownsamplingFactor, ...
    encDecDownsamplingFactor);
if (inputSize-encDecDownsamplingFactor) <= 0 || (inputSize < minInputSize)
    errFlag = 1;
    modInputSize = minInputSize;
else
    [modInputSize, errFlag] = iValidateAndSuggestInputSize(inputSize, sizeFactor,...
        encDownsamplingFactor);
end
end

%--------------------------------------------------------------------------
function validInputSize = iSuggestMinValidInputSize(encoderDepth, ...
    encDownsamplingFactor, encDecDownsamplingFactor)
% Function to suggest nearest valid image size that is greater than 
% encDecDownsamplingFactor in steps of 2. Size greater than
% encDecDownsamplingFactor will be validated against encDownsamplingFactor
% to be a valid size.
val = 2;
while(~(rem(encDecDownsamplingFactor+val-encDownsamplingFactor, ...
        2^(encoderDepth))==0))
    val = val + 2;
end
validInputSize = encDecDownsamplingFactor + val;
end

%--------------------------------------------------------------------------
function [encoder, finalNumChannels] = iCreateEncoder(encoderDepth, ...
    convFilterSize, initialEncoderNumChannels, convolutionPadding)

encoderBlock = @(block) iCreateEncoderBlock(block, encoderDepth, ...
    convFilterSize, initialEncoderNumChannels, convolutionPadding);

encoder = blockedNetwork(encoderBlock,encoderDepth,NamePrefix="Encoder-Stage-");
finalNumChannels = 2*initialEncoderNumChannels * 2^(encoderDepth-1);

end

%--------------------------------------------------------------------------
function encoderBlock = iCreateEncoderBlock(block, encoderDepth, ...
    convFilterSize, initialEncoderNumChannels, convolutionPadding)

encoderNumChannels = initialEncoderNumChannels * 2^(block-1);

% Add convolutional and activation layers for each encoder block.
encoderBlock = [
        iCreateAndInitializeConv3dLayer(convFilterSize, encoderNumChannels, ...
        [num2str(block) '-Conv-1'], convolutionPadding);
        batchNormalizationLayer(Name=[num2str(block) '-BN-1']);
        reluLayer(Name=[num2str(block) '-ReLU-1']);
        iCreateAndInitializeConv3dLayer(convFilterSize, 2*encoderNumChannels, ...
        [num2str(block) '-Conv-2'], convolutionPadding);
        batchNormalizationLayer(Name=[num2str(block) '-BN-2']);
        reluLayer(Name=[num2str(block) '-ReLU-2'])];

% Add a dropout layer in the final encoder block.
if block == encoderDepth
    dropOutLayer = dropoutLayer(0.5,Name=[num2str(block) '-DropOut']);
    encoderBlock = [encoderBlock; dropOutLayer];
end

% Add a max pooling layer at the end of each encoder block.
encoderBlock = [encoderBlock; maxPooling3dLayer(2,Stride=2,Name=[num2str(block) '-MaxPool'])];

end

%--------------------------------------------------------------------------
function finalNumChannels = iFindPretrainedNumChannels(encoder)
    % The assumption is that each pretrained encoder has exactly one output.
    % Obtain input size of the encoder.
    if ~isa(encoder.Layers(1),"nnet.cnn.layer.Image3DInputLayer")
        error(message('vision:semanticseg:encoderImageInputMissing'));
    else
        inputSize = encoder.Layers(1).InputSize;
    end

    % Create a sample input to pass through the encoder.
    exampleInput = dlarray(zeros(inputSize),'SSSC');
    [sizes,formats] = deep.internal.sdk.forwardDataAttributes(encoder,exampleInput);

    % Find the position of the channels
    channelIdx = strfind(formats{1},'C');

    % Obtain the finalNumChannels from the sample forward pass.
    finalNumChannels = sizes{1}(channelIdx);
end

%--------------------------------------------------------------------------
function encoderDecoderBridge = iCreateEncoderDecoderBridge(...
    convFilterSize, finalNumChannels, convolutionPadding)

encoderDecoderBridge = [
    iCreateAndInitializeConv3dLayer(convFilterSize, ...
    finalNumChannels, "-Bridge-Conv-1", convolutionPadding);
    batchNormalizationLayer(Name="Bridge-BN-1");
    reluLayer(Name="-Bridge-ReLU-1");
    iCreateAndInitializeConv3dLayer(convFilterSize, ...
    2*finalNumChannels, "-Bridge-Conv-2", convolutionPadding);
    batchNormalizationLayer(Name="Bridge-BN-2");
    reluLayer(Name="-Bridge-ReLU-2");
    dropoutLayer(0.5,Name="-Bridge-DropOut")];

end

%--------------------------------------------------------------------------
function [decoder, finalDecoderNumChannels] = iCreateDecoder(...
    encoderDepth, upConvFilterSize, convFilterSize,...
    initialDecoderNumChannels, convolutionPadding)

decoderBlock = @(block) iCreateDecoderBlock(block, upConvFilterSize, ...
    convFilterSize, initialDecoderNumChannels, convolutionPadding);

decoder = blockedNetwork(decoderBlock,encoderDepth,NamePrefix="Decoder-Stage-");
finalDecoderNumChannels = initialDecoderNumChannels / 2^(encoderDepth-1);

end

%--------------------------------------------------------------------------
function decoderBlock = iCreateDecoderBlock(block, upConvFilterSize, ...
    convFilterSize, initialDecoderNumChannels, convolutionPadding)

decoderNumChannels = initialDecoderNumChannels / 2^(block-1);

% Add convolutional and activation layers for each encoder block.
decoderBlock = [
        iCreateAndInitializeUpConv3dLayer(upConvFilterSize, ...
            2*decoderNumChannels, [num2str(block) '-UpConv']);
        reluLayer(Name=[num2str(block) '-UpReLU']);
        iCreateAndInitializeConv3dLayer(convFilterSize, decoderNumChannels, ...
        [num2str(block) '-Conv-1'], convolutionPadding);
        batchNormalizationLayer(Name=[num2str(block) '-BN-1']);
        reluLayer(Name=[num2str(block) '-ReLU-1']);
        iCreateAndInitializeConv3dLayer(convFilterSize, decoderNumChannels, ...
        [num2str(block) '-Conv-2'], convolutionPadding);
        batchNormalizationLayer(Name=[num2str(block) '-BN-2']);
        reluLayer(Name=[num2str(block) '-ReLU-2'])];
end

%--------------------------------------------------------------------------
function encoder = iUpdateConvChannels(encoder)
% Find all convolutional layers in the encoder network.
idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.Convolution3DLayer'),encoder.Layers);  
convIdx = find(idx);

% Replace convolutional layers with ones with 'auto' number of channels.
for i = 1:length(convIdx)
    oldConvLayer = encoder.Layers(convIdx(i));
    convLayer = convolution3dLayer(oldConvLayer.FilterSize,...
        oldConvLayer.NumFilters, Padding=oldConvLayer.PaddingSize,...
        Stride=oldConvLayer.Stride, BiasL2Factor=0,...
        WeightsInitializer="he", BiasInitializer="zeros",...
        Name=oldConvLayer.Name);
    encoder = replaceLayer(encoder,oldConvLayer.Name,convLayer);
end

end

%--------------------------------------------------------------------------
function finalNetwork = iCreateFinalNetwork()

finalNetwork = [softmaxLayer(Name="Softmax-Layer")];

end

%--------------------------------------------------------------------------
function convLayer = iCreateAndInitializeConv3dLayer(convFilterSize,...
    outputNumChannels, layerName, convolutionPadding)

convLayer = convolution3dLayer(convFilterSize,outputNumChannels,...
    Padding=convolutionPadding, BiasL2Factor=0, WeightsInitializer="he",...
    BiasInitializer="zeros", Name=layerName);

end

%--------------------------------------------------------------------------
function upConvLayer = iCreateAndInitializeUpConv3dLayer(UpconvFilterSize,...
    outputNumChannels, layerName)

upConvLayer = transposedConv3dLayer(UpconvFilterSize, outputNumChannels,...
    Stride=2, BiasL2Factor=0, WeightsInitializer="he", ...
    BiasInitializer="zeros", Name=layerName);

upConvLayer.BiasLearnRateFactor = 2;
end

%--------------------------------------------------------------------------
function [encoder, encoderOutputNames] = iFindEncoderOutputs(encoder, inputSize)
% Append an input layer, if needed, to the encoder in order to run a dummy
% input through the encoder sub-network and find its outputs.
if ~isa(encoder.Layers(1),"nnet.cnn.layer.Image3DInputLayer")
    inputLayer = image3dInputLayer(inputSize,Name="encoderImageInputLayer");
    encoder = addInputLayer(encoder,inputLayer);
end

encoder = initialize(encoder);

% Create a dummy input to process the encoder network dlarray sizes.
dims = repmat('S',1,numel(inputSize)-1);
dims = [dims,'C'];
dummyEncoderInput = dlarray(zeros(inputSize),dims);

downsampleNames = string.empty();
numLayers = numel(encoder.Layers);
for idx = 1:numLayers
    layer = encoder.Layers(idx);
    if isprop(layer,'Stride') && all(~mod(layer.Stride,2))
        downsampleNames(end+1) = encoder.Layers(idx).Name; %#ok<AGROW>
    end
end

destNames = string(encoder.Connections.Destination);
sourceNames = string(encoder.Connections.Source);
outputNames = string.empty();
for idx = 1:length(downsampleNames)
    downsampleLayerName = downsampleNames(idx);
    sourceIndex = destNames == downsampleLayerName;
    sourceName = sourceNames(sourceIndex);
    outputNames(end+1) = sourceName; %#ok<AGROW>
end

outputNames = unique(outputNames,'stable');
outputNames = iRemoveNamesWithDuplicateSpatialSizes(outputNames,dummyEncoderInput,encoder);
encoderOutputNames = outputNames';

end

%--------------------------------------------------------------------------
function decoderInputNames = iGatherDecoderInputs(decoder)
decoderLayerNames = string({decoder.Layers.Name})';
decoderInputIdx = endsWith(decoderLayerNames,"-UpReLU");
decoderInputNames = flip(decoderLayerNames(decoderInputIdx),1);
end

%--------------------------------------------------------------------------
function outputNames = iRemoveNamesWithDuplicateSpatialSizes(outputNames,dummyEncoderInput,encoder)

% If multiple downsampling layers result in the same activation size, take
% the first one in topological order to handle cases like inceptionv3 where
% downsampling is performed within inception blocks.
if ~isempty(outputNames)
    sampleSkipOutputs = cell(1,length(outputNames));
    [sampleSkipOutputs{:}] = forward(encoder,dummyEncoderInput,'Outputs',outputNames);
    sizes = cellfun(@(x) size(x),sampleSkipOutputs,'UniformOutput',false);
    spatialSizes = cat(1,sizes{:});
    spatialSizes = spatialSizes(:,1:ndims(dummyEncoderInput)-1);
    duplicateSizes = iFindDuplicateSizes(spatialSizes);

    for idx = 1:size(duplicateSizes,1)
        sz = duplicateSizes(idx,:);
        outputsWithDuplicateSize = all(spatialSizes == sz,2);
        duplicateToKeep =  find(outputsWithDuplicateSize,1,'first');
        duplicatesToRemove = outputsWithDuplicateSize;
        duplicatesToRemove(duplicateToKeep) = false;
        outputNames(duplicatesToRemove) = [];
    end
end

end

%--------------------------------------------------------------------------
function duplicateSizes = iFindDuplicateSizes(x)

[~,I] = unique(x, 'rows', 'first');
ixDupRows = setdiff(1:size(x,1), I);
dupRowValues = x(ixDupRows,:);
duplicateSizes = unique(dupRowValues);

end