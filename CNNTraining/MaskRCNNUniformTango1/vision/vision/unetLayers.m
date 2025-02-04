%unetLayers Create U-Net for semantic segmentation using deep learning.
%
%--------------------------------------------------------------------------
%   unetLayers will be removed in a future release. Use unet instead.
%--------------------------------------------------------------------------
%
%   U-Net is a convolutional neural network for semantic image
%   segmentation. It uses a pixelClassificationLayer to predict the
%   categorical label for every pixel in an input image. The network gets
%   its name from the "U" shape created when the layers are arranged in
%   order.
%
%   Use unetLayers to create the network architecture for U-Net. This
%   network must be trained using trainNetwork from Deep Learning Toolbox
%   before it can be used for semantic segmentation.
%
%   lgraph = unetLayers(inputSize, numClasses) returns U-Net layers
%   configured using the following inputs:
%
%   Inputs
%   ------
%   inputSize    - size of the network input image specified as a vector
%                  [H W] or [H W C], where H and W are the image height
%                  and width, and C is the number of image channels. The
%                  number of image channels is 1 if only [H W] vector is
%                  specified.
%
%   numClasses   - number of classes the network should be configured to
%                  classify.
%
%   [lgraph, outputSize] = unetLayers(...) also returns network's output
%   image size as 1-by-3 vector. It consist of height, width, and number of
%   channels. outputSize can be used to configure the datastores during
%   training.
%
%   [...] = unetLayers(inputSize, numClasses, Name, Value) specifies
%   additional name-value pair arguments described below:
%
%   'EncoderDepth'                 U-Net is composed of an encoder
%                                  sub-network and a corresponding decoder
%                                  sub-network. Specify the depth of these
%                                  networks as a scalar D. The depth of
%                                  these networks determines the number of
%                                  times an input image is downsampled or
%                                  upsampled as it is processed. The
%                                  encoder network downsamples the input
%                                  image by a factor of 2^D. The decoder
%                                  network performs the opposite operation
%                                  and upsamples the encoder network output
%                                  by a factor of 2^D. Typical depth of the
%                                  encoder sub-network is 4.
%
%                                  Default: 4
%
%   'NumFirstEncoderFilters'       Specify the number of output channels
%                                  for the first encoder subsection. Each
%                                  of the subsequent encoder subsections
%                                  double the number of output channels.
%                                  The number of output channels in the
%                                  decoder sections is automatically set to
%                                  match the corresponding encoder section.
%
%                                  Default: 64
%
%   'FilterSize'                   Specify the height and width used for
%                                  all convolution layer filters as a
%                                  scalar or vector [H W]. When the size is
%                                  a scalar, the same value is used for H
%                                  and W. Typical values are between 3 and
%                                  7. The value must be odd.
%
%                                  Default: 3
%
%   'ConvolutionPadding'           Specify the padding style of the
%                                  convolution2dLayer in both encoder and
%                                  decoder. If specified as 'valid' then
%                                  valid convolution is performed and
%                                  output feature map size is less than the
%                                  input feature map size. If specified as
%                                  'same' then convolution operation is
%                                  performed such that output feature map
%                                  size is same as input feature map size.
%
%                                  Default: 'same'
%
% Notes
% -----
% - This version of U-Net supports both "same" and "valid" padding for the 
%   convolutional layers to enable a broader set of input image sizes.
%
% - The sections within the U-Net encoder sub-network are made up of two 
%   sets of convolutional and ReLU layers followed by a 2x2 max-pooling 
%   layer. While the sections of the decoder sub-network are made
%   up of transposed convolution layers (for upsampling) followed by two 
%   sets of convolutional and ReLU layers.
%
% - Convolution layer weights in the encoder and decoder sub-networks are
%   initialized using the 'He' weight initialization method. All bias terms
%   are initialized to zero.
%
% - Input size of the network must be selected such that the dimension of 
%   input to each 2x2 max-pooling layer must be even. 
%
% - For seamless segmentation of large images, use the patch-based 
%   approach. randomPatchExtractionDatastore can be used for extracting 
%   image patches. 
%   
% - Consider using "valid" convolution padding option in order to avoid 
%   border artifacts when using a patch based approach.
%
%   Example 1 - Create U-Net with custom encoder/decoder depth.
%   ------------------------------------------------------------
%   % Create U-Net layers with an encoder/decoder depth of 3.
%   inputSize = [480 640 3];
%   numClasses = 5;
%   encoderDepth = 3;
%   lgraph = unetLayers(inputSize, numClasses, 'EncoderDepth',...
%   encoderDepth)
%
%   % Display network.
%   analyzeNetwork(lgraph)
%
%   Example 2 - Create U-Net with "valid" convolution layers.
%   ------------------------------------------------------------
%   % Create U-Net layers with an encoder/decoder depth of 4.
%   inputSize = [572 572 3];
%   numClasses = 5;
%   encoderDepth = 4;
%   lgraph = unetLayers(inputSize, numClasses, 'EncoderDepth', ...
%   encoderDepth, 'ConvolutionPadding','valid');
%
%   % Display network.
%   analyzeNetwork(lgraph)
%
%   Example 3 - Train U-Net.
%   -------------------------
%   % Load training images and pixel labels.
%   dataSetDir = fullfile(toolboxdir('vision'),'visiondata','triangleImages');
%   imageDir = fullfile(dataSetDir, 'trainingImages');
%   labelDir = fullfile(dataSetDir, 'trainingLabels');
%
%   % Create an imageDatastore holding the training images.
%   imds = imageDatastore(imageDir);
%
%   % Define the class names and their associated label IDs.
%   classNames = ["triangle", "background"];
%   labelIDs   = [255 0];
%
%   % Create a pixelLabelDatastore holding the ground truth pixel labels
%   % for the training images.
%   pxds = pixelLabelDatastore(labelDir, classNames, labelIDs);
%
%   % Create U-Net.
%   inputSize = [32 32];
%   numClasses = 2;
%   [lgraph,~] = unetLayers(inputSize, numClasses)
%
%   % Combine image and pixel label data to train a semantic segmentation
%   % network.
%   ds = combine(imds,pxds);
%
%   % Setup training options.
%   options = trainingOptions('sgdm', 'InitialLearnRate', 1e-3, ...
%       'MaxEpochs', 20, 'VerboseFrequency', 10);
%
%   % Train network.
%   net = trainNetwork(ds, lgraph, options)
%
% See also unet3dLayers, segnetLayers, fcnLayers, vgg16, vgg19,
%          pixelClassificationLayer, LayerGraph, trainNetwork,
%          DAGNetwork, semanticseg, randomPatchExtractionDatastore.

% References
% ----------
%
% [1] Olaf Ronneberger, Philipp Fischer, and Thomas Brox, U-Net:
%     Convolutional Networks for Biomedical Image Segmentation, Medical
%     Image Computing and Computer-Assisted Intervention (MICCAI),
%     Springer, LNCS, Vol.9351: 234--241, 2015, available at
%     arXiv:1505.04597
%
% [2] He, Kaiming, et al. "Delving deep into rectifiers: Surpassing
%     human-level performance on imagenet classification." Proceedings of
%     the IEEE international conference on computer vision. 2015.

% Copyright 2017-2024 The MathWorks, Inc.

function [lgraph, outputSize] = unetLayers(inputSize, numClasses, varargin)

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

warning(message('vision:semanticseg:unetLayersDeprecation'));

narginchk(2,10);

args = iParseInputs(inputSize, numClasses, varargin{:});

encoderDepth = args.EncoderDepth;
initialEncoderNumChannels = args.NumFirstEncoderFilters;
inputTileSize = args.inputSize;
convFilterSize = args.FilterSize;
convolutionPadding = args.ConvolutionPadding;

% Create image input layer with default parameters.
inputlayer = imageInputLayer(inputTileSize,'Name','ImageInputLayer');

% Create encoder sub-network from given input parameters.
[encoder, finalNumChannels] = iCreateEncoder(encoderDepth, ...
    convFilterSize, initialEncoderNumChannels, convolutionPadding);

% Create encoder-decoder bridge section of the network.
firstConv = iCreateAndInitializeConvLayer(convFilterSize, ...
    2*finalNumChannels, 'Bridge-Conv-1', convolutionPadding);
firstReLU = reluLayer('Name','Bridge-ReLU-1');

secondConv = iCreateAndInitializeConvLayer(convFilterSize, ...
    2*finalNumChannels, 'Bridge-Conv-2', convolutionPadding);
secondReLU = reluLayer('Name','Bridge-ReLU-2');

encoderDecoderBridge = [firstConv; firstReLU; secondConv; secondReLU];

dropOutLayer = dropoutLayer(0.5,'Name','Bridge-DropOut');
encoderDecoderBridge = [encoderDecoderBridge; dropOutLayer];

% Initialize decoder sub-network parameters and create decoder sub-network.
initialDecoderNumChannels = finalNumChannels;

upConvFilterSize = 2;

decoder = iCreateDecoder(encoderDepth, upConvFilterSize,...
    convFilterSize, initialDecoderNumChannels, convolutionPadding);

% Connect input, encoder, bridge and decoder sub-networks. 
layers = [inputlayer; encoder; encoderDecoderBridge; decoder];

numClasses = args.numClasses;
finalConv = convolution2dLayer(1,numClasses,...
    'BiasL2Factor',0,...
    'Name','Final-ConvolutionLayer', 'Padding', convolutionPadding,...
    'WeightsInitializer', 'he', 'BiasInitializer','zeros');


smLayer = softmaxLayer('Name','Softmax-Layer');

% Create default pixel classification layer. 

pixelClassLayer = pixelClassificationLayer('Name','Segmentation-Layer');

layers = [layers; finalConv; smLayer; pixelClassLayer];

lgraph = layerGraph(layers);

lgraph = iConnectLgraph(lgraph,convolutionPadding, encoderDepth);

% Use network analyzer to calculate output size of the network.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
analysis.applyConstraints();
outputSize = analysis.LayerAnalyzers(end-2,1).Outputs.Size{1};
end

%--------------------------------------------------------------------------
function args = iParseInputs(varargin)

p = inputParser;
p.addRequired('inputSize', @iCheckInputSize);
p.addRequired('numClasses', @iCheckNumClasses);
p.addParameter('FilterSize', 3, @iCheckFilterSize);
p.addParameter('EncoderDepth', 4, @iCheckEncoderDepth);
p.addParameter('NumOutputChannels', '', @iCheckNumFirstEncoderFilters);
p.addParameter('NumFirstEncoderFilters', 64, ...
    @iCheckNumFirstEncoderFilters);
p.addParameter('ConvolutionPadding', 'same', @iCheckConvolutionPadding);

p.parse(varargin{:});

userInput = p.Results;

inputSize  = double(userInput.inputSize);
args.inputSize = inputSize;
args.numClasses = double(userInput.numClasses);

if isscalar(userInput.FilterSize)
    args.FilterSize = [double(userInput.FilterSize) ...
        double(userInput.FilterSize)];
else
    args.FilterSize = double(userInput.FilterSize);
end

args.EncoderDepth = double(userInput.EncoderDepth);

% To avoid breaking backward compatibility we are silently supporting
% NumOuputChannels NV pair.
if isempty(userInput.NumOutputChannels)
    args.NumFirstEncoderFilters = ...
        double(userInput.NumFirstEncoderFilters);
else
    args.NumFirstEncoderFilters = double(userInput.NumOutputChannels);
end

% Mapping of external padding parameter to that of convolution2dLayer
% parameters.
if ~any(strcmp(userInput.ConvolutionPadding,{'same','valid'}))
    userInput.ConvolutionPadding = iValidateConvPaddingPartial(...
        userInput.ConvolutionPadding);
end
if strcmp(userInput.ConvolutionPadding , 'same')
    args.ConvolutionPadding = char(userInput.ConvolutionPadding);
else
    args.ConvolutionPadding = 0;
end

% Validate the input image size. In "same" convolution settings, it should
% be divisible by 2^encoderDepth. In case of "valid" convolution settings,
% convolution layer size reduction value should be subtracted from image
% size to consider to be divisible by 2^encoderDepth.
sizeFactor = 2^args.EncoderDepth;
errId = zeros([1,2]);
if strcmp(args.ConvolutionPadding, 'same') %% "same" convolution setting
    % Convolution layer with "same" padding will not reduce feature map 
    % size after convolution layer therefore encDownsamplingFactor = 0.
    encDownsamplingFactor = 0;
    for idx=1:2 %% For Height and width.
        [args.inputSize(idx),errId(idx)] = iValidateAndSuggestInputSize...
            (args.inputSize(idx), sizeFactor, encDownsamplingFactor);
    end
    errMessage = 'vision:semanticseg:imageSizeIncompatible';
else  %% "valid" convolution setting
    % The UNet paper imposes constraint that input of each 2x2 max-pooling
    % layer must be even in height, width, and depth dimensions. This
    % constraint can be satisfied in "valid" convolution settings by using
    % following things:
    % 1. Calculating the difference between the input size and the value of
    % size reduction caused by convolution layer and max-pooling layer.   
    % 2. Checking that this difference is divisible by 2^args.EncoderDepth.
    % The value of size reduction caused by convolution layer and
    % max-pooling layer together is the encDownSamplingFactor. The "enc"
    % prefix in the name because max-pooling operation happens only in the
    % encoder stage. In Encoder, each convolution layer with "valid"
    % padding (padding=0) will reduce its input feature map size by factor
    % of (args.FilterSize-1), since stride is set to 1. Also there are
    % (args.EncoderDepth) number of max-pooling layers, each of which will
    % reduce its output feature map size by 2 in height, and width.
    % Therefore, encDownsamplingFactor value is compounding effect of these
    % size reductions. Following are few cases for various encoderDepth:
    % (args.EncoderDepth=1), encDownsamplingFactor=2*(args.FilterSize-1) 
    % (args.EncoderDepth=2), encDownsamplingFactor=6*(args.FilterSize-1)
    % (args.EncoderDepth=3), encDownsamplingFactor=14*(args.FilterSize-1)
    % After generalizing, (2^(args.EncoderDepth+1)-2)*(args.FilterSize-1).
    % For e.g.: If args.InputSize = [18 18], args.EncoderDepth = 1,
    % args.FilterSize = 3, then, in encoder there will be only one
    % max-pooling layer. And size of input feature map of max-pooling layer
    % is [18 18] - (2^(2)-2)*2 = [14 14], which is even and divisible by
    % 2^args.EncoderDepth = 2^1 = 2, therefore we consider [18 18] as valid
    % size, and UNet network can be created using these parameters.
    encDownsamplingFactor = ...
        (2^(args.EncoderDepth+1)-2)*(args.FilterSize-1);
    
    % In addition to encDownSamplingFactor, we have the max-pooling and
    % convolution layers that further reduces the input size by a factor of
    % ((2^args.EncoderDepth)*2*(args.FilterSize-1)). The
    % finalEncDownsamplingFactor = encDownsamplingFactor -
    % (2^args.EncoderDepth)*2*(args.FilterSize-1). For e.g. 
    % finalEncDownsamplingFactor = (2^3-2)*2 = 12.
    finalEncDownsamplingFactor = ...
        (2^(args.EncoderDepth+2)-2)*(args.FilterSize-1);
    % The output of encoder will be then, (args.inputSize -
    % finalEncDownsamplingFactor)./2^args.EncoderDepth. For e.g. ([18 18] -
    % 12)./2^1  = [6 6]./2^1 = [3 3]. (ignoring the number of channels, as
    % it is independant of size reduction). Considering decoder, transposed
    % convolution layers will upsample output of encoder by
    % 2^args.EncoderDepth. 2^args.EncoderDepth*((args.inputSize -
    % finalEncDownsamplingFactor) ./2^args.EncoderDepth) = (args.inputSize
    % - finalEncDownsamplingFactor) For e.g. [18 18] - 12 = [6 6]. Followed
    % by convolution layers that will reduce the size by factor of
    % encDownsamplingFactor, since convolution layers are similar in
    % encoder and decoder except last encoder section(not considered while
    % calculating encDownsamplingFactor). For e.g. [6 6] - 4 = [2 2]. The
    % overall, network size reduction is, encDecDownsamplingFactor =
    % finalEncDownsamplingFactor + encDownsamplingFactor. For e.g.
    % encDecDownsamplingFactor = 12 + 4 = 16. So. the 3-D UNet network will
    % reduce the size of input by 16. [18 18]-16 = [2 2]. The [2 2] is
    % output size of the network for input size of [18 18].
    encDecDownsamplingFactor = finalEncDownsamplingFactor + ...
        encDownsamplingFactor;
    
    for idx=1:2 %% For Height and width.
        [args.inputSize(idx),errId(idx)] = ...
            iValidateAndSuggestInputSizeForValidConv(args.inputSize(idx), sizeFactor,...
            encDownsamplingFactor(idx), encDecDownsamplingFactor(idx));
    end
    errMessage = 'vision:semanticseg:imageSizeIncompatibleValidConv';
end
if any(errId)
    error(message(errMessage, mat2str(inputSize), mat2str(args.inputSize)));
end
end 
    
%--------------------------------------------------------------------------
function iCheckInputSize(x)
validateattributes(x, {'numeric'}, ...
    {'vector', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
    mfilename, 'inputSize');

N = numel(x);
if ~(N == 2 || N == 3)
    error(message('vision:semanticseg:imageSizeIncorrect'));
end
end

%--------------------------------------------------------------------------
function iCheckFilterSize(x)
% require odd filter sizes to facilitate "same" output size padding. In the
% future this can be relaxed with asymmetric padding.
if isscalar(x)
    validateattributes(x, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'integer', 'nonsparse', 'positive', 'odd'}, ...
        mfilename, 'FilterSize');
else
    validateattributes(x, {'numeric'}, ...
        {'vector', 'real', 'finite', 'integer', 'nonsparse', 'positive', 'odd', ...
        'ncols', 2}, mfilename, 'FilterSize');
end
end

%--------------------------------------------------------------------------
function iCheckNumClasses(x)
validateattributes(x, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonsparse', '>', 1}, ...
    mfilename, 'numClasses');
end

%--------------------------------------------------------------------------
function iCheckNumFirstEncoderFilters(x)
validateattributes(x, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
    mfilename, 'NumOutputChannels');
end

%--------------------------------------------------------------------------
function iCheckEncoderDepth(x)
validateattributes(x, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
    mfilename, 'EncoderDepth');
end

%--------------------------------------------------------------------------
function iCheckConvolutionPadding(x)
validateattributes(x, {'char', 'string'}, {'nonempty'}, ...
    mfilename, 'ConvolutionPadding');
end

%--------------------------------------------------------------------------
function convPad = iValidateConvPaddingPartial(x)
validStrings = ["valid", "same"];
convPad = validatestring(x, validStrings, mfilename, 'ConvolutionPadding');
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
    % 4x4 input with encoderDepth = 3, lead to modified input of 8x8.
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
    (inputSize, sizeFactor, encDownsamplingFactor,...
    encDecDownsamplingFactor)
% Function to validate and suggest new input size if given input size is
% not valid in case of "valid" convolution settings.

% Input size is smaller than excepted input i.e. encDecDownsamplingFactor.
% For e.g.16x16 input with encoderDepth = 1, lead to modified input of 
% 18x18.
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

encoder = [];
for stage = 1:encoderDepth
    % Double the layer number of channels at each stage of the encoder.
    encoderNumChannels = initialEncoderNumChannels * 2^(stage-1);

    firstConv = iCreateAndInitializeConvLayer(convFilterSize,...
        encoderNumChannels, ['Encoder-Stage-' num2str(stage) ...
        '-Conv-1'], convolutionPadding);

    firstReLU = reluLayer('Name',['Encoder-Stage-' ...
        num2str(stage) '-ReLU-1']);
    
    secondConv = iCreateAndInitializeConvLayer(convFilterSize,...
        encoderNumChannels, ['Encoder-Stage-' num2str(stage) ...
        '-Conv-2'], convolutionPadding);
    secondReLU = reluLayer('Name',['Encoder-Stage-' ...
        num2str(stage) '-ReLU-2']);
    
    encoder = [encoder;firstConv; firstReLU; secondConv; secondReLU];
    
    if stage == encoderDepth
        dropOutLayer = dropoutLayer(0.5,'Name',...
            ['Encoder-Stage-' num2str(stage) '-DropOut']);
        encoder = [encoder; dropOutLayer];
    end
    
    maxPoolLayer = maxPooling2dLayer(2, 'Stride', 2, 'Name',...
        ['Encoder-Stage-' num2str(stage) '-MaxPool']);
    
    encoder = [encoder; maxPoolLayer];
end
finalNumChannels = encoderNumChannels;
end

%--------------------------------------------------------------------------
function [decoder, finalDecoderNumChannels] = iCreateDecoder(...
    encoderDepth, upConvFilterSize, convFilterSize,...
    initialDecoderNumChannels, convolutionPadding)

decoder = [];
for stage = 1:encoderDepth
    % Half the layer number of channels at each stage of the decoder.
    decoderNumChannels = initialDecoderNumChannels / 2^(stage-1);
    
    upConv = iCreateAndInitializeUpConvLayer(upConvFilterSize, ...
        decoderNumChannels, ['Decoder-Stage-' num2str(stage) '-UpConv']);
    upReLU = reluLayer('Name',['Decoder-Stage-' num2str(stage) '-UpReLU']);
    
    % Input feature channels are concatenated with deconvolved features
    % within the decoder.
    depthConcatLayer = depthConcatenationLayer(2, 'Name', ...
        ['Decoder-Stage-' num2str(stage) '-DepthConcatenation']);
    
    firstConv = iCreateAndInitializeConvLayer(convFilterSize, ...
        decoderNumChannels, ['Decoder-Stage-' num2str(stage) ...
        '-Conv-1'], convolutionPadding);
    firstReLU = reluLayer('Name',['Decoder-Stage-' ...
        num2str(stage) '-ReLU-1']);
    
    secondConv = iCreateAndInitializeConvLayer(convFilterSize,...
        decoderNumChannels, ['Decoder-Stage-' num2str(stage)...
        '-Conv-2'], convolutionPadding);
    secondReLU = reluLayer('Name',['Decoder-Stage-' num2str(stage) ...
        '-ReLU-2']);
    
    decoder = [decoder; upConv; upReLU; depthConcatLayer;...
        firstConv; firstReLU; secondConv; secondReLU];
end
finalDecoderNumChannels = decoderNumChannels;
end

%--------------------------------------------------------------------------
function convLayer = iCreateAndInitializeConvLayer(convFilterSize,...
    outputNumChannels, layerName, convolutionPadding)

convLayer = convolution2dLayer(convFilterSize,outputNumChannels,...
    'Padding', convolutionPadding ,'BiasL2Factor',0, ...
    'WeightsInitializer','he',...
    'BiasInitializer','zeros', ...
    'Name',layerName);
end

%--------------------------------------------------------------------------
function upConvLayer = iCreateAndInitializeUpConvLayer(UpconvFilterSize,...
    outputNumChannels, layerName)

upConvLayer = transposedConv2dLayer(UpconvFilterSize, outputNumChannels,...
    'Stride',2, 'BiasL2Factor', 0, 'WeightsInitializer', 'he', ...
    'BiasInitializer', 'zeros', 'Name', layerName);

upConvLayer.BiasLearnRateFactor = 2;
end

%--------------------------------------------------------------------------
function lgraph = iConnectLgraph(lgraph, convolutionPadding, encoderDepth)
for depth = 1:encoderDepth
    if strcmp(convolutionPadding,'same')
        startLayer = sprintf('Encoder-Stage-%d-ReLU-2',depth);
        endLayer = sprintf('Decoder-Stage-%d-DepthConcatenation/in2',...
            encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph,startLayer, endLayer);
    else
        crop = crop2dLayer('centercrop', 'Name', ['Crop2d-',...
            num2str(encoderDepth-depth + 1)]);
        lgraph = addLayers(lgraph,crop);
        startLayer = sprintf('Encoder-Stage-%d-ReLU-2',depth);
        endLayer = sprintf('Crop2d-%d/in',encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph,startLayer, endLayer);
        startLayer2 = sprintf('Decoder-Stage-%d-UpConv',...
            encoderDepth-depth + 1);
        endLayer2 = sprintf('Crop2d-%d/ref',encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph,startLayer2, endLayer2);
        startLayer3 = sprintf('Crop2d-%d',encoderDepth-depth + 1);
        endLayer3 = sprintf('Decoder-Stage-%d-DepthConcatenation/in2',...
            encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph, startLayer3, endLayer3);
    end
end
end
