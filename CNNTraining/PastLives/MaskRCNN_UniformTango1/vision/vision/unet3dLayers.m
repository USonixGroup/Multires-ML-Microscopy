function [lgraph, outputSize] = unet3dLayers(inputSize,numClasses,varargin)
%unet3dLayers Create U-Net for 3-D semantic segmentation using deep learning.
%
%--------------------------------------------------------------------------
%   unet3dLayers will be removed in a future release. Use unet3d instead.
%--------------------------------------------------------------------------
%
%   3-D U-Net is a convolutional neural network for 3-D semantic image
%   segmentation. It uses a pixelClassificationLayer to predict the
%   categorical label for every pixel in an input 3-D image/volume. The
%   network gets its name from the "U" shape created when the layers are
%   arranged in order.
%
%   Use unet3dLayers to create the network architecture for 3-D U-Net. This
%   network must be trained using the trainNetwork function in Deep
%   Learning Toolbox before it can be used for 3-D semantic segmentation.
%
%   lgraph = unet3dLayers(inputSize, numClasses) returns 3-D U-Net layers
%   configured using the following inputs:
%
%   Inputs 
%   ------ 
%   inputSize    - size of the network input image specified as a vector
%                  [H W D] or [H W D C], where H, W and D are the image
%                  height, width, and depth and C is the number of image
%                  channels. The number of image channels is 1 if only [H W
%                  D] vector is specified.
%
%   numClasses   - number of classes the network should be configured to
%                  classify.
%
%   [lgraph, outputSize] = unet3dLayers(...) also returns network's output
%   image size as 1-by-4 vector. It consist of height, width, depth and
%   number of channels. outputSize can be used to configure the datastores
%   during training.
%
%   [...] = unet3dLayers(inputSize, numClasses, Name, Value)
%   specifies additional name-value pair arguments described below:
%
%   'EncoderDepth'                 3-D U-Net is composed of an encoder
%                                  sub-network and a corresponding decoder
%                                  sub-network. Specify the depth of these
%                                  networks as a scalar D. The depth of
%                                  these networks determines the number of
%                                  times an input image is downsampled or
%                                  upsampled as it is processed. Typical
%                                  depth of the encoder sub-network is 3.
%
%                                  Default: 3
%
%   'NumFirstEncoderFilters'       Specify the number of output channels
%                                  for the first encoder subsection. Each
%                                  of the subsequent encoder subsections
%                                  double the number of output channels.
%                                  The number of output channels in the
%                                  decoder sections is automatically set to
%                                  match the corresponding encoder section.
%
%                                  Default: 32
%
%   'FilterSize'                   Specify the height, width and depth used
%                                  for all convolutional layer filters as a
%                                  scalar or vector [H W D]. When the size
%                                  is a scalar, the same value is used for
%                                  H, W and D. Typical values are between 3
%                                  and 7.
%
%                                  Default: 3
%
%   'ConvolutionPadding'           Specify the padding style of the
%                                  convolution3dLayer in both encoder and
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
% - This version of 3-D U-Net supports both "same" and "valid" padding for 
%   the convolutional layers to enable a broader set of input image sizes. 
%
% - The sections within the 3-D U-Net encoder sub-network are
%   made up of two sets of 3-D convolutional, batch normalization and ReLU
%   layers followed by a 2x2x2 3-D max-pooling layer. While the sections of
%   the decoder sub-network are made up of 3-D transposed convolution 
%   layers(for upsampling) followed by two sets of convolutional, batch
%   normalization and ReLU layers.
%
% - Convolution 3-D layer weights in the encoder and decoder sub-networks
%   are initialized using the 'He' weight initialization method. All bias 
%   terms are initialized to zero.
%   
% - Input size of the network must be selected such that the dimension of 
%   input to each 2x2x2 max-pooling layer must be even. 
%
% - For seamless segmentation of large images, use the patch-based 
%   approach. randomPatchExtractionDatastore can be used for extracting 
%   image patches. 
%   
% - Consider using "valid" convolution padding option in order to avoid 
%   border artifacts when using a patch based approach.
%
%   % Example 1 - Create standard 3-D U-Net network. 
%   % ------------------------------------------------------------ 
%   inputSize = [136 136 120 3]; 
%   numClasses = 2; 
%   lgraph = unet3dLayers(inputSize,numClasses);
%
%   % Visualize the created network using network analyzer
%   analyzeNetwork(lgraph)
%
%   % Example 2 - Create 3-D U-Net network with "valid" convolution 
%   % settings. 
%   % ------------------------------------------------------------ 
%   inputSize = [132 132 116 3]; 
%   numClasses = 2; 
%   [lgraph, outputSize] = unet3dLayers(inputSize, numClasses, ...
%                              'ConvolutionPadding', 'valid');
%
%   % Display the network using network analyzer.
%   analyzeNetwork(lgraph)
%
% See also unetLayers, segnetLayers, fcnLayers, deeplabv3plusLayers, 
%          pixelClassificationLayer, LayerGraph, trainNetwork, DAGNetwork, 
%          semanticseg, pixelLabelDatastore, imageDatastore,
%          randomPatchExtractionDatastore.

% References 
% ----------
% 
% [1] Ronneberger, Olaf et al. 3-D U-Net: Learning Dense Volumetric
%     Segmentation from Sparse Annotation, Medical Image Computing and
%     Computer-Assisted Intervention (MICCAI), Springer, LNCS, 2016,
%     available at arXiv:1606.06650.
%
% [2] Olaf Ronneberger, Philipp Fischer, and Thomas Brox, U-Net:
%     Convolutional Networks for Biomedical Image Segmentation, Medical
%     Image Computing and Computer-Assisted Intervention (MICCAI),
%     Springer, LNCS, Vol.9351: 234--241, 2015, available at
%     arXiv:1505.04597

% Copyright 2019-2024 The MathWorks, Inc.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

warning(message('vision:semanticseg:unet3dLayersDeprecation'));

narginchk(2,10);

args = iParseInputs(inputSize, numClasses, varargin{:});

encoderDepth = args.EncoderDepth;
initialEncoderNumChannels = args.NumFirstEncoderFilters;
inputTileSize = args.inputSize;
convFilterSize = args.FilterSize;
convolutionPadding = args.ConvolutionPadding;

% Create 3-D image input layer with default parameters. 
inputlayer = image3dInputLayer(inputTileSize,'Name','ImageInputLayer');

% Create encoder sub-network from given input parameters.
[encoder, finalNumChannels] = iCreateEncoder(encoderDepth, convFilterSize,...
                            initialEncoderNumChannels, convolutionPadding);

% Create encoder-decoder bridge section of the network.
firstConv = iCreateAndInitializeConv3dLayer(convFilterSize, ...
    finalNumChannels, convolutionPadding , 'Bridge-Conv-1');
firstBN = batchNormalizationLayer('name', 'Bridge-BN-1');
firstReLU = reluLayer('Name','Bridge-ReLU-1');

secondConv = iCreateAndInitializeConv3dLayer(convFilterSize, ...
    2*finalNumChannels, convolutionPadding, 'Bridge-Conv-2');
secondBN = batchNormalizationLayer('name', 'Bridge-BN-2');
secondReLU = reluLayer('Name','Bridge-ReLU-2');
    
encoderDecoderBridge = [firstConv; firstBN; firstReLU; secondConv; ...
                        secondBN; secondReLU];

% Initialize decoder sub-network parameters and create decoder sub-network.
initialDecoderNumChannels = finalNumChannels;

upConvFilterSize = 2;

[decoder, ~] = iCreateDecoder(encoderDepth, ...
    upConvFilterSize, convFilterSize, initialDecoderNumChannels, ...
    convolutionPadding);

% Connect input, encoder, bridge and decoder sub-networks. 
layers = [inputlayer; encoder; encoderDecoderBridge; decoder];

% Create final 1x1x1 convolution layer with output number of channels as 
% numClasses. 
numClasses = args.numClasses;
finalConv = convolution3dLayer(1,numClasses, 'Padding', ...
    convolutionPadding, 'WeightsInitializer', 'he', ...
    'BiasInitializer', 'zeros', 'name', 'Final-ConvolutionLayer');

% Create softmax classification layer.
smLayer = softmaxLayer('Name','Softmax-Layer');

% Create pixel classification layer with default parameters.
pixelClassLayer = pixelClassificationLayer('Name','Segmentation-Layer');

% Create final network with all the required layers. 
layers = [layers; finalConv; smLayer; pixelClassLayer];
lgraph = layerGraph(layers);

% Connect encoder-decoder interconnections as per Padding argument.
lgraph = iConnectLgraph(lgraph, convolutionPadding, encoderDepth);

% Use network analyzer to calculate output size of the network.
analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);
analysis.applyConstraints();
outputSize = analysis.LayerAnalyzers(end-2,1).Outputs.Size{1};
end
%--------------------------------------------------------------------------
function args = iParseInputs(varargin)

p = inputParser;
p.addRequired('inputSize', @iCheckinputSize);
p.addRequired('numClasses', @iCheckNumClasses);
p.addParameter('FilterSize', 3, @iCheckFilterSize);
p.addParameter('EncoderDepth', 3, @iCheckEncoderDepth);
p.addParameter('NumFirstEncoderFilters', 32,...
 @iCheckNumFirstEncoderFilters);
p.addParameter('ConvolutionPadding', 'same', @iCheckConvolutionPadding);

p.parse(varargin{:});

userInput = p.Results;

inputSize = double(userInput.inputSize);

args.inputSize  = inputSize;

args.numClasses = double(userInput.numClasses);
if isscalar(userInput.FilterSize)
    args.FilterSize = [double(userInput.FilterSize) ...
        double(userInput.FilterSize) double(userInput.FilterSize)];
else 
    args.FilterSize = double(userInput.FilterSize);
end
args.EncoderDepth = double(userInput.EncoderDepth);
    
args.NumFirstEncoderFilters = double(userInput.NumFirstEncoderFilters);

% Mapping of external padding parameter to that of convolution3dLayer
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
errId = zeros([1,3]);
if strcmp(args.ConvolutionPadding, 'same') %% "same" convolution setting
    % Convolution layer with "same" padding will not reduce feature map 
    % size after convolution layer therefore encDownsamplingFactor = 0.
    encDownsamplingFactor = 0; 
    for idx=1:3 %% for Height, width and Depth
        [args.inputSize(idx),errId(idx)] = iValidateAndSuggestInputSize...
            (args.inputSize(idx), sizeFactor, encDownsamplingFactor);
    end
    errMessage = 'vision:semanticseg:imageSizeIncompatible3d';
else  %% "valid" convolution setting
    % The 3-D UNet paper imposes constraint that input of each 2x2x2
    % max-pooling layer must be even in height, width, and depth
    % dimensions. This constraint can be satisfied in "valid" convolution
    % settings by using following things: 
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
    % reduce its output feature map size by 2 in height, width and depth.
    % Therefore, encDownsamplingFactor value is compounding effect of these
    % size reductions. Following are few cases for various encoderDepth:
    % (args.EncoderDepth=1), encDownsamplingFactor=2*(args.FilterSize-1) 
    % (args.EncoderDepth=2), encDownsamplingFactor=6*(args.FilterSize-1)
    % (args.EncoderDepth=3), encDownsamplingFactor=14*(args.FilterSize-1)
    % After generalizing, (2^(args.EncoderDepth+1)-2)*(args.FilterSize-1).
    % For e.g.: If args.InputSize = [18 18 18], args.EncoderDepth = 1,
    % args.FilterSize = 3, then, in encoder there will be only one
    % max-pooling layer. And size of input feature map of max-pooling layer
    % is [18 18 18] - (2^(2)-2)*2 = [14 14 14], which is even and divisible
    % by 2^args.EncoderDepth = 2^1 = 2, therefore we consider [18 18 18] as
    % valid size, and UNet 3-D network can be created using these
    % parameters.
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
    % finalEncDownsamplingFactor)./2^args.EncoderDepth. For e.g. ([18 18
    % 18] - 12)./2^1  = [6 6 6]./2^1 = [3 3 3]. (ignoring the number of
    % channels, as it is independent of size reduction). Considering
    % decoder, transposed convolution layers will upsample output of
    % encoder by 2^args.EncoderDepth. 2^args.EncoderDepth*((args.inputSize
    % - finalEncDownsamplingFactor) ./2^args.EncoderDepth) =
    % (args.inputSize - finalEncDownsamplingFactor) For e.g. [18 18 18] -
    % 12 = [6 6 6]. Followed by convolution layers that will reduce the
    % size by factor of encDownsamplingFactor, since convolution layers are
    % similar in encoder and decoder except last encoder section(not
    % considered while calculating encDownsamplingFactor). For e.g. [6 6 6]
    % - 4 = [2 2 2]. The overall, network size reduction is,
    % encDecDownsamplingFactor = finalEncDownsamplingFactor +
    % encDownsamplingFactor. For e.g. encDecDownsamplingFactor = 12 + 4 =
    % 16. So. the 3-D UNet network will reduce the size of input by 16. [18
    % 18 18]-16 = [2 2 2]. The [2 2 2] is output size of the network for
    % input size of [18 18 18].
    encDecDownsamplingFactor = finalEncDownsamplingFactor + ...
        encDownsamplingFactor;
    
    for idx = 1:3 %% for Height, width and Depth
        [args.inputSize(idx), errId(idx)] = iValidateAndSuggestInputSizeForValidConv...
        (args.inputSize(idx), sizeFactor, encDownsamplingFactor(idx), ...
        encDecDownsamplingFactor(idx));
    end    
    errMessage = 'vision:semanticseg:imageSizeIncompatibleValidConv';
end

if any(errId)
    error(message(errMessage, mat2str(inputSize), mat2str(args.inputSize)));
end
end
%--------------------------------------------------------------------------
function iCheckinputSize(x)
validateattributes(x, {'numeric'}, ...
    {'nonempty', 'real', 'finite', 'integer', 'positive','row'}, ...
    mfilename, 'inputSize');
isValidSize = isrow(x) && (numel(x)==3 || numel(x)==4);
if ~isValidSize
    error(message('vision:semanticseg:imageSizeIncorrect3d'));
end
end

%--------------------------------------------------------------------------
function iCheckNumClasses(x)
validateattributes(x, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonsparse','positive', '>', 1}, ...
    mfilename, 'numClasses');
end

%--------------------------------------------------------------------------
function iCheckFilterSize(x)

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
function iCheckEncoderDepth(x)
validateattributes(x, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
    mfilename, 'EncoderDepth');
end

%--------------------------------------------------------------------------
function iCheckNumFirstEncoderFilters(x)
validateattributes(x, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'integer', 'nonsparse', 'positive'}, ...
    mfilename, 'NumFirstEncoderFilters');
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
function [modInputSize,errFlag] = iValidateAndSuggestInputSizeForValidConv...
    (inputSize, sizeFactor, encDownsamplingFactor,...
    encDecDownsamplingFactor)
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
function [encoder, finalNumChannels] = iCreateEncoder(encoderDepth, ...
    convFilterSize, initialEncoderNumChannels, convolutionPadding)

encoder = [];
for stage = 1:encoderDepth
    
    encoderNumChannels = initialEncoderNumChannels * 2^(stage-1);
     
    firstConv = iCreateAndInitializeConv3dLayer(convFilterSize, ...
        encoderNumChannels, convolutionPadding, ...
        ['Encoder-Stage-' num2str(stage) '-Conv-1']);
    firstBN = batchNormalizationLayer('name', ...
        ['Encoder-Stage-' num2str(stage) '-BN-1']);
    firstReLU = reluLayer('Name', ...
        ['Encoder-Stage-' num2str(stage) '-ReLU-1']);
    
    % Double the layer number of channels at each stage of the encoder.
    secondConv = iCreateAndInitializeConv3dLayer(convFilterSize, ...
        2*encoderNumChannels, convolutionPadding, ...
        ['Encoder-Stage-' num2str(stage) '-Conv-2']);
    secondBN = batchNormalizationLayer('name', ...
        ['Encoder-Stage-' num2str(stage) '-BN-2']);
    secondReLU = reluLayer('Name', ...
        ['Encoder-Stage-' num2str(stage) '-ReLU-2']);
        
    encoder = [encoder;firstConv; firstBN; firstReLU; secondConv; ...
        secondBN; secondReLU];    
   
    maxPoolLayer = maxPooling3dLayer(2,'stride',2,'name', ...
        ['Encoder-Stage-' num2str(stage) '-MaxPool']);
    
    encoder = [encoder; maxPoolLayer];
end
finalNumChannels = 2*encoderNumChannels;
end

%--------------------------------------------------------------------------
function [decoder, finalDecoderNumChannels] = iCreateDecoder(encoderDepth, ...
    upConvFilterSize, convFilterSize, initialDecoderNumChannels, ...
    convolutionPadding)

decoder = [];
for stage = 1:encoderDepth    
    % Half the layer number of channels at each stage of the decoder.
    decoderNumChannels = initialDecoderNumChannels / 2^(stage-1);
    
    upConv = iCreateAndInitializeUpConv3dLayer(upConvFilterSize, ...
        2*decoderNumChannels, ['Decoder-Stage-' num2str(stage) '-UpConv']);
        
    % Input feature channels are concatenated with deconvolved features 
    % within the decoder.
    depthConcatLayer = concatenationLayer(4, 2, 'Name', ...
        ['Decoder-Stage-' num2str(stage) '-Concatenation']);    
    
    firstConv = iCreateAndInitializeConv3dLayer(convFilterSize, ...
        decoderNumChannels, convolutionPadding, ...
        ['Decoder-Stage-' num2str(stage) '-Conv-1']);
    firstBN = batchNormalizationLayer('name', ...
        ['Decoder-Stage-' num2str(stage) '-BN-1']);
    firstReLU = reluLayer('Name',['Decoder-Stage-' ...
        num2str(stage) '-ReLU-1']);
        
    secondConv = iCreateAndInitializeConv3dLayer(convFilterSize, ...
        decoderNumChannels, convolutionPadding, ...
        ['Decoder-Stage-' num2str(stage) '-Conv-2']);
    secondBN = batchNormalizationLayer('name', ...
        ['Decoder-Stage-' num2str(stage) '-BN-2']);
    secondReLU = reluLayer('Name',['Decoder-Stage-' ...
        num2str(stage) '-ReLU-2']);
    
    decoder = [decoder; upConv; depthConcatLayer; firstConv; firstBN; ...
        firstReLU; secondConv; secondBN; secondReLU];
end
finalDecoderNumChannels = decoderNumChannels;
end

%--------------------------------------------------------------------------
function convLayer = iCreateAndInitializeConv3dLayer(convFilterSize, ...
    outputNumChannels, convolutionPadding , layerName)

convLayer = convolution3dLayer(convFilterSize,outputNumChannels,...
    'Padding', convolutionPadding ,'BiasL2Factor',0, ...
    'WeightsInitializer','he', 'BiasInitializer','zeros', ...
    'Name', layerName);
end

%--------------------------------------------------------------------------
function upConvLayer = iCreateAndInitializeUpConv3dLayer(UpconvFilterSize, ...
    outputNumChannels, layerName)

upConvLayer = transposedConv3dLayer(UpconvFilterSize, outputNumChannels,...
    'Stride', 2, 'BiasL2Factor', 0, 'WeightsInitializer', 'he', ...
    'BiasInitializer', 'zeros', 'Name', layerName);

upConvLayer.BiasLearnRateFactor = 2;
end

%--------------------------------------------------------------------------
function lgraph = iConnectLgraph(lgraph, convolutionPadding, encoderDepth)
for depth = 1:encoderDepth
    if strcmp(convolutionPadding,'same')
        startLayer = sprintf('Encoder-Stage-%d-ReLU-2',depth);
        endLayer = sprintf('Decoder-Stage-%d-Concatenation/in2',...
            encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph,startLayer, endLayer);
    else
        crop = crop3dLayer('Name', ['Crop3d-',...
            num2str(encoderDepth-depth + 1)]);
        lgraph = addLayers(lgraph,crop);
        startLayer = sprintf('Encoder-Stage-%d-ReLU-2',depth);
        endLayer = sprintf('Crop3d-%d/in',encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph,startLayer, endLayer);
        startLayer2 = sprintf('Decoder-Stage-%d-UpConv',...
            encoderDepth-depth + 1);
        endLayer2 = sprintf('Crop3d-%d/ref',encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph,startLayer2, endLayer2);
        startLayer3 = sprintf('Crop3d-%d',encoderDepth-depth + 1);
        endLayer3 = sprintf('Decoder-Stage-%d-Concatenation/in2',...
            encoderDepth-depth + 1);
        lgraph = connectLayers(lgraph,startLayer3, endLayer3);
    end
end
end

