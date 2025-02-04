%dicePixelClassificationLayer Pixel classification layer using generalized
%dice loss for semantic segmentation.
%
%   layer = dicePixelClassificationLayer() creates a pixel classification
%   layer using generalized dice loss for semantic image segmentation
%   networks. The layer computes generalized dice loss for a batch of data
%   processed by a convolutional neural network (CNN).
%
%   layer = dicePixelClassificationLayer('PARAM1', Value) specifies
%   optional parameter name/value pairs for creating the layer:
%
%   'Classes'       Specify the classes into which the pixels or voxels
%                   are classified as a string vector, a categorical
%                   vector, a cell array of character vectors, or 'auto'.
%                   If the value is a categorical vector Y, the entries of
%                   the Classes property will be sorted with the order of
%                   categories(Y). When 'auto' is specified, the classes
%                   are automatically set during training.
%
%                   Default: 'auto'.
%
%   'Name'          A name for the layer. The default is ''.
%
% Notes
% -----
% - The layer automatically ignores undefined pixel labels during training.
%
% - The layer uses Generalized dice loss which uses inverse size of
% expected region as a weighting factor to control the contribution each
% class makes to the loss. This weighting technique can alleviate the
% problem of class imbalance in semantic segmentation problems.
%
% - The Generalized Dice Loss automatically sets weights, which is
% essential in patch based training as it is extremely difficult for one
% set of global weights to address class imbalancing effectively.
%
% Example 1
% ---------
%     % Use dicePixelClassificationLayer to create a 2-D semantic
%     % segmentation network.
% 
% layers = [
%     imageInputLayer([480 640 3])
%     convolution2dLayer(3,16,'Stride',2,'Padding',1)
%     reluLayer
%     transposedConv2dLayer(2,4,'Stride',2)
%     softmaxLayer
%     dicePixelClassificationLayer
%     ]
%
% Example 2
% ---------
%     % Use dicePixelClassificationLayer to create a 3-D semantic
%     % segmentation network.
% 
% layers = [
%     image3dInputLayer([32 32 32 1])
%     convolution3dLayer(3,64,'Padding',1)
%     reluLayer
%     maxPooling3dLayer(2,'Stride',2)
%     convolution3dLayer(3,64,'Padding',1)
%     reluLayer
%     transposedConv3dLayer(4,64,'Stride',2,'Cropping',1)
%     convolution3dLayer(1,2)
%     softmaxLayer
%     dicePixelClassificationLayer
%     ];
%
% See also semanticseg, pixelLabelDatastore, trainNetwork, segnetLayers,
%          fcnLayers, unetLayers, unet3dLayers,
%          nnet.cnn.layer.DicePixelClassificationLayer.

% Copyright 2019 The MathWorks, Inc.

function layer = dicePixelClassificationLayer(varargin)

vision.internal.requiresNeuralToolbox(mfilename);

% Parse the input arguments
args = nnet.cnn.layer.DicePixelClassificationLayer.parseInputs(varargin{:});

% Create an internal representation of a cross entropy layer
internalLayer = nnet.internal.cnn.layer.GeneralizedDiceLoss(args.Name, args.Categories, [], []);

% Pass the internal layer to a function to construct
layer = nnet.cnn.layer.DicePixelClassificationLayer(internalLayer);

end

