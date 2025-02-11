function layer = rcnnBoxRegressionLayer(varargin)

% Copyright 2018-2023 The MathWorks, Inc.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

params = parseInputs(varargin{:});

% Create an internal representation of the layer.
numResponses = []; % will be set during training.
internalLayer = vision.internal.cnn.layer.SmoothL1Loss(params.Name, numResponses);

% Pass the internal layer to a function to construct a user visible layer.
layer = nnet.cnn.layer.RCNNBoxRegressionLayer(internalLayer);

%--------------------------------------------------------------------------
function params = parseInputs(varargin)
p = inputParser;
p.addParameter('Name','',...
    @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
parse(p, varargin{:});

params.Name = char(p.Results.Name);

