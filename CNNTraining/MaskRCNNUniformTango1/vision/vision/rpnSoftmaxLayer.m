function layer = rpnSoftmaxLayer(varargin)

% Copyright 2018-2023 The MathWorks, Inc.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

params = parseInputs(varargin{:});

% Create an internal representation of the layer.
internalLayer = nnet.internal.cnn.layer.RPNSoftmaxLayer(params.Name);

% Pass the internal layer to a function to construct a user visible layer.
layer = nnet.cnn.layer.RPNSoftmaxLayer(internalLayer);

%--------------------------------------------------------------------------
function params = parseInputs(varargin)
p = inputParser;
p.addParameter('Name','',...
    @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
parse(p, varargin{:});

params.Name = char(p.Results.Name);
