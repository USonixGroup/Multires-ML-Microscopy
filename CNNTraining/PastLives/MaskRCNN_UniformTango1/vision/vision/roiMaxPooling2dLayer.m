function layer = roiMaxPooling2dLayer(varargin)

% Copyright 2018-2023 The MathWorks, Inc.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

params = parseInputs(varargin{:});

% Create an internal representation of the layer.
internalLayer = nnet.internal.cnn.layer.ROIMaxPooling2DLayer(...
    params.Name, params.OutputSize);

% Pass the internal layer to a function to construct a user visible layer.
layer = nnet.cnn.layer.ROIMaxPooling2DLayer(internalLayer);

%--------------------------------------------------------------------------
function params = parseInputs(varargin)
p = inputParser;
p.addRequired('outputSize', ...
    @(x)nnet.cnn.layer.ROIMaxPooling2DLayer.validateOutputSize(x,mfilename,'outputSize'));
p.addParameter('Name', '', @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
parse(p, varargin{:});

params.Name = char(p.Results.Name);

params.OutputSize = double(p.Results.outputSize);
