function layer = roiInputLayer(varargin)

% Copyright 2018-2023 The MathWorks, Inc.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

params = parseInputs(varargin{:});
layer = nnet.cnn.layer.ROIInputLayer(...
    nnet.internal.cnn.layer.ROIInputLayer(params.Name));

%--------------------------------------------------------------------------
function params = parseInputs(varargin)
p = inputParser;
p.addParameter('Name','',...
    @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
parse(p, varargin{:});

params.Name = char(p.Results.Name);