function layer = yolov2TransformLayer(varargin)

% Copyright 2018-2024 The MathWorks, Inc.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

% Parse the input arguments.
params = parseInputs(varargin{:});

% Create an internal representation of the layer.
layer = nnet.cnn.layer.YOLOv2TransformLayer(params.Name,params.NumAnchorBoxes);

end
%--------------------------------------------------------------------------
function params = parseInputs(varargin)
p = inputParser;
p.addRequired('NumAnchorBoxes', ...
    @(x)nnet.cnn.layer.YOLOv2TransformLayer.validateParameters(x,mfilename,'NumAnchorBoxes'));
p.addParameter('Name', '', @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
parse(p, varargin{:});
params.Name = char(p.Results.Name);
params.NumAnchorBoxes = double(p.Results.NumAnchorBoxes);
end

