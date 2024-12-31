function layer = yolov2OutputLayer( varargin )

% Copyright 2018-2023 The MathWorks, Inc.

% Parse the input arguments.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);

args = iParseInputArguments(varargin{:});

internalLayer = nnet.internal.cnn.layer.YOLOv2OutputLayer( ...
    args.Name,...
    args.AnchorBoxes,...
    args.LossFactors,...
    args.NumClasses);

% Pass the internal layer to a function to construct.
layer = nnet.cnn.layer.YOLOv2OutputLayer(internalLayer);

% Use class set method to convert to canonical form and set OutputSize too.
layer.Classes = args.Classes;

end

function inputArguments = iParseInputArguments(varargin)
parser = iCreateParser();
parser.parse(varargin{:});
inputArguments = iConvertToCanonicalForm(parser);
end

function p = iCreateParser()
p = inputParser;
p.addRequired('anchorBoxes', ...
    @(x)nnet.cnn.layer.YOLOv2OutputLayer.validateAnchors(x,mfilename));
addParameter(p, 'Name', '', @iAssertValidLayerName);
addParameter(p, 'LossFactors', [5 1 1 1],...
    @(x)nnet.cnn.layer.YOLOv2OutputLayer.validateLossFactors(x,mfilename));
addParameter(p, 'Classes','auto', @iAssertValidClasses);
end

function iAssertValidLayerName(name)
nnet.internal.cnn.layer.paramvalidation.validateLayerName(name);
end

function iAssertValidClasses(value)
nnet.internal.cnn.layer.paramvalidation.validateClasses(value);
if numel(value) == 1
    if (strcmp(cellstr(value),'auto'))
        error(message('vision:yolo:classNameExists'));
    end
end
end

function inputArguments = iConvertToCanonicalForm(p)
inputArguments = struct;
inputArguments.NumClasses = [];
inputArguments.Name = char(p.Results.Name); % make sure name string gets converted to char vector
inputArguments.AnchorBoxes = double(p.Results.anchorBoxes);
inputArguments.LossFactors = double(p.Results.LossFactors);
inputArguments.Classes = p.Results.Classes;
end
