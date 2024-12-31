function layer = regionProposalLayer(anchorBoxes, varargin)

% Copyright 2018-2023 The MathWorks, Inc.

% Check for deep learning toolbox.
vision.internal.requiresNeuralToolbox(mfilename);
params = parseInputs(anchorBoxes, varargin{:});
layer = nnet.cnn.layer.RegionProposalLayer(...
    nnet.internal.cnn.layer.RegionProposalLayer(params.Name, params.AnchorBoxes));

%--------------------------------------------------------------------------
function params = parseInputs(varargin)
p = inputParser;
p.addRequired('anchorBoxes',@iCheckAnchorBoxes);
p.addParameter('Name','',...
    @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
parse(p, varargin{:});

params.Name = char(p.Results.Name);
params.AnchorBoxes = double(p.Results.anchorBoxes);

%--------------------------------------------------------------------------
function iCheckAnchorBoxes(boxSizes)
validateattributes(boxSizes, {'numeric'}, ...
    {'positive','real','nonsparse','finite','integer','nonempty','size',[NaN,2]},...
    mfilename,'anchorBoxes',1);
