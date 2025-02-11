function layer = ssdMergeLayer(varargin)

% Copyright 2019-2023 The MathWorks, Inc.
    
    % Check for deep learning toolbox.
    vision.internal.requiresNeuralToolbox(mfilename);

    params = iParseInputs(varargin{:});

    % Create an internal representation of the layer.
    internalLayer = nnet.internal.cnn.layer.SSDMergeLayer(char(params.Name), ...
                                                          params.NumChannels, ...
                                                          params.NumInputs);

    % Pass the internal layer to a function to construct a user visible layer.
    layer = nnet.cnn.layer.SSDMergeLayer(internalLayer);
end

%--------------------------------------------------------------------------
function params = iParseInputs(varargin)

    parser = inputParser();

    inputNumber = 1;

    validateNumChannels = @(x)validateattributes(x, {'numeric'}, {'scalar', 'positive', 'integer'}, ...
        mfilename, 'NumChannels', inputNumber);
    inputNumber = inputNumber + 1;
    parser.addRequired('NumChannels', validateNumChannels);

    validateNumInputs = @(x)validateattributes(x, {'numeric'}, {'scalar', 'positive', 'integer'}, ...
        mfilename, 'NumInputs', inputNumber);
    parser.addRequired('NumInputs', validateNumInputs);

    parser.addParameter('Name','',...
        @nnet.internal.cnn.layer.paramvalidation.validateLayerName);

    parser.parse(varargin{:});
    params = parser.Results;
end