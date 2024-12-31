function lgraph = fasterRCNNLayers(varargin)

% Copyright 2019-2023 The MathWorks, Inc.

    % Check for deep learning toolbox.
    vision.internal.requiresNeuralToolbox(mfilename);

    [lgraph,params] = iParseInputs(varargin{:});

    lgraph = vision.internal.cnn.RCNNLayers.createFasterRCNN(...
        params.ImageSize,params.NumClasses,lgraph,params.AnchorBoxes,...
        params.FeatureLayer,params.ROIMaxPoolingLayer,params.ROIOutputSize);

end

%--------------------------------------------------------------------------
function [lgraph,params] = iParseInputs(varargin)
    % Parse and validate the input parameters.
    p = inputParser;
    p.addRequired('imageSize',@iCheckImageSize);
    p.addRequired('numClasses',@iCheckNumClasses);
    p.addRequired('anchorBoxes',@iCheckAnchorBoxes);
    p.addRequired('network');
    numArgs = nargin;
    featureLayerSpecified = false;
    featureLayer = [];
    if numArgs > 4
        if mod(numArgs - 4,2) == 1
            featureLayer = varargin{5};
            iCheckScalarText(featureLayer,'featureLayer');
            featureLayerSpecified = true;
            varargin(5) = [];
        end
    end
    p.addParameter('ROIMaxPoolingLayer','auto',@(x)iCheckScalarText(x,'ROIMaxPoolingLayer'));
    p.addParameter('ROIOutputSize','auto',@(x)iCheckROIOutputSize(x));
    parse(p, varargin{:});
    userInput = p.Results;

    params.ImageSize        = double(userInput.imageSize);
    params.NumClasses       = double(userInput.numClasses);
    params.AnchorBoxes      = double(userInput.anchorBoxes);
    params.Network          = iValidateNetworkInput(userInput.network);
    if ~featureLayerSpecified && ~ischar(params.Network)
        error(message('vision:fasterRCNNLayers:featureLayerRequiredForNonChar'));
    else
        params.FeatureLayer     = char(featureLayer);
    end

    if iWasUserSpecified(p, 'ROIMaxPoolingLayer')
        params.ROIMaxPoolingLayer = iValidateROIMaxPoolingLayerOption(userInput.ROIMaxPoolingLayer);
    else
        params.ROIMaxPoolingLayer = userInput.ROIMaxPoolingLayer;
    end

    params.ROIOutputSize = userInput.ROIOutputSize;
    if ~isnumeric(params.ROIOutputSize)
        params.ROIOutputSize = 'auto';
    end

    % Validate that anchorBoxes are smaller than imageSize.
    iValidateAnchorBoxes(params);

    % Create layer graph from the user provided network.
    lgraph = iLayerGraph(params.Network);
end

%--------------------------------------------------------------------------
function iCheckImageSize(x)
    classes = {'numeric'};
    attrs = {'vector', 'nonempty', 'nonzero',...
        'finite', 'nonnan', 'integer', 'nonsparse', 'nonnegative'...
        'numel', 3, 'nrows', 1};
    validateattributes(x,classes,attrs,mfilename,'imageSize');
end

%--------------------------------------------------------------------------
function iCheckNumClasses(x)
    classes = {'numeric'};
    attrs = {'scalar', 'nonempty', 'nonzero',...
        'finite', 'nonnan','integer', 'nonsparse', 'nonnegative'};
    validateattributes(x,classes,attrs,mfilename,'numClasses');
end

%--------------------------------------------------------------------------
function iCheckAnchorBoxes(x)
    classes = {'numeric'};
    attrs = {'2d','ncols',2,'nonempty','nonsparse',...
        'finite','nonnan','positive','integer', 'nonnegative'};
    validateattributes(x,classes,attrs,mfilename,'anchorBoxes');
end

%--------------------------------------------------------------------------
function iCheckScalarText(x,optionName)
    classes = {'char', 'string'};
    attrs = {'scalartext','nrows',1,'nonempty'};
    validateattributes(x,classes,attrs,mfilename,optionName);
end
%--------------------------------------------------------------------------
function iCheckROIOutputSize(x)
try
    if ischar(x) || isstring(x)
        validatestring(x, {'auto'});
    else
        classes = {'numeric'};
        attrs = {'vector','numel',2,'nonempty','nonsparse',...
            'finite','nonnan','positive','integer', 'nonnegative'};
        validateattributes(x,classes,attrs);
    end
catch
    error(message('vision:fasterRCNNLayers:invalidROIOutputSize'));
end
end

%--------------------------------------------------------------------------
function network = iValidateNetworkInput(network)
    isSeriesNetwork = isa(network,'SeriesNetwork');
    isDAGNetwork    = isa(network,'DAGNetwork');
    isLayerGraph    = isa(network,'nnet.cnn.LayerGraph');
    nonString = isDAGNetwork || isSeriesNetwork || isLayerGraph;
    if nonString
        return;
    end
    validNetworkNames = vision.internal.cnn.RCNNLayers.SupportedPretrainedNetworks;
    try
        network = validatestring(network,validNetworkNames);
    catch
        error(message('vision:fasterRCNNLayers:invalidNetworkInput'));
    end
end

%--------------------------------------------------------------------------
function iValidateAnchorBoxes(params)
    imageSize = params.ImageSize;
    anchorBoxes = params.AnchorBoxes;
    for k=1:size(anchorBoxes,1)
        if any(anchorBoxes(k,:)>imageSize(:,1:2))
            error(message('vision:fasterRCNNLayers:smallerAnchorThanImage',...
                    mat2str(anchorBoxes(k,:)),mat2str(imageSize(:,1:2))));
        end
    end
end

%--------------------------------------------------------------------------
function lgraph = iLayerGraph(x)
    % Get layer graph from input network.
    if isa(x,'DAGNetwork')

        lgraph = layerGraph(x);
    elseif isa(x,'SeriesNetwork')

        lgraph = layerGraph(x.Layers);
    else

        lgraph = x;
    end
end

%----------------------------------------------------------------------------------
function tf = iWasUserSpecified(parser,param)
    tf = ~ismember(param,parser.UsingDefaults);
end

%----------------------------------------------------------------------------------
function option = iValidateROIMaxPoolingLayerOption(roiMaxPoolingLayerOption)
    validStrings = {'auto','insert','replace'};
    optionName = 'ROIMaxPoolingLayer';
    option = validatestring(roiMaxPoolingLayerOption, validStrings,...
            mfilename, optionName);
end
