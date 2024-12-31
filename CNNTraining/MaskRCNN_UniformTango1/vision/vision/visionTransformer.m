%

%   Copyright 2023 The MathWorks, Inc.

function [net, classes] = visionTransformer(modelName, options)
arguments
    modelName                                 {mustBeTextScalar} = "base-16-imagenet-384"
    options.DropoutProbability          (1,1) {mustBeNumeric, mustBeNonsparse,...
                                               mustBeInRange(options.DropoutProbability, 0, 1, "exclude-upper")} = 0.1
    options.AttentionDropoutProbability (1,1) {mustBeNumeric, mustBeNonsparse,...
                                               mustBeInRange(options.AttentionDropoutProbability, 0, 1, "exclude-upper")} = 0.1
end

vision.internal.requiresNeuralToolbox(mfilename);

nargoutchk(0, 2)

modelName = iValidateModelName(modelName);

returnClasses = nargout==2;
[net, classes] = iTripwireViTModel(modelName, returnClasses);

defaultDropoutProb = 0.1;
dropoutProb = cast(options.DropoutProbability, 'like', defaultDropoutProb);
attentionDropoutProb = cast(options.AttentionDropoutProbability, 'like', defaultDropoutProb);
if ~isequal(dropoutProb, defaultDropoutProb)
    net = iChangeDropoutProbability(net, dropoutProb);
end
if ~isequal(attentionDropoutProb, defaultDropoutProb)
    net = iChangeAttentionDropoutProbability(net, attentionDropoutProb);
end

net = initialize(net);
end

function value = iValidateModelName(value)
value = validatestring(value, ["base-16-imagenet-384", "small-16-imagenet-384", "tiny-16-imagenet-384"]);
end

function [net, classes] = iTripwireViTModel(modelName, returnClasses)
% Check if support package is installed
breadcrumbFile = 'vision.internal.cnn.supportpackages.IsViTInstalled';
fullPath = which(breadcrumbFile);
if isempty(fullPath)
    name     = 'Computer Vision Toolbox Model for Vision Transformer Network';
    basecode = 'VIT';
    
    throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired', mfilename, name, basecode)));
else
    % Load pretrained network.
    pattern = fullfile(filesep, '+vision','+internal','+cnn','+supportpackages','IsViTInstalled.m');
    idx     = strfind(fullPath, pattern);
    fullPath = fullPath(1:idx);
    switch modelName
        case "base-16-imagenet-384"
            matfile = fullfile(fullPath, 'data', 'vit-base16-imagenet384.mat');
        case "small-16-imagenet-384"
            matfile = fullfile(fullPath, 'data', 'vit-small16-imagenet384.mat');
        case "tiny-16-imagenet-384"
            matfile = fullfile(fullPath, 'data', 'vit-tiny16-imagenet384.mat');
    end
    data = load(matfile);
    net = data.net;

    % Modify network
    net = iModifyNet(net);

    if returnClasses
        % Load classes
        data = load(fullfile(fullPath, 'data', 'classes.mat'));
        classes = data.classes;
    else
        classes = "";
    end
end
end

function net = iModifyNet(net)
% Change the flatten mode option of patch embedding layer to row-major and
% add an input size validation layer to throw helpful error for invalid
% input size.
layer = iGetLayerOfClass(net, 'nnet.cnn.layer.PatchEmbeddingLayer');
name = layer.Name;
newLayer = patchEmbeddingLayer(layer.PatchSize, layer.OutputSize, ...
    Name = name, ...
    Weights = layer.Weights, ...
    Bias = layer.Bias, ...
    SpatialFlattenMode = "row-major");

net = replaceLayer(net, name, newLayer);

% Add function layer to validate input size
net = addLayers(net, functionLayer(@vision.internal.cnn.errorIfInvalidInputSize, Name='validate_inputsize'));
net = disconnectLayers(net,'imageinput',name);
net = connectLayers(net,'imageinput','validate_inputsize');
net = connectLayers(net,'validate_inputsize',name);
end

function net = iChangeDropoutProbability(net, newDropoutProb)
layer = iGetLayerOfClass(net, 'nnet.cnn.layer.DropoutLayer');
for i = 1:numel(layer)
    name = layer(i).Name;
    net = replaceLayer(net, name, dropoutLayer(newDropoutProb, Name=name));
end
end

function net = iChangeAttentionDropoutProbability(net, newDropoutProb)
layer = iGetLayerOfClass(net, 'nnet.cnn.layer.SelfAttentionLayer');
for i = 1:numel(layer)
    currentLayer = layer(i);
    name = currentLayer.Name;
    % Create a new self-attention layer using learnable parameters of
    % current layer and assign new attention dropout probability to the new
    % layer
    newLayer = selfAttentionLayer(currentLayer.NumHeads, currentLayer.NumKeyChannels, ...
                    QueryWeights = currentLayer.QueryWeights, ...
                    QueryBias = currentLayer.QueryBias, ...
                    KeyWeights = currentLayer.KeyWeights, ...
                    KeyBias = currentLayer.KeyBias, ...
                    ValueWeights = currentLayer.ValueWeights, ...
                    ValueBias = currentLayer.ValueBias, ...
                    OutputWeights = currentLayer.OutputWeights, ...
                    OutputBias = currentLayer.OutputBias, ...
                    DropoutProbability = newDropoutProb, ...  
                    Name = name);
    net = replaceLayer(net, name, newLayer);
end
end

function layer = iGetLayerOfClass(net, className)
layer = net.Layers(arrayfun(@(x) isa(x, className), net.Layers));
end