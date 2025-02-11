function lastFCLayerIndex = findLastFullyConnectedLayer(lgraph)
% Search backwards from provided LayerGraph and find first FC layer feeding
% the classification output layer.
%
% Callers should check if lastFCLayerIndex isempty and error if
% appropriate.

%   Copyright 2018-2020 The MathWorks, Inc.

assert(isa(lgraph,'nnet.cnn.LayerGraph'));

% Start search from the classification output layer.
clsLayerIndex = find( ...
    arrayfun(@(x)...
    isa(x,'nnet.cnn.layer.ClassificationOutputLayer'), lgraph.Layers), ...
    1, 'last');
clsLayerName = lgraph.Layers(clsLayerIndex).Name;

index = find(strcmp(clsLayerName,lgraph.Connections.Destination));

% Search backwards from classification layer looking for
% the first fully connected layer. If one is not found, the
% return value is [].
lastFCLayerIndex = [];
names = {lgraph.Layers.Name};
while true
    layerName = lgraph.Connections{index,'Source'};
    
    layerIndex = find(strcmp(layerName,names));
    
    if isempty(layerIndex)
        break
    end
    
    if isa(lgraph.Layers(layerIndex), 'nnet.cnn.layer.FullyConnectedLayer')
        lastFCLayerIndex = layerIndex;
        break
    end
    
    index = find(strcmp(layerName,lgraph.Connections.Destination));
    
    if isempty(index)
        break
    end
end



