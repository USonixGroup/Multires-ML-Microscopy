function lgraph = removeLayersAfterSpecifiedLayer(lgraph,layerName)
% Remove all the layers from the layergraph after the specified layername.

% Copyright 2021 The Mathworks, Inc.

dg = vision.internal.cnn.RCNNLayers.digraph(lgraph);

% Find the last layer.
id = findnode(dg,char(layerName));

% Search for all nodes starting from the feature extraction
% layer.
if ~(sum(id)==0)
    ids = dfsearch(dg,id);
    names = dg.Nodes.Name(ids,:);
    lgraph = removeLayers(lgraph, names(2:end)); % exclude feature extraction layer which is first name.
end
end