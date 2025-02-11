%--------------------------------------------------------------------------
% Remove the augmentations, if present, and issues a warning.
%--------------------------------------------------------------------------
function layerGraphOrLayers = removeAugmentationIfNeeded(layerGraphOrLayers, augmentations)

%   Copyright 2016-2020 The MathWorks, Inc.

augmentations = cellstr(augmentations);
for i = 1:numel(augmentations)
    layerGraphOrLayers = iRemoveAugmentationAndWarn(layerGraphOrLayers, augmentations{i});
end

%--------------------------------------------------------------------------
function layerGraphOrLayers = iRemoveAugmentationAndWarn(layerGraphOrLayers, augmentation)

if isa(layerGraphOrLayers,'nnet.cnn.layer.Layer')
    layers = layerGraphOrLayers;
    isLayerGraph = false;
else
    layers = layerGraphOrLayers.Layers;
    isLayerGraph = true;
end

imageLayerIdx = find(arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),layers),1);

if ismember(augmentation, layers(imageLayerIdx).DataAugmentation)
    warning(message('vision:rcnn:removingAugmentation', augmentation));
    
    augmentations = cellstr(layers(imageLayerIdx).DataAugmentation);
    idx = strcmpi(augmentation, augmentations);
    
    % remove randcrop data augmentation
    augmentations(idx) = [];
    
    if isempty(augmentations)
        augmentations = 'none';
    end
    
    newImageInputLayer = imageInputLayer(layers(imageLayerIdx).InputSize, ...
        'Name', layers(imageLayerIdx).Name,...
        'DataAugmentation', augmentations, ...
        'Normalization', layers(imageLayerIdx).Normalization);
    
    if isLayerGraph
        name = layers(imageLayerIdx).Name;                        
        layerGraphOrLayers = replaceLayer(layerGraphOrLayers, name, newImageInputLayer);        
    else
        layerGraphOrLayers(imageLayerIdx) = newImageInputLayer;
    end
    
end
