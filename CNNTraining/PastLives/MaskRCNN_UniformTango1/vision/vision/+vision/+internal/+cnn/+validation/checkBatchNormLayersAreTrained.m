function checkBatchNormLayersAreTrained(lgraphOrLayerArray)
% error if batch norm layers are not trained. Use this check if the user
% asks to freeze the batch norm layers during training. The only way they
% can be frozen is if they have initial values.

%   Copyright 2018-2020 The MathWorks, Inc.

if isa(lgraphOrLayerArray,'nnet.cnn.LayerGraph')
    larray = lgraphOrLayerArray.Layers;
else
    larray = lgraphOrLayerArray;
end

idx = arrayfun(@(x)...
    isa(x,'nnet.cnn.layer.BatchNormalizationLayer'),...
    larray);

bnLayers = larray(idx);
for i = 1:numel(bnLayers)
    if iIsNotTrained(bnLayers(i))
        error(message('vision:rcnn:unableToFreezeBNLayer'));
    end
end

%--------------------------------------------------------------------------
function tf = iIsNotTrained(bn)
tf = isempty(bn.Offset) || ...
    isempty(bn.Scale) || ...
    isempty(bn.TrainedMean) || ...
    isempty(bn.TrainedVariance);
