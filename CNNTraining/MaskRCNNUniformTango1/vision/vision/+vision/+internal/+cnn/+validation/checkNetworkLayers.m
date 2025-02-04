function checkNetworkLayers(analysis)
% Must have exactly one image input layer and output layer. 
% Must have classification layer.
% Input analysis is produced by nnet.internal.cnn.analyzer.NetworkAnalyzer.

%   Copyright 2016-2020 The MathWorks, Inc.

inputLayerIndex = [analysis.LayerAnalyzers.IsInputLayer];
details = analysis.LayerAnalyzers(inputLayerIndex);
if ~(numel(details) == 1 && isa(details.ExternalLayer, 'nnet.cnn.layer.ImageInputLayer'))
    error(message('vision:rcnn:firstLayerNotImageInputLayer'));
end

outputLayerIndex = [analysis.LayerAnalyzers.IsOutputLayer];
details = analysis.LayerAnalyzers(outputLayerIndex);

if sum(outputLayerIndex) ~= 1 || sum(inputLayerIndex) ~= 1
    % TODO add test case.
    % TODO update message catalog.
    error(message('vision:rcnn:mimoDAGNotSupported'))
end

hasClassificationLayer = isa(details.ExternalLayer,...
    'nnet.cnn.layer.ClassificationOutputLayer');

% Last two layers must be softmax followed by a classification layer
if ~hasClassificationLayer
    error(message('vision:rcnn:lastLayerNotClassificationLayer'));
end
