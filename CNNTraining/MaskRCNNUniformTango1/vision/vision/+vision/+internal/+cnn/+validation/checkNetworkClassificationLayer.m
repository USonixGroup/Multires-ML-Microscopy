function checkNetworkClassificationLayer(analysis, groundTruth)
% The classification layer must support N classes plus a background class.
% Input analysis is produced by nnet.internal.cnn.analyzer.NetworkAnalyzer.
% groundTruth is training data table. First column is image file names,
% remaining columns are class bounding boxes.

%   Copyright 2016-2020 The MathWorks, Inc.

outputLayerIndex = [analysis.LayerAnalyzers.IsOutputLayer];
details = analysis.LayerAnalyzers(outputLayerIndex);

if details.InternalLayer.NumClasses ~= width(groundTruth)
    error(message('vision:rcnn:notEnoughObjectClasses'));
end
