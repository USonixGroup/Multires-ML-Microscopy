function sz = imageInputSizeFromNetworkAnalysis(analysis)
% Returns the input size of the imageInputLayer from the network analysis.
% analysis is returned by nnet.internal.cnn.analyzer.NetworkAnalyzer.

% Copyright 2017 MathWorks, Inc. 

inputLayerIndex = [analysis.LayerAnalyzers.IsInputLayer];
details = analysis.LayerAnalyzers(inputLayerIndex);

% This utility function expects a single image input layer.
assert(isa(details.ExternalLayer,'nnet.cnn.layer.ImageInputLayer'))
assert(numel(details)==1);

sz = details.ExternalLayer.InputSize;

