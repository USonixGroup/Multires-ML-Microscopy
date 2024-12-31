function layer = roiAlignLayer(outputSize, NameValueArgs)

% Copyright 2020-2023 The MathWorks, Inc.

arguments
    outputSize (1,2) double {nnet.cnn.layer.ROIAlignLayer.validateOutputSize(outputSize,'roiAlignLayer.m','OutputSize')}
    NameValueArgs.Name char {nnet.internal.cnn.layer.paramvalidation.validateLayerName} = ''
    NameValueArgs.ROIScale (1,1) double {nnet.cnn.layer.ROIAlignLayer.validateROIScale(NameValueArgs.ROIScale,'roiAlignLayer.m','SamplingRatio')} = 1
    NameValueArgs.SamplingRatio {nnet.cnn.layer.ROIAlignLayer.validateSamplingRatio(NameValueArgs.SamplingRatio,'roiAlignLayer.m','SamplingRatio')} = 'auto'
    
end

if(isstring(NameValueArgs.SamplingRatio))
    NameValueArgs.SamplingRatio = validatestring(NameValueArgs.SamplingRatio,{'auto'}, mfilename, 'SamplingRatio');
end


% Create an internal representation of the layer.
internalLayer = nnet.internal.cnn.layer.ROIAlignLayer(...
    NameValueArgs.Name, outputSize, NameValueArgs.ROIScale, NameValueArgs.SamplingRatio);

% Pass the internal layer to a function to construct a user visible layer.
layer = nnet.cnn.layer.ROIAlignLayer(internalLayer);

