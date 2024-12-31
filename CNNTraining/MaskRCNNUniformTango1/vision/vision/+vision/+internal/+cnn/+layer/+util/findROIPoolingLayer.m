function idx = findROIPoolingLayer(internalLayers)
% Returns index to ROI Average or ROI Max pooling layer. 

%   Copyright 2016-2020 The MathWorks, Inc.

% Can use one isa check because average pooling inherits from max pooling.
idx = find(...
    cellfun( @(x)isa(x,'nnet.internal.cnn.layer.ROIMaxPooling2DLayer'), ...
    internalLayers), 1, 'last');

end
