function tf = needsZeroCenterNormalization(layers)
%

%   Copyright 2016-2020 The MathWorks, Inc.

idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ImageInputLayer'),layers);
tf = ismember(cellstr('zerocenter'), cellstr(layers(idx).Normalization));
