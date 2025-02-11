function out = colorPreprocessingForImageInputSize(sz)
% select the type of color preprocessing to use. This is used in the
% datastores to auto convert images if required.

%   Copyright 2019-2020 The MathWorks, Inc.

isRGB          = numel(sz)==3 && sz(3) == 3;

isMultiChannel = numel(sz)==3 && ( sz(3) == 2 || sz(3) > 3 );

if isRGB
    out.Preprocessing           = 'gray2rgb';
    out.NetworkInputChannelSize = 3;
elseif isMultiChannel
    % if the input image size has a different channel size than that of
    % the network input size, we need to error.
    out.Preprocessing           = 'multi-channel';
    out.NetworkInputChannelSize = sz(3);
else
    out.Preprocessing           = 'rgb2gray';
    out.NetworkInputChannelSize = 1;
end

