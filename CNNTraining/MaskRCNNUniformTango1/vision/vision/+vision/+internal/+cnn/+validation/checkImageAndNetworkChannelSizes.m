function checkImageAndNetworkChannelSizes(I, networkChannelSize)
% If the input image size has a different channel size than that of
% the network input size, we need to error.

%   Copyright 2019-2020 The MathWorks, Inc.

[~, ~, Isize, ~] = size(I);

if Isize ~= networkChannelSize
    error(message('vision:ObjectDetector:invalidInputImageChannelSize', Isize, networkChannelSize));
end
