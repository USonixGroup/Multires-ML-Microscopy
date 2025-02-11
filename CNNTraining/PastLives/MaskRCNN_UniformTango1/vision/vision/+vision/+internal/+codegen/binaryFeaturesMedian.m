function median = binaryFeaturesMedian(features)
% binaryFeaturesMedian is the codegen implementation for 
% visionBinaryFeaturesMedian

% Copyright 2022 The MathWorks, Inc.
%#codegen
numFeatures = size(features, 1);
numBytes = size(features, 2);
median = zeros(1, numBytes, 'uint8');
bitMask = uint8([1, 2, 4, 8, 16, 32, 64, 128]);
% Iterate over the features and compute the median value
for n=1:numBytes
    temp = features(:, n);
    bitCounts = getNumberOfBits(temp, numFeatures, bitMask);
    median(n) = getMedianFromBitCounts(bitCounts, numFeatures, bitMask);
end
end
%------------------------------------------------------------------
function bitCounts = getNumberOfBits(features, numFeatures, bitMask)
bitCounts = zeros(1, 8, 'uint32');
for i = 1:numFeatures
    for j = 1:8
        % Count the number of true bits along each bit position.
        result = bitand(features(i), bitMask(j));
        numSetBits  = numel(find(bitget(result, 1:8)));
        bitCounts(j) = bitCounts(j) + uint32(numSetBits);
    end
end
end
%------------------------------------------------------------------
function median = getMedianFromBitCounts(bitCounts, numFeatures, bitMask)
% Divide the total number of feature by 2.
halfNumFeatures = numFeatures/2;
median = uint8(0);
% Determine the median bit value for each bit position and encode the
% result into a uint8 value. The median value is 1 if the number of true
% bits is greater than half the total number of bits.
for i=1:8
    if bitCounts(i) > halfNumFeatures
        median = median + (bitMask(i) * 1);
    else
        median = median + (bitMask(i) * 0);
    end
end
end