function data = validateImagesAndBoxesTransform(data,networkInputSize)
%validateImagesAndBoxesTransform Scans for valid training boxes in the format provided
% by boxLabelDatastore. Scans for valid images using the network input size information.
% This also returns data as a cell converting from table, if needed.
%
% See also boxLabelDatastore.

% Copyright 2019-2023 The MathWorks, Inc.

if istable(data)
    images = data{:, 1};
    boxes  = data{:, 2};
    labels = data{:, 3};
else
    images = data(:, 1);
    boxes  = data(:, 2);
    labels = data(:, 3);
end

firstNonempty = find(~cellfun(@isempty,boxes),1);
isRotatedRectangle = size(boxes{firstNonempty},2) == 5;

cellfun(@(I)vision.internal.cnn.validation.checkImageAndNetworkChannelSizes(I, networkInputSize(3)), images);

vision.internal.cnn.validation.checkTrainingBoxes(images, boxes, isRotatedRectangle);

data = horzcat(images, boxes, labels);
end
