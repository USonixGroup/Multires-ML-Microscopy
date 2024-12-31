function data = validateImagesAndBoxesTransform(data,colorPreprocessing)
%validateImagesAndBoxesTransform Scans for valid training boxes in the format provided
% by boxLabelDatastore. Scans for valid images using the colorPreprocessing information.
% This also returns data as a cell converting from table, if needed.
%
% See also boxLabelDatastore.

% Copyright 2019 The MathWorks, Inc.

if istable(data)
    images = data{:, 1};
    boxes  = data{:, 2};
    labels = data{:, 3};
else
    images = data(:, 1);
    boxes  = data(:, 2);
    labels = data(:, 3);
end

boxes  = vision.internal.cnn.validation.checkTrainingBoxes(images, boxes);
images = vision.internal.cnn.utils.convertImageToMatchNumberOfNetworkChannels(images, colorPreprocessing);

data = horzcat(images, boxes, labels);
end
