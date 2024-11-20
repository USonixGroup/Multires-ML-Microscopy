clear
clc
close all


% Read the image for inference
%img = imread('visionteam.jpg');

% Define the target size of the image for inference
targetSize = [700 700 3];

% Resize the image maintaining the aspect ratio and scaling the largest
% dimension to the target size.
imgSize = size(img);
[~, maxDim] = max(imgSize);
resizeSize = [NaN NaN];
resizeSize(maxDim) = targetSize(maxDim);

img = imresize(img, resizeSize);

% detect the objects and their masks
maskSubnet = helper.extractMaskNetwork(net);
classNames = trainCats;
numClasses = numel(classNames);
% Add a background class
classNames = [classNames {'background'}];
params = createMaskRCNNConfig(imageSize, numClasses, classNames);
disp(params);

params.Threshold

[boxes, scores, labels, masks] = detectMaskRCNN(net, maskSubnet, img, params, "cpu");

% Visualize Predictions

% Overlay the detected masks on the image using the insertObjectMask
% function.
if(isempty(masks))
    overlayedImage = img;
else
    overlayedImage = insertObjectMask(img, masks);
end
figure, imshow(overlayedImage)

% Show the bounding boxes and labels on the objects
showShape("rectangle", gather(boxes), "Label", labels, "LineColor",'r')