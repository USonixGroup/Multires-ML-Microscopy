function autoLabels = exampleAutomationAlgorithmFunction(I)
% autoLabels = exampleAutomationAlgorithmFunction(I) runs a pretrained ACF
% object detector to label people in the input image I. The predicted
% labels are returned in autoLabels, a struct array with fields Name, Type,
% and Position.

% Copyright 2022 The MathWorks, Inc.

% One-time initialization of the detector. A one-time initialization saves
% time on subsequent runs.
persistent detector
if isempty(detector)
    detector = peopleDetectorACF();
end
 
% Run the detector on the input image, I.
bboxes = detect(detector, I);
 
% Create and fill the autoLabels struct with the predicted bounding box
% locations. The Name and Type of ROI returned by the automation function
% must match one of the labels defined in the labeling app.
autoLabels = struct('Name',{},'Type',{}, 'Position',{});
for i = 1:size(bboxes,1)
    autoLabels(i).Name = 'people';
    autoLabels(i).Type = labelType.Rectangle;
    autoLabels(i).Position = bboxes(i,:);
end