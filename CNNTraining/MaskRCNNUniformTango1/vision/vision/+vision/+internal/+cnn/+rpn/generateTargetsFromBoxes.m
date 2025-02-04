function targets = generateTargetsFromBoxes(boxes, imageSize, featureMapSize, params)
%GENERATETARGETSFROMBOXES Given ground truth boxes, image and feature map size
% generate positive box regression targets. This generates intermediate
% region proposals needed for the calculation using the anchor boxes.

% Copyright 2019 The MathWorks, Inc.

% Scale factor from feature to image space.
scaleFactor          = 1./params.ScaleFactor;

% generate box candidates
regionProposals      = vision.internal.cnn.generateAnchorBoxesInImage(...
                           imageSize,featureMapSize,params.AnchorBoxes,scaleFactor);

% convert from k cells to M-by-2 format.
regionProposals      = (vertcat(regionProposals{:}));

matchAllGroundTruth  = true; % every ground truth box should be assigned to a box.
targets              = vision.internal.rcnn.BoundingBoxRegressionModel.generateRegressionTargetsFromProposals(...
                           regionProposals,boxes,params.PositiveOverlapRange,params.NegativeOverlapRange,matchAllGroundTruth);
end
