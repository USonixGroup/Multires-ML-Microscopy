function [response, weights] = yolov2ResponseAndWeights(X, gt, lossFactors,anchorBoxes,iouThresh,gridSize)
% yolov2ResponseWeights returns response and weights based on activations, 
% ground truths. Response is required to compute loss, weights are required 
% to learn positive predictions and neglect negative predictions.
%
% Input X correspond to predictions obtained form activations of last 
% convolutional layer.
%
% Input gt is groundTruth in [x1, y1, x2, y2, objProb, imgSize] format, 
% where (x1, y1) corresponds to top-left coordinate of boxes and (x2,y2) 
% correspond to bottom-right coordinate of the boxes in the image space, 
% objProb is probability of object being present, imgSize is the size of 
% image.
%
% Input lossFactors consists of loss factors defined by user.
%
% Input anchorBoxes is M-by-2 matrix defined by user.
%
% Input iouThresh is threshold to discard predictions with less confidence.
%
% Input gridSize is the dimension size of activations from 
% transform layer.
%
% Output response consists [iou, boxes, classProb] for each anchor boxes in
% every grid, where iou is IoU with respect to ground truth,boxes are 
% boundingBox coordinates, and classProb consists of probabilities of each 
% class. Output weights consists of weights for each of the response i.e 
% [iouWeights, boxWeights, classWeights]

% Copyright 2018 The MathWorks, Inc.

% Obtain initial predictions from transform layer output.
initPred.conf = X(:,1,:,:);
xySigma = X(:,2:3,:,:);
whExp = X(:,4:5,:,:);
initPred.bbox = [xySigma,whExp];
initPred.class = X(:,6:end,:,:);

% Initialize IoU weights.
iouWeights = zeros(size(initPred.conf),'like',gt);

% Compute downSampling factor of network.
downSamplingFactor = gt(1,6:7)./gridSize(1,1:2);

% Scale anchor boxes with respect to feature map size.
anchorBoxes = anchorBoxes./downSamplingFactor;

% Bounding box coordinates.
xyBbox = nnet.internal.cnn.layer.util.yoloPredictBBox(xySigma,whExp,anchorBoxes,gridSize,downSamplingFactor);

% Obtain Bbox dimensions [x, y, w, h] for every anchor.
xyBbox = permute(xyBbox,[1,3,2]);
xyBbox = reshape(xyBbox,[],4);

% xyBbox is converted to [x,y,w,h], to compute IoU with respect to ground truth.
predictedBoxXYWH = vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(xyBbox);
gtXYWH =  vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(gt(:,1:4));

% Compute IoU between predicted boxes and ground truth.
iou = visionBboxIntersectByUnion(single(predictedBoxXYWH), single(gtXYWH));

% Index of anchor with maximum IoU w.r.t ground truth.
maxAnchorIdx = nnet.internal.cnn.layer.util.maxAnchorIdx(gt(:,1:4),anchorBoxes,downSamplingFactor);

% Compute initial confidence weight.
maxIou = max(iou,[],2);
bestIou = reshape(maxIou,size(anchorBoxes,1),1,size(X,3));
confPenalty = initPred.conf;
confPenalty(bestIou<iouThresh)=0-confPenalty(bestIou<iouThresh);
iouWeights(bestIou<=iouThresh) = lossFactors(2)*confPenalty(bestIou<=iouThresh);

% Compute response and update weights.
[response, weights] = nnet.internal.cnn.layer.util.yolov2UpdateResponseWeights(lossFactors,gt,iou,maxAnchorIdx,downSamplingFactor,iouWeights,initPred,anchorBoxes);

end
