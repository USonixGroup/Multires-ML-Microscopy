function [response, weights] = yolov2UpdateResponseWeights(lossFactors,gt,iou,...
    maxAnchorIdx,downSamplingFactor,iouWeights,initPred,anchors)
% yolov2UpdateResponseWeights returns response and updated weights.
%
% Input lossFactors consists of loss factors defined by user.
%
% Input gt is groundTruth in [x1, y1, x2, y2, objProb, imgSize] format, 
% where (x1, y1) corresponds to top-left coordinate of boxes and (x2,y2) 
% correspond to bottom-right coordinate of the boxes in the image space, 
% objProb is probability of object being present, imgSize is the size of 
% image.
%
% Input iou consists of IOU between predicted boxes and ground truths.
%
% Input maxAnchorIdx consists index of anchor with maximum IoU w.r.t ground 
% truth.
%
% Input downSamplingFactor is a 1-by-2 matrix, with first element
% downsampling in y direction and second element downsampling in x
% direction.
%
% Input iouWeights contains initial IoU weights.
%
% Input initPred contains predicted bounding box, confidence, class values.
%
% Input anchors is M-by-2 matrix defined by user.
%
% Output response consists [ iou, boxes, classProb] for each anchor boxes in
% every grid, where iou is IoU with respect to ground truth, boxes are 
% boundingBox coordinates, and classProb consists of probabilities of each 
% class and Output weights consists of weights for each of the response i.e 
% [iouWeights, boxWeights, classWeights];

% Copyright 2018 The MathWorks, Inc.

outWidth = gt(1,7)/downSamplingFactor(1,2);

% Extract losses defined in LossFactors.
objectScale = lossFactors(1);
coordScale = lossFactors(3);
classScale = lossFactors(4);

% Initialize response parameters.
boxes = zeros(size(initPred.bbox),'like',gt);
boxes(:,1:2,:)=0.5;
boxes(:,3:4,:)=1.0;
ious = zeros(size(initPred.conf),'like',gt);
classProb = zeros(size(initPred.class),'like',gt);

% Initialize weight parameters.
boxWeights = zeros(size(initPred.conf),'like',gt)+0.01;
classWeights = zeros(size(initPred.conf),'like',gt);

% Map ground truth to respective grid cell index.
cellWidth = downSamplingFactor(1,2);
cellHeight = downSamplingFactor(1,1);
cx = (gt(:,1)+gt(:,3))*0.5/cellWidth;
cy = (gt(:,2)+gt(:,4))*0.5/cellHeight;
cellIdx = round((floor(cy)*outWidth)+floor(cx))+1;

% Offset per grid with respect to ground truth.
targetBoxes = zeros(size(gt(:,1:4)),'like',gt);
targetBoxes(:,1)=cx-floor(cx);
targetBoxes(:,2)=cy-floor(cy);
targetBoxes(:,3)=(gt(:,3)-gt(:,1))/cellWidth;
targetBoxes(:,4)=(gt(:,4)-gt(:,2))/cellHeight;

% Reshape IoUs to get IoU value for each of the anchor in every grid.
reshapedIou = permute(iou,[2,1]);
reshapedIou = reshape(reshapedIou,size(gt,1),size(initPred.conf,1),size(initPred.conf,3));
reshapedIou = permute(reshapedIou,[2,1,3]);

% Associate bounding box coordinates, IoUs, class probabilities at
% appropriate cell, anchor indices.
for i=1:size(cellIdx,1)
    if cellIdx(i)<=size(initPred.conf,3) && cellIdx(i)>0
        maxAnchor = maxAnchorIdx(i);
        maxAnchorIou = initPred.conf(maxAnchor,:,cellIdx(i));
        
        targetBoxes(i,3) = targetBoxes(i,3)/anchors(maxAnchor,2);
        targetBoxes(i,4) = targetBoxes(i,4)/anchors(maxAnchor,1);
        
        boxes(maxAnchor,:,cellIdx(i)) = targetBoxes(i,1:4);
        ious(maxAnchor,:,cellIdx(i)) = reshapedIou(maxAnchor,i,cellIdx(i));        
        classProb(maxAnchor,gt(i,5),cellIdx(i)) = 1;        
        
        boxWeights(maxAnchor,:,cellIdx(i)) = coordScale;        
        iouWeights(maxAnchor,:,cellIdx(i)) = objectScale * (1 - maxAnchorIou);
        classWeights(maxAnchor,:,cellIdx(i)) = classScale;
    end
end
response = [ious,boxes,classProb];

boxWeights = repmat(boxWeights,1,size(initPred.bbox,2));
classWeights = repmat(classWeights,1,size(initPred.class,2));
weights = [iouWeights,boxWeights,classWeights];

end
