function idx = maxAnchorIdx(gt,anchors,downSamplingFactor)
% maxAnchorIdx returns index of anchor with maximum IoU with respect to
% ground truth.
%
% Input gt is groundTruth in [x1, y1, x2, y2, objProb, imgSize] format, 
% where (x1, y1) corresponds to top-left coordinate of boxes and (x2,y2) 
% correspond to bottom-right coordinate of the boxes in the image space, 
% objProb is probability of object being present, imgSize is the size of 
% image.
%
% Input anchors is M-by-2 matrix defined by user.
%
% Input downSamplingFactor is a 1-by-2 matrix, with first element
% downsampling in y direction and second element downsampling in x
% direction.
%
% Output idx contains index of anchor with maximum IoU with respect to 
% ground truth.

% Copyright 2018 The MathWorks, Inc.

resizeGT = [gt(:,1)/downSamplingFactor(1,2),gt(:,2)/downSamplingFactor(1,1),gt(:,3)/downSamplingFactor(1,2),gt(:,4)/downSamplingFactor(1,1)];
anchorIoU = zeros(size(anchors,1),size(gt,1),'like',gt);
gtXYWH =  vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(resizeGT);

for gtIdx= 1:size(resizeGT,1)
    
    % Compute anchor center with respect to ground truths.
    anchorCenterX = (gtXYWH(gtIdx,1)+gtXYWH(gtIdx,3)/2);
    anchorCenterX  = repmat(anchorCenterX, [size(anchors,1),1]);
    anchorCenterX = anchorCenterX - (anchors(:,1)/2);
    
    % Compute anchor center with respect to ground truths.
    anchorCenterY = (gtXYWH(gtIdx,2)+gtXYWH(gtIdx,4)/2);
    anchorCenterY = repmat(anchorCenterY,[size(anchors,1),1]);
    anchorCenterY = anchorCenterY - (anchors(:,2)/2);
    
    anchorBox = [anchorCenterX,anchorCenterY,anchors];
    
    % AnchorIoU is a M-by-N matrix, where M is size(anchors,1) and N is 
    % size(gt,1).
    anchorIoU(:,gtIdx) = vision.internal.cnn.boxAssignmentUtils.bboxOverlapIoU(anchorBox,gtXYWH(gtIdx,:));
end

% Get index of anchor with maximum IOU.
[~,idx] = max(anchorIoU,[],1);

end
