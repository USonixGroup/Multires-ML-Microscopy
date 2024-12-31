function boxes = roundGroundTruthBoxes(boxes)
%roundGroundTruthBoxes Round fractional ground truth boxes to nearest pixel
% centers.
%
% See also boxLabelDatastore.

% Copyright 2019 The MathWorks, Inc.

    xymax = round(boxes(:,1:2) + boxes(:,3:4) - 1);
    xymin = round(boxes(:,1:2));
    boxes = [xymin xymax-xymin+1];
end
