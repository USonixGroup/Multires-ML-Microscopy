function offsets2D = poseTranslationTargetsXY(boxes,centroids2D)
% Calculate the X,Y offsets of 2D centroids relative to their 2D bounding boxes

% Copyright 2023 The MathWorks, Inc.

offsets2D = zeros(size(boxes, 1), 2);
% boxes are in format xywh
offsets2D(:,1) = (centroids2D(:,1) - boxes(:,1)) ./ (boxes(:,3) + 1e-6);
offsets2D(:,2) = (centroids2D(:,2) - boxes(:,2)) ./ (boxes(:,4) + 1e-6);

offsets2D(offsets2D > 1) = 1;
offsets2D(offsets2D < 0) = 0;

end