function [bbox,valid] = crop(bbox,win,threshold,inPixel)
% bboxB = crop(bboxA,win,threshold) crops bounding boxes in bboxA to the
% crop window. The threshold must be a positive scalar, <= 1. Bounding
% boxes that overlap crop window boundary are clipped if the amount of
% overlap is at or above the threshold.
%
% Specify an inPixel value of 1 when the bbox is in pixel coordinates.
% Otherwise, it is 0.
%
% bbox and win is a box in the minmax format: [xmin ymin xmax ymax], where
% all coordinates are assumed to be in spatial coordinates.
%
% This function is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.

if nargin == 3
    % assume bbox and win are in spatial coordinates.
    inPixel = 0;
end

xmin = bbox(:,1);
ymin = bbox(:,2);
xmax = bbox(:,3);
ymax = bbox(:,4);

wxmin = win(1);
wymin = win(2);
wxmax = win(3);
wymax = win(4);

if threshold == 1
    % Check that box corners are within cropping window bounds when
    % threshold is 1.
    valid = xmin >= wxmin ...
        & xmax <= wxmax ...
        & ymin >= wymin ...
        & ymax <= wymax;
    
    bbox = bbox(valid,:);
    
else
    % Compute the intersection rectangles.
    ixmin = max(xmin,wxmin);
    iymin = max(ymin,wymin);
    ixmax = min(xmax,wxmax);
    iymax = min(ymax,wymax);
    
    % Compute the width and height of the intersection rectangles. Set the
    % maximum width and height to zero. A non-positive value means boxes do not
    % intersect.
    iwidth  = max(ixmax - ixmin + inPixel, 0);
    iheight = max(iymax - iymin + inPixel, 0);
    
    areaOfIntersection = single(iwidth .* iheight);
    
    ratio = areaOfIntersection ./ (iBoxArea(xmin,ymin,xmax,ymax,inPixel) + eps('single'));
    
    valid = ratio(:,1) >= threshold;
    
    bbox = [ixmin(valid,:) iymin(valid,:) ixmax(valid,:) iymax(valid,:)];
end
%--------------------------------------------------------------------------
function a = iBoxArea(xmin,ymin,xmax,ymax,inPixel)
a = single((xmax-xmin+inPixel).*(ymax-ymin+inPixel));
