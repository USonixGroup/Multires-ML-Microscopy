function bbox = minmax2xywh(bbox)
% Convert minmax bbox in spatial coordinates to xywh bbox in pixel
% coordinates.
%
% The width and height of the output bbox is saturated to 1. Therefore any
% box with sub-pixel extent will always be returned as box with width
% height of 1. This matches the behavior of IMRESIZE and IMWARP where if
% you transform the image to sub-pixel extent, the output size is always
% 1x1.
%
% N.B. This function assumes the box coordinates have already been
% clipped.
%
% This function is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.

% Round to nearest pixel center.
bbox = max(1,round(bbox));

bbox(:,[3 4]) = bbox(:,[3 4]) - bbox(:,[1 2]) + 1;