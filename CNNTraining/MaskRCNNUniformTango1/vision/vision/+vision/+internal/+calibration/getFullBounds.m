function [xBounds, yBounds] = getFullBounds(undistortedMask, xBoundsBig,...
                                            yBoundsBig)
% getFullBounds Get the bounding box of the central connected component of the
% undistortedMask

% Copyright 2023 The MathWorks, Inc.

%#codegen

    % We have to consider the possibility that there may be more than one
    % connected component in the undistorted image.  Our distortion model can
    % result in an additional ring of valid pixels around the "correct" valid
    % region.
    props = regionprops(logical(undistortedMask), 'Centroid', 'BoundingBox');
    
    % find centroid closest to the center
    center = round(size(undistortedMask) ./ 2);
    dists = zeros(1, numel(props));
    for i = 1:numel(props)
        dists(i) = (props(i).Centroid(1) - center(2)).^2 + ...
            (props(i).Centroid(2) - center(1)).^2;
    end
    [~, idx] = min(dists);
    bbox = props(idx).BoundingBox;
    
    
    xBounds = zeros(1, 2, 'like', xBoundsBig);
    yBounds = zeros(1, 2, 'like', yBoundsBig);
    
    xBounds(1) = ceil(xBoundsBig(1) + bbox(1)-1);
    xBounds(2) = floor(xBounds(1) + bbox(3));
    
    yBounds(1) = ceil(yBoundsBig(1) + bbox(2)-1);
    yBounds(2) = floor(yBounds(1) + bbox(4));
end