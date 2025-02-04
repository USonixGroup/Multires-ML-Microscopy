function [lowerBounds3D,upperBounds3D] = poseTranslationBounds3D(boxes,xyzImg)
% bboxBounds3D get 3D corners in world coordinates from 2D-boxes
%   boxes is a numPrediction-by-4 array in format xywh.
%   xyzImg is H-by-W-by-3 image containing the X, Y and Z (depth) values in
%   world coordinates corresponding to each pixel position in the input 
%   RGB image.

% Copyright 2023 The MathWorks, Inc.

    numPreds = size(boxes,1);
    lowerBounds3D = zeros(numPreds, 3);
    upperBounds3D = zeros(numPreds, 3);

    function z = iPercentiles(data, p)
        data = data(:);
        z = interp1(linspace(1/numel(data),1,numel(data)), sort(data), p);
    end

    % Ensure 2D bounding box coordinates are integers
    boxes = round(boxes);
    
    % The "XYZ image" contains the 3-D world-coordinates for every pixel
    % position in the input image. For each 2D bbox, taking the min/max  
    % of the corresponding XYZ Image patch will give the 3-D bounds
    % corresponding to that box in world-coordinates.
    for i = 1:numPreds
        xyzPatch = xyzImg(...
            boxes(i,2):boxes(i,2)+boxes(i,4)-1,...
            boxes(i,1):boxes(i,1)+boxes(i,3)-1,...
            :);
        lowerBounds3D(i,1:2) = min(xyzPatch(:,:,1:2), [], [1 2]);
        upperBounds3D(i,1:2) = max(xyzPatch(:,:,1:2), [], [1 2]);

        % Use robust statistics for estimating bounds of the depth 
        % channel (Z) to provide some robustness to noisy depth maps.
        z = iPercentiles(xyzPatch(:,:,3), [0.05 0.95]);
        lowerBounds3D(i,3) = z(1);
        upperBounds3D(i,3) = z(2);
    end
end
