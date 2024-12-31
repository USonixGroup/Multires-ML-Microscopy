function [lowerBounds3D,upperBounds3D] = regionProposalBounds3D(obj, boxes, xyzImg)
% regionProposalBounds3D get 3D corners in world coordinates from 2D-boxes
%   boxes is a 5xnumProposal array in format xyxyb (b = batch index).
%   xyzImg is H-by-W-by-3 image containing the X, Y and Z (depth) values in
%   world coordinates corresponding to each pixel in the input RGB image.

% Copyright 2023 The MathWorks, Inc.

    lowerBounds3D = cell(1,length(boxes));
    upperBounds3D = cell(1,length(boxes));

    % Clip roi boxes to image bounds
    if ndims(xyzImg) == 3
        % single image in the batch
        imgSize = size(xyzImg);
        boxes = iClipRoiToImage(boxes, imgSize);
    else
        % handle a batch of multiple images
        numBatches = size(xyzImg, 4);
        for b = 1:numBatches
            selBoxesInBatch = boxes(5,:) == b;
            imgSize = size(xyzImg(:,:,:,b));
            boxes(:,selBoxesInBatch) = iClipRoiToImage(boxes(:,selBoxesInBatch), imgSize);
        end
    end

    % The "XYZ image" contains the 3-D world-coordinates for every pixel
    % position in the input image. For each roi box, taking the min/max of 
    % the corresponding XYZ Image patch will give the 3-D bounds
    % corresponding to that roi in world-coordinates.
    for i = 1:length(boxes)
        if ndims(xyzImg) == 3
            % single image in the batch
            xyzPatch = xyzImg(boxes(2,i):boxes(4,i), ...
                boxes(1,i):boxes(3,i),  :);
        else
            % pick the right image from the batch of images
            batchId = boxes(5,i);
            xyzPatch = xyzImg(boxes(2,i):boxes(4,i), ...
                boxes(1,i):boxes(3,i), :, batchId);
        end
        lowerBounds3D{i} = min(xyzPatch, [], [1 2]);
        upperBounds3D{i} = max(xyzPatch, [], [1 2]);

        % use robust statistics for estimating bounds of the depth image
        % (provides some robustness to noisy depth maps)
        z = iPercentiles(xyzPatch(:,:,3), [0.05 0.95]);
        lowerBounds3D{i}(3) = z(1);
        upperBounds3D{i}(3) = z(2);
    end
    lowerBounds3D = cat(4, lowerBounds3D{:}); % 1 x 1 x 3 x numProposals
    upperBounds3D = cat(4, upperBounds3D{:});
end

function boxes = iClipRoiToImage(boxes, imgSize)
    x1 = boxes(1,:); y1 = boxes(2,:); 
    x2 = boxes(3,:); y2 = boxes(4,:); 
    b = boxes(5,:);

    x1(x1 < 1) = 1;
    y1(y1 < 1) = 1;

    x1(x1>imgSize(2)) = imgSize(2);
    y1(y1>imgSize(1)) = imgSize(1);

    x2(x2 < 1) = 1;
    y2(y2 < 1) = 1;

    x2(x2 > imgSize(2)) = imgSize(2);
    y2(y2 > imgSize(1)) = imgSize(1);

    boxes = [x1; y1; x2; y2; b];
end

function z = iPercentiles(data, p)
    data = data(:);
    z = interp1(linspace(1/numel(data),1,numel(data)), sort(data), p);
end
