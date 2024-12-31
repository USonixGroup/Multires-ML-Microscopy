function [resizedImage, scale] = resizePreservingAspectRatio(img, targetSize, padValue)
    % Compute the scale to resize the groundtruth bounding boxes.

    %   Copyright 2023 The MathWorks, Inc.

    imgSize = size(img);

    % Optimization - Skip resize if targetSize and imgSize already equal.
    if isequal(imgSize(1:2),targetSize)
        resizedImage = img;
        scale = 1.0;
        return
    end

    % Compute Aspect Ratio.
    imgAspectRatio = imgSize(2)/imgSize(1);

    resizeRowsToTargetOutputSize = ceil(imgAspectRatio*targetSize(1));
    resizeColsToTargetOutputSize = ceil(targetSize(2)/imgAspectRatio);
    padSizeIfResizeRowsToTarget = resizeRowsToTargetOutputSize-targetSize(2);
    padSizeIfResizeColsToTarget = resizeColsToTargetOutputSize-targetSize(1);

    % Resize and pad image to final size
    if padSizeIfResizeRowsToTarget < padSizeIfResizeColsToTarget
        scale = targetSize(1)/imgSize(1);
        resizedImage = imresize(img,[targetSize(1),nan]);
        resizedImage(:,end+1:targetSize(2),:) = padValue;
    elseif padSizeIfResizeColsToTarget < padSizeIfResizeRowsToTarget
        scale = targetSize(2)/imgSize(2);
        resizedImage = imresize(img,[nan,targetSize(2)]);
        resizedImage(end+1:targetSize(1),:,:) = padValue;
    else
        scale = targetSize(1)/imgSize(1);
        resizedImage = imresize(img,targetSize(1:2));
    end

end
