function [scores, labels, masks, keepIdxs] = matrixNMS(masks, labels, scores, maskArea, numPreNMS, maxPerImg, kernel, sigma, filterThresh)
%#codegen

% Copyright 2023-2024 The MathWorks, Inc.

% Step 1: Consider only top nms_pre pedictions
[scores, sortedIdx] = sort(scores,'descend');
numPreNMS = min(numPreNMS, length(labels));
keepIdxs = sortedIdx(1:numPreNMS);
scores = scores(1:numPreNMS);
masks = masks(:,:, keepIdxs);
maskArea = maskArea(keepIdxs);
labels = labels(keepIdxs);

% Step 2: Flatten out the masks & compute iou in vectorized form
numMasks = length(labels);
flatmasks = reshape(masks, [], numMasks);

intersection = flatmasks'*flatmasks;
expanded_area = repmat(maskArea, [1 numMasks]);
union = expanded_area+expanded_area'-intersection;

ious = triu(intersection./union, 1);

% Step 3: Label specific matrix
expanded_labels = repmat(labels, [1 numMasks]);
label_matrix = triu((expanded_labels == expanded_labels'),1);

% Step 4:  Iou compensation
compensate_iou = max(ious.*label_matrix);
compensate_iou = repmat(compensate_iou', [1, numMasks]);

% Step 5: IOU decay
decay_iou = ious.*label_matrix;

if(strcmp(kernel,'gaussian'))
    decay_matrix = exp(-1*sigma*(decay_iou.^2));
    compensate_matrix = exp(-1*sigma*(compensate_iou.^2));
    decay_coefficient = min(decay_matrix./compensate_matrix);

elseif(strcmp(kernel,'linear'))

else
    assert(false,'Unsupported kernel');
end

% Update scores
scores = scores.*decay_coefficient';

% Threshold Scores
keep = scores > filterThresh;
keepIdxs = keepIdxs(keep);

scores = scores(keep);
masks = masks(:,:,keep);
labels = labels(keep);
