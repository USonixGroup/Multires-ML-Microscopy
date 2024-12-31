function [ T ] = generatePoseRotationTargets(groundTruthRotations, assignment, positiveIdx, numClasses)
% Compute regression targets for per-class rotation quaternion estimation 

% Copyright 2023 The MathWorks, Inc.

    % Only compute targets for proposal assigned to boxes.
    numImagesInBatch = length(groundTruthRotations);
    for i = 1:numImagesInBatch
        numProposals = size(assignment{i},1);
        assignedPositiveGroundTruthRotations = groundTruthRotations{i}(assignment{i}(positiveIdx{i}),:);
        targets = zeros(numProposals, 4, 'like', groundTruthRotations{i});
        targets(positiveIdx{i},:) = assignedPositiveGroundTruthRotations;
        T{i} = repmat(targets,1,numClasses); % -> [4*numClasses N], per class rotation response.
    end
end