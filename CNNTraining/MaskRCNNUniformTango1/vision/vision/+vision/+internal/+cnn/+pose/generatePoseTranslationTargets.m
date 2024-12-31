function [ T ] = generatePoseTranslationTargets(gTranslations,assignment,positiveIdx,proposals,intrinsics,xyzImg,gTruthBoxes,numClasses)
% Compute regression targets for per-class 3D translation estimation

% Copyright 2023 The MathWorks, Inc.

    % Only compute targets for proposal assigned to boxes. 
    numImagesInBatch = length(gTranslations);
    for i = 1:numImagesInBatch
        numProposals = size(assignment{i},1);

        gBoxCenters = cat(2, gTruthBoxes{i}(:,1) + 0.5*gTruthBoxes{i}(:,3),...
            gTruthBoxes{i}(:,2) + 0.5*gTruthBoxes{i}(:,4));

        % Translation targets XY - 2D centroid coordinates as offsets
        % relative to proposal bounding boxes
        centroids2D = single(world2img(gTranslations{i}, rigidtform3d(), intrinsics{i}));
        invalidSel = or( or( or( centroids2D(:,1) < 1, centroids2D(:,2) < 1),...
            centroids2D(:,1) >  size(xyzImg{i},2)), centroids2D(:,2) > size(xyzImg{i},1));
        centroids2D(invalidSel,:) = gBoxCenters(invalidSel,:); % replace invalid 2D centroids with gTruth box centers

        assignedCentroids2D = centroids2D(assignment{i}(positiveIdx{i}),:);
        bboxes = proposals{i}(positiveIdx{i},1:4);
        translationOffsetsXY = vision.internal.cnn.pose.poseTranslationTargetsXY(bboxes,assignedCentroids2D);

        % Translation targets Z - offsets relative to minimum and maximum
        % bound on the depth
        [lowerBoundZ,upperBoundZ] = vision.internal.cnn.pose.poseTranslationBoundsZ(...
            bboxes,xyzImg{i},assignedCentroids2D,intrinsics{i}.K);
        translationOffsetsZ = vision.internal.cnn.pose.poseTranslationTargetsZ(...
            lowerBoundZ,upperBoundZ,gTranslations{i}(assignment{i}(positiveIdx{i}),3));

        assignedPositiveGroundTruthTranslations = cat(2, translationOffsetsXY, translationOffsetsZ);

        targets = zeros(numProposals, 3, 'like', gTranslations{1});
        targets(positiveIdx{i},:) = assignedPositiveGroundTruthTranslations;
        T{i} = repmat(targets,1,numClasses); % -> [3*numClasses N], per class translation response. 
    end
end