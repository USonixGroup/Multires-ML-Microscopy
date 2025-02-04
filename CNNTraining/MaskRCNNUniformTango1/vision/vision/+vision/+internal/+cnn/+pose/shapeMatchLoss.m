function loss = shapeMatchLoss(qPred, qTarget, instanceWeightsReg, ptClouds, assignments, batchIds, useClosestPoints)
%SHAPEMATCHLOSS Compute ShapeMatchLoss between predicted and ground-truth
%rotations.
% The ShapeMatch Loss applies the ground-truth and predicted rotations to
% the point cloud of the object, then computes the average L2 distances
% between the closest points of the two rotated point clouds. This
% computation is identical to a one-sided Chamfer distance.
%
% Inputs:
% ------
%   qPred                     Predicted rotation quaternions (numProposals-by-4)
%
%   qTarget                   Target rotation quaternions (numProposals-by-4)
%
%   instanceWeightsReg        Instance weights, 0 for background (numProposals-by-numClasses*4)
%
%   ptClouds        Cell array 1-by-batchSize. Each element is an array of
%                   point clouds N-by-3-by-numGTObjects.
%
%   assignments     Indices of ground truth boxes assigned to
%                   each box. A batchSize-by-1 cell array holding vector of 
%                   length size(boxes{i},1).
%                   If a box is not assigned to a ground truth box, the 
%                   assignment index is 0.
%
%   batchIds        Integers indicating which image a proposal came from.
%
%
%   useClosestPoints   Logical flag to control using closest-points matching
%                      for symmetric shapes.
%
% Reference:
% [1] Yu Xiang, Tanner Schmidt, Venkatraman Narayanan and Dieter Fox,
%     "PoseCNN: A Convolutional Neural Network for 6D Object Pose Estimation
%     in Cluttered Scenes," Robotics: Science and Systems (RSS), 2018.

% Copyright 2023 The MathWorks, Inc.

    arguments
        qPred
        qTarget
        instanceWeightsReg
        ptClouds
        assignments
        batchIds
        useClosestPoints = true
    end

    instanceWeights = max(instanceWeightsReg, [], 3);

    % Select the positive matches between prediction and ground-truth
    selector = squeeze(instanceWeights > 0);
    predQuaternions = squeeze(qPred(1,1,:,selector));        % numClasses*4 x numPositives
    targetQuaternions = squeeze(qTarget(1,1,:,selector));    % numClasses*4 x numPositives
    assignmentsFlat = cat(1, assignments{:});
    assignmentSelected = assignmentsFlat(selector);
    batchIdSelected = batchIds(selector);
    numPositives = size(predQuaternions,2);
    assignedRegWeights = squeeze(instanceWeightsReg(1,1,:,selector));

    minDist = 0;
    for i = 1:numPositives
        % batch and pred-gt association information
        batchIndex = batchIdSelected(i);
        gtIndex = assignmentSelected(i);
        ptCloud = ptClouds{batchIndex}(:,:,gtIndex); % numPoints x 3
        numPoints = size(ptCloud,1);

        thisPredQuaternion = predQuaternions(assignedRegWeights(:,i) > 0, i);
        predShape = vision.internal.quaternion.quaternionToRotation(thisPredQuaternion) * ptCloud';        % 3 x numPoints

        thisTargetQuaternion = targetQuaternions(assignedRegWeights(:,i) > 0, i);
        targetShape = vision.internal.quaternion.quaternionToRotation(thisTargetQuaternion) * ptCloud';    % 3 x numPoints

        if useClosestPoints
            % ---------------------------------------------------------------------
            %   Stop tracing gradients in this block
            % ---------------------------------------------------------------------
            predShapeRaw = extractdata(predShape);
            % For each point in predicted shape, find its closest point in the 
            % target shape.
            [~, minIndices] = pdist2(targetShape', predShapeRaw',...
                "euclidean", "smallest",1);      
            % ---------------------------------------------------------------------
            
            % Gradients are now traced in the closest-distance computation
            nearestTargetPoints = targetShape(:,minIndices);
            minDist = minDist + sum( sum((predShape - nearestTargetPoints).^2) ) / (2*numPoints) ;
        else
            % Compare difference directly between target and prediction
            % points
            minDist = minDist + sum( sum( (predShape - targetShape).^2 ) )/ (2*numPoints) ;
        end
 
    end

    loss = minDist ./ numPositives ;

end

