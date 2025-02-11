function metrics = compareTrajectories(estimatedPoses, groundTruthPoses, args)

% Copyright 2024 The MathWorks Inc.

%#codegen

arguments
    estimatedPoses {mustBeA(estimatedPoses,"rigidtform3d"),mustBeVector,mustBeNonempty,mustHaveMoreThanThreeElements}
    groundTruthPoses {mustBeA(groundTruthPoses,"rigidtform3d"),mustBeVector,mustBeNonempty,mustHaveSameSize(estimatedPoses,groundTruthPoses)}
    args.AlignmentType string {mustBeTextScalar,validatestring(args.AlignmentType, {'rigid','similarity','none'}, "compareTrajectories")} = "rigid"
    args.EvaluationInterval (1,1) {mustBePositive,mustBeFinite,mustBeNonsparse,mustBeReal} = 1
end

numPoses = numel(groundTruthPoses);
dataType = class(estimatedPoses(1).A);

% Extract the translation component to align the trajectories
if isempty(coder.target())
    groundTruthTranslation = vertcat(groundTruthPoses.Translation);
    estimatedTranslation   = vertcat(estimatedPoses.Translation);
else
    groundTruthTranslation = zeros(numPoses, 3, dataType);
    estimatedTranslation   = zeros(numPoses, 3, dataType);
    for i = 1:numPoses
        groundTruthTranslation(i, :) = groundTruthPoses(i).Translation;
        estimatedTranslation(i, :) = estimatedPoses(i).Translation;
    end
end

% Align the estimated poses with the ground truth poses
if args.AlignmentType == "rigid"
    [align_R, align_t] = computeTransform(estimatedTranslation, groundTruthTranslation, false);
    alignedEstimationTranslation = (align_R * estimatedTranslation' + align_t)';
elseif args.AlignmentType == "similarity"
    [align_R, align_t, scale] = computeTransform(estimatedTranslation, groundTruthTranslation, true);
    alignedEstimationTranslation = (align_R*scale*estimatedTranslation' + align_t)';
else
    align_R = eye(3, dataType);
    alignedEstimationTranslation = estimatedTranslation;
end

% Compute aligned estimated poses
if isempty(coder.target())
    alignedEstimatedPoses = estimatedPoses;
    if args.AlignmentType ~= "none"
        for i = 1:numel(estimatedPoses)
            newRotationMatrix = align_R * estimatedPoses(i).R;
            [u,~,v] = svd(newRotationMatrix); % Make sure newRotationMatrix is a numerically valid rotation matrix
            alignedEstimatedPoses(i).R = u*v';
            alignedEstimatedPoses(i).Translation = alignedEstimationTranslation(i, 1:3);
        end
    end
else
    newRotationMatrix = align_R * estimatedPoses(1).R;
    [u,~,v] = svd(newRotationMatrix); 
    alignedEstimatedPoses = rigidtform3d(u*v', alignedEstimationTranslation(1, :));

    for i = 2:numel(estimatedPoses)
        if args.AlignmentType ~= "none"
            newRotationMatrix = align_R * estimatedPoses(i).R;
            [u,~,v] = svd(newRotationMatrix);
            alignedEstimatedPoses = [alignedEstimatedPoses, rigidtform3d(u*v', alignedEstimationTranslation(i, 1:3))]; %#ok<*AGROW>
        else
            alignedEstimatedPoses = [alignedEstimatedPoses, estimatedPoses(i)];
        end
    end
end

% Compute absolute pose errors
absoluteTranslationError = vecnorm((groundTruthTranslation - alignedEstimationTranslation)');
absoluteRotationError    = zeros(1, numPoses, dataType);
for i = 1:numPoses
    absoluteRotationError(i) = rad2deg(norm(rotmat2vec3d(alignedEstimatedPoses(i).R \ groundTruthPoses(i).R)));
end

% Compute cumulative trajectory length at each pose using the ground truth data
cumDistances = cumsum([0, vecnorm((groundTruthTranslation(2:end,:) - groundTruthTranslation(1:end-1,:))')]);

% Initialize the right bound index
rightBoundIndices = 1:numPoses;

for i = 1:numPoses-1
    segmentDistances = cumDistances(i+1:end) - cumDistances(i);
    [~, shift] = min(abs(segmentDistances - args.EvaluationInterval));
    rightBoundIndices(i) = i + shift;
end

% Compute relative pose errors
relativeTranslationError = zeros(size(absoluteTranslationError), dataType);
relativeRotationError    = zeros(size(absoluteRotationError), dataType);
for i = 1:numPoses
    % Compute relative pose between two relative poses, one from the
    % estimated trajectory, the other one from the ground truth trajectory
    relPoseEst = alignedEstimatedPoses(i).A \ alignedEstimatedPoses(rightBoundIndices(i)).A;
    relPoseGTruth = groundTruthPoses(i).A \ groundTruthPoses(rightBoundIndices(i)).A;
    diffTform = relPoseEst \ relPoseGTruth;
    relativeTranslationError(i) = norm(diffTform(1:3,4));
    relativeRotationError(i) = rad2deg(norm(rotmat2vec3d(diffTform(1:3,1:3))));
end


metrics = trajectoryErrorMetrics.create(alignedEstimationTranslation, groundTruthTranslation,...
    absoluteRotationError, absoluteTranslationError,...
    relativeRotationError, relativeTranslationError);

end

function [R, t, scale] = computeTransform(points1, points2, isSimilarity)
%computeTransform computes a rigid or similarity transformation between two  
%  sets of 3-D points. points1 and points2 must be M-by-3 arrays of [x,y,z] 
%  coordinates.
%
% Umeyama, Shinji. "Least-squares estimation of transformation parameters 
% between two point patterns." IEEE Transactions on Pattern Analysis & 
% Machine Intelligence 13, no. 04 (1991): 376-380

numPoints = size(points1, 1);

% Find data centroid and deviations from centroid
centroid1 = mean(points1);  % equation 34
centroid2 = mean(points2);  % equation 35

normPoints1 = bsxfun(@minus, points1, centroid1);
normPoints2 = bsxfun(@minus, points2, centroid2);

% Covariance matrix
C = normPoints1'*normPoints2/numPoints;  % equation 38

[U,S,V] = svd(C);

% Handle the reflection case
D = diag([1 1 sign(det(U*V'))]); % equation 39

% Compute rotation
R = V*D*U'; % equation 40

% Compute scale
if isSimilarity
    signa_x = mean( sum(normPoints1.^2, 2) ); % equation 36
    scale = trace(S*D)/signa_x; % equation 42
else
    scale = 1;
end

% Compute the translation
t = centroid2' - scale * R*centroid1'; % equation 41
end

function mustHaveMoreThanThreeElements(poses)
coder.internal.errorIf(numel(poses)<3, ...
    "vision:compareTrajectories:mustHaveAtLeastThreePoses");
end


function mustHaveSameSize(poses1, poses2)
coder.internal.errorIf(numel(poses1)~=numel(poses2),...
    "vision:compareTrajectories:mustHaveSameNumberOfPoses");
end
