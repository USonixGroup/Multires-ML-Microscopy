function varargout = sparseBA(optimType, mfilename, varargin)

% The variants of Levenberg-Marquardt are based on Sparse Bundle Adjustment
% (SBA) technical report by Lourakis and Argyros and general framework for
% graph optimization by Kümmerle et al.
% 
% Class syntax (wpSet):
% ---------------------
% Full:           [wpSet, vSet, pointIndex, reprojectionErrors] = sparseBA(...)
% Structure-only: [wpSet, pointIndex, reprojectionErrors] = sparseBA(...)
%
% Non-class syntax (xyzPoints):
% -----------------------------
% Full:           [xyzRefinedPoints, refinedPoses, reprojectionErrors] = sparseBA(...)
% Structure-only: [xyzRefinedPoints, reprojectionErrors] = sparseBA(...)
% Motion-only:    [refinedPose, reprojectionErrors] = sparseBA(...)

% Copyright 2019-2023 The MathWorks, Inc.
%
% References
% ----------
% [1] M.I.A. Lourakis and A.A. Argyros (2009). "SBA: A Software Package for
%     Generic Sparse Bundle Adjustment". ACM Transactions on Mathematical
%     Software (ACM) 36 (1): 1-30.
%
% [2] Kümmerle, R., Grisetti, G., Strasdat, H., Konolige, K., & Burgard, W.
%     (2011). g2o: A general framework for graph optimization. In 2011 IEEE
%     International Conference on Robotics and Automation. pp. 3607-3613.

%#codegen

%==========================================================================
% Data Parsing
%==========================================================================

[xyzPoints, measurements, visibility, intrinsics, ...
maxIterations, absTol, relTol, isUndistorted, verbose, ...
returnPointType, cameraPoses, fixedCameraIndex, solver, wpSet, vSet, mapPointIdx] = ...
vision.internal.bundleAdjust.validateAndParseInputs(optimType, mfilename, varargin{:});

%==========================================================================
% Convert Data Format
%==========================================================================
 
[cameraMatrices, quaternionBases, intrinsics, returnPoseType] = ...
    vision.internal.bundleAdjust.convertInputDataFormat(cameraPoses, ...
    intrinsics, isUndistorted, optimType);

%==========================================================================
% Optimization
%==========================================================================
isFull      = strncmpi(optimType, 'full', 1);
isMotion    = strncmpi(optimType, 'motion', 1);
isClassSyntax = ~isempty(wpSet);
curMeanErr = 0;

% Only full bundle adjustment supports preconditioned-conjugate-gradient solver
 if strncmpi(solver, 'preconditioned-conjugate-gradient', 1)
     
    optimizationOptions = struct('verbose', verbose, ...
        'absoluteTolerance', absTol, ...
        'relativeTolerance', relTol, ...
        'maxIterations', maxIterations, ...
        'Solver', solver);

    [xyzRefinedPoints, refinedPosesTform, reprojError] = vision.internal.buildable.visionG2OBundleAdjustBuildable.bundleAdjustG2O(xyzPoints, measurements, cameraMatrices,...
            quaternionBases, visibility, intrinsics, optimizationOptions, fixedCameraIndex);

    refinedPoses = cameraPoses;
    refinedPoseCg = cameraPoses;

    sizePoses = size(refinedPoseCg.ViewId, 1);
    t = cast([reshape(refinedPosesTform(1:9, 1), 3, 3), ...
        zeros(3, 1); refinedPosesTform(10:12, 1)', 1], returnPoseType);
    poseArr = inOutTypeMatchPreConjugate(cameraPoses, t(1:3,1:3), t(4, 1:3));
    
    for j = 2:sizePoses
        if ~isfield(refinedPoseCg, 'Location')
            t = cast([reshape(refinedPosesTform(1:9, j), 3, 3), ...
                zeros(3, 1); refinedPosesTform(10:12, j)', 1], returnPoseType);
            poseArr = [poseArr, inOutTypeMatchPreConjugate(cameraPoses, t(1:3,1:3), t(4, 1:3))];
        else
            refinedPoseCg.Location{j}     = cast(refinedPosesTform(10:12, j)', returnPoseType);
            refinedPoseCg.Orientation{j}  = cast(reshape(refinedPosesTform(1:9, j), 3, 3), returnPoseType);
        end
    end

    if ~isfield(refinedPoses, 'Location')
        tc = cell(sizePoses, 1);
        for i = 1: sizePoses
            tc{i} = poseArr(i);
        end
        
        refinedPoseCg.AbsolutePose = poseArr;
        refinedPoseTable = table(refinedPoseCg.ViewId, tc, ...
            'VariableNames', {'ViewId', 'AbsolutePose'});
    else
        refinedPoseTable = table(refinedPoseCg.ViewId, ...
            refinedPoseCg.Orientation, refinedPoseCg.Location,...
            'VariableNames', {'ViewId', 'Orientation', 'Location'});
    end
    refinedPoses = refinedPoseCg;

    if isClassSyntax
        posesSize = numel(refinedPoses.AbsolutePose);
        tc = cell(posesSize, 1);
        for i = 1: posesSize
            tc{i} = refinedPoses.AbsolutePose(i);
        end
        viewsTable = table(refinedPoses.ViewId, tc, 'VariableNames',{'ViewId', 'AbsolutePose'});
        vSet = updateView(vSet, viewsTable);
        wpSet = updateWorldPoints(wpSet, mapPointIdx, xyzRefinedPoints);
        varargout{1} = wpSet;
        varargout{2} = vSet;
        varargout{3} = mapPointIdx;
    else
        varargout{1} = cast(xyzRefinedPoints, returnPointType);
        varargout{2} = refinedPoseTable;
    end

    if nargout-isClassSyntax > 2 % Output reprojection error
        % Calculate reprojection error
        curMeanErr = reprojError(1,:).^2+reprojError(2,:).^2;
        reprojectionErrors = vision.internal.bundleAdjust.computeReprojectionErrors(visibility, curMeanErr, 'double');
        varargout{isClassSyntax+3} = reprojectionErrors;
    end

else % 'sparse-linear-algebra'
% List of status code
% Terminating conditions:
%   1 - small gradient ||J'e||_inf
%   2 - small increment ||dp||
%   3 - max iterations
%   4 - small relative reduction in ||e||
%   5 - small ||e||
%   6 - Failed to converge

statusCode = struct('NoStop',               int32(0),...
    'SmallGrad',            int32(1),...
    'SmallIncreX',          int32(2),...
    'MaxIters',             int32(3),...
    'SmallRelDecreFunVal',  int32(4),...
    'SmallAbsFunVal',       int32(5),...
    'NoConverge',           int32(6));

% Damping factors
v               = 2;
mu              = -inf;
tau             = 1e-3;
% Iteration counter
iter            = 0;
% Internal thresholds
gradTol         = 1e-12;
increTol        = 1e-12;

%Stopping flag
stopCondition   = statusCode.NoStop;

if isMotion
    jj = eye(6, 'logical');
elseif isFull
    [numPoints, numViews] = size(visibility);
    jj = (repmat(eye(6), 1, numViews) > 0);
    ii = (repmat(eye(3), 1, numPoints) > 0);
else
    [numPoints, ~] = size(visibility);
    ii = (repmat(eye(3), 1, numPoints) > 0);
end

while (stopCondition == statusCode.NoStop)
    iter =  iter + 1;
    
    if iter > maxIterations
        stopCondition = statusCode.MaxIters;
        break;
    end
    if isMotion
        [errors, Uj, eaj] = vision.internal.buildable.visionSBAAuxiliaryBuildable.visionSBAAuxiliary(xyzPoints, ...
            measurements, cameraMatrices, quaternionBases, visibility, ...
            intrinsics, 'motion', fixedCameraIndex);
        g = eaj(:);
        pL2 = norm(cameraMatrices(:));
    elseif isFull
        [errors, Uj, Vi, Wij, eaj, ebi] = vision.internal.buildable.visionSBAAuxiliaryBuildable.visionSBAAuxiliary(xyzPoints, ...
            measurements, cameraMatrices, quaternionBases, visibility, ...
            intrinsics, 'full', fixedCameraIndex);
        g = [eaj(:); ebi(:)];
        pL2 = norm([cameraMatrices(:); xyzPoints(:)]);
    else
        [errors, Vi, ebi] = vision.internal.buildable.visionSBAAuxiliaryBuildable.visionSBAAuxiliary(xyzPoints, ...
            measurements, cameraMatrices, quaternionBases, visibility, ...
            intrinsics, 'struct', fixedCameraIndex);
       g = ebi(:);
       pL2 = norm(xyzPoints(:));
    end

    curMeanErr = errors(1,:).^2+errors(2,:).^2;
    e1 = sum(curMeanErr);

    meanReprojError = e1 / numel(curMeanErr);

    if ~isfinite(meanReprojError)
        stopCondition = statusCode.NoConverge;
        break;
    end

    if meanReprojError < absTol
        stopCondition = statusCode.SmallAbsFunVal;
        break;
    end

    g_inf = norm(g, Inf);

    if g_inf < gradTol
        stopCondition = statusCode.SmallGrad;
        break;
    end

    if iter == 1
        if isMotion
            jj = eye(6, 'logical');
            mu = max(mu,max(Uj(jj)));
        elseif isFull
            jj = (repmat(eye(6), 1, numViews) > 0);
            ii = (repmat(eye(3), 1, numPoints) > 0);
            mu = max(mu,max(Uj(jj)));
            mu = max(mu,max(Vi(ii)));
        else
            ii = (repmat(eye(3), 1, numPoints) > 0);
            mu = max(mu,max(Vi(ii)));
        end
        mu = tau * mu;
    end
    
    while true
        
        if isMotion
            % Augment U with the increased damping factor
            Uj(jj) = Uj(jj) + mu;

            % Solve for camera poses
            Xa = Uj \ eaj;

            delta = Xa;
        elseif isFull
            % Augment U, V with the increased damping factor
            Uj(jj) = Uj(jj) + mu;
            Vi(ii) = Vi(ii) + mu;
            [S, e, Vii] = vision.internal.buildable.visionSBASchurComplementBuildable.visionSBASchurComplement(Uj, Vi, Wij, eaj, ebi, visibility);

            % Solve for camera poses
            Xa = S \ e(:);
            % Solve for 3-D pints
            Xb = vision.internal.buildable.visionSBASolvePointsBuildable.visionSBASolvePoints(Wij, Xa, Vii, ebi, visibility);
            delta = [Xa; Xb];

            else % isStructure
            % Augment V with the increased damping factor
            Vi(ii) = Vi(ii) + mu;

            % Solve for 3-D pints
            Xb = zeros(3, numPoints);
            for i = 1:numPoints
                Xb(:, i) = Vi(:, (i-1)*3+1:3*i) \ ebi(:, i);
            end
            XbT   = Xb';
            delta = XbT(:);
         end

        if ~isfinite(delta)
            stopCondition = statusCode.NoConverge;
            break;
        elseif (norm(delta) <= increTol * pL2)
            stopCondition = statusCode.SmallIncreX;
            break;
        end

        % Try update camera poses and world points locations
        if isMotion
            newCameraMatrices = cameraMatrices + Xa;
            newXYZPoints = xyzPoints;
        elseif isFull
            newCameraMatrices = cameraMatrices + reshape(Xa, 6, numViews);
            newXYZPoints = xyzPoints + reshape(Xb, 3, numPoints);
        else % isStructure
            newXYZPoints = xyzPoints + Xb;
            newCameraMatrices = cameraMatrices;
        end

        newErrors = vision.internal.buildable.visionSBAAuxiliaryBuildable.visionSBAAuxiliary(newXYZPoints, measurements, ...
                newCameraMatrices, quaternionBases, visibility, intrinsics, ...
                optimType, fixedCameraIndex);

        e2 = sum(newErrors(1,:).^2 + newErrors(2,:).^2);
        dF = e1-e2;
        dL = (delta'*(mu*delta+g));

        if (dL > 0 && dF > 0)
            % Reduction in error, increment is accepted
            tmp = 2*dF/dL-1;
            tmp = 1-tmp^3;
            mu = mu * max(1/3, tmp);
            v = 2;
            if ((sqrt(e1)-sqrt(e2))^2 < relTol*e1)
                stopCondition = statusCode.SmallRelDecreFunVal;
            end
            cameraMatrices = newCameraMatrices;
            xyzPoints      = newXYZPoints;
            break;
        else
            mu = mu*v;
            v2 = 2*v;
            if (v2 <= v) % v has wrapped around, too many failed attempts to increase the damping factor
                stopCondition = statusCode.NoConverge;
                break;
            end
            v = v2;
        end
    end
end
%==========================================================================
% Variable-length Output
%==========================================================================
if isMotion
    cameraMatrices(1:3) = vision.internal.buildable.visionSBAUpdateRotationVectorBuildable.visionSBAUpdateRotation(quaternionBases, cameraMatrices(1:3));
    R = vision.internal.calibration.rodriguesVectorToMatrix(cameraMatrices(1:3));
    t = cameraMatrices(4:6)';
    refinedPose = inOutTypeMatch(cameraPoses, cast(R, returnPoseType), cast(t, returnPoseType));

    varargout{1} = refinedPose;
    if nargout > 1 % Output reprojection error
        varargout{2} = vision.internal.bundleAdjust.computeReprojectionErrors(visibility, curMeanErr, returnPointType);
    end
elseif isFull
    % Refined camera poses
    cameraMatrices(1:3, :) = vision.internal.buildable.visionSBAUpdateRotationVectorBuildable.visionSBAUpdateRotation(quaternionBases, cameraMatrices(1:3, :));

    refinedPosesStr = cameraPoses;

        R = vision.internal.calibration.rodriguesVectorToMatrix(cast(cameraMatrices(1:3, 1), returnPoseType));
        t = cast(cameraMatrices(4:6, 1)', returnPoseType);
        poseArr = inOutTypeMatch(cameraPoses, R, t);

        if isfield(refinedPosesStr, 'Location')
            refinedPosesStr.Location{1}     = -t*R;
            refinedPosesStr.Orientation{1}  = R;
        end
        
        for j = 2:numViews
            R = vision.internal.calibration.rodriguesVectorToMatrix(cast(cameraMatrices(1:3, j), returnPoseType));
            t = cast(cameraMatrices(4:6, j)', returnPoseType);
    
            if ~isfield(refinedPosesStr, 'Location')
                poseArr(j) = inOutTypeMatch(cameraPoses, R, t);
            else
                refinedPosesStr.Location{j}     = -t*R;
                refinedPosesStr.Orientation{j}  = R;
            end
        end
        
        if ~isfield(refinedPosesStr, 'Location')
            tc = cell(numViews, 1);
            for i = 1: numViews
                tc{i} = poseArr(i);
            end
            
            refinedPosesStr.AbsolutePose = poseArr;
            refinedPoseTable = table(refinedPosesStr.ViewId, tc, ...
                'VariableNames', {'ViewId', 'AbsolutePose'});
        else
            refinedPoseTable = table(refinedPosesStr.ViewId, ...
                refinedPosesStr.Orientation, refinedPosesStr.Location,...
                'VariableNames', {'ViewId', 'Orientation', 'Location'});
        end
        refinedPoses = refinedPosesStr;
        if isClassSyntax
            wpSet = updateWorldPoints(wpSet, mapPointIdx, xyzPoints');
            posesSize = numel(refinedPoses.AbsolutePose);
            tc = cell(posesSize, 1);
            for i = 1: posesSize
                tc{i} = refinedPoses.AbsolutePose(i);
            end
            viewsTable = table(refinedPoses.ViewId, tc, 'VariableNames',{'ViewId', 'AbsolutePose'});
            vSet = updateView(vSet, viewsTable);
            varargout{1} = wpSet;
            varargout{2} = vSet;
            varargout{3} = mapPointIdx;
        else
            varargout{1} = cast(xyzPoints', returnPointType);
            varargout{2} = refinedPoseTable;
        end
    

    if nargout-isClassSyntax > 2 % Output reprojection error
        varargout{isClassSyntax+3} = vision.internal.bundleAdjust.computeReprojectionErrors(visibility, curMeanErr, returnPointType);
    end
else
    % isStructure
    if isClassSyntax
         wpSet = updateWorldPoints(wpSet, mapPointIdx, xyzPoints');
         varargout{1} = wpSet;
         varargout{2} = mapPointIdx;
    else
        
         varargout{1} = cast(xyzPoints', returnPointType);
    end

    if nargout-isClassSyntax > 1 % Output reprojection erro
         varargout{isClassSyntax+2} = vision.internal.bundleAdjust.computeReprojectionErrors(visibility, curMeanErr, returnPointType);
     end
end
end
end

%==========================================================================
% Determine output based on input
%==========================================================================
function refinedPose = inOutTypeMatch(cameraPose, R, t)
    hasAbsolutePose = isfield(cameraPose, 'AbsolutePose');
    if hasAbsolutePose
        % Check if object is rigid3d, else it is defaulted to rigidtform3d
        if isa(cameraPose.AbsolutePose, 'images.internal.coder.rigid3d')
            refinedPose = rigid3d(R,-t*R);
        else % must transpose rotation matrix if rigidtform3d
            refinedPose = rigidtform3d(R',-t*R);
        end
    else
        % Check if object is rigid3d, else it is defaulted to rigidtform3d
        if isa(cameraPose, 'images.internal.coder.rigid3d')
            refinedPose = rigid3d(R,-t*R);
        else % must transpose rotation matrix if rigidtform3d
            refinedPose = rigidtform3d(R',-t*R);
        end
    end
end

function refinedPose = inOutTypeMatchPreConjugate(cameraPose, R, t)
    hasAbsolutePose = isfield(cameraPose, 'AbsolutePose');
    if hasAbsolutePose
        % Check if object is rigid3d, else it is defaulted to rigidtform3d
        if isa(cameraPose.AbsolutePose, 'images.internal.coder.rigid3d')
            refinedPose = rigid3d(R, t);
        else % must transpose rotation matrix if rigidtform3d
            pose = rigid3d(R, t);
            refinedPose = rigidtform3d(pose.T');
        end
    else
        % Check if object is rigid3d, else it is defaulted to rigidtform3d
        if isa(cameraPose, 'images.internal.coder.rigid3d')
            refinedPose = rigid3d(R, t);
        else % must transpose rotation matrix if rigidtform3d
            pose = rigid3d(R, t);
            refinedPose = rigidtform3d(pose.T');
        end
    end
end
