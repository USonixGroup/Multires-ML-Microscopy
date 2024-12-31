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
if isempty(coder.target)

%==========================================================================
% Data Parsing
%==========================================================================
[xyzPoints, measurements, visibility, intrinsics, ...
    maxIterations, absTol, relTol, isUndistorted, verbose, ...
    returnPointType, cameraPoses, fixedCameraIndex, solver, wpSet, vSet, mapPointIdx] = ...
    vision.internal.bundleAdjust.validateAndParseInputs(optimType, mfilename, varargin{:});

printer = vision.internal.MessagePrinter.configure(verbose);
printer.printMessage('vision:sfm:sbaInitialization');

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

printer.printMessage('vision:sfm:sbaStart');

% Only full bundle adjustment supports preconditioned-conjugate-gradient solver
if strncmpi(solver, 'preconditioned-conjugate-gradient', 1)
    optimizationOptions = struct('verbose', verbose, ...
        'absoluteTolerance', absTol, ...
        'relativeTolerance', relTol, ...
        'maxIterations', maxIterations, ...
        'Solver', solver);

    [xyzRefinedPoints, refinedPosesTform, reprojError] = visionG2OBundleAdjust(xyzPoints, measurements, cameraMatrices,...
        quaternionBases, visibility, intrinsics, optimizationOptions, fixedCameraIndex);

    refinedPoses = cameraPoses;
    hasAbsolutePose = ismember('AbsolutePose', refinedPoses.Properties.VariableNames);
    for j = 1:size(refinedPoses, 1)
        if hasAbsolutePose
            refinedPoses.AbsolutePose(j).T = cast([reshape(refinedPosesTform(1:9, j), 3, 3), ...
                zeros(3, 1); refinedPosesTform(10:12, j)', 1], returnPoseType);
        else
            refinedPoses.Location{j}     = cast(refinedPosesTform(10:12, j)', returnPoseType);
            refinedPoses.Orientation{j}  = cast(reshape(refinedPosesTform(1:9, j), 3, 3), returnPoseType);
        end
    end

    if isClassSyntax
        if ~isempty(wpSet.PointIds)
            varargout{1} = updateWorldPoints(wpSet, wpSet.PointIds(mapPointIdx), xyzRefinedPoints);
        else
            varargout{1} = updateWorldPoints(wpSet, mapPointIdx, xyzRefinedPoints);
        end
        varargout{2} = updateView(vSet, refinedPoses);
        varargout{3} = mapPointIdx;
    else
        varargout{1} = cast(xyzRefinedPoints, returnPointType);
        varargout{2} = refinedPoses;
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

% Stopping flag
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

% Disable the warnings about conditioning for singular and
% nearly singular matrices

wState  = warning;                           % Save warning state
cleanUp = onCleanup(@() warning(wState));    % Restore warning state at exit

warning('off','MATLAB:nearlySingularMatrix');
warning('off','MATLAB:singularMatrix');
warning('off','MATLAB:rankDeficientMatrix');

while (stopCondition == statusCode.NoStop)
    iter =  iter + 1;
    if iter > maxIterations
        stopCondition = statusCode.MaxIters;
        break;
    end

    printer.printMessageNoReturn('vision:sfm:sbaIteration', iter);

    % Compute derivative submatrices A_ij (w.r.t camera poses), B_ij (w.r.t points)
    % See Fig.4 in [1]
    if isMotion
        [errors, Uj, eaj] = visionSBAAuxiliaryVariable(xyzPoints, ...
            measurements, cameraMatrices, quaternionBases, visibility, ...
            intrinsics, 'motion');
        g = eaj(:);
        p_L2 = norm(cameraMatrices(:));
    elseif isFull
        [errors, Uj, Vi, Wij, eaj, ebi] = visionSBAAuxiliaryVariable(xyzPoints, ...
            measurements, cameraMatrices, quaternionBases, visibility, ...
            intrinsics, 'full', fixedCameraIndex);
        g = [eaj(:); ebi(:)];
        p_L2 = norm([cameraMatrices(:); xyzPoints(:)]);
    else % isStructure
        [errors, Vi, ebi] = visionSBAAuxiliaryVariable(xyzPoints, ...
            measurements, cameraMatrices, quaternionBases, visibility, ...
            intrinsics, 'structure', fixedCameraIndex);
        g = ebi(:);
        p_L2 = norm(xyzPoints(:));
    end

    curMeanErr = errors(1,:).^2+errors(2,:).^2;
    if iter == 1
        initMeanErr = curMeanErr;
    end
    e1 = sum(curMeanErr);

    meanReprojError = e1 / numel(curMeanErr);
    if verbose
        printer.printMessage('vision:sfm:sbaMeanSquareError', num2str(meanReprojError));
    end

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

            [S, e, Vii] = visionSBASchurComplement(Uj, Vi, Wij, eaj, ebi, visibility);

            % Solve for camera poses
            Xa = S \ e(:);

            % Solve for 3-D pints
            Xb = visionSBASolvePoints(Wij, Xa, Vii, ebi, visibility);

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
        elseif (norm(delta) <= increTol * p_L2)
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

        % Evaluate function value
        newErrors = visionSBAAuxiliaryVariable(newXYZPoints, measurements, ...
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

% Print the final metric
if verbose
    switch stopCondition
        case statusCode.SmallGrad
            printer.printMessage('vision:sfm:sbaStopCondSmallGrad');
        case statusCode.SmallIncreX
            printer.printMessage('vision:sfm:sbaStopCondSmallChangeOfX');
        case statusCode.MaxIters
            printer.printMessage('vision:sfm:sbaStopCondMaxIteration');
        case statusCode.SmallRelDecreFunVal
            printer.printMessage('vision:sfm:sbaStopCondSmallRelChangeOfFunVal');
        case statusCode.SmallAbsFunVal
            printer.printMessage('vision:sfm:sbaStopCondSmallAbsFunVal');
        case statusCode.NoConverge
            printer.printMessage('vision:sfm:sbaStopCondNotConverge');
    end

    printer.printMessage('vision:sfm:sbaReportInitialError', num2str(mean(sqrt(initMeanErr))));
    printer.printMessage('vision:sfm:sbaReportFinalError', num2str(mean(sqrt(curMeanErr))));
end
%==========================================================================
% Variable-length Output
%==========================================================================
if isMotion
    cameraMatrices(1:3) = visionSBAUpdateRotationVector(quaternionBases, cameraMatrices(1:3));
    R = vision.internal.calibration.rodriguesVectorToMatrix(cast(cameraMatrices(1:3), returnPoseType));
    t = cast(cameraMatrices(4:6)', returnPoseType);

    % Determines the output type based on the input type. In motion mode,
    % cameraPoses is a rigidtform3d or rigid3d object.
    refinedPose = inOutTypeMatch(cameraPoses, R, t);
    varargout{1} = refinedPose;
    if nargout > 1 % Output reprojection error
        varargout{2} = vision.internal.bundleAdjust.computeReprojectionErrors(visibility, curMeanErr, returnPointType);
    end
elseif isFull
    % Refined camera poses
    cameraMatrices(1:3, :) = visionSBAUpdateRotationVector(quaternionBases, cameraMatrices(1:3, :));
    refinedPoses = cameraPoses;

    hasAbsolutePose = ismember('AbsolutePose', refinedPoses.Properties.VariableNames);
    for j = 1:numViews
        R = vision.internal.calibration.rodriguesVectorToMatrix(cast(cameraMatrices(1:3, j), returnPoseType));
        t = cast(cameraMatrices(4:6, j)', returnPoseType);

        if hasAbsolutePose
            % Determine the output type based on the input type.
            refinedPoses.AbsolutePose(j) = inOutTypeMatch(refinedPoses{j,'AbsolutePose'}, R, t);
        else
            refinedPoses.Location{j}     = -t*R;
            refinedPoses.Orientation{j}  = R;
        end
    end

    if isClassSyntax
        if ~isempty(wpSet.PointIds)
            varargout{1} = updateWorldPoints(wpSet, wpSet.PointIds(mapPointIdx), xyzPoints');
        else
            varargout{1} = updateWorldPoints(wpSet, mapPointIdx, xyzPoints');
        end
        varargout{2} = updateView(vSet, refinedPoses);
        varargout{3} = mapPointIdx;
    else
        varargout{1} = cast(xyzPoints', returnPointType);
        varargout{2} = refinedPoses;
    end

    if nargout-isClassSyntax > 2 % Output reprojection error
        varargout{isClassSyntax+3} = vision.internal.bundleAdjust.computeReprojectionErrors(visibility, curMeanErr, returnPointType);
    end

else % isStructure
    if isClassSyntax
        if ~isempty(wpSet.PointIds)
            varargout{1} = updateWorldPoints(wpSet, wpSet.PointIds(mapPointIdx), xyzPoints');
        else
            varargout{1} = updateWorldPoints(wpSet, mapPointIdx, xyzPoints');
        end
        varargout{2} = mapPointIdx;
    else
        varargout{1} = cast(xyzPoints', returnPointType);
    end

    if nargout-isClassSyntax > 1 % Output reprojection error
        varargout{isClassSyntax+2} = vision.internal.bundleAdjust.computeReprojectionErrors(visibility, curMeanErr, returnPointType);
    end
end
end
else
    [varargout{1:nargout}] = vision.internal.codegen.bundleAdjust.sparseBA(optimType, mfilename, varargin{:});
end
end

%==========================================================================
% Determine output based on input
%==========================================================================
function refinedPose = inOutTypeMatch(cameraPose, R, t)
    % Check if object is rigid3d, else it is defaulted to rigidtform3d
    if isa(cameraPose, 'rigid3d')
        refinedPose = rigid3d(R,-t*R);
    else % must transpose rotation matrix if rigidtform3d
        refinedPose = rigidtform3d(R',-t*R);
    end
end
