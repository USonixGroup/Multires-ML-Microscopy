classdef visionG2OBundleAdjustBuildable < coder.ExternalDependency %#codegen
% visionG2OBundleAdjustBuildable - encapsulate visionG2OBundleAdjust implementation library

% Copyright 2021 The MathWorks, Inc.
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'visionG2OBundleAdjustBuildable';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo, context)
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                                                    'bundleAdjust',{});
            vision.internal.buildable.portableOptimizePosesBuildInfo(buildInfo, context);
        end

        function [xyzRefinedPoints, refinedPosesTform, reprojError] = ...
                bundleAdjustG2O(xyzPoints, measurements, cameraMatrices,...
                quaternionBases, visibility, intrinsics, optimizationOptions, fixedCameraIndex)

            coder.inline('always');
            coder.cinclude('cvstCG_bundleAdjust.h');

            fcnName = 'visionbundleAdjust';
            numPoints = int32(size(xyzPoints, 2));
            numMeasures = int32(size(measurements, 2));
            
            numViews = int32(size(cameraMatrices, 2));
            
            % computing irs and jcs array for sparse array
            [irsd, ~] = find(visibility);
            irs = int32(irsd-1);
            jcsd = zeros(1, size(visibility, 2) + 1);
            jcsd(1) = 0;
            for i = 2:size(visibility, 2) + 1
                jcsd(i) = nnz(visibility(:, i - 1)) + jcsd(i - 1); 
            end

            jcs = int32(jcsd);
            
            % obtaining optimization options from NV pairs input
            absTol = optimizationOptions.absoluteTolerance;
            relTol = optimizationOptions.relativeTolerance;
            maxIterations = int32(optimizationOptions.maxIterations);
            
            % intrinsics input to internal function call
            focal = intrinsics.focalLength;
            center = intrinsics.principalPoint;
            radDistort = intrinsics.radialDistortion;
            tangDistort = intrinsics.tangentialDistortion;
            skew = intrinsics.skew;

            radialCoefs = int32(numel(radDistort));
            numCameraIndices = int32(numel(fixedCameraIndex));
            bSingleCamera = isscalar(intrinsics);
            
            % initialize outputs
            xyzRefinedPoints = zeros([numPoints, 3]);
            refinedPosesTform = zeros([12, numViews]);
            reprojError = zeros([2, numMeasures]);

            if coder.isColumnMajor
                % call internal functions to compute refined points and
                % reprojection errors in column major format
                coder.ceval(fcnName, numPoints, coder.ref(xyzPoints), ...
                    coder.ref(measurements), numMeasures, numViews, ...
                    coder.ref(cameraMatrices), coder.ref(quaternionBases), ...
                    bSingleCamera, irs, jcs, absTol, relTol, maxIterations, ...
                    coder.ref(fixedCameraIndex), numCameraIndices, focal, ...
                    center, skew, coder.ref(radDistort), ...
                    coder.ref(tangDistort), radialCoefs, coder.ref(xyzRefinedPoints), ...
                    coder.ref(refinedPosesTform), coder.ref(reprojError));
            else
                xyzRefinedPointsRow = zeros([3, numPoints]);
                refinedPosesTformRow = zeros([numViews, 12]);
                reprojErrorRow = zeros([numMeasures, 2]);

                % call internal functions to compute refined points and
                % reprojection errors in row major format
                coder.ceval('-row', fcnName, numPoints, xyzPoints', ...
                    measurements', numMeasures, numViews, ...
                    cameraMatrices', quaternionBases', ...
                    bSingleCamera, irs', jcs', absTol, relTol, maxIterations, ...
                    coder.ref(fixedCameraIndex), numCameraIndices, focal',...
                    center', skew, radDistort, ...
                    tangDistort, radialCoefs, coder.ref(xyzRefinedPointsRow), ...
                    coder.ref(refinedPosesTformRow), coder.ref(reprojErrorRow));
                
                xyzRefinedPoints = xyzRefinedPointsRow';
                refinedPosesTform = refinedPosesTformRow';
                reprojError = reprojErrorRow';
            end
        end
    end
end