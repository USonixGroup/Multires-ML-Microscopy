function [score, gradient, hessian] = computeSGH(qryPoints,qryPointsTransformedMat,...
    Ja,Ha,fixedVoxelMeans,fixedVoxelICov,nghbrsIdxMat,numNgbrsMat,numRefPoints,d1,d2)
% Compute Score, Gradient and Hessian

% compute the final score, Jacobian
% and Hessian based on the computed coefficients.
% [1] Martin Magnusson, The three-Dimensional Normal-Distributions
% Transform - an Efficient Representation for Registration, Surface
% Analysis, and Loop Detection, Thesis, 2013

% $matlabroot/toolbox/vision/builtins/src/vision/visionNDTComputeScoreDerivatives.cpp
% $matlabroot/toolbox/vision/builtins/src/vision/include/ndtUtils.hpp

% Copyright 2021-2022 The MathWorks, Inc.

%#codegen

numQryPoints = size(qryPoints, 1);
maxNgbrsMat = max(max(numNgbrsMat));
maxNgbrs = maxNgbrsMat(1);

score = zeros(1, 1, 'like', qryPoints);
gradient = zeros(6, 1, 'like', qryPoints);
hessian = zeros(6, 6, 'like', qryPoints);

% If neighbors exist compute S,G & H
if maxNgbrs
    for ptIter = 1:numQryPoints
        for ngbrIter = 1:numNgbrsMat(ptIter)
            % Mean-centering the query point => xk = T(p,x) - Muk, where,
            % x -> qyrPoint
            % T(p,x) -> Transformation of x with poseMat (qryPointTransformed)
            % Muk -> Mean value of the fixed point cloud's voxel (fixedVoxelMean)
            ngbrIdx = nghbrsIdxMat(ngbrIter,ptIter);
            voxelMean = [fixedVoxelMeans(ngbrIdx);...
                fixedVoxelMeans(ngbrIdx + numRefPoints);...
                fixedVoxelMeans(ngbrIdx + 2*numRefPoints)];

            qryPointMeanSubtracted = [qryPointsTransformedMat(ptIter,1) - voxelMean(1);...
                qryPointsTransformedMat(ptIter,2) - voxelMean(2);
                qryPointsTransformedMat(ptIter,3) - voxelMean(3)];

            % Voxel Covariance
            voxelICov = fixedVoxelICov(:,:,ngbrIdx);

            % Product of xk'*Cov*xk
            prod = [0, 0, 0];
            prod(1) = voxelICov(1,1)*qryPointMeanSubtracted(1) + ...
                voxelICov(1,2)*qryPointMeanSubtracted(2) + ...
                voxelICov(1,3)*qryPointMeanSubtracted(3);

            prod(2) = voxelICov(2,1)*qryPointMeanSubtracted(1) + ...
                voxelICov(2,2)*qryPointMeanSubtracted(2) + ...
                voxelICov(2,3)*qryPointMeanSubtracted(3);

            prod(3) = voxelICov(3,1)*qryPointMeanSubtracted(1) + ...
                voxelICov(3,2)*qryPointMeanSubtracted(2) + ...
                voxelICov(3,3)*qryPointMeanSubtracted(3);

            prodValue = qryPointMeanSubtracted(1).*prod(1) + ...
                qryPointMeanSubtracted(2).*prod(2) + ...
                qryPointMeanSubtracted(3).*prod(3);

            % Compute score, refer to Equation 6.9 in [1]
            score = score + double(-d1 * exp(-d2 * (prodValue/2.0)));

            % Evaluate Jacobian and Hessian at this point
            [Jp,Hp] = vision.internal.codegen.pc.jacobianHessian(Ja,Ha,qryPoints(ptIter, :));
            [grad, hess] = vision.internal.codegen.pc.gradientHessian(qryPointMeanSubtracted, voxelICov, d1, d2, Jp, Hp);
            
            for i = 1:6
                gradient(i) = gradient(i) + double(grad(i));
            end
            
            for cIter = 1:6
                for rIter = 1:6                        
                    hessian(rIter, cIter) = hessian(rIter, cIter) + double(hess(rIter, cIter));
                end
            end

        end
    end
    
else
    score = 0;
    gradient = zeros(6, 1, 'like', qryPoints);
    hessian = zeros(6, 6, 'like', qryPoints);
end
end
