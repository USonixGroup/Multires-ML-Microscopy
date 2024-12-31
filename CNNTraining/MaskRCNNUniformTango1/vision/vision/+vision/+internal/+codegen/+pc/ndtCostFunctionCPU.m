function [scoreSumDouble, gradientSumDouble, hessianSumDouble] = ...
    ndtCostFunctionCPU(inPoseArray, ndtArgsStruct)
% NDT Cost Function

% compute the cost of the NDT
% optimization function. The function computes the score, Jacobian, Hessian
% matrices based on input pose matrix.

% [1] Martin Magnusson, The Three-Dimensional Normal-Distributions
% Transform - an Efficient Representation for Registration, Surface
% Analysis, and Loop Detection, Thesis, 2013

% $matlabroot/toolbox/vision/builtins/src/vision/visionNDTComputeScoreDerivatives.cpp
% $matlabroot/toolbox/vision/builtins/src/vision/include/ndtUtils.hpp

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

% Extract the params for the algorithm
qryPoints = ndtArgsStruct.ps;
fixedVoxelMeans = ndtArgsStruct.mvals;
fixedVoxelICov = ndtArgsStruct.iCov;
d1 = ndtArgsStruct.d1;
d2 = ndtArgsStruct.d2;
voxelSize = ndtArgsStruct.radius;

numQryPoints = size(qryPoints, 1);
numRefPoints = size(fixedVoxelMeans, 1);

% Compute Jacobian and Hessian using computeJaHa function
[R,Ja,Ha] = vision.internal.codegen.pc.computeJaHa(inPoseArray);

% Transform the points and compute score, gradient and Hessian

% Tranform the points based on the current rotation and translation.
qryPointsTransformedMat = coder.nullcopy(qryPoints);
for ptIter = 1:numQryPoints
    tmpArr = R * qryPoints(ptIter,1:3)';
    qryPointsTransformedMat(ptIter,:) = tmpArr + inPoseArray(1:3);
end

% Compute neighbours in radius
[nghbrsIdxMat,numNgbrsMat] = vision.internal.codegen.pc.multiQueryRadiusSearch(...
    fixedVoxelMeans, qryPointsTransformedMat, voxelSize);

% Compute score, gradient and Hessian
[scoreSumDouble, gradientSumDouble, hessianSumDouble] = ...
    vision.internal.codegen.pc.computeSGH(qryPoints, qryPointsTransformedMat,...
    Ja, Ha, fixedVoxelMeans, fixedVoxelICov, nghbrsIdxMat, numNgbrsMat, numRefPoints, d1, d2);

end