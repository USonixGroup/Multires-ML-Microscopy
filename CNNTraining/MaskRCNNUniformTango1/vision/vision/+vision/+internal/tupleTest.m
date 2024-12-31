function indices = tupleTest(points1, points2, scale, isNormalized)
%
%   indices = vision.internal.tupleTest(points1, points2, scale,
%   isNormalized) returns a P-by-1 matrix, indices, containing indices to
%   the matched points in points1 and points2.
%
%   points1 and points2 are matrices of size M1-by-N matrix and an M2-by-N.
%   scale is a positive scalar value of range (0,1).
%   isNormalized is a boolean, set this to true, to perform normalization
%   of input points.

%  Copyright 2020 The MathWorks, Inc.
%#codegen

% Determine point class
if (isa(points1, 'double'))
    ptClass = 'double';
else
    ptClass = 'single';
end

% Cast all to have same class
points1In = cast(points1, ptClass);
points2In = cast(points2, ptClass);
tupleScale = cast(scale, ptClass);

% Normalize the points
if(isNormalized)
     [normalizedA, normalizedB]  = jointNormalize(points1In, points2In);
else
    normalizedA = points1In;
    normalizedB = points2In;
end

numPoints = size(points1, 1);
numIters = uint32(size(normalizedA, 1) * 100);
% Generate random indices
randInds = randi(numPoints, [1 numIters*3], 'uint32') - 1;

% Perform tuple test
if isSimMode
    indices = vision.internal.tupleTestImpl(normalizedA, normalizedB, ...
        randInds, tupleScale, numIters);
else
    indices = vision.internal.buildable.tupleTestBuildable.tupleTestImpl(...
        normalizedA, normalizedB, randInds, tupleScale, numIters);
end
indices = find(indices);

%==========================================================================
% Normalize input points
% X = (X-mu)/scale
%==========================================================================
function [normalizedA, normalizedB]  = jointNormalize(ptA, ptB)

ptAMean = mean(ptA);
ptBMean = mean(ptB);

normalizedA = bsxfun(@minus, ptA, ptAMean);
normalizedB = bsxfun(@minus, ptB, ptBMean);

ptANorm = max(sqrt(sum(normalizedA.^2, 2)));
ptBNorm = max(sqrt(sum(normalizedB.^2, 2)));
scale   = max(ptANorm, ptBNorm);

if(scale <= eps(cast(1, class(scale))))
    normalizedA = ptA;
    normalizedB = ptB;
else
    normalizedA = bsxfun(@rdivide, normalizedA, scale);
    normalizedB = bsxfun(@rdivide, normalizedB, scale);
end

%==========================================================================
function flag = isSimMode()

flag = isempty(coder.target);
