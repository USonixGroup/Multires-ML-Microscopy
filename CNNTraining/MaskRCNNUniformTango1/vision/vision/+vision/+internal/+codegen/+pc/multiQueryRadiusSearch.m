function [nghbrsIdx, numNgbrs] = multiQueryRadiusSearch(meanCoordinates, queryPoints, radius)
% Multi-Query nearest neighbhour in radius search.

% Multi-query radius search implementation. 
% Given an array of query points, this function computes the nearest
% points in a given radius.

% Ref:[1] Martin Magnusson, The three-Dimensional Normal-Distributions
% Transform - an Efficient Representation for Registration, Surface
% Analysis, and Loop Detection, Thesis, 2013

% $matlabroot/toolbox/vision/builtins/src/vision/include/ndtUtils.hpp
% matlab/toolbox/vision/vision/+vision/+internal/+buildable/multiQueryRadiusSearchImpl.m

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

% Number of reference points.
numRefPoints = size(meanCoordinates, 1);
numQryPoints = size(queryPoints, 1);

% Compute number of neighbours for each query point
nghbrsIdx = coder.nullcopy(zeros(numRefPoints, numQryPoints, 'uint32'));
numNgbrs = zeros(numQryPoints, 1, 'uint32');

for qryIter = 1:numQryPoints
    for refIter = 1:numRefPoints
        qryPt = queryPoints(qryIter, :);
        refPt = meanCoordinates(refIter, :);
        diffMat = [qryPt(1) - refPt(1), qryPt(2) - refPt(2), qryPt(3) - refPt(3)];
        diffMat_sum = diffMat(1).^2 + diffMat(2).^2 + diffMat(3).^2;
        
        if diffMat_sum < radius*radius
            oldVal = numNgbrs(qryIter);
            numNgbrs(qryIter) = numNgbrs(qryIter) + uint32(1);
            nghbrsIdx(oldVal+1, qryIter) = refIter;
        end
    end
end
end
