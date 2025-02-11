function [normPoints, normalVector] = estimateNormalVectorWithPCA(xyzPoints, numPoints, dims)
%   Estimate normal vector with PCA. This file generates portable code.
%
%   Inputs:
%   -------
%    xyzPoints    : input X-Y-Z column major Nx3 matrix.
%    numPoints    : number of points.
%    dims         : dimension of input points.
%
%   Outputs:
%   -------
%    normPoints   : buffer for storing the normalized points, a Nx3 matrix.
%    normalVector : buffer for normal vector, 1x3 vector.

% Copyright 2021 The MathWorks, Inc.
%#codegen
    
    if nargin < 3
        dims = 3;
    end
    
    % Allocate memory for normPoints and normalVector.
    normalVector = coder.nullcopy(zeros(3, 1, 'like',xyzPoints));
    
    % Calculate the mean value for each coordinate.
    meanValue = zeros(3, 1, 'like', xyzPoints);
    for d = 0 : (dims - 1)
        j = d * numPoints;
        for k = 1 : numPoints
            meanValue(d + 1) = meanValue(d + 1) + xyzPoints(j + k);
        end
    end
    for d = 1:dims
        meanValue(d) = meanValue(d)/double(numPoints);
    end
    
    % Zero-center the input points.
    normPoints = iZeroCenter(xyzPoints, meanValue, numPoints, dims);
    
    % Compute the covariance matrix.
    covariance = iCovarianceMatrix(xyzPoints, normPoints, numPoints, dims);
    
    % Make sure the covariance matrix is symmetric.
    for d = 0:dims-1
        for n = (d+1):2
           covariance(n * dims + (d + 1)) = covariance(d * dims + (n + 1));
        end
    end
    
    % Find the eigen values and vectors.
    covariance = reshape(covariance,3,3);
    [eigenVector, eigenValue, ~] = vision.internal.codegen.pc.eig3(covariance);
    
    % Find the eigen vector associated with the minimum eigen value.
    minEigenValue = cast(eigenValue(1), 'like', xyzPoints) ;
    idx = cast(1, 'uint32');
    for k = 2:3
        if eigenValue(k) < minEigenValue
            minEigenValue = eigenValue(k);
            idx = uint32(k);
        end
    end
    
    for d = 1 : 3
        normalVector(d) = eigenVector((idx - 1) * 3 + d);
    end
end

function normPoints = iZeroCenter(xyzPoints, meanValue, numPoints, dims)
    coder.inline('always');
    normPoints = coder.nullcopy(zeros(size(xyzPoints), 'like',xyzPoints));
    for d = 0:(dims-1)
        j = d * numPoints;
        for k = 1: numPoints
            normPoints(j + k) = xyzPoints(j + k) - meanValue(d + 1);
        end
    end
end

function covariance = iCovarianceMatrix(xyzPoints, normPoints, numPoints, dims)
    coder.inline('always');
    covariance = coder.nullcopy(zeros(9, 1, 'like', xyzPoints));
    for d = 0:(dims-1)
        m = d * numPoints;
        for j = 0:d
            val = zeros(1, 'like', xyzPoints);
            n = j * numPoints;
            for k = 1:numPoints
                val = val + normPoints(m + k) * normPoints(n + k);
            end
            covariance(j * 3 + (d + 1)) = val;
        end
    end
end
