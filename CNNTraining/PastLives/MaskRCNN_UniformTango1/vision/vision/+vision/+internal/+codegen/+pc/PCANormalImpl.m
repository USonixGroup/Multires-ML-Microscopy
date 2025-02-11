function mNormals = PCANormalImpl(mPoints, mIndices, mValid, M, numNeighbors)
%   Compute normal vector with PCA. This file generates portable code.
%   simulation and shared library code generation version is found in
%   PCANormal.hpp
%
%   Inputs:
%   -------
%    mPoints    : input Mx3 matrix (single or double)
%    mIndices   : [NUM of Neighbors]xM uint32 matrix, 
%                   indices of neighbor points in mPoints.
%    mValid     : Mx1 vector indicating the actual number of neighbors 
%                   in indices.
%   Outputs:
%   -------
%    mNormals   : Mx3 matrix (single or double), normals for each points.

% Copyright 2021 The MathWorks, Inc.
%#codegen

    % Create buffer for neighboring points
    neighbors = coder.nullcopy(zeros(numNeighbors*3, 1, 'like',mPoints ));
    normalVector = coder.nullcopy(zeros(3, 1, 'like',mPoints));

    % Allocate mNormals
    mNormals = coder.nullcopy(zeros(size(mPoints), 'like',mPoints));

    for i = 0 : M - 1
        if mValid(i + 1) < numNeighbors

            % Not enough neighbor points to apply PCA
            for d = 0 : 2
                mNormals(d * M + i + 1) = nan;
                continue;
            end
        end
        for k = 0 : numNeighbors - 1
            % index is one-based
            index = mIndices(i*numNeighbors + k + 1 ) - 1;
            for d = 0 : 2
                neighbors(k + 1 + d * numNeighbors) = mPoints(index + 1 + d * M);
            end
        end

        [~, normalVector] = vision.internal.codegen.pc.estimateNormalVectorWithPCA(neighbors, numNeighbors);

        for d = 0 : 2
            mNormals(d * M + i + 1) = normalVector(d + 1);
        end
    end
end
