classdef DistanceMetricStrategy
% DistanceMetricStrategy An abstract interface for computing distance
% metrics between feature vectors.

% Copyright 2020-2023 The MathWorks, Inc.
%#codegen

    properties
        % FeatureDimension The dimension that represents the length of a
        %                  feature vector when processing a matrix of
        %                  feature vectors.
        %
        %                  Default: 2 (features are row vectors)
        FeatureDimension (1,1) {iMustBeOneOrTwo(FeatureDimension)} = 2
    end

    methods (Abstract)
        %------------------------------------------------------------------
        %pdist Pairwise distance between feature vectors.
        % D = pdist(A, B) returns pairwise distances between vectors in
        % A and B. The output D is an M-by-N matrix where M is the number
        % of features in A and N is the number of features in B.
        pdist(A,B)

        %------------------------------------------------------------------
        %dist Distance between features vectors.
        % d = dist(A,B) returns the distance between feature vectors in A
        % and B. The output d is an M element vector where M is the number
        % of features in A and B.
        dist(A,B)
    end

    methods
        %------------------------------------------------------------------
        %sqdist Squared distance metric between features.
        % d = sqdist(A, B) return the squared distance metric between
        % features in A and B. The output is an M-element column vector
        % where M is the number of features in A and B. Used for K-Means++
        % initialization.
        function d = sqdist(this,A,B)
            d = this.dist(A,B).^2;
        end
    end

end

%--------------------------------------------------------------------------
function iMustBeOneOrTwo(x)
    assert(x == 1 || x == 2)
end
