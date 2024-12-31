classdef DistanceMetricSSD < vision.internal.kmeans.DistanceMetricStrategy
% DistanceMetricSSD Sum-of-squared distance metric.

% Copyright 2020-2023 The MathWorks, Inc.
%#codegen
    methods

        %------------------------------------------------------------------
        %dist Sum of squared distances between features.
        % d = dist(A,B) returns the distance between each row vector in A
        % and B. The output d is an M-by-1 vector where M is size(A,1). A
        % and B must have the same size.
        function d = dist(this,A,B)
            if coder.target('MATLAB')
                d(:,1) = sum((A-B).^2, this.FeatureDimension);
            else
                d = zeros(size(A, 1), 1, class(A));
                if this.FeatureDimension == 1
                    d(:,1) = sum((A-B).^2, 1);
                else
                    d(:,1) = sum((A-B).^2, 2);
                end
            end
        end

        %------------------------------------------------------------------
        %pdist Pairwise sum-of-square distances between feature vectors.
        % d = pdist(A,B) returns the pairwise distances between row vectors
        % in A and B. The output d is an M-by-N matrix, where M is
        % size(A,1) and N is size(B,1).
        function D = pdist(this,A,B)
            if coder.target('MATLAB')
                if this.FeatureDimension == 2
                    % Transpose feature vectors prior to calling built-in.
                    A = A';
                    B = B';
                end
                D = visionSSDMetric(A,B);
            elseif coder.internal.isTargetMATLABHost()
                if this.FeatureDimension == 2
                    D = vision.internal.buildable.ComputeMetricBuildable.ComputeMetric_core(A, B, 'ssd', size(A, 1), ...
                                                                                            size(B, 1), class(A));
                else
                    A = A';
                    C = B';
                    D = vision.internal.buildable.ComputeMetricBuildable.ComputeMetric_core(A, C, 'ssd', size(A, 1), ...
                                                                                            size(C, 1), class(A));
                end
            else
                if this.FeatureDimension == 2
                    N1 = size(A, 1);
                    N2 = size(B, 1);
                    aDims = size(A, 2);
                    D = zeros(N1, N2, class(A));
                    for c = 1:N2
                        for r = 1:N1
                            D(r, c) = sum((A(r, 1:aDims) - B(c, 1:aDims)).^2);
                        end
                    end
                else
                    A = A';
                    C = B';
                    N1 = size(A, 1);
                    N2 = size(C, 1);
                    aDims = size(A, 2);
                    D = zeros(N1,N2, class(A));
                    for c = 1:N2
                        for r = 1:N1
                            D(r, c) = sum((A(r, 1:aDims) - C(c, 1:aDims)).^2);
                        end
                    end
                end
            end
        end

        %------------------------------------------------------------------
        %sqdist Squared distance.
        % d = sqdist(A,B) return the squared distance. This is the same as
        % the dist method for the SSD metric.
        function d = sqdist(this,A,B)
        % SSD is already a squared distance. Override default
        % implementation and call into pdist to use the built-in which
        % is faster. Currently, sqdist is only used for k-means++ init
        % where B contains a one feature vector, so calling into pdist
        % is OK.
            d = this.pdist(A,B);
        end
    end
end
