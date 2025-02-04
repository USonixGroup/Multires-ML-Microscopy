classdef DistanceMetricHamming < vision.internal.kmeans.DistanceMetricStrategy
% DistanceMetricHamming Hamming distance metric.

% Copyright 2020-2023 The MathWorks, Inc.
%#codegen

    properties (Constant, Access = private)
        % LUT Look-up table that stores mapping from uint8 value to number
        %     of true bits per value.
        LUT = iInitializeLookUpTable();
    end

    methods
        %------------------------------------------------------------------
        %dist Hamming distance between binary feature vectors.
        % d = dist(A,B)
        function d = dist(this,A,B)
        % Determine bit differences between A and B using a look-up
        % table.
            assert(isa(A,'uint8') && isa(B,'uint8'));
            bitvec = single(bitxor(A,B));

            % Return distances in M-by-1 vector.
            coder.varsize('d');
            if this.FeatureDimension == 1
                d = zeros(size(bitvec, 2), 1, class(this.LUT));
                d(:,1) = sum(this.LUT(bitvec+1), 1);
            else
                if isempty(coder.target)
                    d = sum(this.LUT(bitvec+1), 2);
                else
                    v = coder.nullcopy(zeros(size(bitvec), class(this.LUT)));
                    lut = this.LUT;
                    bitvec1 = bitvec+1;
                    for i=1:size(bitvec, 1)
                        for j=1:size(bitvec, 2)
                            v(i, j) = lut(bitvec1(i, j));
                        end
                    end
                    d = sum(v, 2);
                end
            end
        end

        %------------------------------------------------------------------
        %pdist Pairwise Hamming distance between binary feature vectors.
        % D = pdist(A,B) returns the pairwise Hamming distance between
        % binary feature vectors in A and B. The output D is an M-by-N
        % matrix where M is the number of features in A and N is the number
        % of features in B.
        function D = pdist(this,A,B)
            if this.FeatureDimension == 2
                % Transpose A and B prior to calling built-in.
                A = A';
                B = B';
            end
            D = images.internal.builtins.hammingMetric(A,B);
        end
    end
end

%--------------------------------------------------------------------------
% Create look-up table for counting bits in a uint8 byte.
function lut = iInitializeLookUpTable()
    lut = zeros(256, 1, 'single');
    for i = 0:255
        lut(i+1) = sum(dec2bin(i)-'0');
    end
end
