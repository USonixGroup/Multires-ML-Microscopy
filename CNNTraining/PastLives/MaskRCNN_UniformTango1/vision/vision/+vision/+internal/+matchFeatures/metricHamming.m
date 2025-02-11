function scores = metricHamming(features1, features2, N1, N2, outputClass)
%metricHamming Calculating the hamming distance between two groups of features

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

persistent lookupTable; % lookup table for counting bits

% Need to obtain feature vector length to perform explicit row indexing
% needed for code generation of variable sized inputs
if isempty(coder.target)
    % call optimized builtin function
    scores = images.internal.builtins.hammingMetric(features1,features2);
else
    if coder.internal.isTargetMATLABHost
        features1 = features1';
        features2 = features2';
        scores = vision.internal.buildable.ComputeMetricBuildable.ComputeMetric_core(features1, features2, 'hamming', N1, N2, outputClass);
    else
        % for portable C code generation
        vector_length = size(features1, 1);
        scores = zeros(N1, N2, outputClass);

        if isempty(lookupTable)
            lookupTable = zeros(256, 1, outputClass);
            for i = 0:255
                lookupTable(i+1) = sum(dec2bin(i)-'0');
            end
        end
        col = coder.internal.indexInt(N2);
        row = coder.internal.indexInt(N1);
        if ~coder.internal.isInParallelRegion
            parfor c = 1 : col
                for r = 1 : row
                    temp = bitxor(features1(1:vector_length, r),...
                        features2(1:vector_length, c));
                    idx = double(temp) + 1; % cast needed to avoid integer math
                    scores(r,c) = sum(lookupTable(idx));
                end
            end
        else
            for c = 1:col
                for r = 1:row
                    temp = bitxor(features1(1:vector_length, r),...
                        features2(1:vector_length, c));
                    idx = double(temp) + 1; % cast needed to avoid integer math
                    scores(r,c) = sum(lookupTable(idx));
                end
            end
        end
    end
end
