function scores = metricSAD(features1, features2, N1, N2, outputClass)
%metricSAD Calculating the Sum of Absolute Differences between two groups of features

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

% Need to obtain feature vector length to perform explicit row indexing
% needed for code generation of variable sized inputs

if isempty(coder.target)
    % call optimized builtin function
    scores = images.internal.builtins.SADMetric(features1,features2);
else
    if coder.internal.isTargetMATLABHost
        features1 = features1';
        features2 = features2';
        scores = vision.internal.buildable.ComputeMetricBuildable.ComputeMetric_core(features1,features2, 'sad', N1, N2, outputClass);
    else
        % for portable C code generation
        vector_length = size(features1, 1);
        scores = zeros(N1, N2, outputClass);

        for c = 1:N2
            for r = 1:N1
                scores(r, c) = sum(abs(features1(1:vector_length, r) - ...
                    features2(1:vector_length, c)));
            end
        end
    end
end