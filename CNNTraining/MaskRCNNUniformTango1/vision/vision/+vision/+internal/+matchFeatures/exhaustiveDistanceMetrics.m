function scores = exhaustiveDistanceMetrics(features1, features2, ...
    numFeatures1, numFeatures2, outputClass, metric)
%exhaustiveDistanceMetrics Compute distance metrics

%   Copyright 2020 The MathWorks, Inc.

%#codegen

switch metric
    case 'sad'
        % Generate correspondence matrix using Sum of Absolute Differences
        scores = vision.internal.matchFeatures.metricSAD(features1, features2, numFeatures1, numFeatures2, outputClass);
    case 'normxcorr'
        % Generate correspondence matrix using Normalized Cross-correlation
        scores = features1' * features2;
    case 'ssd'
        % Generate correspondence matrix using Sum of Squared Differences
        scores = vision.internal.matchFeatures.metricSSD(features1, features2, numFeatures1, numFeatures2, outputClass);
    otherwise % 'hamming'
        % Generate correspondence matrix using Hamming distance
        scores = vision.internal.matchFeatures.metricHamming(uint8(features1), uint8(features2), numFeatures1, numFeatures2, outputClass);
end