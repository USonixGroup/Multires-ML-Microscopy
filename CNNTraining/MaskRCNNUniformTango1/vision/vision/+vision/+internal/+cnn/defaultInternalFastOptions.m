function s = defaultInternalFastOptions()
%

%   Copyright 2017-2020 The MathWorks, Inc.

s = struct(...
    'BoxRegressionMeanStd',[], ...   
    'MiniBatchPadWithNegatives', true, ... % honor fraction and truncate minibatch if needed
    'SmoothL1Normalization', 'batch', ..., % 'valid'= just positives', 'batch' = batch size (num pos + num neg)
    'FastForegroundFraction', 0.50);
