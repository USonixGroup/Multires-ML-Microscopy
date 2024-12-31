function s = defaultInternalEndToEndOptions()
% Default training options. 

%   Copyright 2018-2020 The MathWorks, Inc.

s = struct(...
    'BoxRegressionMeanStd',[], ...    
    'MiniBatchPadWithNegatives', true, ... % honor fraction and truncate minibatch if needed
    'SmoothL1Normalization', 'valid', ..., % 'valid'= just positives', 'batch' = batch size (num pos + num neg    
    'NumStrongestRegionsBeforeProposalNMS', 12000, ... % top N before NMS
    'MinScore',0, ...                    % keep proposals with score >= min score
    'ProposalsOutsideImage', 'clip', ...  % discard or clip proposal boxes outside image borders
    'OverlapThreshold', 0.7, ...         % overlap threshold for NMS
    'FastForegroundFraction', 0.5, ...
    'RPNForegroundFraction', 0.5 ...
    );
