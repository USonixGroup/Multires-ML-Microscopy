function s = defaultInternalRPNOptions()
%

%   Copyright 2017-2020 The MathWorks, Inc.

s.RPNMiniBatchParams = struct(...
    'BoxRegressionMeanStd',[], ...    
    'MiniBatchPadWithNegatives', true, ... % honor fraction and truncate minibatch if needed
    'SmoothL1Normalization', 'batch', ..., % 'valid'= just positives', 'batch' = batch size (num pos + num neg
    'RPNForegroundFraction', 0.5);

s.ProposalParams = struct(...   
    'NumStrongestRegions', 2000,...                  % top N after NMS
    'NumStrongestRegionsBeforeProposalNMS', Inf, ... % top N before NMS
    'MinScore',0, ...                    % keep proposals with score >= min score
    'ProposalsOutsideImage', 'discard', ...  % discard or clip proposal boxes outside image borders
    'OverlapThreshold', 0.7 ...         % overlap threshold for NMS
    );
