function warnIfLossHasNaNs(info,msg)
% Warn using WarnginLogger if last 5 iterations have NaNs. This is a heuristic. 

%   Copyright 2018-2020 The MathWorks, Inc.

loss = iTail(info.TrainingLoss,5);
rmse = iTail(info.TrainingRMSE,5);
if ~iAllElementsAreFinite(loss) || ~iAllElementsAreFinite(rmse)
    % network is not trained properly. Defer warning so that it gets
    % displayed one at the end of training.
    vision.internal.cnn.WarningLogger.deferredWarning(msg); 
end

%--------------------------------------------------------------------------
function x = iTail(x,n)
% return the last n elements in x.
n = min(n,numel(x));
x = x(end-n+1:end);

%--------------------------------------------------------------------------
function tf = iAllElementsAreFinite(x)
tf = all(isfinite(x(:)));
