function [targetMean, targetStd] = meanStdForTargets(targetsInCell)
%MEANSTDFORTARGETS Calculate mean and standard deviation for targets.

% Copyright 2019 The MathWorks, Inc.
    allTargets     = horzcat(targetsInCell{:});

    if isempty(allTargets)
        % Maybe increasing positive overlap range would help.
        error(message('vision:rcnn:noTrainingSamples'));
    end

    meanAllTargets = mean(allTargets,2);
    stdAllTargets  = std(allTargets,0,2);
    stdAllTargets  = stdAllTargets + eps(class(stdAllTargets));
    targetMean     = meanAllTargets';
    targetStd      = stdAllTargets';
end
