function monitor = checkExperimentMonitor(parser, userInput, options, name)
%

%   Copyright 2021 The MathWorks, Inc.

    if iWasUserSpecified(parser,'ExperimentMonitor')
        classes = {'experiments.Monitor'};
        attrs = {'scalar', 'nonempty'};
        validateattributes(userInput.ExperimentMonitor,...
            classes,attrs,name,'ExperimentMonitor');
        monitor = userInput.ExperimentMonitor;
        if ~isempty(monitor.Metrics)
            error(message('vision:ObjectDetector:nonEmptyMetricsInfoInExperimentMonitor'));
        end       
        if isequal(options.Plots,'training-progress')
            error(message('vision:ObjectDetector:trainingPlotWithExperimentMonitor'));
        end
    else
        monitor = [];
    end
end

%--------------------------------------------------------------------------
function tf = iWasUserSpecified(parser,param)
tf = ~ismember(param,parser.UsingDefaults);
end
