classdef ExperimentMonitorReporter < nnet.internal.cnn.util.Reporter
    % ExperimentMonitorReporter Reporter to record training info to the Experiment Manager app
    
    %   Copyright 2021 The MathWorks, Inc.

    properties (Access = private)
        % experiments.Monitor object
        Monitor
        % Monitor Metrics and Info field information
        MonitorFieldInformation
    end

    methods
        function this = ExperimentMonitorReporter(monitor, monitorFieldInformation)
            this.Monitor = monitor;
            this.MonitorFieldInformation = monitorFieldInformation;
            setupMonitor(this);
        end

        function setup( ~, ~ )
        end

        function start( this )
            % Add state to info then call callbacks
            this.Monitor.Status = "Start";
        end

        function reportIteration( this, summary, ~ )
            % Edit the Info struct with fields from summary input, and call
            % callbacks
            this.Monitor.Status = "Iteration";
            updateMetrics( this, summary);
            notifyInterruptOnStop(this);
        end

        function reportEpoch(~, ~, ~, ~)
            % End of epoch does not trigger anything on the monitor 
        end

        function finish( ~, ~, ~ )
            % finish does not trigger anything on the monitor 
        end

        function computeFinalIteration(~, ~, ~)
            % computing final iteration does not trigger anything on the monitor 
        end

        function reportFinalIteration(this,~)
            % reporting final validation
            this.Monitor.Status = "Done";
            notifyInterruptOnStop(this);
        end
    end

    methods (Access = private)

        function updateMetrics(this,summary)

            numMetrics = numel(this.Monitor.Metrics);
            metricsStruct = [];
            for ii = 1:numMetrics
                field = this.Monitor.Metrics(ii);
                val = iGatherAndConvert(summary.(field));
                if ~isempty(val)
                    metricsStruct.(field) = val;
                end
            end
            if ~isempty(metricsStruct)
                this.Monitor.recordMetrics(summary.Iteration,metricsStruct);
            end
            maxEpochs = this.MonitorFieldInformation.trainingOptions.MaxEpochs;
            currentEpoch = summary.Epoch;
            this.Monitor.Progress = (currentEpoch/maxEpochs)*100;
        end

        
        function setupMonitor(this)

            this.Monitor.XLabel = "Iteration";
            validationSpecified = this.MonitorFieldInformation.ValidationSpecified;

            numGroups = numel(this.MonitorFieldInformation.Groups);

            groupNames = cell(numGroups,1);
            groupMetrics = cell(numGroups,1);

            % Collect all the metrics.
            for ii = 1:numGroups
                group = this.MonitorFieldInformation.Groups(ii);

                metrics = group.TrainingMetrics;
                if validationSpecified
                    metrics = [metrics, group.ValidationMetrics];
                end
                groupNames{ii} = group.Name;
                groupMetrics{ii} = metrics(:);
            end
            this.Monitor.Metrics = vertcat(groupMetrics{:});
            % Setup subplots based on groups.
            for ii = 1:numGroups
                groupSubPlot(this.Monitor,groupNames{ii},groupMetrics{ii});
            end

        end

        function notifyInterruptOnStop(this)
            if this.Monitor.Stop
                event.AffectedObject = this.Monitor;
                localCallbackInterrupt(this,[], event);
            end
        end

        function localCallbackInterrupt(this, ~, event)
            if event.AffectedObject.Stop
                stopReason = nnet.internal.cnn.util.TrainingStopReason.StopButton;
                evtData = nnet.internal.cnn.util.TrainingInterruptEventData( stopReason );

                notify( this, 'TrainingInterruptEvent', evtData);
            end
        end
    end
end


function val = iGatherAndConvert(val)
% Gather if gpuArray and convert to double if numeric
if isnumeric(val)
    val = double( gather( val ) );
end
end
