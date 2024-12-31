classdef mAPObjectDetectionMetric < images.dltrain.internal.Metric

    % Copyright 2023 The MathWorks, Inc.

    properties (Access=private)
        DetectionResults
        DetectionGroundTruth
    end

    properties
        Name
        OverlapThreshold
    end

    methods

        function self = mAPObjectDetectionMetric(NV)

            arguments
                NV.Name {mustBeTextScalar} = "mAP50";
                NV.OverlapThreshold {mustBeNumeric,mustBeInRange(NV.OverlapThreshold,0.0,1.0)} = 0.5;
            end

            self.Name = NV.Name;
            self.OverlapThreshold = NV.OverlapThreshold;

            self.ValidationOnly = true;
            self.InferenceMethodMode = "predictForValidation";
            self.Maximize = true;
        end

    end

    methods (Hidden)

        function this = initialize(this, Y,T,varargin)
            % Do an update/eval/reset to make sure this metric makes sense
            % with the example supplied data.
            this = reset(this);
            this = update(this,Y,T);
            evaluate(this);
            this = reset(this);
        end

        function self = update(self,Y,T)
            self.DetectionResults = vertcat(self.DetectionResults,Y);

            % This expects ground truth is a cell array of the form
            % {boxes,labels}.
            boxes = T(:,1);
            labels = T(:,2);
            gtTable = table(boxes,labels);

            self.DetectionGroundTruth = vertcat(self.DetectionGroundTruth,gtTable);
        end

        function metric = evaluate(self)
            try
                metrics = evaluateObjectDetection(self.DetectionResults,self.DetectionGroundTruth,self.OverlapThreshold,Verbose=false);
                metric = metrics.DatasetMetrics.mAPOverlapAvg;
            catch ME
                if ME.identifier == "MATLAB:categorical:InvalidComparisonTypes"
                    % gecked as G3059021, we don't handle case where
                    % DetectionResults contains no boxes.
                    metric = 0.0;
                else
                    rethrow(ME)
                end
            end
        end

        function self = aggregate(m1,m2) %#ok<STOUT>
            % Stub for now
        end

        function self = reset(self)
            self.DetectionResults = [];
            self.DetectionGroundTruth = [];
        end

    end

end

