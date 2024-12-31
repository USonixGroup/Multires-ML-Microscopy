classdef FastRCNNAxesConfig < vision.internal.cnn.AxesConfig
% Plot config for Fast R-CNN.

%   Copyright 2020-2023 The MathWorks, Inc.

    properties
        % AxesConfiguration   Holds a cell array of AxesProperties for each
        % axis and a MetricRowDataFactory. Also calculates number of Axes.        
        AxesConfiguration
        
    end
    
    methods
        function this = FastRCNNAxesConfig()
            
            cellArrayofAxesProperties = {iAccuracyProperties(), iLossProperties()};
            
            metricRowDataFactory = nnet.internal.cnn.ui.info.ClassificationMetricRowDataFactory();
            
            this.AxesConfiguration = nnet.internal.cnn.ui.AxesConfiguration(cellArrayofAxesProperties,...
                metricRowDataFactory);            
        end

    end
 
end

function axesProperties = iAccuracyProperties(~)

    axesProperties = struct();

    % Line Colors for training, training(smoothed), validation
    axesProperties.Color = vision.internal.cnn.utils.Colors.ClassificationAccuracyLineColor;
    axesProperties.SmoothedColor = vision.internal.cnn.utils.Colors.ClassificationAccuracySmoothedLineColor;
    axesProperties.ValidationColor = vision.internal.cnn.utils.Colors.ClassificationValidationAccuracyLineColor;

    % Y-axis limits
    % Accuracy ranges from 0 to 100. Setting Y-axis limit to 105 makes
    % sure that the accuracy line does not merge into the top of the plot
    % when it is 100.
    axesProperties.YLims = [0, 105];

    % Variable names for Mini-Batch-Summary
    axesProperties.TrainingMetric = 'Accuracy';
    axesProperties.ValidationMetric = 'ValidationAccuracy';
    axesProperties.FinalValidationMetric = 'FinalValidationAccuracy';

    % Tag Suffix
    axesProperties.ViewTagSuffix = 'CLASSIFICATION_ACCURACY';

    % Size Fraction dictates how much of plotting space is taken up by each
    % plot
    axesProperties.SizeFraction = 0.50;

    % Legend Info
    axesProperties.LineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationTrainingAccuracyLineLegendName');
    axesProperties.LegendSectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendAccuracySectionName');
    axesProperties.TrainingLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendTrainingAccuracySubsectionName');
    axesProperties.ValidationLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendValidationAccuracySubsectionName');
    axesProperties.TrainingTrendLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationTrainingAccuracyTrendLineLegendName');
    axesProperties.TrainingLegendTrendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendTrainingTrendAccuracySubsectionName');
    axesProperties.ValidationLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationValidationAccuracyLineLegendName');

    % X & Y axes labels
    axesProperties.XLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationAccuracyAxesXLabel');
    axesProperties.YLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationAccuracyAxesYLabel');
end

        
function lossAxesProperties = iLossProperties(~)
    lossAxesProperties = struct();

    % Line Colors for training, training(smoothed), and validation
    lossAxesProperties.Color = vision.internal.cnn.utils.Colors.ClassificationLossLineColor;
    lossAxesProperties.SmoothedColor = vision.internal.cnn.utils.Colors.ClassificationLossSmoothedLineColor;
    lossAxesProperties.ValidationColor = vision.internal.cnn.utils.Colors.ClassificationValidationLossLineColor;

    % Y-axis limits
    lossAxesProperties.YLims = [0, 0];

    % Variable names for Mini-Batch Summary
    lossAxesProperties.TrainingMetric = 'Loss';
    lossAxesProperties.ValidationMetric = 'ValidationLoss';
    lossAxesProperties.FinalValidationMetric = 'FinalValidationLoss';

    % Tag Suffix
    lossAxesProperties.ViewTagSuffix = 'CLASSIFICATION_LOSS';

    % Size Fraction dictates how much of plotting space is taken up by each
    % plot
    lossAxesProperties.SizeFraction = 0.50;

    % Legend Info
    lossAxesProperties.LineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationTrainingLossLineLegendName');
    lossAxesProperties.LegendSectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendLossSectionName');
    lossAxesProperties.TrainingLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendTrainingLossSubsectionName');
    lossAxesProperties.ValidationLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendValidationLossSubsectionName');
    lossAxesProperties.TrainingTrendLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationTrainingLossTrendLineLegendName');
    lossAxesProperties.TrainingLegendTrendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLegendTrainingTrendLossSubsectionName');
    lossAxesProperties.ValidationLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationValidationLossLineLegendName');

    % X & Y axes labels
    lossAxesProperties.XLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLossAxesXLabel');
    lossAxesProperties.YLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:ClassificationLossAxesYLabel');

end
        
