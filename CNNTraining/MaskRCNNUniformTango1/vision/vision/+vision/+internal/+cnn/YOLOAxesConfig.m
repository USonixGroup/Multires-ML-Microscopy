classdef YOLOAxesConfig < vision.internal.cnn.AxesConfig
% Plot config for YOLO.

%   Copyright 2020-2023 The MathWorks, Inc.

    properties
        % AxesConfiguration   Holds a cell array of AxesProperties for each
        % axis and a MetricRowDataFactory. Also calculates number of Axes.
        AxesConfiguration
    
    end
    
    methods
        function this = YOLOAxesConfig()

            cellArrayofAxesProperties = {iRMSEProperties(), iLossProperties()};
            
            metricRowDataFactory = nnet.internal.cnn.ui.info.RegressionMetricRowDataFactory();
            
            this.AxesConfiguration = nnet.internal.cnn.ui.AxesConfiguration(cellArrayofAxesProperties,...
                metricRowDataFactory);            
        end

    end
 
end


function axesProperties = iRMSEProperties(~)
            
    axesProperties = struct();

    % Line Colors for training, training(smoothed), validation
    axesProperties.Color = vision.internal.cnn.utils.Colors.RegressionRMSELineColor;
    axesProperties.SmoothedColor = vision.internal.cnn.utils.Colors.RegressionRMSESmoothedLineColor;
    axesProperties.ValidationColor = vision.internal.cnn.utils.Colors.RegressionValidationRMSELineColor;

    % Y-axis Limits
    axesProperties.YLims = [0, 0];

    % Variable Names for Mini-Batch Summary
    axesProperties.TrainingMetric = 'RMSE';
    axesProperties.ValidationMetric = 'ValidationRMSE';
    axesProperties.FinalValidationMetric = 'FinalValidationRMSE';

    % Tag Suffix
    axesProperties.ViewTagSuffix = 'REGRESSION_RMSE';

    % Size Fraction dictates how much of plotting space is taken up by each
    % axes
    axesProperties.SizeFraction = 0.50;

    % Legend Info
    axesProperties.LineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionTrainingRMSELineLegendName');
    axesProperties.LegendSectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendRMSESectionName');
    axesProperties.TrainingLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendTrainingRMSESubsectionName');
    axesProperties.ValidationLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendValidationRMSESubsectionName');
    axesProperties.TrainingTrendLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionTrainingRMSETrendLineLegendName');
    axesProperties.TrainingLegendTrendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendTrainingTrendRMSESubsectionName');
    axesProperties.ValidationLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionValidationRMSELineLegendName');

    % X & Y axes labels
    axesProperties.XLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionRMSEAxesXLabel');
    axesProperties.YLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionRMSEAxesYLabel');

end
        
        
function lossAxesProperties = iLossProperties(~)
    
    lossAxesProperties = struct();

    % Line Colors for training, training(smoothed), and validation
    lossAxesProperties.Color = vision.internal.cnn.utils.Colors.RegressionLossLineColor;
    lossAxesProperties.SmoothedColor = vision.internal.cnn.utils.Colors.RegressionLossSmoothedLineColor;
    lossAxesProperties.ValidationColor = vision.internal.cnn.utils.Colors.RegressionValidationLossLineColor;

    % Y-axis limits
    lossAxesProperties.YLims = [0, 0];

    % Variable names for Mini-Batch Summary
    lossAxesProperties.TrainingMetric = 'Loss';
    lossAxesProperties.ValidationMetric = 'ValidationLoss';
    lossAxesProperties.FinalValidationMetric = 'FinalValidationLoss';

    % Tag Suffix
    lossAxesProperties.ViewTagSuffix = 'REGRESSION_LOSS';

    % Size Fraction dictates how much of plotting space is taken up by each
    % axes
    lossAxesProperties.SizeFraction = 0.50;

    % Legend Info
    lossAxesProperties.LineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionTrainingLossLineLegendName');
    lossAxesProperties.LegendSectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendLossSectionName');
    lossAxesProperties.TrainingLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendTrainingLossSubsectionName');
    lossAxesProperties.ValidationLegendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendValidationLossSubsectionName');
    lossAxesProperties.TrainingTrendLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionTrainingLossTrendLineLegendName');
    lossAxesProperties.TrainingLegendTrendSubsectionName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLegendTrainingTrendLossSubsectionName');
    lossAxesProperties.ValidationLineLegendName = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionValidationLossLineLegendName');

    % X & Y axes labels
    lossAxesProperties.XLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLossAxesXLabel');
    lossAxesProperties.YLabel = vision.getMessage('nnet_cnn:internal:cnn:ui:trainingplot:RegressionLossAxesYLabel');

end

   
