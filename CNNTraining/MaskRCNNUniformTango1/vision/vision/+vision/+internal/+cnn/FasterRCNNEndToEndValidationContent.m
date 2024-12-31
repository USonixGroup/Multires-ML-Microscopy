classdef (Sealed) FasterRCNNEndToEndValidationContent < nnet.internal.cnn.util.traininginfo.ContentStrategy
     % FasterRCNNEndToEndValidationContent content strategy. Used to populate
     % training and validation info output.

    %   Copyright 2019-2021 The MathWorks, Inc.

    properties
        FieldNames = {'TrainingLoss', 'RPNTrainingAccuracy', 'RPNTrainingRMSE', 'TrainingAccuracy', 'TrainingRMSE', 'BaseLearnRate', 'ValidationLoss', 'ValidationAccuracy', 'ValidationRMSE'};
        SummaryNames = {'Loss', 'RPNAccuracy', 'RPNRMSE', 'Accuracy', 'RMSE', 'LearnRate', 'ValidationLoss','ValidationAccuracy', 'ValidationRMSE'};

        FinalValidationFieldNames = {'FinalValidationLoss', 'FinalValidationRMSE'};
        FinalValidationSummaryNames = {'FinalValidationLoss', 'FinalValidationRMSE'};
        
        OutputNetworkIterFieldName = {'OutputNetworkIteration'};
        OutputNetworkIterSummaryName = {'OutputNetworkIteration'};
    end

    methods (Static)
        function fields = monitorSummaryFields(validationSpecified)

            group1.Name              = "RPN_Accuracy";
            group1.TrainingMetrics   = "RPNAccuracy";
            group1.ValidationMetrics = [];

            group2.Name              = "RPN_RMSE";
            group2.TrainingMetrics   = "RPNRMSE";
            group2.ValidationMetrics = [];

            group3.Name              = "Accuracy";
            group3.TrainingMetrics   = "Accuracy";
            group3.ValidationMetrics = "ValidationAccuracy";

            group4.Name              = "RMSE";
            group4.TrainingMetrics   = "RMSE";
            group4.ValidationMetrics = "ValidationRMSE";

            group5.Name              = "Loss";
            group5.TrainingMetrics   = "Loss";
            group5.ValidationMetrics = "ValidationLoss";

            group6.Name = "Info";
            group6.TrainingMetrics = "LearnRate";
            group6.ValidationMetrics = [];

            fields.ValidationSpecified = validationSpecified;
            fields.Groups              = [group1,group2,group3,group4,...
                                    group5, group6];
        end
    end
end
