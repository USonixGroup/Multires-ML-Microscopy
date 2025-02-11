classdef (Sealed) FastRCNNAndRPNValidationContent < nnet.internal.cnn.util.traininginfo.ContentStrategy
     % FastRCNNAndRPNValidationContent Validation content strategy. Used to populate training
     % info output.

     % Copyright 2019-2021 The MathWorks, Inc.

    properties
        FieldNames = {'TrainingLoss', 'TrainingAccuracy', 'TrainingRMSE', 'BaseLearnRate', 'ValidationLoss', 'ValidationAccuracy', 'ValidationRMSE'};
        SummaryNames = {'Loss', 'Accuracy', 'RMSE', 'LearnRate', 'ValidationLoss','ValidationAccuracy', 'ValidationRMSE'};

        FinalValidationFieldNames = {'FinalValidationLoss', 'FinalValidationRMSE'};
        FinalValidationSummaryNames = {'FinalValidationLoss', 'FinalValidationRMSE'};
        
        OutputNetworkIterFieldName = {'OutputNetworkIteration'};
        OutputNetworkIterSummaryName = {'OutputNetworkIteration'};
    end

    methods (Static)
        function fields = monitorSummaryFields(validationSpecified)

            group1.Name              = "Accuracy";
            group1.TrainingMetrics   = "Accuracy";
            group1.ValidationMetrics = "ValidationAccuracy";

            group2.Name              = "RMSE";
            group2.TrainingMetrics   = "RMSE";
            group2.ValidationMetrics = "ValidationRMSE";

            group3.Name              = "Loss";
            group3.TrainingMetrics   = "Loss";
            group3.ValidationMetrics = "ValidationLoss";

            group4.Name = "Info";
            group4.TrainingMetrics = "LearnRate";
            group4.ValidationMetrics = [];

            fields.ValidationSpecified = validationSpecified;
            fields.Groups              = [group1,group2,group3,group4];
        end
    end
end
