classdef (Sealed) FasterRCNNEndToEndTrainingContent < nnet.internal.cnn.util.traininginfo.ContentStrategy
     % FasterRCNNEndToEndTrainingContent content strategy. Used to populate
     % training info output.
    
    %   Copyright 2016-2021 The MathWorks, Inc.
    
    properties
        FieldNames = {'TrainingLoss', 'RPNTrainingAccuracy', 'RPNTrainingRMSE', 'TrainingAccuracy', 'TrainingRMSE', 'BaseLearnRate'};
        SummaryNames = {'Loss', 'RPNAccuracy', 'RPNRMSE', 'Accuracy', 'RMSE', 'LearnRate'};
        
        FinalValidationFieldNames = {};
        FinalValidationSummaryNames = {};
        
        OutputNetworkIterFieldName = {'OutputNetworkIteration'};
        OutputNetworkIterSummaryName = {'OutputNetworkIteration'};
    end
end