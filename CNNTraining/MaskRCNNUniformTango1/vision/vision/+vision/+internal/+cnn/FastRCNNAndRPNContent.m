classdef (Sealed) FastRCNNAndRPNContent < nnet.internal.cnn.util.traininginfo.ContentStrategy
     % FastRCNNAndRPNContent content strategy. Used to populate training
     % info output.
    
    %   Copyright 2016-2021 The MathWorks, Inc.
    
    properties
        FieldNames = {'TrainingLoss', 'TrainingAccuracy', 'TrainingRMSE', 'BaseLearnRate'};
        SummaryNames = {'Loss', 'Accuracy', 'RMSE', 'LearnRate'};
        
        FinalValidationFieldNames = {};
        FinalValidationSummaryNames = {};
        
        OutputNetworkIterFieldName = {'OutputNetworkIteration'};
        OutputNetworkIterSummaryName = {'OutputNetworkIteration'};
    end
end