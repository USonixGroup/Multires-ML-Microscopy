% YOLOv2Summary Internal class to create a summary function for YOLOv2 training.

% Copyright 2018-2022 The MathWorks, Inc.
classdef YOLOv2Summary < nnet.internal.cnn.util.Summary

    properties
        % Epoch (int)   Number of current epoch.
        Epoch
        
        % Iteration (int)   Number of current iteration.
        Iteration
        
        % Time (double)   Time spent since training started.
        Time
        
        % Loss (double)   Current loss.
        Loss
        
        % ValidationLoss (double)   Current validation loss.
        ValidationLoss
        
        % FinalValidationLoss (double)   Final validation loss.
        FinalValidationLoss
        
        % LearnRate (double)   Current learning rate.
        LearnRate
        
        % Predictions   4-D array of network predictions.
        Predictions
        
        % Response   4-D array of responses.
        Response
        
        % ValidationPredictions   4-D array of validation predictions.
        ValidationPredictions
        
        % ValidationResponse   4-D array of validation responses.
        ValidationResponse
        
        % FinalValidationPredictions   4-D array of final validation 
        % predictions.
        FinalValidationPredictions
        
        % FinalValidationResponse   4-D array of final validation responses.
        FinalValidationResponse
        
        % OutputNetworkIteration (double) Iteration corresponding to the
        % output network
        OutputNetworkIteration
    end
    
    properties (SetAccess = protected)
        % Accuracy (double)   Current accuracy for the model.
        Accuracy
        
        % RMSE (double)   Current RMSE for a regression problem.
        RMSE
        
        % ValidationAccuracy (double)   Current validation accuracy for the
        % model.
        ValidationAccuracy
        
        % ValidationRMSE (double)   Current validation RMSE for the model.
        ValidationRMSE
        
        % FinalValidationAccuracy (double)   Final validation accuracy for 
        % the model.
        FinalValidationAccuracy
        
        % FinalValidationRMSE (double)   Final validation RMSE for the model.
        FinalValidationRMSE
    end
    
    properties (Access = private)
        % Maximum number of epochs.
        MaxEpochs
        % Logical to indicate if current epoch is done.
        CurrentEpochIsDone
    end

    methods
        
        function this = YOLOv2Summary(~,maxEpochs)
            arguments
                ~
                maxEpochs = []
            end
            this.MaxEpochs = maxEpochs;
        end
        
        function update( this, predictions, response, epoch, iteration, elapsedTime, miniBatchLoss, learnRate, currentEpochIsDone)
            % update   Use this function to update all the
            % properties of the class without having to individually fill
            % in each property.
            this.Predictions = predictions;
            this.Response = response;
            this.Epoch = epoch;
            this.CurrentEpochIsDone = currentEpochIsDone;
            this.Iteration = iteration;
            this.Time = elapsedTime;
            this.Loss = gather(miniBatchLoss);
            this.LearnRate = learnRate;
            this.Accuracy = NaN;
            this.RMSE = sqrt(this.Loss);
        end
        
        % gather  Ensure all properties are stored on the host.
        function gather( this )
            this.Epoch = gather(this.Epoch);
            this.CurrentEpochIsDone = gather(this.CurrentEpochIsDone);
            this.Iteration = gather(this.Iteration);
            this.Loss = gather(this.Loss);
            this.LearnRate = gather(this.LearnRate);
            this.Accuracy = gather(this.Accuracy);
            this.RMSE = gather(this.RMSE);
            this.ValidationLoss = gather(this.ValidationLoss);
            this.ValidationAccuracy = gather(this.ValidationAccuracy); 
            this.ValidationRMSE = gather(this.ValidationRMSE);
            this.FinalValidationLoss = gather(this.FinalValidationLoss);
            this.FinalValidationAccuracy = gather(this.FinalValidationAccuracy);
            this.FinalValidationRMSE = gather(this.FinalValidationRMSE);
        end

        function tf = isLastIteration(this)
            tf = ~isempty(this.MaxEpochs) && this.Epoch == this.MaxEpochs && this.CurrentEpochIsDone;
        end

        function rmse = get.ValidationRMSE( this )
            % get.ValidationRMSE   Get the current validation RMSE.
            rmse = sqrt(this.ValidationLoss);
        end

        function rmse = get.FinalValidationRMSE( this )
            % get.ValidationRMSE   Get the current validation RMSE.
            rmse = sqrt(this.FinalValidationLoss);
        end
    end
    
    methods(Static)
        %--------------------------------------------------------------------------
        function summaryFcn = makeSummary(~,executionSettings)
            % Create a summary function for yolov2 training.
            if executionSettings.useParallel
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.ParallelYOLOv2Summary(outputfmt,maxEpochs);
            else
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.YOLOv2Summary(outputfmt,maxEpochs);
            end
            
        end

        function fields = monitorSummaryFields(validationSpecified)

            group1.Name              = "RMSE";
            group1.TrainingMetrics   = "RMSE";
            group1.ValidationMetrics = "ValidationRMSE";
            
            group2.Name              = "Loss";
            group2.TrainingMetrics   = "Loss";
            group2.ValidationMetrics = "ValidationLoss";

            group3.Name = "Info";
            group3.TrainingMetrics = "LearnRate";
            group3.ValidationMetrics = [];

            fields.ValidationSpecified = validationSpecified;
            fields.Groups              = [group1,group2,group3];
        end
    end
end
