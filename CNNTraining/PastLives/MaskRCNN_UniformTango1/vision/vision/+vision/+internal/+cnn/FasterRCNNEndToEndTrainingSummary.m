classdef FasterRCNNEndToEndTrainingSummary <  nnet.internal.cnn.util.Summary & vision.internal.cnn.FasterRCNNEndToEndLossUtilities
    % Summary for Faster R-CNN end-to-end training.
    
    %   Copyright 2018-2022 The MathWorks, Inc.
    
    properties
        % Epoch (int)   Number of current epoch
        Epoch
        
        % Iteration (int)   Number of current iteration
        Iteration
        
        % Time (double)   Time spent since training started
        Time
        
        % Loss (double)   Current loss
        Loss
        
        % ValidationLoss (double)   Current validation loss
        ValidationLoss
        
        % FinalValidationLoss (double)   Final validation loss
        FinalValidationLoss
        
        % LearnRate (double)   Current learning rate
        LearnRate
        
        % Predictions   4-D array of network predictions
        Predictions
        
        % Response   4-D array of responses
        Response
        
        % ValidationPredictions   4-D array of validation predictions
        ValidationPredictions
        
        % ValidationResponse   4-D array of validation responses
        ValidationResponse
        
        % FinalValidationPredictions   4-D array of final validation 
        % predictions
        FinalValidationPredictions
        
        % FinalValidationResponse   4-D array of final validation responses
        FinalValidationResponse
        
        % OutputNetworkIteration (double) Iteration corresponding to the
        % output network
        OutputNetworkIteration
        
    end
    
    properties (SetAccess = protected)  
        % Accuracy (double)   Current accuracy for a classification problem
        Accuracy
        
        % RMSE (double)   Current RMSE for a regression problem
        RMSE 
        
        % Accuracy (double)   Current accuracy for a classification problem
        RPNAccuracy
        
        % RMSE (double)   Current RMSE for a regression problem
        RPNRMSE 
        
        % ValidationAccuracy (double)   Current validation accuracy for a
        % classification problem
        ValidationAccuracy
        
        % ValidationRMSE (double)   Current validation RMSE for a
        % regression problem
        ValidationRMSE 
        
        % FinalValidationAccuracy (double)   Final validation accuracy for a
        % classification problem
        FinalValidationAccuracy
        
        % FinalValidationRMSE (double)   Final validation RMSE for a
        % regression problem
        FinalValidationRMSE
        
        % NumObservationsFast Two element vector. First element is number
        % of observations for classification, which includes positive and
        % negatives. Second is number of observations for RMSE, which
        % includes just positives.
        NumObservationsFast
        
        % NumObservationsRPN
        NumObservationsRPN
    end
    
    properties (Access = private)
        % Order The order of the classification and regression predictions
        % output from the network. Specifies which entry in the prediction
        % cell array corresponds to the classification prediction?
        Order

        % Maximum number of epochs.
        MaxEpochs
        % Logical to indicate if current epoch is done.
        CurrentEpochIsDone
    end
    methods
        
        function this = FasterRCNNEndToEndTrainingSummary(order,classNames,boxMatcher,proposalParameters,~,maxEpochs)
            % Set the order of the classification and regression
            % predictions output from the network.
            arguments
                order
                classNames
                boxMatcher
                proposalParameters
                ~
                maxEpochs = []
            end
            
            this.Order = order;
            this.ClassNames = classNames;
            
            this.PositiveOverlapRange = boxMatcher.PositiveOverlapRange;            
            this.NegativeOverlapRange = boxMatcher.NegativeOverlapRange;
            this.ProposalParameters   = proposalParameters;
            this.IsGeneralDatastore   = ~isempty(this.ProposalParameters);

            if this.IsGeneralDatastore
                % Use the removed softmax layer to compute forward prediciton.
                this.RPNSoftmaxLayer      = nnet.internal.cnn.layer.RPNSoftmaxLayer('rpnSoftmax');
                this.ClassCategoricals    = categorical(classNames, classNames);
            end

            this.MaxEpochs = maxEpochs;        
        end
      
        function update( this, predictions, response, epoch, iteration, elapsedTime, miniBatchLoss, learnRate, currentEpochIsDone )
            % update   Use this function to update all the
            % properties of the class without having to individually fill
            % in each property.
            
            this.Predictions = predictions;
            this.Response = response;
            this.Epoch = epoch;
            this.Iteration = iteration;
            this.Time = elapsedTime;
            this.Loss = gather(miniBatchLoss);
            this.LearnRate = learnRate;
            this.CurrentEpochIsDone = currentEpochIsDone;
                 
             if isempty(this.Predictions) || isempty(this.Response)
                this.Accuracy           = [];
                this.RMSE               = [];
                this.RPNAccuracy        = [];
                this.RPNRMSE            = [];
                this.NumObservationsRPN = 0;
                this.NumObservationsFast = [0 0];
            
                return   
             end
            
            % RPN

            if ~this.IsGeneralDatastore
                [this.RPNAccuracy, this.NumObservationsRPN] = iClassificationAccuracyMetric(...
                    this.Predictions{this.Order(1)},...
                    this.Response{this.Order(1)});

                this.RPNRMSE =  iWeightedRMSEMetric(...
                    this.Predictions{this.Order(2)},...
                    this.Response{this.Order(2)});
            else
                % Modified RPN for datastore inputs.
                Y = this.Predictions{this.Order(1)};
                T = this.Response{this.Order(1)};

                [Ycls, Tcls, Yreg, Treg, Wreg, hasResponsesInBatch] = ...
                    rpnClassificationRegressionPredictionResponses(this, Y, T);

                if ~hasResponsesInBatch
                    this.RPNAccuracy        = [];
                    this.RPNRMSE            = [];
                    this.NumObservationsRPN = 0;
                else
                    % For the training summary, just reshaping the classification
                    % prediction does not match the softmax accuracy.
                    % Call softmax forward to get softmax prediction.
                    Zcls = this.RPNSoftmaxLayer.forward(Ycls);

                    [this.RPNAccuracy, this.NumObservationsRPN] = iClassificationAccuracyMetric(Zcls,Tcls);
                    this.RPNRMSE =  iWeightedRMSEMetric(Yreg, {Treg, Wreg});
                end
            end

            % Fast
            [this.Accuracy, this.RMSE, this.NumObservationsFast] = fastMetricsFromPredictionResponse(this,...
                    {this.Predictions},{this.Response});
        end

        function gather( this )
            this.Epoch = gather(this.Epoch);
            this.CurrentEpochIsDone = gather(this.CurrentEpochIsDone);
            this.Iteration = gather(this.Iteration);
            this.Loss = gather(this.Loss);
            this.LearnRate = gather(this.LearnRate);
            this.Accuracy = gather(this.Accuracy);
            this.RMSE = gather(this.RMSE);
            this.RPNAccuracy = gather(this.RPNAccuracy);
            this.RPNRMSE = gather(this.RPNRMSE);
            this.NumObservationsFast = gather(this.NumObservationsFast);
            this.NumObservationsRPN = gather(this.NumObservationsRPN);
            this.ValidationLoss = gather(this.ValidationLoss);
            this.ValidationAccuracy = gather(this.ValidationAccuracy); 
            this.ValidationRMSE = gather(this.ValidationRMSE);
            this.FinalValidationLoss = gather(this.FinalValidationLoss);
            this.FinalValidationAccuracy = gather(this.FinalValidationAccuracy);
            this.FinalValidationRMSE = gather(this.FinalValidationRMSE);
        end
        % gather  Ensure all properties are stored on the host

        function set.ValidationLoss( this, validationLoss )
            this.ValidationLoss = validationLoss;

            % The only place we can update the metrics, until we have a method like
            % updateValidationSummary.
            setValidationMetrics(this);
        end

        function tf = isLastIteration(this)
            tf = ~isempty(this.MaxEpochs) && this.Epoch == this.MaxEpochs && this.CurrentEpochIsDone;
        end


        function set.FinalValidationLoss( this, validationLoss )
            this.FinalValidationLoss = validationLoss;

            % The only place we can update the metrics, until we have a method like
            % updateValidationSummary.
            if ~isempty(validationLoss) && ...
                    ~isempty(this.ValidationPredictions) && ...
                    ~isempty(this.ValidationResponse) && ...
                    this.IsGeneralDatastore

                [~, this.FinalValidationRMSE] = fastMetricsFromPredictionResponse(this,...
                    this.ValidationPredictions,...
                    this.ValidationResponse);
            else
                this.FinalValidationRMSE = [];
            end
        end
    end
       
    methods(Access = protected)
        
        function setValidationMetrics(this)
            if ~isempty(this.ValidationLoss) && ...
                    ~isempty(this.ValidationPredictions) && ...
                    ~isempty(this.ValidationResponse) && ...
                    this.IsGeneralDatastore

                [this.ValidationAccuracy, this.ValidationRMSE] = fastMetricsFromPredictionResponse(this,...
                    this.ValidationPredictions,this.ValidationResponse);
            else
                this.ValidationAccuracy = [];
                this.ValidationRMSE     = [];
            end 
        end

        function [accuracy, rmse, numObservationsFast] = fastMetricsFromPredictionResponse(this,predictionSet,responseSet)
            % Calcuate accuracy and RMSE from the given predictions and
            % responses. predictionSet and responseSet must be a cell array, i.e.,
            % each cell array element contains a batch's worth of predictions and responses.
            numPredictions = numel(predictionSet);

            Ycls = cell(numPredictions,1);
            Tcls = cell(numPredictions,1);
            Yreg = cell(numPredictions,1);
            Treg = cell(numPredictions,1);
            Wreg = cell(numPredictions,1);

            hasResponses = false(numPredictions,1);

            for ii = 1:numPredictions
                predictions = predictionSet{ii};
                response    = responseSet{ii};

                if ~this.IsGeneralDatastore
                    Y = predictions{this.Order(3)};
                    T = response{this.Order(3)};
                else
                    Y = predictions{this.Order(2)};
                    T = response{this.Order(2)};
                end

                [Ycls{ii}, Tcls{ii}, Yreg{ii}, Treg{ii}, Wreg{ii}, hasResponses(ii)] = ...
                    fastRCNNClassificationRegressionPredictionResponses(this, Y, T);
            end
            if any(hasResponses)
                Ycls = cat(4,Ycls{:});
                Tcls = cat(4,Tcls{:});
                Yreg = cat(4,Yreg{:});
                Treg = cat(4,Treg{:});
                Wreg = cat(4,Wreg{:});
                [accuracy, numObservationsFast(1)] = iClassificationAccuracyMetric(Ycls,Tcls);
                [rmse, numObservationsFast(2)] = iWeightedRMSEMetric(Yreg,{Treg,Wreg});
            else
                accuracy = [];
                rmse     = [];
                numObservationsFast = [0 0];
            end
        end
    end
    
    methods(Static)
        %--------------------------------------------------------------------------
        function summaryFcn = makeSummary(internalNetwork, executionSettings, classNames, boxMatcher, proposalParameters)                       
            % Create a summary function for fast and faster r-cnn training.

            if nargin == 4
                proposalParameters = [];
            end
                       
            order = iMetricOrder(internalNetwork, proposalParameters);
            
            if executionSettings.useParallel
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.ParallelFasterRCNNEndToEndTrainingSummary(order,classNames,boxMatcher,proposalParameters,outputfmt,maxEpochs);
            else
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.FasterRCNNEndToEndTrainingSummary(order,classNames,boxMatcher,proposalParameters,outputfmt,maxEpochs);
            end
        end
    end
end

%--------------------------------------------------------------------------
function [rmse, numObservations] = iWeightedRMSEMetric(Y,response)
% response input holds two arrays: {T , W}.
% W holds instance weights. From RMSE, use W as pure indicator matrix.
T = response{1};
W = response{2};
mask = W > 0;
squares = mask.*((Y-T).^2);
numObservations = nnz(mask);
if numObservations == 0
    rmse = [];
else
    rmse = sqrt( sum( squares(:) ) / max(numObservations,1) );
    rmse = gather(rmse);
end
end

%--------------------------------------------------------------------------
function [accuracy, numObservations] = iClassificationAccuracyMetric(Y,T)
[~, yIntegers] = max(Y, [], 3);
[t, tIntegers] = max(T, [], 3);
x = (t.* (yIntegers == tIntegers));

numObservations = nnz(T); 
if numObservations == 0
    % None of the proposals where marked as positive or negative sample
    % based on the positive/negative overlap ratios. In this case, the Fast
    % R-CNN sub-network does nothing during training and we should report
    % [] accuracy.
    accuracy = [];
else
    accuracy = gather( 100 * (sum(x(:)) / numObservations) );
end
end

%--------------------------------------------------------------------------
function order = iMetricOrder(internalNetwork, proposalParameters)
% figure out the order of the prediction/response output required for the
% printing the verbose summary.

outputLayers = cellfun(@(x)class(x),internalNetwork.OutputLayers,'UniformOutput',false);
if isempty(proposalParameters)
    % Define corresponding layer types for when training input is a table.
    expectedLayers = {'vision.internal.cnn.layer.RPNCrossEntropy',...
        'vision.internal.cnn.layer.SmoothL1Loss',...
        'vision.internal.cnn.layer.FastRCNNOutputInternalLayer'};
else
    % Define corresponding layer types for when training input is a datastore.
    expectedLayers = {'vision.internal.cnn.layer.RPNOutputInternalLayer',...
        'vision.internal.cnn.layer.FastRCNNOutputInternalLayer'};
end

[~,order] = ismember(expectedLayers,outputLayers);
end

%--------------------------------------------------------------------------
