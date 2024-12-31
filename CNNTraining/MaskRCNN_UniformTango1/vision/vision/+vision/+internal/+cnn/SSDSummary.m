classdef SSDSummary <  nnet.internal.cnn.util.Summary
% SSDSummary Internal class to create a summary function for SSD training.

% Copyright 2019-2022 The MathWorks, Inc.
    %----------------------------------------------------------------------
    properties
        % Epoch (int) Number of current epoch
        Epoch

        % Iteration (int) Number of current iteration
        Iteration

        % Time (double) Time spent since training started
        Time

        % Loss (double) Current loss
        Loss

        % ValidationLoss (double) Current validation loss
        ValidationLoss

        % FinalValidationLoss (double)   Final validation loss
        FinalValidationLoss
        
        % LearnRate (double) Current learning rate
        LearnRate

        % Predictions 4-D array of network predictions
        Predictions

        % Response 4-D array of responses
        Response

        % ValidationPredictions 4-D array of validation predictions
        ValidationPredictions

        % ValidationResponse 4-D array of validation responses
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
    %----------------------------------------------------------------------
    properties (SetAccess = protected)
        % Accuracy (double) Current accuracy for a classification problem
        Accuracy

        % RMSE (double) Current RMSE for a regression problem
        RMSE

        % ValidationAccuracy (double) Current validation accuracy for a
        % classification problem
        ValidationAccuracy

        % ValidationRMSE (double) Current validation RMSE for a
        % regression problem
        ValidationRMSE
        
        % FinalValidationAccuracy (double)   Final validation accuracy for a
        % classification problem
        FinalValidationAccuracy
        
        % FinalValidationRMSE (double)   Final validation RMSE for a
        % regression problem
        FinalValidationRMSE

        % NumObservationsSSD
        % Two element vector. First element is number of observations for
        % classification, which includes positive and negatives.
        % Second is number of observations for RMSE, which includes just positives.
        NumObservationsSSD
    end
    %----------------------------------------------------------------------
    properties (Access = private)
        ClassIdx
        RegressIdx
        % Maximum number of epochs.
        MaxEpochs
        % Logical to indicate if current epoch is done.
        CurrentEpochIsDone
    end
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = SSDSummary(classIdx, regressIdx, ~, maxEpochs)
            % Set the order of the classification and regression
            % predictions output from the network.
            arguments
                classIdx
                regressIdx
                ~
                maxEpochs = []
            end

            this.ClassIdx = classIdx;
            this.RegressIdx = regressIdx;
            this.MaxEpochs = maxEpochs;
        end
        %------------------------------------------------------------------
        function update(this, predictions, response, epoch, iteration, ...
                        elapsedTime, miniBatchLoss, learnRate, currentEpochIsDone)
            % update   Use this function to update all the
            % properties of the class without having to individually fill
            % in each property.

            this.Predictions = predictions;
            this.Response    = response;
            this.Epoch       = epoch;
            this.Iteration   = iteration;
            this.Time        = elapsedTime;
            this.Loss        = gather(miniBatchLoss);
            this.LearnRate   = learnRate;

            this.CurrentEpochIsDone = currentEpochIsDone;
            if isempty(this.Predictions) || isempty(this.Response)
                this.Accuracy           = [];
                this.RMSE               = [];

                return
            end

            [accuracy,rmse] = getAccuracyRMSE(this,this.Predictions,this.Response);
            this.Accuracy = mean(accuracy);
            this.RMSE = mean(rmse);
        end
        %------------------------------------------------------------------
        function gather(this)
            % gather  Ensure all properties are stored on the host
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
        %------------------------------------------------------------------
        function set.ValidationLoss( this, validationLoss )
            this.ValidationLoss = validationLoss;

            % The only place we can update the metrics, until we have a method like
            % updateValidationSummary.
            setValidationMetrics(this);
        end
        %------------------------------------------------------------------
        function set.FinalValidationLoss( this, validationLoss )
            this.FinalValidationLoss = validationLoss;
            if ~isempty(validationLoss) && ...
                    ~isempty(this.ValidationPredictions) && ...
                    ~isempty(this.ValidationResponse)

                [~, this.FinalValidationRMSE] = metricsFromPredictionResponse(this,...
                    this.ValidationPredictions,this.ValidationResponse);
            else
                this.FinalValidationRMSE = [];
            end
        end
        %------------------------------------------------------------------
        function tf = isLastIteration(this)
            tf = ~isempty(this.MaxEpochs) && this.Epoch == this.MaxEpochs && this.CurrentEpochIsDone;
        end
        %------------------------------------------------------------------
        function rmse = get.FinalValidationRMSE( this )
            % get.ValidationRMSE   Get the current validation RMSE.
            rmse = sqrt(this.FinalValidationLoss);
        end
    end
    %----------------------------------------------------------------------
    methods(Access = private)
        %------------------------------------------------------------------
        function setValidationMetrics(this)
            if ~isempty(this.ValidationLoss) && ...
                    ~isempty(this.ValidationPredictions) && ...
                    ~isempty(this.ValidationResponse)

                [this.ValidationAccuracy, this.ValidationRMSE] = metricsFromPredictionResponse(this,...
                    this.ValidationPredictions,this.ValidationResponse);
            else
                this.ValidationAccuracy = [];
                this.ValidationRMSE     = [];
            end
        end
        %------------------------------------------------------------------
        function [accuracy, rmse] = metricsFromPredictionResponse(this,predictionSet,responseSet)
            % Calcuate accuracy and RMSE from the given predictions and
            % responses. predictionSet and responseSet must be a cell array, i.e.,
            % each cell array element contains a batch's worth of predictions and responses.
            numPredictions = numel(predictionSet);

            accuracy = cell(numPredictions,1);
            rmse     = cell(numPredictions,1);

            for ii = 1:numPredictions
                predictions = predictionSet{ii};
                response    = responseSet{ii};
                [accuracy{ii}, rmse{ii}] =  getAccuracyRMSE(this,predictions,response);
            end
            accuracy = vertcat(accuracy{:});
            rmse     = vertcat(rmse{:});
            accuracy = mean(accuracy);
            rmse     = mean(rmse);
        end

        function [accuracy, rmse] = getAccuracyRMSE(this,predictions,response)
            numClassIdx = numel(this.ClassIdx);
            accuracy = cell(numClassIdx, 1);
            numObservations = zeros(numClassIdx,1);

            for idx = 1:numClassIdx
                Y = predictions{this.ClassIdx(idx)};
                T = response{this.ClassIdx(idx)};
                if isempty(Y) || isempty(T)
                    continue;
                end
                [accuracy{idx},numObservations(idx)] = iClassificationAccuracyMetric(Y,T);
            end
            accuracy = vertcat(accuracy{:});
            this.NumObservationsSSD(1) = sum(numObservations);

            numRegressIdx = numel(this.RegressIdx);
            rmse = cell(numRegressIdx, 1);
            numObservations = zeros(numRegressIdx,1);
            for idx = 1:numRegressIdx
                Y = predictions{this.RegressIdx(idx)};
                resp = response{this.RegressIdx(idx)};
                if isempty(Y) || isempty(resp)
                    continue;
                end
                [rmse{idx},numObservations(idx,2)] = iWeightedRMSEMetric(Y,resp);
            end
            rmse = vertcat(rmse{:});
            this.NumObservationsSSD(2) = sum(numObservations);
        end
    end
    %----------------------------------------------------------------------
    methods(Static)
        %------------------------------------------------------------------
        function summaryFcn = makeSummary(internalNetwork, executionSettings)
            % Create a summary function for SSD training.
            classIdx = iGetClassificationLayerIndices(internalNetwork);
            regressIdx = iGetRegressionLayerIndices(internalNetwork);

            if executionSettings.useParallel
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.ParallelSSDSummary(classIdx, regressIdx, outputfmt, maxEpochs);
            else
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.SSDSummary(classIdx, regressIdx, outputfmt, maxEpochs);
            end
        end
    end
end

%--------------------------------------------------------------------------
function [rmse,numObservations] = iWeightedRMSEMetric(Y, response)

    % response input holds two arrays: {T , W}.
    % W holds instance weights. From RMSE, use W as pure indicator matrix.
    T = response{1};
    W = response{2};
    mask = W > 0;
    numObservations = nnz(mask);
    if numObservations == 0
        rmse = [];
    else
        squares = mask.*((Y-T).^2);
        rmse = sqrt(sum(squares(:)) / nnz(T));
        rmse = gather(rmse);
    end
end

%--------------------------------------------------------------------------
function [accuracy,numObservations] = iClassificationAccuracyMetric(Y, T)

    [~,~,nclasses,~] = size(Y);
    positiveFlag = logical(T(:,:,nclasses+1,:));
    T = T(:,:,1:nclasses,:);
    % Observations are encoded in T as non-zero values.
    positiveObservations = sum(sum(squeeze(positiveFlag)));
    [~, yIntegers] = max(Y, [], 3);
    [t, tIntegers] = max(T, [], 3);
    x = (t.* (yIntegers == tIntegers));
    numObservations = nnz(t);
    if positiveObservations == 0
        % None of the proposals where marked as positive or negative sample
        % based on the positive/negative overlap ratios. In this case, the Fast
        % SSD sub-network does nothing during training and we should report
        % [] accuracy.
        accuracy = [];
    else
        accuracy = gather( 100 * (sum(x(:)) / numObservations) );
    end
end

%--------------------------------------------------------------------------
function indices = iGetClassificationLayerIndices(internalNetwork)

    outputLayers = cellfun(@(x)class(x), internalNetwork.OutputLayers, 'UniformOutput', false);
    expectedLayers = 'nnet.internal.cnn.layer.CustomClassificationLayer';

    indices = find(strcmp(outputLayers, expectedLayers));
end

%--------------------------------------------------------------------------
function indices = iGetRegressionLayerIndices(internalNetwork)

    outputLayers = cellfun(@(x)class(x), internalNetwork.OutputLayers, 'UniformOutput', false);
    expectedLayers = 'vision.internal.cnn.layer.SmoothL1Loss';

    indices = find(strcmp(outputLayers, expectedLayers));
end
