classdef FastRCNNSummary <  nnet.internal.cnn.util.Summary & vision.internal.cnn.FasterRCNNEndToEndLossUtilities
%

%   Copyright 2017-2022 The MathWorks, Inc.
    
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
        
        % FinalValidationResponse   4-D array of final validation 
        % responses
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
        
        % ValidationAccuracy (double)   Current validation accuracy for a
        % classification problem
        ValidationAccuracy
        
        % ValidationRMSE (double)   Current validation RMSE for a
        % regression problem
        ValidationRMSE
        
        % FinalValidationAccuracy (double)   Final validation accuracy for 
        % a classification problem
        FinalValidationAccuracy
        
        % FinalValidationRMSE (double)   Final validation RMSE for a
        % regression problem
        FinalValidationRMSE
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
        
        function this = FastRCNNSummary(order,rpnFasterRCNN,proposalParameters,~,maxEpochs)
            % Set the order of the classification and regression
            % predictions output from the network.
            arguments
                order
                rpnFasterRCNN
                proposalParameters
                ~
                maxEpochs = []
            end
            
            this.Order = order;
            if rpnFasterRCNN
                % Use the removed softmax layer to compute forward prediciton.
                this.RPNSoftmaxLayer    = nnet.internal.cnn.layer.RPNSoftmaxLayer('rpnSoftmax');
                this.ProposalParameters = proposalParameters;
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

            [this.Accuracy, this.RMSE] = metricsFromPredictionResponse(this,...
                {this.Predictions},...
                {this.Response});
        end
        
        % gather  Ensure all properties are stored on the host
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

        function set.FinalValidationLoss( this, validationLoss )
            this.FinalValidationLoss = validationLoss;

            % The only place we can update the metrics, until we have a method like
            % updateValidationSummary.
            if ~isempty(validationLoss) && ...
                    ~isempty(this.ValidationPredictions) && ...
                    ~isempty(this.ValidationResponse)

                [~, this.FinalValidationRMSE] = metricsFromPredictionResponse(this,...
                    this.ValidationPredictions,...
                    this.ValidationResponse);
            else
                this.FinalValidationRMSE = [];
            end
        end

        function set.ValidationLoss( this, validationLoss )
            this.ValidationLoss = validationLoss;

            % The only place we can update the metrics, until we have a method like
            % updateValidationSummary.
            setValidationMetrics(this);
        end

        function tf = isLastIteration(this)
            tf = ~isempty(this.MaxEpochs) && this.Epoch == this.MaxEpochs && this.CurrentEpochIsDone;
        end

    end
    
    methods(Access = protected)

        function setValidationMetrics(this)
            if ~isempty(this.ValidationLoss) && ...
                    ~isempty(this.ValidationPredictions) && ...
                    ~isempty(this.ValidationResponse)

                [this.ValidationAccuracy, this.ValidationRMSE] = metricsFromPredictionResponse(this,...
                    this.ValidationPredictions,...
                    this.ValidationResponse);
            else
                this.ValidationAccuracy = [];
                this.ValidationRMSE     = [];
            end
        end

        function [accuracy, rmse] = metricsFromPredictionResponse(this,predictionSet,responseSet)
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
                if isempty(predictions) || isempty(response)
                    continue;
                end

                if ~isempty(this.RPNSoftmaxLayer)
                    % For Faster RCNN's RPN training, we set the removed softmax layer for metric calculations.
                    Y = predictions;
                    T = response{this.Order(1)};
                    [Ycls{ii}, Tcls{ii}, Yreg{ii}, Treg{ii}, Wreg{ii}, hasResponses(ii)] = ...
                        rpnClassificationRegressionPredictionResponses(this, Y, T);
                    if hasResponses(ii)
                        % For the training summary, just reshaping the classification
                        % prediction does not match the softmax accuracy.
                        % Call softmax forward to get softmax prediction.
                        Ycls{ii} = this.RPNSoftmaxLayer.forward(Ycls{ii});
                    else
                        [Ycls{ii}, Tcls{ii}, Yreg{ii}, Treg{ii}, Wreg{ii}] = deal([]);
                    end
                else
                    % For Fast RCNN we can just use the prediction and response
                    % values from the network.
                    hasResponses(ii) = true;
                    Ycls{ii} = predictions{this.Order(1)};
                    Tcls{ii} = response{this.Order(1)};

                    Yreg{ii} = predictions{this.Order(2)};
                    Treg{ii} = response{this.Order(2)}{1};
                    Wreg{ii} = response{this.Order(2)}{2};
                end
            end
            if any(hasResponses)
                Ycls = cat(4,Ycls{:});
                Tcls = cat(4,Tcls{:});
                Yreg = cat(4,Yreg{:});
                Treg = cat(4,Treg{:});
                Wreg = cat(4,Wreg{:});
                accuracy = iClassificationAccuracyMetric(Ycls,Tcls);
                rmse = iWeightedRMSEMetric(Yreg,{Treg,Wreg});
            else
                accuracy = [];
                rmse     = [];
            end
        end
    end

    methods(Static)
        %--------------------------------------------------------------------------
        function summaryFcn = makeSummary(internalNetwork, executionSettings, proposalParameters)
            % Create a summary function for fast and faster r-cnn training.
            if nargin == 2
                proposalParameters = [];
            end
            [order, rpnFasterRCNN] = iMetricOrder(internalNetwork);
            
            if executionSettings.useParallel
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.ParallelFastRCNNSummary(order,rpnFasterRCNN,proposalParameters,outputfmt,maxEpochs);
            else
                summaryFcn = @(outputfmt,maxEpochs)...
                    vision.internal.cnn.FastRCNNSummary(order,rpnFasterRCNN,proposalParameters,outputfmt,maxEpochs);
            end
        end
    end
end

%--------------------------------------------------------------------------
function rmse = iWeightedRMSEMetric(Y,response)
% response input holds two arrays: {T , W}.
% W holds instance weights. From RMSE, use W as pure indicator matrix.
T = response{1};
W = response{2};
squares = (W > 0).*((Y-T).^2);
rmse = sqrt( sum( squares(:) ) / nnz(T) );
rmse = gather(rmse);
end

%--------------------------------------------------------------------------
function accuracy = iClassificationAccuracyMetric(Y,T)
[~, yIntegers] = max(Y, [], 3);
[t, tIntegers] = max(T, [], 3);
x = (t.* (yIntegers == tIntegers));
numObservations = nnz(t);
accuracy = gather( 100 * (sum(x(:)) / numObservations) );
accuracy = gather(accuracy);
end

%--------------------------------------------------------------------------
function [order, rpnFasterRCNN] = iMetricOrder(internalNetwork)
% figure out the order of the prediction/response output required for the
% printing the verbose summary.

rpnFasterRCNN = false;
outputLayers = cellfun(@(x)class(x),internalNetwork.OutputLayers,'UniformOutput',false);
if ismember('vision.internal.cnn.layer.RPNOutputInternalLayer',outputLayers)
    % RPN from Faster RCNN
    % Define corresponding layer type.
    expectedLayers = {'vision.internal.cnn.layer.RPNOutputInternalLayer'};
    rpnFasterRCNN = true;
elseif ismember('vision.internal.cnn.layer.RPNCrossEntropy',outputLayers)
    % RPN
    % Define corresponding layer type.
    expectedLayers = {'vision.internal.cnn.layer.RPNCrossEntropy',...
        'vision.internal.cnn.layer.SmoothL1Loss'};
elseif ismember('nnet.internal.cnn.layer.SoftmaxCrossentropy',outputLayers)
    % Fast with softmax+crossentropy layer fusion
    % Define corresponding layer type.
    expectedLayers = {'nnet.internal.cnn.layer.SoftmaxCrossentropy',...
        'vision.internal.cnn.layer.SmoothL1Loss'};
else
    % Fast
    % Define corresponding layer type.
    expectedLayers = {'nnet.internal.cnn.layer.CrossEntropy',...
        'vision.internal.cnn.layer.SmoothL1Loss'};
end

[~,order] = ismember(expectedLayers,outputLayers);

end
