classdef ValidationPredictDatastoreStrategy
% ValidationPredictDatastoreStrategy   Predict strategy for validation reporter
% for CNNs that use generic datastores for validation data. A generic datastore has unknown
% length (number of observations) and cannot preallocate output buffers to hold predictions and
% responses. The dispatcher data is for MIMO networks.

%   Copyright 2019-2021 The MathWorks, Inc.

    properties(Access = private)
        % Precision (nnet.internal.cnn.util.Precision)   A precision object
        Precision

        % ShuffleOption   Controls when to shuffle the data. It can be:
        % 'once', 'every-epoch', or 'never'
        ShuffleOption

        % ExecutionStrategy Strategy for host or GPU.
        ExecutionStrategy
    end

    methods
        function this = ValidationPredictDatastoreStrategy( precision, environment, shuffleOption )
            this.Precision = precision;
            this.ShuffleOption = shuffleOption;
            if environment == "gpu"
                this.ExecutionStrategy = nnet.internal.cnn.TrainerGPUStrategy;
            else
                this.ExecutionStrategy = nnet.internal.cnn.TrainerHostStrategy;
            end
        end

        function [predictions, response, loss] = predictAndComputeLoss( this, net, data )
            % predictAndComputeLoss   Use a network and data dispatcher to
            % calculate a set of predictions and calculate the loss
            [predictions, response] = this.predict( net, data );
            loss = [];
            for ii = 1:numel(predictions)
                loss = [loss net.loss( predictions{ii}, response{ii} )];%#ok<AGROW>
            end
            loss = mean(loss);
        end
    end

    methods(Access = protected)
        function [predictions, responses] = predict( this, net, data )

            predictions = cell(0,1);
            responses = cell(0,1);

            % When 'shuffle' is set to 'every-epoch', validation data is
            % shuffled everytime we compute validation metrics. This is
            % because one epoch corresponds to an entire pass over the
            % dataset, so each time we compute a validation iteration using
            % all the data we should shuffle
            if isequal(this.ShuffleOption, 'every-epoch')
                data.shuffle();
            end
            data.start();
            while ~data.IsDone
                [X, Y] = data.next();
                X = this.ExecutionStrategy.environment(X);
                Y = this.ExecutionStrategy.environment(Y);
                currentBatchPredictions = net.predict( X );
                % Predictions from MIMO Networks are in cell arrays
                predictions{end+1} = currentBatchPredictions; %#ok<AGROW>
                responses{end+1} = Y; %#ok<AGROW>
            end
        end
    end
end
