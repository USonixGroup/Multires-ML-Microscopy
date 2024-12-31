classdef ParallelFasterRCNNEndToEndTrainingSummary < vision.internal.cnn.FasterRCNNEndToEndTrainingSummary
%

%   Copyright 2018-2022 The MathWorks, Inc.
    
    methods
        function this = ParallelFasterRCNNEndToEndTrainingSummary(varargin)
            this = this@vision.internal.cnn.FasterRCNNEndToEndTrainingSummary(varargin{:});
        end
        
        function update( this, predictions, response, epoch, iteration, elapsedTime, miniBatchLoss, learnRate, lossFunctionType,currentIterationIsDone )
            % update  Overload to merge output between workers
            update@vision.internal.cnn.FasterRCNNEndToEndTrainingSummary(this, predictions, response, epoch, iteration, elapsedTime, miniBatchLoss, learnRate, currentIterationIsDone);
            merge(this, lossFunctionType);
        end
    end
    
    methods( Access = private )
        function merge( this, lossFunctionType )
            % merge   Only call within SPMD block, to merge current Loss
            % and Accuracy between workers. Note that predictions and
            % response are NOT merged because this isn't needed.
            subBatchSizes = [this.NumObservationsRPN this.NumObservationsFast];
            mergedLossAndAccuracy = spmdReduce(@iBlendLossAndAccuracy, ...
                { this.Loss, this.Accuracy, this.RMSE, this.RPNAccuracy, this.RPNRMSE, subBatchSizes, lossFunctionType } );
           
            this.Loss        = mergedLossAndAccuracy{1};
            this.Accuracy    = mergedLossAndAccuracy{2};
            this.RMSE        = mergedLossAndAccuracy{3};
            this.RPNAccuracy = mergedLossAndAccuracy{4};
            this.RPNRMSE     = mergedLossAndAccuracy{5};
        end
    end
    
end

function [val, n] = iWeightedBlend(val1, n1, val2, n2)
% Blend two values with appropriate weights

% Values may be empty in parallel cases and when a metric cannot be
% computed.
if isempty(val1)
    val1 = 0;
    n1 = 0;
end

if isempty(val2)
    val2 = 0;
    n2 = 0;
end

n = n1 + n2;
if n == 0
    val = 0;
else
    val = (n1*val1/n) + (n2*val2/n);
end
end


function blended = iBlendLossAndAccuracy(args1, args2)
% Blend the loss and accuracy. Function in binary form for use by GOP

% Retrieve individual arguments
[loss1, accuracy1, rmse1, rpnAcc1, rpnRMSE1, batchSizes1, outputType1] = deal( args1{:} );
[loss2, accuracy2, rmse2, rpnAcc2, rpnRMSE2, batchSizes2, ~] = deal( args2{:} );

n = zeros(1,3);

% Blend accuracy
n1 = batchSizes1(2);
n2 = batchSizes2(2);
[accuracy, n(2)] = iWeightedBlend( accuracy1, n1, accuracy2, n2 );

% Blend rmse
n1 = batchSizes1(3);
n2 = batchSizes2(3);
[rmse, n(3)] = iWeightedBlend( rmse1, n1, rmse2, n2 );

% Blend accuracy
n1 = batchSizes1(1);
n2 = batchSizes2(1);
[rpnAccuracy, n(1)] = iWeightedBlend( rpnAcc1, n1, rpnAcc2, n2 );

% Blend rmse
[rpnRMSE, ~] = iWeightedBlend( rpnRMSE1, n1, rpnRMSE2, n2 );

% Blend loss based on loss function type
switch ( outputType1 )
    otherwise
        % Default which works for most loss types
        [loss, ~] = iWeightedBlend( loss1, n1, loss2, n2 );
end

% Combine into a single output
blended = {loss, accuracy, rmse, rpnAccuracy, rpnRMSE, n, outputType1};
end
