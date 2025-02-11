%

% Copyright 2020-2022 The MathWorks, Inc.
classdef ParallelSSDSummary < vision.internal.cnn.SSDSummary
    
    methods
        function this = ParallelSSDSummary(varargin)
            this = this@vision.internal.cnn.SSDSummary(varargin{:});
        end
        
        function update( this, predictions, response, epoch, iteration, elapsedTime, miniBatchLoss, learnRate, lossFunctionType,currentIterationIsDone )
            % update  Overload to merge output between workers.
            update@vision.internal.cnn.SSDSummary(this, predictions, response, epoch, iteration, elapsedTime, miniBatchLoss, learnRate, currentIterationIsDone);
            merge(this, lossFunctionType);
        end
    end
    
    methods( Access = private )
        function merge( this, lossFunctionType )
            % merge   Only call within SPMD block, to merge current Loss
            % and Accuracy between workers. Note that predictions and
            % response are NOT merged because this isn't needed.
            subBatchSizes = this.NumObservationsSSD;
            mergedLossAndAccuracy = spmdReduce(@iBlendLossAndAccuracy, ...
                { this.Loss, this.Accuracy, this.RMSE, subBatchSizes, lossFunctionType } );
            this.Loss = mergedLossAndAccuracy{1};
            this.Accuracy = mergedLossAndAccuracy{2};
            this.RMSE = mergedLossAndAccuracy{3};
        end
    end
end

function [val, n] = iWeightedBlend(val1, n1, val2, n2)
% Blend two values with appropriate weights.

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
[loss1, accuracy1, rmse1, batchSizes1, outputType1] = deal( args1{:} );
[loss2, accuracy2, rmse2, batchSizes2, ~] = deal( args2{:} );

n = zeros(1,2);

% Blend accuracy
n1 = batchSizes1(1);
n2 = batchSizes2(1);
[accuracy, n(2)] = iWeightedBlend( accuracy1, n1, accuracy2, n2 );

% Blend rmse
n1 = batchSizes1(2);
n2 = batchSizes2(2);
[rmse, n(3)] = iWeightedBlend( rmse1, n1, rmse2, n2 );

% Blend loss based on loss function type
switch ( outputType1 )
    otherwise
        % Default which works for most loss types
        [loss, ~] = iWeightedBlend( loss1, n1, loss2, n2 );
end

% Combine into a single output.
blended = {loss, accuracy, rmse, n, outputType1};
end
