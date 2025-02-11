classdef ReidentificationLoss < images.dltrain.internal.Loss
% This class defines the loss for reidentificationNetwork object training.

% Copyright 2023 The MathWorks, Inc.

    properties
        Categories
        LossFunction
        Margin
        NumClasses
        Scale
        MetricNames = "Loss";
    end

    methods
        function obj = ReidentificationLoss(numClasses, labelCategories,...
                lossFunction, margin, scale)
        % Construct an object for the ReidentificationLoss class.
            obj.NumClasses = numClasses;
            obj.Categories = labelCategories;
            obj.LossFunction = lossFunction;
            obj.Margin = margin;
            obj.Scale = scale;
        end

        function [loss, lossData] = lossFcn(obj, preds, gtLabels)
            % Extract categorical elements from ground truth cell.
            if isa(gtLabels,"cell")
                labels = categorical([],1:obj.NumClasses,obj.Categories);
                for i = 1:length(gtLabels)
                    labels(end+1) = gtLabels{i}; %#ok<AGROW>
                end
            else
                labels = gtLabels;
            end

            % Generate target matrix for batch.
            targets = onehotencode(labels,1);

            % Add back in the additive margin for non-target predictions.
            if strcmp(obj.LossFunction, "additive-margin-softmax")
                % Additive margin softmax's margin and scale parameters
                % help control class clustering and separation. When m is
                % large, the intra-class clustering is decreased. When s
                % is large, the inter-class separation is increased. When
                % fewer classes are being used, however, these parameters
                % can result in overlapping classes. To combat this, an
                % approximated class value of 30 has been chosen as the
                % threshold for these parameters. When the number of classes
                % is 30 or fewer, the parameters will be adjusted to better
                % fit the most situations. Otherwise, the optimized values
                % presented by the additive margin softmax paper will be
                % used. This margin has already been subtracted from all
                % logits in the reidentificationFCSoftmaxLayer. Here, the
                % margin is added back in for all non-target predictions.
                % s is a scaling factor to be applied to scale the
                % hypersphere in which the features reside.
                m = obj.Margin;
                s = obj.Scale;
                targetIdx = targets == 1;
                preds(targetIdx) = preds(targetIdx) - m;
                preds = exp(s*preds) ./ sum(exp(s*preds),1);
            end

            % The classification loss.
            loss = crossentropy(preds,targets);
            lossData.Loss = loss;
        end

    end
end