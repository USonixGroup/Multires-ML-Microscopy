classdef SoloV2Loss < images.dltrain.internal.Loss
% This class defines the loss for solov2 object training

% Copyright 2023 The MathWorks, Inc.
    
    properties
        GridSizes
        ObjSizeRanges
        NumClasses
        MetricNames = "Loss";
    end

    methods
        function obj = SoloV2Loss(gridSizes, objSizerange, numClasses)
        % Construct an object for the SoloV2Loss class
        % gridSizes    - The grid size for each FPN level.
        % objSizerange - The range of object sizes, each FPN level is
        %                trained on.
        % numClasses   - Number of classes the solov2 network is to be
        %                trained on.
            obj.GridSizes = gridSizes;
            obj.ObjSizeRanges = objSizerange;
            obj.NumClasses = numClasses;
        end

        function [loss, lossData] = lossFcn (obj, mlvlKernelPreds, mlvlClsPreds,maskFeats,...
                                                  gtBoxes, gtLabels, gtMasks)
        
            
            
            % Generate targets and predictions for the network outputs
            [mlvlMaskPredsBatch, mlvlPosMaskTargetsBatch, mlvlLabelsTargetsBatch, mlvlPosMasksBatch] =...
                    vision.internal.solov2.generateSoloV2Targets(mlvlKernelPreds, mlvlClsPreds,...
                                          maskFeats, gtLabels, gtMasks, gtBoxes,...
                                          obj.ObjSizeRanges, obj.GridSizes, obj.NumClasses);
        
           
            % Check for no positive samples
            if(isempty(mlvlMaskPredsBatch))
                lossMask = dlarray(0);
                numPos=0;
            else
                % Calculate Mask loss
                mlvlMaskPredsBatch = sigmoid(mlvlMaskPredsBatch);
    
                % Calculate number of positive samples
                numPos = sum(mlvlPosMasksBatch, "all");
                
                % Reshape the mask format from HxWxnumPreds to HxWx1xnumPreds
                mlvlMaskPredsBatch = reshape(mlvlMaskPredsBatch, size(mlvlMaskPredsBatch,1), size(mlvlMaskPredsBatch,2), 1,[]);
                mlvlPosMaskTargetsBatch = reshape(mlvlPosMaskTargetsBatch, size(mlvlPosMaskTargetsBatch,1), size(mlvlPosMaskTargetsBatch,2), 1,[]);
                
                % Mask loss is dice loss between the preds and target
                lossMask = 1 - mean(generalizedDice(dlarray(mlvlMaskPredsBatch,'SSCB'),...
                                               dlarray(single(mlvlPosMaskTargetsBatch),'SSCB')), "all");
            end     
            

            % ClassificationLoss - accumulate over each FPN level
            lossCls = 0;
            for idx = 1:length(mlvlLabelsTargetsBatch)
                lvlLabelTarget= onehotencode(categorical(mlvlLabelsTargetsBatch{idx}, 1:obj.NumClasses+1), 3);
                % drop background class
                lvlLabelTarget(:,:,end,:) = [];
                lossCls = lossCls + sum(focalCrossEntropy(mlvlClsPreds{idx}, dlarray(lvlLabelTarget, 'SSCB'),'Reduction', 'none', 'TargetCategories', 'independent'), 'all');
            end
            
            % Normalize over positive samples
            lossCls = lossCls/(numPos+1);
            
            % The classification loss & Mask loss is weighted 1x & 3x.
            loss = 1.0*lossCls + 3.0*lossMask;
            lossData.Loss = loss;
        end
    end
end