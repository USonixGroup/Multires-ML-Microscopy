classdef PoseMaskRCNNLoss < images.dltrain.internal.Loss
% This class defines the loss for posemaskrcnn training

% Copyright 2023 The MathWorks, Inc.
    properties (Access=private)
        params
    end

    properties
        MetricNames = ["Loss","RPNLoss","RMSE","ClassLoss","MaskLoss","RLoss","TLoss"]
    end

    methods

        function obj = PoseMaskRCNNLoss(params)
            obj.params = params;
        end

        function [loss, lossData] = lossFcn (obj, YRPNClass, YRPNRegDeltas, proposal, YRCNNClass, YRCNNReg, YPoseRot, YPoseTrans, YMask,...
                gTruthBoxes, gTruthLabels, gTruthMasks, gtTruthRotation, gTruthTranslation, gIntrinsics, xyzImg, gPointClouds)
        
        % Proposals are 5 x NumProposals (Due to batch restrictions from custom RPL layer)
        proposals = gather(extractdata(proposal));
        
        % Convert proposals to numProposals x 5 (as expected by the rest of post processing code)
        proposals =proposals';
        
        proposals(:,1:4) = vision.internal.cnn.maskrcnn.boxUtils.x1y1x2y2ToXYWH(proposals(:,1:4));
        
        numImagesInBatch = size(gTruthBoxes,1);
        %Convert numProposalsx5 Proposals to numImagesInBatchx1 (Group by image index)
        proposals = vision.internal.cnn.maskrcnn.groupProposalsByImageIndex(proposals, numImagesInBatch);
        
        
        % Generate RCNN response targets
        %--------------------------------
        
        % Step 1: Match ground truth boxes to proposals
        assignAllGTruthBoxes = false; % default: false
        [assignment, positiveIndex, negativeIndex] = vision.internal.cnn.maskrcnn.bboxMatchAndAssign(...
                                                                proposals, gTruthBoxes,...
                                                                obj.params.PositiveOverlapRange, obj.params.NegativeOverlapRange,...
                                                                assignAllGTruthBoxes);
                      
        % Step 2: Calculate regression targets as (dx, dy, log(dw), log(dh))
        regressionTargets = vision.internal.cnn.maskrcnn.generateRegressionTargets(gTruthBoxes, proposals,...
                                                                assignment, positiveIndex,...
                                                                obj.params.NumClasses);
         
        classNames = categories(gTruthLabels{1});
        
        % Step 3: Assign groundtruth labels to proposals
        classificationTargets = vision.internal.cnn.maskrcnn.generateClassificationTargets (gTruthLabels, assignment,...
                                                     positiveIndex, negativeIndex,...
                                                     classNames, obj.params.BackgroundClass);
                                                 
        % Step 4: Calculate instance weights for bounding box regression
        instanceWeightsReg = vision.internal.cnn.maskrcnn.regressionResponseInstanceWeights (classificationTargets, obj.params.BackgroundClass);
         
        % Step 5: Generate mask targets
        if obj.params.MaskLossWeight > 0
         
            % Crop and resize the instances based on proposal bboxes and network output size
            maskOutputSize = obj.params.MaskOutputSize;
            croppedMasks = vision.internal.cnn.maskrcnn.cropandResizeMasks (gTruthMasks, gTruthBoxes, maskOutputSize);
             
            % Generate mask targets
            maskTargets = vision.internal.cnn.maskrcnn.generateMaskTargets(croppedMasks, assignment, classificationTargets, obj.params);
        else
            maskTargets = 0;
        end


        % Stage 2 (RCNN) Loss
        % --------------------
        
        % *Classification loss*
        classificationTargets = cat(1, classificationTargets{:})';
        % onehotencode labels                       
        classificationTargets = onehotencode(classificationTargets,1);
        classificationTargets(isnan(classificationTargets)) = 0;
        
        classificationTargets = reshape(classificationTargets ,1, 1, size(YRCNNClass,3),[]);
        LossRCNNClass = vision.internal.cnn.maskrcnn.CrossEntropy(YRCNNClass, classificationTargets);
        
         
        % *Weighted regression loss*
        regressionTargets = cat(1,regressionTargets{:})';
        instanceWeightsReg = cat(1, instanceWeightsReg{:})';
        
        regressionTargets = reshape(regressionTargets, 1, 1, size(YRCNNReg,3),[]);
        instanceWeightsReg = reshape(instanceWeightsReg, 1, 1, size(YRCNNReg,3),[]);
        LossRCNNReg = vision.internal.cnn.maskrcnn.smoothL1(YRCNNReg, single(regressionTargets), single(instanceWeightsReg));
         
        % *Mask Loss (Weighted cross entropy)*
        if obj.params.MaskLossWeight > 0
            maskTargets= cat(4,maskTargets{:});
            LossRCNNMask = vision.internal.cnn.maskrcnn.SpatialCrossEntropy(...
                YMask, single(maskTargets), cat(1,positiveIndex{:}));
        else
            LossRCNNMask = 0;
        end


        % * 6-DoF Pose Losses (rotation and translation) *
        % -----------------------------------------------------------------

        % Calculate instance weighting for pose prediction losses
        % re-using the instance weights from the bbox regression loss
         

        if obj.params.RotationLossWeight > 0

            % Rotation targets (numProposals x numClasses*4)
            rotationTargets = vision.internal.cnn.pose.generatePoseRotationTargets(...
                gtTruthRotation, assignment, positiveIndex, obj.params.NumClasses);
            
            % Rotation Loss (Weighted)
            rotationTargets = cat(1,rotationTargets{:})';
            rotationTargets = reshape(rotationTargets, 1, 1, size(YPoseRot,3),[]);

            if isempty(gPointClouds{:})
                % Direct quaternion regression (default) - per-class predictions
                % - re-using the bbox regression instance weights, since both
                %   bbox targets and quaternions are N x numClasses*4.
                LossRCNNRot = vision.internal.cnn.maskrcnn.smoothL1(...
                    YPoseRot, single(rotationTargets), single(instanceWeightsReg));
            else
                % Apply rotations to object point cloud then compute distance
                % between closest pairs of points (specified by internal options).
                LossRCNNRot = vision.internal.cnn.pose.shapeMatchLoss(...
                        YPoseRot, single(rotationTargets), single(instanceWeightsReg),...
                        gPointClouds, assignment, proposal(5,:), true); % useClosestPoints
            end
            
        else
            LossRCNNRot = 0.0; 
        end

        if obj.params.TranslationLossWeight > 0

            % Translation targets (numProposals x numClasses*3)
            translationTargets = vision.internal.cnn.pose.generatePoseTranslationTargets(...
                gTruthTranslation, assignment, positiveIndex, proposals, gIntrinsics, xyzImg, gTruthBoxes, obj.params.NumClasses);
    
            % Translation Loss (Weighted)
            translationTargets = cat(1,translationTargets{:})';
            translationTargets = reshape(translationTargets, 1, 1, size(YPoseTrans,3),[]);

            % translation loss instance weights - sum-to-1 and non-zero for
            % assigned ground-truth classes
            instanceWeightPoseTrans = instanceWeightsReg > 0;
            instanceWeightPoseTrans = squeeze(instanceWeightPoseTrans);
            instanceWeightPoseTrans(4:4:end,:) = [];
            instanceWeightPoseTrans = reshape(instanceWeightPoseTrans,...
                1, 1, size(instanceWeightPoseTrans,1), size(instanceWeightPoseTrans,2));
            instanceWeightPoseTrans = instanceWeightPoseTrans / nnz(instanceWeightPoseTrans);

            LossRCNNTrans = vision.internal.cnn.maskrcnn.smoothL1(...
                YPoseTrans, single(translationTargets), single(instanceWeightPoseTrans));

        else
            LossRCNNTrans = 0.0;
        end
        % -----------------------------------------------------------------

        
        % Total Stage 2 loss
        LossRCNN = obj.params.BoxLossWeight * LossRCNNReg + ...
            obj.params.ClassLossWeight * LossRCNNClass + ...
            obj.params.MaskLossWeight * LossRCNNMask + ...
            obj.params.RotationLossWeight * LossRCNNRot + ...
            obj.params.TranslationLossWeight * LossRCNNTrans;

         
        % Generate RPN response targets
        %--------------------------------
        featureSize = size(YRPNRegDeltas);
        imageSize = obj.params.ImageSize;
        [RPNRegressionTargets, RPNRegWeights, assignedLabelsRPN] = vision.internal.cnn.maskrcnn.rpnRegressionResponse(featureSize, gTruthBoxes, imageSize, obj.params);
        
        RPNClassificationTargets = onehotencode(assignedLabelsRPN, 3);
        % detectron2 uses ony foreground class for classification.
        RPNClassificationTargets(:,:,2,:) = [];
        RPNClassificationTargets  = reshape(RPNClassificationTargets, featureSize(1), featureSize(2), [], numImagesInBatch );
        RPNClassificationTargets(isnan(RPNClassificationTargets)) = 0;
        
        
        % Stage 1 (RPN) Loss
        % --------------------
        YRPNClass = softmax(YRPNClass);
        LossRPNClass = vision.internal.cnn.maskrcnn.CrossEntropy(YRPNClass, RPNClassificationTargets);
         
        LossRPNReg = vision.internal.cnn.maskrcnn.smoothL1(YRPNRegDeltas, RPNRegressionTargets, RPNRegWeights);
         
        LossRPN = LossRPNClass + LossRPNReg;
        
        
        % Total Loss
        %------------
        loss = LossRCNN + (obj.params.RPNLossWeight * LossRPN);
    
        lossData.Loss = loss;
        lossData.RPNLoss = LossRPN;
        lossData.RMSE = LossRCNNReg;
        lossData.ClassLoss = LossRCNNClass;
        lossData.MaskLoss = LossRCNNMask; 
        lossData.RLoss = LossRCNNRot;         
        lossData.TLoss = LossRCNNTrans;     
        end
    end

end
