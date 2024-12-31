classdef YOLOv2TransformHostStrategy < nnet.internal.cnn.layer.util.ExecutionStrategy
    % YOLOv2TransformHostStrategy  Execution strategy for running YOLO v2
    % transform algorithm on the host. According to this layer, the
    % IoU predictions, box predictions, score predictions are predicted
    % from activations of last convolutional layer.
    
    %   Copyright 2018 The MathWorks, Inc.
    
    methods
        function Z = forward(~, X, numAnchorBoxes)
            % Input X consists of activations from last convolutional layer.
            %
            % Input numAnchorBoxes are number of anchors defined by user.
            %
            % Output Z consists of predicted [IoU , box coordinates, class
            % scores] for each grid in final activations.
            
            inSize = size(X);
            
            % Input X is reshaped to get the predictions along channels.
            reshapedAct = reshape(X,inSize(1)*inSize(2),numAnchorBoxes,inSize(3)/numAnchorBoxes,[]);
            
            % Squash the IoU, x, y cooridnaes to range from 0 to 1.
            iouPred = 1./(1+exp(-1*(reshapedAct(:,:,1,:))));
            xyPred = reshapedAct(:, :,2:3,:);
            sigmaXY = 1./(1+exp(-1*xyPred));
            
            % Get w, h predictions from feature map.
            whPred = reshapedAct(:,:,4:5,:);
            expWH = exp(whPred);
            
            % Compute probabilities of class predictions.
            scorePred = reshapedAct(:,:,6:end,:);
            probPred = nnet.internal.cnnhost.softmaxForward(scorePred, 3);
            
            % Reshaping back to original dimension of X.
            Z = cat(3,iouPred,sigmaXY,expWH,probPred);
            Z = reshape(Z,inSize);
        end
        
        function [dX, dW] = backward(~,X,Z,dLdZ, this)
            % Backward function of decodeActivations, to compute dX with respect to
            % input data. This function computes dX and converts them to feature
            % dimension of last convolutional layer.
            
            numAnchorBoxes = this.NumAnchorBoxes;
            
            inSize = size(X);
            
            % Extract [iou,x,y,w,h,classProb] from input.
            reshapedZ = reshape(Z,inSize(1)*inSize(2),numAnchorBoxes,inSize(3)/numAnchorBoxes,[]);

            iouPred = reshapedZ(:,:,1,:);
            xyPred = reshapedZ(:,:,2:3,:);
            whPred = reshapedZ(:,:,4:5,:);
            scorePred = reshapedZ(:,:,6:end,:);

            % Extract [iou,x,y,w,h,classProb] from gradients.
            reshapedDZ = reshape(dLdZ,inSize(1)*inSize(2),numAnchorBoxes,inSize(3)/numAnchorBoxes,[]);

            iouPredDerivative = reshapedDZ(:,:,1,:);
            xyPredDerivative = reshapedDZ(:,:,2:3,:);
            whPredDerivative = reshapedDZ(:,:,4:5,:);
            scorePredDerivative = reshapedDZ(:,:,6:end,:);
            
            % Use inverse of sigmoid function to get original range for iou predictions.
            diou = iouPred .*(1.0-iouPred);
            diou = diou.* iouPredDerivative;            
            
            % Use inverse of sigmoid function to get original range for x,y predictions.
            dxy = xyPred .*(1.0-xyPred);
            dxy = dxy.* xyPredDerivative;
            
            % Use inverse of exponential function to get original range for w,h predictions.
            dwh = whPred;
            dwh = dwh.* whPredDerivative;
            
            % Use inverse of softmax function to get original range.
            scorePred = nnet.internal.cnn.util.boundAwayFromZero(scorePred);
            dScore = nnet.internal.cnnhost.softmaxBackward(scorePred, scorePredDerivative, 3);
            
            dX = cat(3,diou,dxy,dwh,dScore);
            dX = reshape(dX,inSize);
            dW = []; % no learnable parameters.
        end
    end
end
