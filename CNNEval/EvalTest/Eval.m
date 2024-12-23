clear
clc
close all

%% run image segmentation
load("RESNET101_net_checkpoint__800__2024_11_13__11_27_26.mat")
dsTest= imageDatastore("TestIms/")
dsResults= segmentObjects(net, dsTest, "Threshold",0.5)

%% evaluate
clc
clear

dsResults = fileDatastore("./SegmentObjectResults/", ReadFcn=@(x)SegMATReader(x)); %training data
dsTruth  = fileDatastore("./TestDSFs", ReadFcn=@(x)TestMATReader(x)); %training data

function metrics = evaluateInstanceSegmentation(dsResults, dsTruth, threshold)

    numPred = numel(dsResults);
    numTruth = numel(dsTruth);
    
    % Initialize variables
    truePositive = 0;
    iouScores = zeros(numPred, numTruth);
    
    % Calculate IoU matrix
    for i = 1:numPred
        for j = 1:numTruth
            iouScores(i, j) = calculateIoU(dsResults{i}, dsTruth{j});
        end
    end
    
    % Match predictions to ground truth based on threshold
    matched = iouScores > threshold;
    
    % Determine true positives
    for i = 1:numTruth
        if any(matched(:, i))
            truePositive = truePositive + 1;
        end
    end
    
    % Calculate false positives and false negatives
    falsePositive = numPred - truePositive;
    falseNegative = numTruth - truePositive;
    
    % Metrics calculations
    precision = truePositive / (truePositive + falsePositive);
    recall = truePositive / (truePositive + falseNegative);
    f1Score = 2 * (precision * recall) / (precision + recall);
    meanIoU = mean(iouScores(matched));
    
    % Output metrics
    metrics = struct();
    metrics.Precision = precision;
    metrics.Recall = recall;
    metrics.F1Score = f1Score;
    metrics.MeanIoU = meanIoU;
end

function iou = calculateIoU(mask1, mask2)
    % Calculate Intersection over Union (IoU) for two binary masks.
    % Inputs:
    %   mask1 - Binary mask for prediction
    %   mask2 - Binary mask for ground truth
    % Output:
    %   iou - Intersection over Union score
    
    intersection = sum(mask1(:) & mask2(:));
    union = sum(mask1(:) | mask2(:));
    iou = intersection / union;
end

%%
mean(cell2mat(metrics.ImageMetrics.AP))