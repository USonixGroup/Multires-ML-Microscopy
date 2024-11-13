clear
clc
close all


load("trainedMaskRCNN-2024-11-12-00-45-27.mat")
dsTest= imageDatastore("TestIms/")
dsResults= segmentObjects(net, dsTest, "Threshold",0.5)

%%
dsResults = fileDatastore("./SegmentObjectResults05", ReadFcn=@(x)SegMATReader(x)); %training data

dsTruth  = fileDatastore("./TestDSFs", ReadFcn=@(x)TestMATReader(x)); %training data

metrics=evaluateInstanceSegmentation(dsResults,dsTruth, 0.5, "Verbose",true)

%%
mean(cell2mat(metrics.ImageMetrics.AP))