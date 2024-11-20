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

metrics=evaluateInstanceSegmentation(dsResults,dsTruth, 0.5, "Verbose",true)

%%
mean(cell2mat(metrics.ImageMetrics.AP))