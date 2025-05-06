clear
clc
close all

% run image segmentation
load("NETEFF_YEYEY.mat", 'net')

%net.OverlapThresholdRPN = 0.3;
net.OverlapThresholdPrediction = 0.3;
%net.ScoreThreshold = 0.1;
%%



dsTest= fileDatastore("./SingleDS/", ReadFcn=@(x)TestIMsMATReader(x));

tic
dsResults= segmentObjects(net, dsTest, "Threshold",0.001,"MinSize",[2 2],"MaxSize",[80 80],"NumStrongestRegions",1000,"SelectStrongest",true);
toc
%% evaluate



dsResults = fileDatastore("./SegmentObjectResults1/", ReadFcn=@(x)SegMATReader(x)); %segmented data
dsTruth  = fileDatastore("./SingleDS", ReadFcn=@(x)TestMATReader(x)); %training data
j=1;
 for i=[0.5:0.05:0.95]
tic
metrics = evaluateInstanceSegmentation(dsResults, dsTruth, i,"Verbose",true);
toc
cellmetrics{j} = metrics;
j=j+1;
save(n)
end





