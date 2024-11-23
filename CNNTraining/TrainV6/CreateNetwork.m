clear
clc
close all


addpath('./src');


%trainClassNames = ["CellA"];
imageSize=[520 704 3];

trainCats = {'CellA'};


classNames = trainCats;
numClasses = numel(classNames);
% Add a background class
classNames = [classNames {'background'}];


% datdir="/home/zcemydo/Scratch/TrainV2/";
% ds = fileDatastore([datdir+"/DSFs"], ReadFcn=@(x)cocoAnnotationMATReader(x)); %training datatrainDS = transform(ds, @(x)helper.preprocessData(x, imageSize));


params = createMaskRCNNConfig(imageSize, numClasses, classNames);


    params.NumRegionsToSample = 2000;
    params.NumStrongestRegions = 1000;
    params.RPNROIPerImage=1000;
    params.MinSize=[3 3]

    params.AnchorBoxes=[
    16 16
    32 32
    64 64
    128 128
    256 256
    16 8
    8 16
    32 16
    16 32
    32 64
    64 32
    10 10
    20 20
    30 30
    40 40
    10 20
    10 30
    10 40
    20 10
    20 30
    20 40
    30 10
    30 20
    30 40
    40 10
    40 20
    40 30 ]
   
    params.NumAnchors=size(params.AnchorBoxes,1)

disp(params);


dlnet = createMaskRCNN(numClasses, params, 'resnet101'); %or resnet 101
%%
save("NET101.mat", "dlnet", 'params')

