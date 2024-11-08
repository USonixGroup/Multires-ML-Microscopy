clear
clc
close all

%%


ds = fileDatastore("./DSFs", ReadFcn=@(x)cocoAnnotationMATReader(x));

%%
data = preview(ds)
trainClassNames = ["CellA"];
imageSizeTrain = [520 740 3];

net = maskrcnn("resnet50-coco",trainClassNames,InputSize=imageSizeTrain)


%%
options = trainingOptions("sgdm", ...
    InitialLearnRate=0.001, ...
    LearnRateSchedule="piecewise", ...
    LearnRateDropPeriod=1, ...
    LearnRateDropFactor=0.95, ...
    Plot="none", ...
    Momentum=0.9, ...
    MaxEpochs=10, ...
    MiniBatchSize=2, ...
    BatchNormalizationStatistics="moving", ...
    ResetInputNormalization=false, ...
    ExecutionEnvironment="gpu", ...
    VerboseFrequency=50);
%%

doTraining = true;
if doTraining
    [net,info] = trainMaskRCNN(ds,net,options,FreezeSubNetwork="backbone");
    modelDateTime = string(datetime("now",Format="yyyy-MM-dd-HH-mm-ss"));
    save("trainedMaskRCNN-"+modelDateTime+".mat","net");
end


%%
read(ds)

