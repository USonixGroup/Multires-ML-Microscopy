clear
clc
close all

%addpath("./vision") %when running on Myriad, needed to load the vision
%toolbox

ds = fileDatastore("./DSFs", ReadFcn=@(x)cocoAnnotationMATReader(x)); %training data
read(ds)
%%
valds=fileDatastore("./ValDSFs", ReadFcn=@(x)cocoAnnotationMATReader(x)); %validation data
read(valds)

%%
data = preview(ds)
trainClassNames = ["CellA"];
imageSizeTrain = [520 704 3];

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
    ValidationData=valds, ...
    ValidationFrequency=20, ...
    BatchNormalizationStatistics="moving", ...
    ResetInputNormalization=false, ...
    ExecutionEnvironment="auto", ...
    VerboseFrequency=10);


%%

doTraining = true;
if doTraining
    [net,info] = trainMaskRCNN(ds,net,options) %,FreezeSubNetwork="backbone"); %to freeze a specific subnetwork from 
    modelDateTime = string(datetime("now",Format="yyyy-MM-dd-HH-mm-ss"));
    save("trainedMaskRCNN-"+modelDateTime+".mat","net", "info");
end



% %% utility to use model to test certain images
% im1=imread("../JSON_FORMATTING/LiveCellsIms1/livecell_test_images/A172_Phase_C7_1_00d00h00m_3.tif");
% 
% [masks,labels,scores,boxes] = segmentObjects(net,im1,Threshold=0.5);
% 
% %%
% imshow(insertObjectMask(im1,masks, Color=lines(size(masks, 3))))