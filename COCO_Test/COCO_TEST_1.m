clear
clc
close all
%%


imageFolder = fullfile("LiceCellsIms1", "livecell_train_val_images");
annotationFile = fullfile("livecell_coco_train.json");



cocoAPIDir = fullfile("./","cocoapi-master","MatlabAPI");
addpath(cocoAPIDir);

unpackAnnotationDir = fullfile("./","annotations_unpacked","matFiles");
if ~exist(unpackAnnotationDir,'dir')
    mkdir(unpackAnnotationDir)
end
trainClassNames = ["Cell1"];

%%

unpackAnnotations(trainClassNames,annotationFile,imageFolder,unpackAnnotationDir);
