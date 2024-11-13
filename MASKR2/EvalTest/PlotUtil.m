clear
clc
close all
%%
load("trainedMaskRCNN-2024-11-12-00-45-27.mat")

%%
im=imread("TestIms/A172_Phase_C7_1_02d00h00m_1.tif");

[masks,labels,scores,boxes] = segmentObjects(net,im,Threshold=0.9);


overlayedImage = insertObjectMask(im,masks);
imshow(overlayedImage);
%showShape("rectangle",boxes,Label=labels,LineColor=[1 0 0]);


