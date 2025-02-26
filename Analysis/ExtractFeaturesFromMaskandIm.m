clear
clc
close all
load("label_A172_Phase_A7_1_00d04h00m_1.tif.mat") %test file for one image, will turn into function later
im = rescale(im);


numCells = size(masks, 3); %find number of cells
maskIms = masks .* im(:,:,:); %pixels within each maskt

lpp=1; %micrometers/pixel

Area = squeeze(sum(masks,1:2)) * lpp^2; %total number of pixels per mask times constant
%Area = table(Area);

for i=[1:numCells]

    Perim(i, 1) = sum(bwperim(masks(:,:,i)),"all");
    FeretDiameter(i,:) = bwferet(masks(:,:,i));

end
FeretDiameter{:,1} = FeretDiameter{:,1} * lpp; %convert diameters to micrometers (but not coordinates in the image)


% Phase Contrast Intensity stats for each mask
[PCIstats, IntensityDistStats] = IntensityStats(maskIms);


%%
sizeX = size(im,2);
sizeY = size(im,1);

[X Y] = meshgrid(1:sizeX, 1:sizeY);

%find weighted coordinate for intensity in each mask
WCx = squeeze(sum(maskIms .* X, 1:2))./Area; 
WCy = squeeze(sum(maskIms .* Y, 1:2))./Area; 

%use bounding box values to find position relative to cell itself, normalize by the size of the bounding box

NWCx = bbox(:,1)- WCx;
NWCy = bbox(:,2)- WCy;


%%


%%



%%
imshow(a(:,:,3))



%%
function [PCIstats, IntensityDistStats] = IntensityStats(maskIms)
Intensities = reshape(maskIms, [], size(maskIms,3), 1);
for i=[1:size(maskIms,3)]
    
    ins = Intensities(:,i);
    ins = ins(ins ~= 0); %remove 0 values
    ins = sort(ins);

    Mean(i,:) = mean(ins);
    Min(i,:) = min(ins);
    Max(i,:) = max(ins);
    P5(i,:) = ins(ceil(0.05*length(ins))); %5th percentile
    P95(i,:) = ins(ceil(0.95*length(ins))); %95th percentile

    Skewness(i,:) = skewness(ins);
    Kurtosis(i,:) = kurtosis(ins);
    

end
    PCIstats = table(Mean, Min, Max, P5, P95);
    IntensityDistStats = table(Skewness, Kurtosis);

    

end