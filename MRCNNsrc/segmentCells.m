function [masks labels scores boxes] = segmentCells(net, Image, Options)
% SEGMENTCELLS De-noises image via DWT-thresholding and uses a Mask R-CNN network to segment cells in an image
    arguments %checks and default values
        net MRCNN 
        Image = [];
        Options.Denoise (1,1) logical =1;
        Options.Wavelet char = 'db5'
        Options.Level (1,1) {mustBeInteger, mustBeReal} = 4 
        Options.DWTThreshold {mustBeGreaterThanOrEqual(Options.DWTThreshold, 0), mustBeLessThan(Options.DWTThreshold, 1), mustBeReal(Options.DWTThreshold)}= 0.02; 
        Options.SegmentThreshold (1,1) {mustBeGreaterThanOrEqual(Options.SegmentThreshold, 0), mustBeLessThan(Options.SegmentThreshold, 1), mustBeReal(Options.SegmentThreshold)}= 0.5;
        Options.NumstrongestRegions (1,1) = Inf;
        Options.SelectStrongest logical = 1;
        Options.MinSize (1,2) = [8 8];
        Options.MaxSize (1,2) = [64 64];
        Options.ShowMasks (1,1) logical = 0;
        Options.ShowScores (1,1) logical = 0;
    end

if Options.Denoise==1 %apply denoising
    Image = DWT_Denoise(Image, "Level",Options.Level,"Threshold",Options.DWTThreshold,"Wavelet",Options.Wavelet);
end

Image = rescale(Image); %scale [0,1] to pass to Mask R CNN Network
[masks,labels,scores,boxes] = segmentObjects(net,Image,Threshold=Options.SegmentThreshold,NumStrongestRegions=Options.NumstrongestRegions, SelectStrongest=Options.SelectStrongest, MinSize=Options.MinSize,MaxSize=Options.MaxSize);

if Options.ShowMasks==1 %display masks
    if(isempty(masks))
        overlayedImage = Image(:,:,1);
    else
        overlayedImage = insertObjectMask(Image(:,:,1), masks,Color=lines(size(masks, 3)) );
    end
    
    figure, imshow(overlayedImage)
    if Options.ShowScores==1 %display bounding boxes and confidence scores [0,1]
        showShape("rectangle", gather(boxes), "Label", scores, "LineColor",'r');
    end


end