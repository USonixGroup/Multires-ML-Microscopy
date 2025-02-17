function [masks labels scores boxes] = segmentCells(net, Image, Options)
    arguments
        Image
        Options.Wavelet char = 'db5' % default value
        Options.Level (1,1) {mustBeInteger, mustBeReal} = 4 % default value
        Options.DWTThreshold {mustBeGreaterThanOrEqual(Options.DWTThreshold, 0), mustBeLessThan(Options.DWTThreshold, 1), mustBeReal(Options.DWTThreshold)}= 0.1; % default value
        Options.SegmentThreshold (1,1) {mustBeGreaterThanOrEqual(Options.SegmentThreshold, 0), mustBeLessThan(Options.SegmentThreshold, 1), mustBeReal(Options.SegmentThreshold)}= 0.5;
        Options.NumstrongestRegions (1,1) = Inf;
        Options.SelectStrongest logical = 1;
        Options.MinSize (1,2) = [8 8];
        Options.MaxSize (1,2) = [64 64];
        Options.ShowIm (1,1) logical = 0;
        Options.ShowScores (1,1) logical =0;
    end


Image = DWT_Denoise(Image, Options)


[masks,labels,scores,boxes] = segmentObjects(net,Image,Threshold=Options.SegmentThreshold,NumStrongestRegions=Options.NumstrongestRegions, SelectStrongest=Options.SelectStrongest, MinSize=Options.MinSize,MaxSize=Options.MaxSize);

if Options.ShowIm==1
    if(isempty(masks))
        overlayedImage = Image(:,:,1);
    else
        overlayedImage = insertObjectMask(Image(:,:,1), masks,Color=lines(size(masks, 3)) );
    end
    
    figure, imshow(overlayedImage)
    if Options.ShowScores==1
        showShape("rectangle", gather(boxes), "Label", scores, "LineColor",'r')
    end


end