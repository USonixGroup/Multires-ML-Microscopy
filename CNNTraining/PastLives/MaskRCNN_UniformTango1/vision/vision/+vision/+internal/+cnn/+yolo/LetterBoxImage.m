function [Inew,bboxnew] = LetterBoxImage(I,targetSize,varargin)
% LetterBoxImage returns a resized image by preserving the width and height
% aspect ratio of input Image I. targetSize is a 1-by-2 vector consisting 
% the target dimension.
%
% Input I can be uint8, uint16, int16, double, single, or logical, and must
% be real and non-sparse.

% Copyright 2021-2024 The MathWorks, Inc.

[Irow,Icol,Ichannels] = size(I);
bboxnew = [];

% Compute aspect Ratio.
arI = Irow./Icol;

% Preserve the maximum dimension based on the aspect ratio.
if arI<1  
    % width changed
    IcolFin = targetSize(1,2);
    IrowFin = floor(IcolFin.*arI);
else 
    % height changed
    IrowFin = targetSize(1,1);
    IcolFin = floor(IrowFin./arI);
end

% Resize the input image.
Itmp = imresize(I,[IrowFin,IcolFin]);

% Initialize Inew with gray values.
Inew = ones([targetSize,Ichannels],'like',I).*0.5;

% Compute the offset.
if arI<1
    buff = targetSize(1,1)-IrowFin;
else
    buff = targetSize(1,2)-IcolFin;
end

% Place the resized image on the canvas image.
if (buff==0)
    Inew = Itmp;
    if ~isempty(varargin)
        imgSize = size(I,1:2);
        if ~isempty(varargin{1,1})
            boxScale = targetSize./imgSize;
            bboxnew = bboxresize(varargin{1,1},boxScale);
        end
    end
else
    % When buffVal <=1, leave out the last row/column by starting from 1.
    buffVal = max(floor(buff/2),1);
    if arI<1
        Inew(buffVal:buffVal+IrowFin-1,:,:) = Itmp;
        if ~isempty(varargin)
            % Resize bounding boxes.
            bboxnew = iScaleBboxes(varargin{1,1},size(Itmp),size(I));
            if ~isempty(bboxnew)
                bboxnew(:,2) = bboxnew(:,2)+buffVal;
            end
        end
    else
        Inew(:,buffVal:buffVal+IcolFin-1,:) = Itmp;
        if ~isempty(varargin)
            % Resize bounding boxes.
            bboxnew = iScaleBboxes(varargin{1,1},size(Itmp),size(I));
            if ~isempty(bboxnew)
                bboxnew(:,1) = bboxnew(:,1)+buffVal;
            end
        end
    end
end

end

%--------------------------------------------------------------------------
function bboxPred = iScaleBboxes(bboxes,imSz,newImSz)
bboxPred = [];
scale   = imSz(1:2)./newImSz(1:2);
[info.ScaleX,info.ScaleY] = deal(scale(2),scale(1));
    if size(bboxes,2) == 4
        % scale axis aligned boxes 
        bboxesX1Y1X2Y2 = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(bboxes);
        % Saturate X2,Y2 to the original image dimension, to remove the gray scale
        % scale detections if any.
        bboxesX1Y1X2Y2(:,3) = min(bboxesX1Y1X2Y2(:,3),newImSz(1,2));
        bboxesX1Y1X2Y2(:,4) = min(bboxesX1Y1X2Y2(:,4),newImSz(1,1));
        
        % Scale the boxes to the image dimension.
        bboxesX1Y1X2Y2 = vision.internal.cnn.boxUtils.scaleX1X2Y1Y2(bboxesX1Y1X2Y2, info.ScaleX, info.ScaleY);
        bboxPred = vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(bboxesX1Y1X2Y2);

    elseif size(bboxes,2) == 5
        % Scale rotated boxes 
        % Convert rotated bbox to vertices for scaling.
        bboxPointsTemp = bbox2points(bboxes);

        % Remap bboxPointsTemp to be an M-by-8 matrix, where M is the number of
        % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
        bboxPoints = reshape(bboxPointsTemp,[8,size(bboxes,1)])';

        % Scale the boxes to the image dimension
        % Scale [x1 x2 x3 x4 y1 y2 y3 y4] box in the x-direction by info.ScaleX and 
        % in the y-direction by info.ScaleY.

        % Scale the vertices based on scaling factor input.
        bboxPoints(:,1:4) = bboxPoints(:,1:4).* info.ScaleX;
        bboxPoints(:,5:8) = bboxPoints(:,5:8).* info.ScaleY;
        
        % Convert scaled vertices back to bbox format.
        w = max(hypot(bboxPoints(:,2:3) - bboxPoints(:,[1 4]), ...
                   bboxPoints(:,6:7) - bboxPoints(:,[5 8])),[],2);
        h = max(hypot(bboxPoints(:,3:4) - bboxPoints(:,[2 1]), ...
                   bboxPoints(:,7:8) - bboxPoints(:,[6 5])),[],2);

        % Re-scale the center point
        xCtr = info.ScaleX * bboxes(:,1);
        yCtr = info.ScaleY * bboxes(:,2);

        bboxPred = [xCtr yCtr w h bboxes(:,5)];
    end
end

