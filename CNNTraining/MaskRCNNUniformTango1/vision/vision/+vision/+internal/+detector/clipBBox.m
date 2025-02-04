%==========================================================================
% Clip the bounding box when it is positioned outside the image. This can
% happen when detections occur near the image boundaries. 
%==========================================================================
function clippedBBox = clipBBox(bbox, imgSize)
%

%   Copyright 2016-2023 The MathWorks, Inc.

%#codegen

% Grab image width and height.
imWidth = imgSize(2);
imHeight = imgSize(1);

if size(bbox,2) == 4
    % The original bounding boxes all overlap the image. Therefore, a check to
    % remove non-overlapping boxes is not required.

    % bounding boxes are returned as doubles
    clippedBBox  = double(bbox);
    
    % The original bounding boxes all overlap the image. Therefore, a check to
    % remove non-overlapping boxes is not required.
    
    % Get coordinates of upper-left (x1,y1) and bottom-right (x2,y2) corners. 
    x1 = clippedBBox(:,1);
    y1 = clippedBBox(:,2);
    
    x2 = clippedBBox(:,1) + clippedBBox(:,3) - 1;
    y2 = clippedBBox(:,2) + clippedBBox(:,4) - 1;
    
    x1(x1 < 1) = 1;
    y1(y1 < 1) = 1;
    
    x2(x2 > imWidth) = imWidth;
    y2(y2 > imHeight) = imHeight;
    
    clippedBBox = [x1 y1 x2-x1+1 y2-y1+1];
else
    angle = double(bbox(:,5));
    bboxPointsTemp = bbox2points(bbox);

    % Remap bboxPoints to be an M-by-8 matrix, where M is the number of
    % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
    bboxPoints = reshape(bboxPointsTemp,[8,size(bbox,1)])';
    
    % Clamp vertices to image boundaries.
    bboxPoints(:,1:4) = max( min( bboxPoints(:,1:4), imWidth  ), 1 );
    bboxPoints(:,5:8) = max( min( bboxPoints(:,5:8), imHeight ), 1 );
    
    % Since any of the sides of the bounding box may
    % have been clipped, calculate and use the minimum
    % widths and heights to reflect the clipping.
    dLength = hypot(bboxPoints(:,3:4) - bboxPoints(:,1:2), ...
                    bboxPoints(:,7:8) - bboxPoints(:,5:6))/2;
    w = round(max(hypot(bboxPoints(:,2:3) - bboxPoints(:,[1 4]), ...
        bboxPoints(:,6:7) - bboxPoints(:,[5 8])),[],2),5);
    h = round(max(hypot(bboxPoints(:,3:4) - bboxPoints(:,[2 1]), ...
               bboxPoints(:,7:8) - bboxPoints(:,[6 5])),[],2),5);
    
    % Use the potentially clipped diagonal lengths to
    % calculate where the new center coordinate should
    % be.
    if dLength(:,1) <= dLength(:,2)
        xCtr = (bboxPoints(:,1) + bboxPoints(:,3))/2;
        yCtr = (bboxPoints(:,5) + bboxPoints(:,7))/2;
    else
        xCtr = (bboxPoints(:,2) + bboxPoints(:,4))/2;
        yCtr = (bboxPoints(:,6) + bboxPoints(:,8))/2;
    end
    
    clippedBBoxTemp = [xCtr yCtr w h angle];

    % Since more than one rotated bounding box's vertices can be clipped,
    % maintaining the same input angle can result in a vertice going
    % outside the image boundary. Angle is to be maintained, so reclip if
    % the new rotated rectangle bounding box is not within the image
    % boundary.
    isValidIdx = vision.internal.detector.isValidROI(clippedBBoxTemp, imgSize);
    if any(~isValidIdx)
        clippedBBoxTemp(~isValidIdx,:) = vision.internal.detector.clipBBox(clippedBBoxTemp(~isValidIdx,:), imgSize);
    end

    clippedBBox = clippedBBoxTemp;

end
