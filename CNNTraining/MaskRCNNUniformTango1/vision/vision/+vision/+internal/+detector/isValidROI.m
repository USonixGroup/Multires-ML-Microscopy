function validROI = isValidROI(roi, imageSize)
% Return a logical column vector with length equal to size(roi,1).
% validROI(k) is true if roi(k,:) is within the image boundary.
%
% An ROI is valid if it is not empty, is fully contained within the
% image boundary, and has > 0 width/height.

% Copyright 2018-2023 The MathWorks, Inc. 

if ~iMatrixWithFourOrFiveColumns(roi)
    validROI = false;    
else
    imWidth = imageSize(2);
    imHeight = imageSize(1);
    if size(roi,2) == 4
        validROI = ...
            roi(:,1) >= 1 & ...
            roi(:,2) >= 1 & ...
            roi(:,1)+roi(:,3)-1 <= imWidth & ...
            roi(:,2)+roi(:,4)-1 <= imHeight;

    else
        % Convert rotated rectangle bounding box to vertices.
        bboxPointsTemp = bbox2points(roi);

        % Remap bboxPoints to be an M-by-8 matrix, where M is the number of
        % bounding boxes and each row is specified as [x1 x2 x3 x4 y1 y2 y3 y4].
        bboxPoints = round(reshape(bboxPointsTemp,[8,size(roi,1)])');

        validROI = ...
            all(bboxPoints >= 1 & ...
            [bboxPoints(:,1:4) <= imWidth bboxPoints(:,5:8) <= imHeight],2);
    end

    validROI = validROI & roi(:,3) > 0 & roi(:,4) > 0;
    
end

%----------------------------------
function tf = iMatrixWithFourOrFiveColumns(roi)
tf = ~isempty(roi) && ismatrix(roi) && (size(roi,2) == 4 || size(roi,2) == 5);
