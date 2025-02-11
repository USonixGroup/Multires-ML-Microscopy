function [boxes, noBoxesRemoved, hasNoBoxes] = hasValidTrainingBoxes(imageSize, boxes, isRotatedRectangle)
%hasValidTrainingBoxes Scans for valid training boxes in the format provided
% by boxLabelDatastore.
%
% See also boxLabelDatastore.

% Copyright 2019-2023 The MathWorks, Inc.

    narginchk(2,3)

    if nargin == 2
        isRotatedRectangle = false;
    end

    % Scans one training data row and removes invalid boxes,
    % returning a clean row of boxes.

    boxes = iManageEmptyBoxes(boxes, isRotatedRectangle);

    % Round fractional ground truth boxes to nearest pixel centers.
    if ~isRotatedRectangle
        boxes = vision.internal.cnn.utils.roundGroundTruthBoxes(boxes);
    end
 
    % Clip boxes to within image bounds.
    boxes = vision.internal.detector.clipBBox(boxes, imageSize);

    % Remove any invalid boxes that may be left over.
    validBoxes = vision.internal.detector.isValidROI(boxes, imageSize);

    noBoxesRemoved       = all(validBoxes);
    if ~isempty(boxes)        
        boxes(~validBoxes,:) = [];
    end
    
    hasNoBoxes           = isempty(boxes);
end

%--------------------------------------------------------------------------
function boxes = iManageEmptyBoxes(boxes, isRotatedRectangle)
    if isempty(boxes)
        % Return canonical empty box.
        if iscell(boxes)
            if ~isRotatedRectangle
                boxes = zeros(0,4);
            else
                boxes = zeros(0,5);
            end
        else
            if ~isRotatedRectangle
                boxes = zeros(0,4,'like',boxes);
            else
                boxes = zeros(0,5,'like',boxes);
            end
        end
    end
end
