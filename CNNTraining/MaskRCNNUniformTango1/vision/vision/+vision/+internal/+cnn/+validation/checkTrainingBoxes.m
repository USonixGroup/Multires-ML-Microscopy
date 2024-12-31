function boxes = checkTrainingBoxes(images,boxes,isRotatedRectangle)
%checkTrainingBoxes Scans for valid training boxes in the format provided
% by boxLabelDatastore.
%
% See also boxLabelDatastore.

% Copyright 2019-2023 The MathWorks, Inc.

    narginchk(2,3)

    if nargin == 2
        isRotatedRectangle = false;
    end

    for ii = 1:size(boxes, 1)
        [boxes{ii},allValidBoxes] = vision.internal.cnn.utils.hasValidTrainingBoxes(size(images{ii}),boxes{ii},isRotatedRectangle);
        if ~allValidBoxes
            error(message('vision:ObjectDetector:invalidBoxesInTrainingData'));
        end
    end
end
