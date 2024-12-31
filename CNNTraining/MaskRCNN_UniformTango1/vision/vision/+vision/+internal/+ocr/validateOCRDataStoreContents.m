function data = validateOCRDataStoreContents(data, varargin)
% Validator function for contents of datastore used in OCR functions.
    
% Copyright 2022-2023 The MathWorks, Inc.

    narginchk(1,2);

    if nargin == 2
        isGtruthTextRequired = varargin{1};
    else
        isGtruthTextRequired = true;
    end
    
    % Validate read output class.
    classes = {'cell', 'table'};
    attrs   = {'ndims', 2};
    validateattributes(data, classes, attrs);

    if isGtruthTextRequired
        minSize = 3; % {Image, Bounding box, Text}
    else
        minSize = 2; % {Image, Bounding box}
    end

     % Validate read output size.
    ncols = size(data,2);
    if ~any(ncols >= minSize)
       error(message('vision:ocr:readOutputNotCellTable', minSize));
    end

    % Validate image.
    image = iGetElement(data, 1);
    imgDims = iCheckImage(image);

    % Validate bounding boxes.
    boxes = iGetElement(data, 2);
    iCheckBoxes(boxes, imgDims);

    % Validate labels if required.
    if minSize >= 3
        labels = iGetElement(data, 3);
        iCheckLabels(labels);
    end

    function elementData = iGetElement(data, index)
        if istable(data) % data is a table containing 3 columns.
            elementData = data{1,index};
        else % data is a 1-by-3 cell array.
            elementData = data(1,index);
        end
    end
end

%--------------------------------------------------------------------------
function imgDims = iCheckImage(image)

    if ~iscell(image)
        error(message('vision:ocr:readOutputInvalidImage'));
    end
    
    I = image{1};
    
    if ismatrix(I) 
        nDims = 2;
    else
        nDims = 3;
    end
    
    classes        = {'numeric'};
    attrs          = {'nonempty', 'nonsparse', 'ndims', nDims};
    funcName       = mfilename;
    varDescription = vision.getMessage('vision:ocr:imageDescription');
    varIndex       = 1;
    
    try
        validateattributes(I, classes, attrs, funcName, varDescription, varIndex);
        imgDims = size(I, 1:2);
    catch me
        msg = message('vision:ocr:readOutputInvalidImage');
        throw(addCause(MException(msg), me));
    end
end

%--------------------------------------------------------------------------
function iCheckBoxes(boxes, imgDims)

    if ~iscell(boxes)
        error(message('vision:ocr:readOutputInvalidBbox'));
    end
    
    boxes          = boxes{1};
    classes        = {'numeric'};
    attrs          = {'nonempty', 'nonsparse', '2d', 'ncols', 4};
    
    funcName       = mfilename;
    varDescription = vision.getMessage('vision:ocr:bboxDescription');
    varIndex       = 2;
    try
        validateattributes(boxes, classes, attrs, funcName, varDescription, varIndex);
    catch me
        msg = message('vision:ocr:readOutputInvalidBbox');
        throw(addCause(MException(msg), me));
    end

    iCheckBoxPositions(boxes, imgDims);
end

%--------------------------------------------------------------------------
function iCheckBoxPositions(boxes, imgDims)

    points = bbox2points(boxes);

    % Get the upper-left and lower-right corner points of all bboxes.
    x1 = [points(1,1,:)];
    y1 = [points(1,2,:)];
    x2 = [points(3,1,:)];
    y2 = [points(3,2,:)];
    
    if any(x1 < 1 | y1 < 1 | x2 > imgDims(2) | y2 > imgDims(1))
        error(message('vision:ocr:bboxOutsideImage'))
    end
end

%--------------------------------------------------------------------------
function iCheckLabels(labels)

    if ~iscell(labels)
        error(message('vision:ocr:readOutputInvalidLabel'));
    end

    labels         = labels{1};
    classes        = {'string'};
    attrs          = {'nonempty', 'nonsparse', '2d'};
    funcName       = mfilename;
    varDescription = vision.getMessage('vision:ocr:labelDescription');
    varIndex       = 3;
    try
        validateattributes(labels, classes, attrs, funcName, varDescription, varIndex);
    catch me
        msg = message('vision:ocr:readOutputInvalidLabel');
        throw(addCause(MException(msg), me));
    end
end