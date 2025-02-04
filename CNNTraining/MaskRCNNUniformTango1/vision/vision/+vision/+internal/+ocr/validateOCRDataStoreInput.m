function cpy = validateOCRDataStoreInput(ds, varargin)
% Validator function for input ground truth datastores in OCR functions to
% check for class type and output class and size.
    
% Copyright 2022-2023 The MathWorks, Inc.

    narginchk(1,2);

    if nargin == 2
        isGtruthTextRequired = varargin{1};
    else
        isGtruthTextRequired = true;
    end
    
    % Validate input class.
    validClasses = {'matlab.io.Datastore', 'matlab.io.datastore.Datastore'};
    validateattributes(ds, validClasses,{});

    % Backup input datastore to read inside.
    cpy = copy(ds);
    reset(cpy);
    sampleData = read(cpy);
    reset(cpy);

    if isGtruthTextRequired
        minSize = 3; % {Image, Bounding box, Text}
    else
        minSize = 2; % {Image, Bounding box}
    end

    % Validate read output class.
    try 
        classes = {'cell', 'table'};
        attrs   = {'ndims', 2};
        validateattributes(sampleData, classes, attrs);
    catch ex
        error(message('vision:ocr:readOutputNotCellTable', minSize));
    end

    % Validate read output size.
    ncols = size(sampleData,2);
    if ~any(ncols >= minSize)
       error(message('vision:ocr:readOutputNotCellTable', minSize));
    end
end