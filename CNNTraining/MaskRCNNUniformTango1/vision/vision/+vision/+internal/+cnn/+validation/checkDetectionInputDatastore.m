function sampleImage = checkDetectionInputDatastore(datastore, filename)
%CHECKDETECTIONINPUTDATASTORE Verify that the input datastore for
% detection is infact a datastore and its read method returns
% a numeric array or an M-by-N column table or cell, with images
% in the first column.

% Copyright 2019 The MathWorks, Inc.

    narginchk(2,2);
    varDescription = 'Datastore input for detect';
    varIndex       = 2;

    try
        validateattributes(datastore, {'matlab.io.Datastore', 'matlab.io.datastore.Datastore'},...
            {'nonempty'}, filename, varDescription, varIndex);
    catch me
        msg = message('vision:ObjectDetector:invalidDetectionInput');
        throw(addCause(MException(msg), me));
    end

    sampleData = iReadNonEmpty(datastore);

    if isnumeric(sampleData)
        sampleImage = sampleData;
        iCheckImages({sampleData}, filename);
    else
        classes = {'cell', 'table'};
        attrs   = {'ndims', 2};
        varDescription = 'Read output of training datastore input';
        try
            validateattributes(sampleData, classes, attrs, filename, varDescription, varIndex);
        catch me
            msg = message('vision:ObjectDetector:invalidDetectionDatastoreReadOutput');
            throw(addCause(MException(msg), me));
        end
        if istable(sampleData)
            images = sampleData{1,1};
        else
            images = sampleData(1,1);
        end
        iCheckImages(images, filename);
        sampleImage = images{1};
    end
end

function iCheckImages(images, filename)
    msg = message('vision:ObjectDetector:invalidDetectionDatastoreReadOutput');
    if ~iscell(images)
        error(msg);
    end
    I = images{1};
    if ndims(I) == 2
        nDims = 2;
    else
        nDims = 3;
    end
    classes        = {'numeric'};
    attrs          = {'nonempty', 'nonsparse', 'ndims', nDims};
    varDescription = 'Images in read output of datastore';
    varIndex       = 1;
    try
        validateattributes(I, classes, attrs, filename, varDescription, varIndex);
    catch me
        throw(addCause(MException(msg), me));
    end
end

function data = iReadNonEmpty(datastore)
    cpy = copy(datastore);
    reset(cpy);
    data = {};
    while hasdata(cpy) && isempty(data)
        data = read(cpy);
    end
    if isempty(data)
        error(message('vision:ObjectDetector:noDataFromDatastore'));
    end
end
