function data = checkAndGetEvaluationInputDatastore(gtds, boxValidationFcn, filename)
%CHECKANDGETEVALUATIONINPUTDATASTORE Verify that the input datastore for
% evaluation is infact a datastore and its read method returns
% the data in M-by-3 or M-by-2 table or cell. Boxes and labels
% in the first and second column, when M-by-2. Boxes and labels
% in the second and third column, when M-by-3.
%
% Returns validated ground truth data in a cell array (M-by-2).

% Copyright 2019-2020 The MathWorks, Inc.

    try
        validateattributes(gtds, {'matlab.io.Datastore', 'matlab.io.datastore.Datastore'},...
            {'nonempty'}, filename);
    catch me
        error(message('vision:ObjectDetector:invalidEvaluationDatastoreInput'));
    end

    ds = transform(gtds,@(x)iTransformForBoxesLabels(x,filename));
    data = iReadallBoxesLabels(ds);

    mSize = iCheckBoxes(data(:,1), boxValidationFcn);
    iCheckLabels(data(:,2), mSize, filename);
end


%----------------------------------------------------------------------------------
function mSize = iCheckBoxes(boxes, boxValidationFcn)
    mSize = zeros(size(boxes,1), 1);
    for ii = 1:size(boxes, 1)
        mSize(ii) = iCheckEachRowBoxes(boxes(ii), boxValidationFcn);
    end
end

%----------------------------------------------------------------------------------
function mSize = iCheckEachRowBoxes(boxes, boxValidationFcn)
    if ~iscell(boxes)
        error(message('vision:ObjectDetector:evaluationReadOutputInvalidBboxNotInCell'));
    end
    boxes = boxes{1};
    try
        boxValidationFcn(boxes);
    catch me
        error(message('vision:ObjectDetector:evaluationReadOutputInvalidBbox', me.message(1:end-1)));
    end
    if isempty(boxes)
        mSize = 0;
    else
        mSize = size(boxes,1);
    end
end

%----------------------------------------------------------------------------------
function iCheckLabels(labels, mSize, filename)
    if numel(mSize) ~= size(labels, 1)
        error(message('vision:ObjectDetector:evaluationReadOutputInvalidLabel'));
    end
    for ii = 1:size(labels, 1)
        iCheckEachRowLabels(labels(ii), mSize(ii), filename);
    end
end

%----------------------------------------------------------------------------------
function iCheckEachRowLabels(labels, mSize, filename)
    if ~iscell(labels)
        error(message('vision:ObjectDetector:evaluationReadOutputInvalidLabel'));
    end
    labels         = labels{1};
    classes        = {'categorical'};
    attrs          = {'nonsparse', '2d', 'ncols', 1, 'nrows', mSize};
    try
        validateattributes(labels, classes, attrs, filename);
    catch me
        error(message('vision:ObjectDetector:evaluationReadOutputInvalidLabel'));
    end
end

%----------------------------------------------------------------------------------
function data = iTransformForBoxesLabels(data,filename)
    classes = {'cell', 'table'};
    attrs   = {'ndims', 2, 'nonempty'};
    try
        validateattributes(data, classes, attrs, filename);
    catch me
        error(message('vision:ObjectDetector:evaluationReadOutputInvalid'));
    end

    switch size(data, 2)
        case 3
            cols = [2,3];
        case 2
            cols = [1,2];
        otherwise
            error(message('vision:ObjectDetector:evaluationReadOutputInvalid'));
    end
    if istable(data)
        data = data{:,cols};
    else
        data = data(:,cols);
    end
end

%----------------------------------------------------------------------------------
function data = iReadallBoxesLabels(ds)
    try
        data = readall(ds);
    catch e
        badTransformId    = 'MATLAB:datastoreio:transformeddatastore:badTransformDef';
        objectDetectorMsg = 'vision:ObjectDetector:';

        if isequal(e.identifier, badTransformId) && ~isempty(e.cause) && ...
                any(contains(e.cause{1}.identifier, objectDetectorMsg))
            throwAsCaller(e.cause{1});
        else
            throwAsCaller(e);
        end
    end
end
