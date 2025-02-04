function [anchorBoxes, meanIoU] = estimateAnchorBoxes(datastore, numAnchors, varargin)

    % Copyright 2019-2023 The MathWorks, Inc.

    [boxes, numAnchors, params] = iParseInputs(datastore, numAnchors, varargin{:});

    widthHeight = iPrefixXYCoordinates(boxes(:,3:4));

    [anchorBoxes, clusterAssignments, dists] = ...
        vision.internal.approximateKMeans(widthHeight,numAnchors,...
        'DistanceMethod','bboxIoU','UseParallel',params.UseParallel);
    % Round anchor boxes to integer-values.
    % Return anchor boxes in [height,width] format
    anchorBoxes = round(anchorBoxes(:, [4,3]));

    if nargout == 2
        sumd    = splitapply(@sum,dists,clusterAssignments);
        counts  = splitapply(@numel,clusterAssignments,clusterAssignments);
        % protect against division by zero.
        counts  = counts + eps(class(counts));
        meanIoU = mean(1 - sumd./(counts));
    end
end

%----------------------------------------------------------------------------------
function [boxes, numAnchors, userInput] = iParseInputs(datastore, numAnchors, varargin)
    p = inputParser;
    p.addParameter('UseParallel', vision.internal.useParallelPreference());
    p.addParameter('BboxColumn', 1);
    p.parse(varargin{:});

    userInput = p.Results;
    userInput.UsingDefaults = p.UsingDefaults;

    vision.internal.inputValidation.validateLogical(userInput.UseParallel,'UseParallel');

    boxes = iCheckBoxesFromDatastore(datastore);

    % Round fractional ground truth boxes to nearest pixel centers. If the
    % bounding boxes are rotated rectangles, use their width and height to
    % create axis-aligned bounding boxes for estimating the anchor boxes.
    if size(boxes,2) == 4
        % The boxes are axis-aligned.
        boxes = vision.internal.cnn.utils.roundGroundTruthBoxes(boxes);
    else
        % The boxes are rotated rectangle bounding boxes.
        xymax = round(boxes(:,1:2) + boxes(:,3:4)/2 - 1);
        xymin = round(boxes(:,1:2) - boxes(:,3:4)/2 + 1);
        boxes = [xymin xymax-xymin+1];
    end

    numAnchors = iValidateNumAnchors(numAnchors, intmax('uint32'));
end

%----------------------------------------------------------------------------------
function numeric = iValidateNumericValue(numeric, maxValue, varargin)
    classes  = {'numeric'};
    attrs    = {'scalar', 'nonempty', 'nonsparse',...
        'nonzero',...
        'nonnegative',... % this covers non complex
        '<=', maxValue};
    funcName = mfilename;

    validateattributes(numeric, classes, attrs, funcName, varargin{:});

    numeric = double(numeric);
end

%----------------------------------------------------------------------------------
function numAnchors = iValidateNumAnchors(numAnchors, maxValue)
    try
        numAnchors = iValidateNumericValue(numAnchors, maxValue);
    catch
        error(message('vision:estimateAnchorBoxes:invalidNumAnchorsValue'));
    end
end

%----------------------------------------------------------------------------------
function boxes = iCheckBoxesFromDatastore(datastore)

    if isa(datastore, 'boxLabelDatastore')
        boxes = datastore.LabelData(:,1);
        boxes = vertcat(boxes{:});
        return;
    end
    try
        validateattributes(datastore, ...
            {'matlab.io.Datastore', 'matlab.io.datastore.Datastore'},...
            {'nonempty'}, mfilename);
    catch
        error(message('vision:estimateAnchorBoxes:invalidDatastoreInput'));
    end
    ds = transform(datastore,@iTransformForBboxColumn);
    boxes = iReadallBoxes(ds);
    boxes = iCheckBoxes(boxes);
end

%----------------------------------------------------------------------------------
function boxes = iCheckBoxes(boxes)
    boxes = vertcat(boxes{:});

    classes        = {'numeric'};

    % Allow bboxes to be either M-by-4 for axis-aligned bounding boxes or
    % M-by-5 for rotated rectangle bounding boxes.
    if size(boxes,2) == 4
        numCols = 4;
    else
        numCols = 5;
    end

    attrs          = {'nonempty', 'nonsparse', '2d', 'ncols', numCols};

    try
        validateattributes(boxes, classes, attrs, mfilename);
    catch me
        error(message('vision:estimateAnchorBoxes:readOutputInvalidBbox'));
    end
    boxes = double(boxes);
end

%----------------------------------------------------------------------------------
function boxes = iPrefixXYCoordinates(boxes)
    n = size(boxes,1);
    xy = ones(n,2,'like',boxes);
    boxes = [xy, boxes(:,end-1:end)];
end

%----------------------------------------------------------------------------------
function data = iTransformForBboxColumn(data)
    classes = {'cell', 'table'};
    attrs   = {'ndims', 2};
    try
        validateattributes(data, classes, attrs, mfilename);
    catch me
        error(message('vision:estimateAnchorBoxes:readOutputNotCellTable'));
    end

    nCols = size(data, 2);
    switch nCols
        case 3
            bboxColumn = 2;
        case {1,2}
            bboxColumn = 1;
        otherwise
            error(message('vision:estimateAnchorBoxes:readOutputNotCellTable'));
    end

    if istable(data)
        data = data{:,bboxColumn};
    else
        data = data(:,bboxColumn);
    end
end

%----------------------------------------------------------------------------------
function boxes = iReadallBoxes(ds)
    try
        boxes = readall(ds);
    catch e
        badTransformId         = 'MATLAB:datastoreio:transformeddatastore:badTransformDef';
        readOutputNotCellTable = 'vision:estimateAnchorBoxes:readOutputNotCellTable';

        if isequal(e.identifier, badTransformId) && ~isempty(e.cause) && ...
                isequal(e.cause{1}.identifier, readOutputNotCellTable)
            error(message('vision:estimateAnchorBoxes:readOutputNotCellTable'));
        else
            throwAsCaller(e);
        end
    end
end
