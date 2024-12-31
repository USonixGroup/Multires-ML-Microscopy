function classNames = checkGroundTruthDatastore(trainingDs, options )
%CHECKGROUNDTRUTHDATASTORE Verify that the input datastore for
% training is infact a datastore and its read method returns
% the data in Mx3 column table or cell, with images in the first
% column, boxes in the second and the labels in the third.
%
% Optionally specify an options struct to control the following aspects of
% the datastore output usin the following struct fields:
%
%    * CheckLabels      Specify 'auto' or 'always'. When 'always', labels
%                       are always checked. Otherwise, they are only checked
%                       if present.
%
%    * AllowEmptyBoxes  Specify a logical. When true, empty boxes are
%                       permitted. 

% Copyright 2019-2024 The MathWorks, Inc.

% Fill in any missing options that may have been excluded
if nargin == 1
    options.CheckLabels = 'always';
    options.AllowEmptyBoxes = false;
    options.AllowRotatedBoxes = false;
end

options = populateOptions(options);

funcName       = mfilename;
varDescription = 'Training datastore input';
varIndex       = 1;

try
    validateattributes(trainingDs, {'matlab.io.Datastore', 'matlab.io.datastore.Datastore'},...
        {'nonempty'}, funcName, varDescription, varIndex);
catch me
    msg = message('vision:ObjectDetector:invalidTrainingInput');
    throw(addCause(MException(msg), me));
end

cpy = copy(trainingDs);
reset(cpy);
sampleData = read(cpy);

classes = {'cell', 'table'};
attrs   = {'ndims', 2};
varDescription = 'Read output of training datastore input';

try
    validateattributes(sampleData, classes, attrs, funcName, varDescription, varIndex);
catch me
    msg = message('vision:ObjectDetector:readOutputNotCellTable');
    throw(addCause(MException(msg), me));
end

% Check whether data has enough columns for labels.
alwaysCheckLabels = strcmp(options.CheckLabels,'always');
if alwaysCheckLabels
    ncols = size(sampleData,2);
    if ~any(ncols == 3)
        error( message('vision:ObjectDetector:readOutputNotCellTable'));
    end
    hasLabels = true;
else
    ncols = size(sampleData,2);
    if ~any(ncols == [2 3])
        error( message('vision:ObjectDetector:readOutputNotCellTwoOrThreeColTable'));
    end
    hasLabels = size(sampleData,2) == 3;
end

if options.IsYOLO
    if istable(sampleData)
        [nrows, ~] = cellfun(@size,sampleData{:,2},UniformOutput=false);
        while ~any(cell2mat(nrows))
            try
                sampleData = read(cpy);
                [nrows, ~] = cellfun(@size,sampleData{:,2},UniformOutput=false);
            catch
                error(message('vision:ObjectDetector:trainingDataHasNoBoxes'));
            end
        end
        idx = find(cell2mat(nrows)>0,1,'first');
        images = sampleData{idx,1};
        boxes  = sampleData{idx,2};
    else
        % Find the number of rows that have ground truth.
        [nrows, ~] = cellfun(@size,sampleData(:,2),UniformOutput=false);

        % If none of the rows have ground truth, read next batch of data.
        while ~any(cell2mat(nrows))
            try
                sampleData = read(cpy);
                [nrows, ~] = cellfun(@size,sampleData(:,2),UniformOutput=false);
            catch
                % Error out when none of the images have ground truth.
                error(message('vision:ObjectDetector:trainingDataHasNoBoxes'));
            end
        end

        % Find the first index with non-zero ground truth.
        idx = find(cell2mat(nrows)>0,1,'first');

        % Extract the images, box information from the sample data.
        images = sampleData(idx,1);
        boxes  = sampleData(idx,2);
    end
else
    idx = 1;
    if istable(sampleData)
        images = sampleData{idx,1};
        boxes  = sampleData{idx,2};
    else
        images = sampleData(idx,1);
        boxes  = sampleData(idx,2);
    end

end

iCheckImages(images);
mSize = iCheckBoxes(boxes, options.AllowEmptyBoxes, options.AllowRotatedBoxes);

if alwaysCheckLabels || hasLabels
    if istable(sampleData)
        labels = sampleData{idx,3};
    else
        labels = sampleData(idx,3);
    end
    if mSize > 0
        % Check labels if we have boxes.
        classNames = iCheckLabels(labels, mSize);
    else
        classNames = '';
    end
else
    classNames = '';
end
end

%--------------------------------------------------------------------------
function iCheckImages(images)
if ~iscell(images)
    error(message('vision:ObjectDetector:readOutputInvalidImage'));
end

I = images{1};

if ismatrix(I) 
    nDims = 2;
else
    nDims = 3;
end

classes        = {'numeric'};
attrs          = {'nonempty', 'nonsparse', 'ndims', nDims};
funcName       = mfilename;
varDescription = 'Images in read output of training datastore input';
varIndex       = 1;

try
    validateattributes(I, classes, attrs, funcName, varDescription, varIndex);
catch me
    msg = message('vision:ObjectDetector:readOutputInvalidImage');
    throw(addCause(MException(msg), me));
end

end

%--------------------------------------------------------------------------
function mSize = iCheckBoxes(boxes, allowEmpties, allowRotatedBoxes)
if ~iscell(boxes)
    error(message('vision:ObjectDetector:readOutputInvalidBbox'));
end

% Define bounding box validation parameters.
boxes          = boxes{1};
classes        = {'numeric'};
attrs          = {'nonempty', 'nonsparse', '2d', 'ncols', 4};

funcName       = mfilename;
varDescription = 'Boxes in read output of training datastore input';
varIndex       = 2;
try
    % Skip empty bounding boxes when allowed
    if allowEmpties && isempty(boxes) 
        % Skip check. 
    else
        % For detectors that support rotated rectangle bounding boxes, also 
        % verify that the angle is valid
        if allowRotatedBoxes && size(boxes,2) == 5
            attrsYaw = {'nonempty', 'nonnan', 'finite', 'nonsparse'};
            validateattributes(boxes(:,1:4), classes, attrs);
            validateattributes(boxes(:,5), classes, attrsYaw);
        else
            validateattributes(boxes, classes, attrs, funcName, varDescription, varIndex);
        end
    end
catch me
    msg = message('vision:ObjectDetector:readOutputInvalidBbox');
    throw(addCause(MException(msg), me));
end
mSize = size(boxes,1);
end

%--------------------------------------------------------------------------
function classes = iCheckLabels(labels, mSize)
if ~iscell(labels)
    error(message('vision:ObjectDetector:readOutputInvalidLabel'));
end
labels         = labels{1};
classes        = {'categorical'};
attrs          = {'nonempty', 'nonsparse', '2d', 'ncols', 1, 'nrows', mSize};
funcName       = mfilename;
varDescription = 'Labels in read output of training datastore input';
varIndex       = 3;
try
    validateattributes(labels, classes, attrs, funcName, varDescription, varIndex);
catch me
    msg = message('vision:ObjectDetector:readOutputInvalidLabel');
    throw(addCause(MException(msg), me));
end
classes = categories(labels);

% Make sure classes output is a row vector.
classes = classes(:);
classes = classes';
end

%--------------------------------------------------------------------------
function options = populateOptions(options)
if ~isfield(options,'CheckLabels')
    options.CheckLabels = 'always';
end

if ~isfield(options,'AllowEmptyBoxes')
    options.AllowEmptyBoxes = false;
end

if ~isfield(options,'AllowRotatedBoxes')
    options.AllowRotatedBoxes = false;
end
if ~isfield(options,'IsYOLO')
    options.IsYOLO = false;
end
end
