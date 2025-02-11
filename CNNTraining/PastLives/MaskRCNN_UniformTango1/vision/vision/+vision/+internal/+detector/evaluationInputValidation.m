function [gtds, classes] = evaluationInputValidation(detectionResults, gTruth, filename, checkScore, supportedBoxFormat, varargin)
% validate inputs for detection evaluation functions

%   Copyright 2016-2020 The MathWorks, Inc.

% Get box validation function based on the supported box formats.
boxValidationFcn = iBoxValidator(supportedBoxFormat, filename);

if istable(gTruth)
    % verify ground truth table
    checkGroundTruthTable(gTruth, height(detectionResults), boxValidationFcn, filename);
    classes = gTruth.Properties.VariableNames;
    
    gtds = boxLabelDatastore(gTruth);
else
    data = vision.internal.detector.checkAndGetEvaluationInputDatastore(gTruth, boxValidationFcn, filename);
    classes = categories(data{1,2});
    gtds = vision.internal.cnn.datastore.InMemoryDatastore(data);
end
classes = string(classes);

if checkScore
    % verify detection result: (bbox, scores, labels)
    checkDetectionResultsTable(detectionResults, classes, boxValidationFcn, filename);
else
    % verify detection results: (bbox, labels)
    checkBoundingBoxTable(detectionResults, classes, boxValidationFcn, filename);
end

% check optional threshold input
if ~isempty(varargin)
    validateattributes(varargin{1}, {'single','double'}, ...
        {'real','scalar','nonsparse','>=',0,'<=',1}, filename, 'threshold');
end
end

%==========================================================================
function checkDetectionResultsTable(detectionResults, classNames, boxValidationFcn, mfilename)

validateattributes(detectionResults, {'table'},{'nonempty'}, mfilename, 'detectionResults');

if width(detectionResults) < 2
    error(message('vision:ObjectDetector:detectionResultsTableWidthLessThanTwo'));
end

ismulcls = (width(detectionResults) > 2);

% When ground truth has more than one class, detection results must include
% labels.
if numel(classNames) > 1 && ~ismulcls
    error(message('vision:ObjectDetector:detectionResultsMustHaveLabels'));
end

if ismulcls
    classNames = categorical(classNames);
    msg = '{';
    for n = 1:numel(classNames)
        msg = [msg char(classNames(n)) ','];
    end
    msg = [msg(1:end-1) '}'];
end

for i = 1:height(detectionResults)
    % check bounding boxes
    try
        iAssertIsScalarCell(detectionResults{i, 1});
        if ~isempty(detectionResults{i, 1}{1})
            
            bbox = detectionResults{i, 1}{1};
            
            boxValidationFcn(bbox);
            
        end
    catch ME
        error(message('vision:ObjectDetector:invalidBboxInDetectionTable', i, ME.message(1:end-1)));
    end
    
    % check scores
    try
        if ~isempty(detectionResults{i, 1}{1})
            iAssertIsScalarCell(detectionResults{i, 2});
            validateattributes(detectionResults{i, 2}{1},{'single','double'},...
                {'vector','real','nonsparse','numel',size(detectionResults{i, 1}{1},1)});
        end
    catch ME
        error(message('vision:ObjectDetector:invalidScoreInDetectionTable', i, ME.message(1:end-1)));
    end
    
    % for multi-class detection, check labels
    if ismulcls
        try
            if ~isempty(detectionResults{i, 1}{1})
                iAssertIsScalarCell(detectionResults{i, 3});
                validateattributes(detectionResults{i, 3}{1},{'categorical'},...
                    {'vector','numel',size(detectionResults{i, 1}{1},1)});
            end
        catch ME
            error(message('vision:ObjectDetector:invalidLabelInDetectionTable', i, ME.message(1:end-1)));
        end
        
        if ~isempty(detectionResults{i, 1}{1})
            labels = categories(detectionResults{i, 3}{1});
            if any(isundefined(detectionResults{i, 3}{1}))||~all(ismember(labels, classNames))
                error(message('vision:ObjectDetector:undefinedLabelInDetectionTable', i, msg));
            end
        end
    end
    
end
end

%==========================================================================
function checkGroundTruthTable(groundTruthData, numExpectedRows, boxValidationFcn, mfilename)

validateattributes(groundTruthData, {'table'}, ...
    {'nonempty','nrows',numExpectedRows}, mfilename, 'groundTruthData');

for n = 1 : width(groundTruthData)
    for i = 1:numExpectedRows
        try
            iAssertIsScalarCell(groundTruthData{i, n});
            if ~isempty(groundTruthData{i, n}{1})
                bbox = groundTruthData{i, n}{1};
                boxValidationFcn(bbox);
            end
        catch ME
            error(message('vision:ObjectDetector:invalidBboxInTrainingDataTable', i, n, ME.message(1:end-1)));
        end
    end
end
end

%==========================================================================
function checkBoundingBoxTable(boundingBoxes, classNames, boxValidationFcn, mfilename)

validateattributes(boundingBoxes, {'table'},{'nonempty'}, mfilename, 'boundingBoxes');

ismulcls = (width(boundingBoxes) >= 2);
if ismulcls
    classNames = categorical(classNames);
    msg = '{';
    for n = 1:numel(classNames)
        msg = [msg char(classNames(n)) ','];
    end
    msg = [msg(1:end-1) '}'];
end

for i = 1:height(boundingBoxes)
    iAssertIsScalarCell(boundingBoxes{i, 1});
    bbox = boundingBoxes{i, 1}{1};
    % check bounding boxes
    try
        if ~isempty(bbox)
            boxValidationFcn(bbox)
        end
    catch ME
        error(message('vision:ObjectDetector:invalidBboxInBboxTable', i, ME.message(1:end-1)));
    end
    
    % for multi-class, check labels
    if ismulcls
        try
            if ~isempty(bbox)
                iAssertIsScalarCell(boundingBoxes{i, 2});
                validateattributes(boundingBoxes{i, 2}{1},{'categorical'},...
                    {'vector','numel',size(bbox,1)});
            end
        catch ME
            error(message('vision:ObjectDetector:invalidLabelInBboxTable', i, ME.message(1:end-1)));
        end
        
        if ~isempty(bbox)
            labels = categories(boundingBoxes{i, 2}{1});
            if any(isundefined(boundingBoxes{i, 2}{1}))||~all(ismember(labels, classNames))
                error(message('vision:ObjectDetector:undefinedLabelInBboxTable', i, msg));
            end
        end
    end
    
end
end

%--------------------------------------------------------------------------
function iAssertIsScalarCell(x)
validateattributes(x, {'cell'}, {'scalar'});
end

%--------------------------------------------------------------------------
function fcn = iBoxValidator(format,filename)
assert(isscalar(format));
assert(any(format == [4 5],'all'));
fcn = @(x)iValidateBox(x,format,filename);
end

%--------------------------------------------------------------------------
function iValidateBox(bbox,rowLength,filename)
if ~isempty(bbox)
    validateattributes(bbox, ...
        {'numeric'},{'real','nonsparse','2d', 'finite', 'size', [NaN, rowLength]},...
        filename);
    if (any(bbox(:,3)<=0) || any(bbox(:,4)<=0))
        error(message('vision:visionlib:invalidBboxHeightWidth'));
    end
end
end
