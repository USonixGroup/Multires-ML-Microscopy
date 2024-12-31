function [selectedBbox, selectedScore, selectedLabel, index] = ...
    selectStrongestBboxMulticlass(bbox, score, label, varargin)

% Copyright 2017-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

nargoutchk(0,4)

[ratioType, overlapThreshold, numStrongest] = iParseInputs(bbox,score,label,varargin{:});

if isempty(bbox)
    selectedBbox = bbox;
    selectedScore = score;
    selectedLabel = label;
    index = [];
    return;
end

if strncmpi(ratioType, 'Union', 1)
    isDivByUnion = true;
else
    isDivByUnion = false;
end

if ~isfloat(bbox)
    inputBbox = single(bbox);
else
    inputBbox = bbox;
end

% Convert labels to numeric values.
inputLabel = iCategoricalLabelsToNumeric(label, class(inputBbox));

% Sort the bbox according to the score.
if isGPUTarget()
    [~, ind] = gpucoder.sort(score, 'descend');
else
    [~, ind] = sort(score, 'descend');
end

% Reorder boxes and labels based on scores.
inputBbox  = inputBbox(ind, :);
inputLabel = inputLabel(ind,:);

isCodegen = ~isempty(coder.target);
switch size(inputBbox,2)
    case 4
        % axis-aligned
        selectedIndex = iOverlapSuppressionAxisAligned(...
            inputBbox, inputLabel, overlapThreshold, isDivByUnion, numStrongest, isCodegen);
    case 5
        % rotated rectangle
        selectedIndex = iOverlapSuppressionRotatedRect(...
            inputBbox, inputLabel, overlapThreshold, isDivByUnion, numStrongest, isCodegen);
    otherwise
        % This code path is required for codegen support when all
        % dimensions of bbox are variable size. Runtime checks exist to
        % ensure the size of bbox is 4 or 5.
        selectedIndex = [];

end

% Reorder the indices back to the pre-sorted order.
index = coder.nullcopy(selectedIndex);
index(ind) = selectedIndex;

selectedBbox = bbox(index, :);
selectedScore = score(index);
selectedLabel = label(index);

% Return an index list instead of logical vector.
index = find(index);

end

%--------------------------------------------------------------------------
function selectedIndex = iOverlapSuppressionAxisAligned(...
    bbox, label, threshold, isDivByUnion, numStrongest, isCodegen)

if isCodegen
    isRotatedRectangles = false;
    if isGPUTarget()
        selectedIndex = selectStrongestBboxCodegen(bbox, ...
            threshold, isDivByUnion, numStrongest, isRotatedRectangles, label);
    else
        selectedIndex = vision.internal.detector.selectStrongestBboxCodegen(...
            bbox, threshold, isDivByUnion, numStrongest, isRotatedRectangles, label);
    end
else
    selectedIndex = visionBboxOverlapSuppression(bbox, ...
        threshold, isDivByUnion, numStrongest, label);
end
end

%--------------------------------------------------------------------------
function selectedIndex = iOverlapSuppressionRotatedRect(...
    bbox, label, threshold, isDivByUnion, numStrongest, isCodegen)

if ~isCodegen
    [x,y] = vision.internal.bbox.bbox2poly(bbox);

    selectedIndex = visionRotatedBBoxOverlapSuppression(...
        x, y, label, threshold, isDivByUnion, numStrongest);
else
    isRotatedRectangles = true;
    if isGPUTarget()
        selectedIndex = selectStrongestBboxCodegen(bbox, ...
            threshold, isDivByUnion, numStrongest, isRotatedRectangles, label);
    else
        selectedIndex = vision.internal.detector.selectStrongestBboxCodegen(...
            bbox, threshold, isDivByUnion, numStrongest, isRotatedRectangles, label);
    end
end
end
%--------------------------------------------------------------------------
function selectedIndex = selectStrongestBboxCodegen(bbox, ...
    threshold, isDivByUnion, numStrongest, isRotatedRectangles, label)
% Wrapper function for vision.internal.detector.selectStrongestBboxCodegen
% to prevent the kernel creation inside the function.
coder.inline('never')
selectedIndex = vision.internal.detector.selectStrongestBboxCodegen(...
    bbox, threshold, isDivByUnion, numStrongest, isRotatedRectangles, label);
end
%--------------------------------------------------------------------------
function iCheckInputBboxScoreAndLabel(bbox,score,label)
vision.internal.detector.selectStrongestValidation.checkInputBboxAndScore(bbox, score, mfilename);
iCheckLabel(label)

coder.internal.errorIf(~isempty(label) && (numel(label) ~= size(bbox,1)) ,...
    'vision:visionlib:unmatchedBboxAndLabel');
end

%--------------------------------------------------------------------------
function iCheckLabel(value)

if isa(value,'categorical')
    validateattributes(value,{'categorical'}, {'size',[NaN, 1]}, ...
        mfilename, 'label', 3);
else
    validateattributes(value,{'uint8', 'int8', 'uint16', 'int16', 'uint32', ...
        'int32', 'double', 'single'}, {'real','nonsparse','finite','integer','size',[NaN, 1]}, ...
        mfilename, 'label', 3);
end
end

%--------------------------------------------------------------------------
function labels = iCategoricalLabelsToNumeric(labels,class)
labels = cast(labels,class);
end

%--------------------------------------------------------------------------
function [ratioType, overlapThreshold, numStrongest] = ...
    iParseInputs(bbox,score,label,varargin)
% Parse and check inputs
iCheckInputBboxScoreAndLabel(bbox, score, label);
if isempty(coder.target)
    [ratioType, overlapThreshold, numStrongest] = ...
        vision.internal.detector.selectStrongestValidation.parseOptInputs(varargin{:});
else
    [ratioType, overlapThreshold, numStrongest] = ...
        vision.internal.detector.selectStrongestValidation.parseOptInputsCodegen(varargin{:});
end

if ~isempty(varargin)
    iValidateParams(ratioType, overlapThreshold, numStrongest);
end

isVector = numel(numStrongest) > 1;
isCategoricalLabels = isa(label,'categorical');

% When numStrongest is a vector, label must be a categorical.
coder.internal.errorIf(isVector && ~isCategoricalLabels,...
    'vision:selectStrongestBbox:mustBeCategoricalForNumStrongestVector');

% Issue a compile time error if labels do not have constant categories.
% This happens if labels are created without providing a value set when
% creating the categorical values: categorical(data).
if isCategoricalLabels && ~isempty(coder.target)
    cats = categories(label);
    eml_invariant(eml_is_const(numel(cats)), ...
        eml_message('vision:selectStrongestBbox:categoriesMustBeConst'));
end

% When numStrongest is a vector, the length of the vector must equal the
% number of categories in label.
coder.internal.errorIf(isVector && numel(categories(label)) ~= numel(numStrongest),...
    'vision:selectStrongestBbox:numelNumStrongestMustEqualNumCategories');

numStrongest = double(numStrongest);
end

%--------------------------------------------------------------------------
function iValidateParams(ratioType, overlapThreshold, numStrongest)
vision.internal.detector.selectStrongestValidation.checkOverlapThreshold(overlapThreshold,mfilename)
vision.internal.detector.selectStrongestValidation.checkRatioType(ratioType);
vision.internal.detector.selectStrongestValidation.checkNumStrongestScalarOrVector(numStrongest);
end

%--------------------------------------------------------------------------
function status = isGPUTarget()
status = coder.gpu.internal.isGpuEnabled;
end