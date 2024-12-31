function [selectedBbox, selectedScore, index] = ...
    selectStrongestBbox(bbox, score, varargin)

% Copyright 2013-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>
coder.gpu.kernelfun;
isCodegen = ~isempty(coder.target);

% Parse and check inputs
[ratioType, overlapThreshold, numStrongest] = iParseInputs(bbox,score,varargin{:});

if isempty(bbox)
    selectedBbox = bbox;
    selectedScore = score;
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

% sort the bbox according to the score
if isGPUTarget()
    [~, ind] = gpucoder.sort(score, 'descend');
else
    [~, ind] = sort(score, 'descend');
end

inputBbox = inputBbox(ind, :);

switch size(inputBbox,2)
    case 4
        % axis-aligned
        selectedIndex = iOverlapSuppressionAxisAligned(...
            inputBbox, overlapThreshold, isDivByUnion, numStrongest, isCodegen);
    case 5
        % rotated rectangle
        selectedIndex = iOverlapSuppressionRotatedRect(...
            inputBbox, overlapThreshold, isDivByUnion, numStrongest, isCodegen);
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

% Return an index list instead of logical vector.
index = find(index);
end

%--------------------------------------------------------------------------
function selectedIndex = iOverlapSuppressionAxisAligned(...
    bbox, threshold, isDivByUnion, numStrongest, isCodegen)

if isCodegen
    isRotatedRectangles = false;
    if isGPUTarget()
        selectedIndex = selectStrongestBboxCodegen(bbox, ...
            threshold, isDivByUnion, numStrongest, isRotatedRectangles);
    else
        selectedIndex = vision.internal.detector.selectStrongestBboxCodegen(...
            bbox, threshold, isDivByUnion, numStrongest, isRotatedRectangles);
    end

else
    unusedLabelInput = []; % disable label based processing.
    selectedIndex = visionBboxOverlapSuppression(...
        bbox, threshold, isDivByUnion, numStrongest, unusedLabelInput);
end
end

%--------------------------------------------------------------------------
function selectedIndex = iOverlapSuppressionRotatedRect(...
    bbox, threshold, isDivByUnion, numStrongest, isCodegen)

if ~isCodegen
    [x,y] = vision.internal.bbox.bbox2poly(bbox);

    unusedLabelInput = []; % disable label based processing.

    selectedIndex = visionRotatedBBoxOverlapSuppression(...
        x, y, unusedLabelInput, threshold, isDivByUnion, numStrongest);
else
    isRotatedRectangles = true;
    if isGPUTarget()
        selectedIndex = selectStrongestBboxCodegen(bbox, ...
            threshold, isDivByUnion, numStrongest, isRotatedRectangles);
    else
        selectedIndex = vision.internal.detector.selectStrongestBboxCodegen(...
            bbox, threshold, isDivByUnion, numStrongest, isRotatedRectangles);
    end
end
end

%--------------------------------------------------------------------------
function selectedIndex = selectStrongestBboxCodegen(bbox, ...
    threshold, isDivByUnion, numStrongest, isRotatedRectangles)
% Wrapper function for vision.internal.detector.selectStrongestBboxCodegen
% to prevent the kernel creation inside the function.
coder.inline('never')
selectedIndex = vision.internal.detector.selectStrongestBboxCodegen(...
    bbox, threshold, isDivByUnion, numStrongest, isRotatedRectangles);
end

%--------------------------------------------------------------------------
function [ratioType, overlapThreshold, numStrongest] = ...
    iParseInputs(bbox,score,varargin)
% Parse and check inputs

vision.internal.detector.selectStrongestValidation.checkInputBboxAndScore(...
    bbox, score, mfilename);

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

numStrongest = double(numStrongest);

end

%--------------------------------------------------------------------------
function iValidateParams(ratioType, overlapThreshold, numStrongest)
vision.internal.detector.selectStrongestValidation.checkOverlapThreshold(overlapThreshold,mfilename)
vision.internal.detector.selectStrongestValidation.checkRatioType(ratioType);
vision.internal.detector.selectStrongestValidation.checkNumStrongestScalar(numStrongest);
end

%--------------------------------------------------------------------------
function status = isGPUTarget()
status = coder.gpu.internal.isGpuEnabled;
end
