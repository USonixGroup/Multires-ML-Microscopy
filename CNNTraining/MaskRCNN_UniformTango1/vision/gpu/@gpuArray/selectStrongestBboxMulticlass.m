function [selectedBbox, selectedScore, selectedLabel, index] = ...
    selectStrongestBboxMulticlass(bbox, score, label, varargin)
%selectStrongestBboxMulticlass Select strongest multiclass bounding boxes from overlapping clusters.
%   selectedBboxes = selectStrongestBboxMulticlass(bboxes,scores,labels)
%   returns selected bounding boxes that have a high confidence score. The
%   function uses greedy non-maximal suppression (NMS) to eliminate
%   overlapping bounding boxes only if they have the same class label. The
%   selected boxes are returned in selectedBboxes.
%
%   Inputs
%   ------
%   bboxes - an M-by-4 gpuArray representing axis-aligned. Each row in
%            bboxes defines one bounding box.
%
%   scores - an M-by-1 gpuArray of scores corresponding to the input
%            bounding boxes.
%
%   labels - is an M-by-1 vector of categorical or numeric labels
%            corresponding to the input bounding boxes. When the labels are
%            numeric, they can be contained in a gpuArray.
%
%   [..., selectedScores, selectedLabels, index] = selectStrongestBboxMulticlass(...)
%   additionally returns the scores, labels, and index associated with the
%   selected bounding boxes.
%
%   [...] = selectStrongestBboxMulticlass(..., Name, Value) specifies
%   additional name-value pairs described below:
%
%   'RatioType'         A string, 'Union' or 'Min', specifying the
%                       denominator of bounding box overlap ratio.
%                       See bboxOverlapRatio for detailed explanation of
%                       the ratio definition.
%
%                       Default: 'Union'
%
%   'OverlapThreshold'  A scalar from 0 to 1. All bounding boxes around a
%                       reference box are removed if their overlap ratio
%                       is above this threshold.
%
%                       Default: 0.5
%
%   'NumStrongest'      Specify the maximum number of strongest boxes to
%                       select as a positive scalar or inf. Use this
%                       parameter to reduce processing time when you have a
%                       priori knowledge about the maximum number of boxes.
%                       Set the value to inf to select all the strongest,
%                       non-overlapping, bounding boxes.
%
%                       When the labels input is a categorical, you can
%                       also specify a vector that contains the maximum
%                       number of strongest boxes for each category in the
%                       labels input. The length of the specified vector
%                       must equal the number of categories in labels.
%
%                       Default: inf
%
%  Class Support
%  -------------
%  bbox and score must be real, finite, and nonsparse. They must both be
%  gpuArray objects with an underlying datatype of uint8, int8, uint16,
%  int16, uint32, int32, single or double. In addition, the class of labels
%  can be categorical, numeric, or gpuArray. selectedBbox and selectedScore
%  are both gpuArray objects with the same underlying type as the input
%  bbox and score, respectively. The class of selectedLabels is the same as
%  the input labels. selectedIndex is a gpuArray with an underlying type of
%  double.
%
%  Notes
%  -----
%  - Only axis-aligned bounding boxes are supported when the
%    inputs are gpuArray.
%
%  Example
%  -------
%  % Create detectors using two different models. These will be used to
%  % generate multiclass detection results.
%  detectorInria = peopleDetectorACF('inria-100x41' );
%  detectorCaltech = peopleDetectorACF('caltech-50x21');
%
%  % Apply the detectors.
%  I = imread('visionteam1.jpg');
%  [bboxesInria, scoresInria] = detect(detectorInria,I,'SelectStrongest',false);
%  [bboxesCaltech, scoresCaltech] = detect(detectorCaltech,I,'SelectStrongest',false);
%
%  % Create categorical labels for each the result of each detector.
%  labelsInria = repelem("inria",numel(scoresInria),1);
%  labelsInria = categorical(labelsInria,{'inria','caltech'});
%  labelsCaltech = repelem("caltech",numel(scoresCaltech),1);
%  labelsCaltech = categorical(labelsCaltech,{'inria','caltech'});
%
%  % Combine results from all detectors to for multiclass detection results.
%  allBBoxes = [bboxesInria;bboxesCaltech];
%  allScores = [scoresInria;scoresCaltech];
%  allLabels = [labelsInria;labelsCaltech];
%
%  % Move boxes and scores to the GPU.
%  allBBoxes = gpuArray(allBBoxes);
%  allScores = gpuArray(allScores);
%
%  % Run multiclass non-maximal suppression
%  [bboxes, scores, labels] = selectStrongestBboxMulticlass(...
%       allBBoxes,allScores,allLabels,...
%       'RatioType','Min','OverlapThreshold',0.65);
%
%  % Gather the results for display.
%  bboxes = gather(bboxes);
%  scores = gather(scores);
%
%  % Annotate detected people
%  annotations = string(labels) + ": " + string(scores);
%  I = insertObjectAnnotation(I, 'rectangle', bboxes, annotations);
%  figure
%  imshow(I)
%  title('Detected people, scores, and labels')
%
%  See also bboxOverlapRatio, selectStrongestBbox, gpuArray.

%  Copyright 2020-2024 The MathWorks, Inc.

if(nargin > 3)
    % Gather parameters to the host. The @gpuArray overload gets called if
    % any of the inputs are gpuArray.
    [varargin{:}] = gather(varargin{:});
end

% If the data is on the CPU, dispatch to the CPU version. You don't
% want to run on the GPU unless the data is on the GPU.
% xxx what about non gpu labels
if ~isa(bbox, 'gpuArray') && ~isa(score, 'gpuArray')
    [selectedBbox, selectedScore, index] = ...
        selectStrongestBboxMulticlass(bbox,score,label,varargin{:});
    return
end

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

% Process data as double or single. Integer datatypes are processed as
% single.
if ~isfloat(bbox)
    inputBbox = single(bbox);
else
    inputBbox = bbox;
end

% Make parameter types match underlying box type.
boxType = classUnderlying(inputBbox);
overlapThreshold = cast(overlapThreshold, boxType);

if iscategorical(label)
    classes = categories(label);
    numClasses = numel(classes);
    
    % Convert classes to class IDs for faster comparisons.
    classes = 1:numel(classes);
else
    classes = unique(label);
    numClasses = numel(classes);
end

% Cast to single. Undefined categorical labels will be NaN and will not be
% selected.
inputLabel = single(label);

% Determine whether we are choosing the top-k across all boxes or the top-k
% per class.
chooseTopK = isscalar(numStrongest) && isfinite(numStrongest);
if isscalar(numStrongest)
    numStrongest = repelem(numStrongest,numel(classes));
end

% Sort the input bbox according to the score.
[~, ord]  = sort(score, 'descend');
inputBbox = inputBbox(ord,:);
inputLabel = inputLabel(ord,:);

boxGroups = cell(numClasses,1);
validGroups = true(numClasses,1);
numPerGroup = zeros(numClasses,1);

groupIndices = inputLabel == reshape(classes,1,[]);

for i = 1:numel(classes)
    boxIndices = groupIndices(:,i);
    boxGroups{i} = inputBbox(boxIndices,:);
    numPerGroup(i) = size(boxGroups{i},1);
end

% Remove box groups that have less than 1 box.
validGroups(numPerGroup < 1) = false;
boxGroups = boxGroups(validGroups);
numStrongest = numStrongest(validGroups);
numPerGroup = numPerGroup(validGroups);

% Run algorithm per group.
idxCell = visiongpuBboxOverlapSuppressionByGroup(...
    boxGroups, overlapThreshold, isDivByUnion, numStrongest, numPerGroup);

idxCellValues = vertcat(idxCell{:});
groupIndices(groupIndices) = idxCellValues;

index = gpuArray(any(groupIndices,2));

if chooseTopK
    % Find the top-k boxes in the sorted index vector. These correspond
    % to the top-k boxes across all classes.
    index = find(index,numStrongest(1));
    
    % Sort the indices to return selected items in the original input
    % order.
    index = sort(ord(index,:),'ascend');
    
else
    % Reorder the indices back to the pre-sorted order.
    index(ord,:) = index;
    
    % Return an index list instead of logical vector.
    index = find(index);
end

% Return selected boxes and scores.
selectedBbox = bbox(index,:);
selectedScore = score(index);
selectedLabel = label(index);

end

%--------------------------------------------------------------------------
function iCheckInputBboxScoreAndLabel(bbox,score,label)

iAssertBoxesAndScoresAreGpuArray(bbox,score);

iAssertBoxesAreAxisAligned(bbox)

vision.internal.detector.selectStrongestValidation.checkInputBboxAndScore(bbox, score, mfilename);

iCheckLabel(label)

if ~isempty(label) && (numel(label) ~= size(bbox,1))
    error(message('vision:visionlib:unmatchedBboxAndLabel'))
end

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
function [ratioType, overlapThreshold, numStrongest] = ...
    iParseInputs(bbox,score,label,varargin)
% Parse and check inputs
iCheckInputBboxScoreAndLabel(bbox, score, label);

[ratioType, overlapThreshold, numStrongest] = ...
    vision.internal.detector.selectStrongestValidation.parseOptInputs(varargin{:});

if ~isempty(varargin)
    iValidateParams(ratioType, overlapThreshold, numStrongest);
end

isVector = numel(numStrongest) > 1;
isCategoricalLabels = isa(label,'categorical');

% When numStrongest is a vector, label must be a categorical.
if isVector && ~isCategoricalLabels
    error(message('vision:selectStrongestBbox:mustBeCategoricalForNumStrongestVector'))
end

% When numStrongest is a vector, the length of the vector must equal the
% number of categories in label.
if isVector && numel(categories(label)) ~= numel(numStrongest)
    error(message('vision:selectStrongestBbox:numelNumStrongestMustEqualNumCategories'))
end

numStrongest = double(numStrongest);
end

%--------------------------------------------------------------------------
function iValidateParams(ratioType, overlapThreshold, numStrongest)
vision.internal.detector.selectStrongestValidation.checkOverlapThreshold(overlapThreshold,mfilename)
vision.internal.detector.selectStrongestValidation.checkRatioType(ratioType);
vision.internal.detector.selectStrongestValidation.checkNumStrongestScalarOrVector(numStrongest);
end

%--------------------------------------------------------------------------
function iAssertBoxesAreAxisAligned(bbox)
if ~(size(bbox,2) == 4)
    error(message('vision:selectStrongestBbox:mustAxisAlignedGPU'));
end
end

%--------------------------------------------------------------------------
function iAssertBoxesAndScoresAreGpuArray(bbox,score)
if ~(isgpuarray(bbox) && isgpuarray(score))
    error(message('vision:selectStrongestBbox:boxAndScoreMustBeGpuArray'));
end
end