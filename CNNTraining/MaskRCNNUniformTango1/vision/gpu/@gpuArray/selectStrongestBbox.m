function [selectedBbox, selectedScore, index] = ...
                selectStrongestBbox(bbox, score, varargin)
%selectStrongestBbox Select strongest bounding boxes from overlapping clusters.
%
%  [selectedBbox, selectedScore] = selectStrongestBbox(bbox, score)
%  eliminates bounding boxes using non-maximal suppression (NMS) and
%  returns only the boxes that have high confidence scores. The bounding
%  boxes in bbox can be axis-aligned rectangles or rotated rectangles. bbox
%  is an M-by-4 gpuArray where each row defines one axis-aligned bounding
%  box. The second input, score, is a M-by-1 gpuArray of scores
%  corresponding to the input bounding boxes. The first output,
%  selectedBbox, is an K-by-4 gpuArray containing the selected boxes. The
%  second output, selectedScores, is a K-by-1 vector of scores
%  corresponding to the selected bounding boxes.
%
%  [..., index] = selectStrongestBbox(bbox, score) additionally returns the
%  index vector associated with selectedBbox. This vector contains the
%  indices to the selected boxes in the bbox input.
%   
%  [...] = selectStrongestBbox(... , Name, Value) specifies additional
%  name-value pairs described below:
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
%                       Default: inf
%
%  Class Support 
%  ------------- 
%  bbox and score must be real, finite, and nonsparse. They must both be
%  gpuArray objects with an underlying datatype of uint8, int8, uint16,
%  int16, uint32, int32, single or double. selectedBbox and selectedScore
%  are both gpuArray objects with the same underlying type as the input
%  bbox and score, respectively. selectedIndex is a gpuArray with an
%  underlying type of double.
%
%  Notes 
%  ----- 
%  - Only axis-aligned bounding boxes are supported when the
%    inputs are gpuArray.
% 
%  Example 
%  ------- 
%  % Load a pretrained ACF people detector.
%  peopleDetector = peopleDetectorACF();
%
%  % Detect people in an image. Disable the default non-maximal suppression
%  % used by the detector.
%  I = imread('visionteam1.jpg'); 
%  [bbox, score] = detect(peopleDetector, I, 'SelectStrongest', false); 
%
%  % Move boxes and scores to the GPU.
%  bbox = gpuArray(bbox);
%  score = gpuArray(score);
%
%  % Run non-maximal suppression with custom threshold.
%  [selectedBbox, selectedScore] = selectStrongestBbox(bbox, score, 'OverlapThreshold', 0.3); 
%
%  % Gather the results for display.
%  bbox = gather(bbox);
%  score = gather(score);
%  selectedBbox = gather(selectedBbox);
%  selectedScore = gather(selectedScore);
%
%  % Display results.
%  I1 = insertObjectAnnotation(I, 'rectangle', bbox, score, 'Color', 'r');
%  I2 = insertObjectAnnotation(I, 'rectangle', selectedBbox, selectedScore, 'Color', 'r');
%
%  figure, imshow(I1); title('Detected people and detection scores before suppression'); 
%  figure, imshow(I2); title('Detected people and detection scores after suppression');
%
%  See also bboxOverlapRatio, selectStrongestBboxMulticlass, gpuArray.

%  Copyright 2020-2024 The MathWorks, Inc.

if(nargin > 2)
    % Gather parameters to the host. The @gpuArray overload gets called if
    % any of the inputs are gpuArray.
    [varargin{:}] = gather(varargin{:});
end

% If the data is on the CPU, dispatch to the CPU version. You don't
% want to run on the GPU unless the data is on the GPU.
if ~isa(bbox, 'gpuArray') && ~isa(score, 'gpuArray')    
    [selectedBbox, selectedScore, index] = ...
        selectStrongestBbox(bbox,scores,varargin{:});
    return
end

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
numStrongest = cast(numStrongest, boxType);

% Sort the input bbox according to the score.
[~, ind]  = sort(score, 'descend'); 
inputBbox = inputBbox(ind,:);

selectedIndex = visiongpuBboxOverlapSuppression(...
    inputBbox, overlapThreshold, isDivByUnion, numStrongest);

% Reorder the indices back to the pre-sorted order.
index = gpuArray.false(numel(score),1);
index(ind,:) = selectedIndex;

% Return an index list instead of logical vector.
index = find(index);

% Return selected boxes and scores. 
selectedBbox = bbox(index,:);
selectedScore = score(index);
end

%--------------------------------------------------------------------------
function [ratioType, overlapThreshold, numStrongest] = ...
    iParseInputs(bbox,score,varargin)
% Parse and check inputs

iAssertBoxesAndScoresAreGpuArray(bbox,score);

iAssertBoxesAreAxisAligned(bbox)

vision.internal.detector.selectStrongestValidation.checkInputBboxAndScore(...
    bbox, score, mfilename);

[ratioType, overlapThreshold, numStrongest] = ...
    vision.internal.detector.selectStrongestValidation.parseOptInputs(varargin{:});

if ~isempty(varargin)
    iValidateParams(ratioType, overlapThreshold, numStrongest);
end

numStrongest = double(numStrongest);
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

%--------------------------------------------------------------------------
function iValidateParams(ratioType, overlapThreshold, numStrongest)
vision.internal.detector.selectStrongestValidation.checkOverlapThreshold(overlapThreshold,mfilename)
vision.internal.detector.selectStrongestValidation.checkRatioType(ratioType);
vision.internal.detector.selectStrongestValidation.checkNumStrongestScalar(numStrongest);
end