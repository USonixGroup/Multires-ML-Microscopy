function labelNames = checkGroundTruthLabelDefinitions(gTruth, type, varargin)
% labelNames = checkGroundTruthLabelDefinitions(gTruth, type) verifies that
% the input array of groundTruth objects, gTruth, contains consistent label
% definitions of type, type and returns the label definition name in
% labelNames. 
% 
% labelNames = checkGroundTruthLabelDefinitions(gTruth, type, originalIndices) 
% optionally accepts the original indices of the groundTruth objects in gTruth 
% if they were filtered out in a previous validation function like
% vision.internal.trainingData.checkGroundTruthSources. originalIndices is
% used to inform the user about the inconsistent ground truth location in
% the validation error message.

% Copyright 2022 The MathWorks, Inc.

narginchk(2,3)

% Populate original indices if they were not provided.
if nargin == 2
    originalIndices = 1:numel(gTruth);
else
    originalIndices = varargin{1};
end

% sort by name
labelDefsStd = sortrows(gTruth(1).LabelDefinitions, 1);

% remove non-rectangle and discard description
labelDefsStd = labelDefsStd(labelDefsStd.Type == type, 1:2);

for n = 2 : numel(gTruth)
    labelDefs = sortrows(gTruth(n).LabelDefinitions, 1);
    labelDefs = labelDefs(labelDefs.Type == type, 1:2);
    
    if ~isequal(labelDefs, labelDefsStd)
        error( message('vision:objectDetectorTrainingData:consistentLabelDefinitions', originalIndices(n)) )
    end
end

labelNames = labelDefsStd.Name;
end