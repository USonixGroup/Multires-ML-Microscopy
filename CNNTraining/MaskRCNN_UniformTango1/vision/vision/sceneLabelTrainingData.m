function [imds,groupInfo] = sceneLabelTrainingData(gTruth)

%sceneLabelTrainingData Create training data for image classification from groundTruth.
%   sceneLabelTrainingData creates a datastore that can be used to train a
%   classification network from scene labeled ground truth data.
%
%   imds = sceneLabelTrainingData(gTruth) creates an image datastore from
%   ground truth in gTruth. gTruth is a scalar groundTruth object. The
%   output imageDatastore, imds, contains images in gTruth with scene labels
%   along with corresponding scalar valued Labels in the Labels property of
%   imds.
%
%   [__,groupInfo] = sceneLabelTrainingData(gTruth) additionally returns groupInfo,
%   a struct with fieldnames which describe scene label groups and string array values
%   that correspond to scene label names in each group.

%   Copyright 2022 The MathWorks, Inc.

arguments
    gTruth {mustBeA(gTruth,'groundTruth'),iMustBeValidGroundTruthObject}
end

if ~isscalar(gTruth)
    gTruth = iMergeGroundTruthArray(gTruth);
end

sceneLabelNames = string(gTruth.LabelDefinitions.Name(gTruth.LabelDefinitions.Type == "Scene"));

singleObservationHasMultipleSceneLabels = any(sum(table2array(gTruth.LabelData(:,sceneLabelNames)),2)>1);
if singleObservationHasMultipleSceneLabels
    error(message('vision:sceneLabelTrainingData:multiLabelObservationsNotSupported'));
end

sceneLabelIdx = false(size(gTruth.LabelData,1),length(sceneLabelNames));
for idx = 1:length(sceneLabelNames)
    sceneLabelIdx(:,idx) = gTruth.LabelData.(sceneLabelNames(idx));
end

rowsToInclude = find(any(sceneLabelIdx,2));
catLookup = categorical(sceneLabelNames);

datastoreFilesToInclude = gTruth.DataSource.Source(any(sceneLabelIdx,2));
labels = repmat(catLookup(1),length(datastoreFilesToInclude),1);
for idx = 1:length(rowsToInclude)
    thisRow = gTruth.LabelData(rowsToInclude(idx),:);
    lookupIdx = thisRow{1,sceneLabelNames};
    labels(idx) = catLookup(lookupIdx);
end

imds = imageDatastore(datastoreFilesToInclude);
imds.Labels = labels;

groupInfo = iFormGroupInfo(sceneLabelNames,gTruth.LabelDefinitions);

end

function groupInfo = iFormGroupInfo(sceneLabelNames,labelDefs)

% Filter only label definitions which are sceneLabels
labelDefs = labelDefs(cellfun(@(c) any(c==sceneLabelNames),labelDefs.Name),:);

fields = unique(labelDefs.Group)';
c = cell(length(fields),1);
groupInfo = cell2struct(c,fields);

for idx = 1:size(labelDefs,1)
    groupInfo.(labelDefs.Group{idx}) = horzcat(groupInfo.(labelDefs.Group{idx}),sceneLabelNames(idx));
end
end

function iMustBeValidGroundTruthObject(gTruth)

mustBeNonempty(gTruth);

labelDefs = vertcat(gTruth.LabelDefinitions);
if ~any(labelDefs.Type == "Scene")
    error(message('vision:sceneLabelTrainingData:NoSceneLabelsInGroundTruth'));
end

for idx = 1:length(gTruth)
    if isa(gTruth(idx).LabelData,'timetable')
        error(message(('vision:sceneLabelTrainingData:TimeDataUnsupported')));
    end
end

end

function gTruthNew = iMergeGroundTruthArray(gTruthArray)
labelDefs = {gTruthArray.LabelDefinitions};
allDefsEqual = all(cellfun(@(c) isequal(c,labelDefs{1}),labelDefs));
if ~allDefsEqual
    error(message('vision:sceneLabelTrainingData:ArrayOfGroundTruthLabelDefinitionsDifferent'));
end

source = gTruthArray(1).DataSource.Source;
for idx = 2:length(gTruthArray)
    source = vertcat(source,gTruthArray(idx).DataSource.Source); %#ok<AGROW> 
end
dataSourceNew = groundTruthDataSource(source);

gTruthNew = groundTruth(dataSourceNew, labelDefs{1}, vertcat(gTruthArray.LabelData));
end