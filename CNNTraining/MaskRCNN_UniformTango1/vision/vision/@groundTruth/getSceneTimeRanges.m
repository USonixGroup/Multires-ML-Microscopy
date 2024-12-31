function [timeRanges,sceneLabels] = getSceneTimeRanges(gTruth)
%getSceneTimeRanges Time ranges of scene labels from ground truth data.
%
%   timeRanges = getSceneTimeRanges(gTruth) returns the time ranges
%   defining the start and end times over which scene labels are applied
%   to ground truth data sources in gTruth.
%
%   [...,sceneLabels] = getSceneTimeRanges(...) optionally returns the scene
%   labels associated with the time ranges.
%
%   Inputs
%   ------
%   gTruth       An array of groundTruth or groundTruthMultisignal objects.
%
%   Outputs
%   -------
%   timeRanges   A cell array of duration matrices specifying the time ranges
%                of scene labels. Each duration matrix is of size T-by-2.
%                Each row in this matrix corresponds to a time range for
%                which that scene label has been applied in the ground truth.
%                T is the number of time ranges. Rows in the matrix are of the
%                form [rangeStart, rangeEnd], where rangeStart and rangeEnd
%                specify the start and end of a time range for an applied scene
%                label.
%                timeRanges is an M-by-1 cell array for a groundTruth input or
%                an M-by-N cell array for a groundTruthMultisignal input. M is
%                the number of elements in gTruth and N is the number of signals
%                in gTruth.
%
%   sceneLabels  A cell array of categorical vectors specifying the scene labels
%                for each time range. Each categorical vector is of size T-by-1,
%                where T is the number of time ranges in a ground truth data
%                source.
%                sceneLabels is an M-by-1 cell array for a groundTruth input or
%                an M-by-N cell array for a groundTruthMultisignal input. M is
%                the number of elements in gTruth and N is the number of signals
%                in gTruth.
%
%   Note:
%   -----
%   - In order to select and obtain scene time ranges for specific signals in
%     a groundTruthMultisignal object, use selectLabelsBySignalName function
%     before calling getSceneTimeRanges function.
%
%   Example : Gather scene time ranges and scene labels.
%   ----------------------------------------------------
%   % Load ground truth scene label definitions and label data.
%   data             = load('groundTruthSceneLabels.mat');
%   labelDefinitions = data.labelDefinitions;
%   labelData        = data.labelData;
%
%   % Create ground truth datasource with a video file.
%   gSource = groundTruthDataSource('viptrain.avi');
%
%   % Create a ground truth object.
%   gTruth = groundTruth(gSource, labelDefinitions, labelData);
%
%   % Gather all the scene time ranges and the scene labels.
%   [timeRanges, sceneLabels] = sceneTimeRanges(gTruth);
%
%   % Display the scene time ranges for the ground truth data.
%   timeRanges{1}
%
%   % Display the corresponding scene labels for the ground truth data.
%   sceneLabels{1}
%
%   See also gatherLabelData, writeFrames, writeVideoScenes, groundTruthDataSource,
%            groundTruth, objectDetectorTrainingData, pixelLabelTrainingData.

% Copyright 2021 The MathWorks, Inc.

    arguments
        % Required inputs.
        gTruth  {mustBeNonempty,mustBeVector,iValidateGroundTruth(gTruth)}
    end
   
    numGtruth = numel(gTruth);
    timeRanges = cell(numGtruth,1);
    sceneLabels = cell(numGtruth,1);
    classNames = getUniqueClassNames(gTruth);

    for ii = 1:numGtruth
        labelData = gTruth(ii).LabelData;
        labelDefs = gTruth(ii).LabelDefinitions;
        labels = getSceneLabels(labelDefs);
        numLabels = numel(labels);
        lbls = cell(numLabels,1);
        durations = cell(numLabels,1);
      
        % For each of the labels collect the time ranges
        % represented by the duration matrices.
        time = labelData.Time;
        for jj = 1:numel(labels)
            lblName = labels(jj);
            logicalVec = labelData.(lblName);
            durations{jj} = getDurations(time, logicalVec);
            m = size(durations{jj},1);
            lbls{jj} = categorical(repmat(lblName, m, 1), classNames);
        end
        timeRanges{ii} = vertcat(durations{:});
        sceneLabels{ii} = vertcat(lbls{:});
    end

end

%--------------------------------------------------------------------------
function durations = getDurations(time, logicalVec)
    % getDurations Obtain duration matrices for a label
    % specified by the logical vector, logicalVec. The
    % duration matrices are of size M-by-2, where M is the
    % of time ranges, the first column is the start time
    % and the second column is the end time of the time range.
    indices = find(diff([false; logicalVec]));
    numIndices = numel(indices);

    % When the last frame is labeled, odd number of
    % indices are found.
    numDurations = ceil(numIndices/2);
    durations = seconds(zeros(numDurations,2));

    durationIdx = 1;
    maxTime = time(end);

    for i = 1:2:numIndices
        firstIdx = indices(i);

        if i ~= numIndices
            secondIdx = indices(i+1);

            rowVal = [time(firstIdx) time(secondIdx)];
        else
            % When the last frame is labeled.
            rowVal = [time(firstIdx) maxTime];
        end

        durations(durationIdx,:) = rowVal;
        durationIdx = durationIdx + 1;
    end
end

%--------------------------------------------------------------------------
function labels = getSceneLabels(labelDefs)
    scenes = labelDefs.Type == labelType.Scene;
    labels = string(labelDefs.Name(scenes));
end

%--------------------------------------------------------------------------
function classNames = getUniqueClassNames(gTruth)
    numGtruth = numel(gTruth);
    classNames = cell(size(gTruth));
    for ii = 1:numGtruth
        labelDefs = gTruth(ii).LabelDefinitions;
        classNames{ii} = getSceneLabels(labelDefs);
    end
    classNames = unique(vertcat(classNames{:}));
end

%--------------------------------------------------------------------------
function iValidateGroundTruth(gTruth)
    numGtruth = numel(gTruth);
    for ii = 1:numGtruth
        % Must be a timetable of labels to avoid ground truth data
        % from imageLabeler or without time stamps.
        if ~istimetable(gTruth(ii).LabelData) || ~any(gTruth(ii).LabelDefinitions.Type == labelType.Scene)
            error(message('vision:groundTruth:SceneTimeRangesNoSceneLabels', string(ii)));
        end
    end
end
