function [timeRanges,sceneLabels] = sceneTimeRanges(gTruth)
%sceneTimeRanges Time ranges of scene labels from ground truth data.
%
%   timeRanges = sceneTimeRanges(gTruth) returns the time ranges
%   defining the start and end times over which scene labels are applied
%   to ground truth data sources in gTruth.
%
%   [...,sceneLabels] = sceneTimeRanges(...) optionally returns the scene
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
%                timeRanges is an M-by-1 cell array. M is the number of
%                elements in gTruth.
%
%   sceneLabels  A cell array of categorical vectors specifying the scene labels
%                for each time range. Each categorical vector is of size T-by-1,
%                where T is the number of time ranges in a ground truth data
%                source.
%                sceneLabels is an M-by-1 cell array. M is the number of
%                elements in gTruth.
%
%   Note:
%   -----
%   - In order to select and obtain scene time ranges for specific signals in
%     a groundTruthMultisignal object, use selectLabelsBySignalName function
%     before calling sceneTimeRanges function.
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
%   See also groundTruth/gatherLabelData, writeVideoScenes, groundTruthDataSource,
%            groundTruth, objectDetectorTrainingData, pixelLabelTrainingData.

try
    [timeRanges,sceneLabels] = getSceneTimeRanges(gTruth);
catch exc
    throw(exc);
end
