function metrics = evaluateDetectionAOS(...
    detectionResults, groundTruthData, varargin)

% Copyright 2019-2023 The MathWorks, Inc.

narginchk(2, 3);

% Validate user inputs
boxFormat = 5; % rotated rectangle
[gtds, classes] = vision.internal.detector.evaluationInputValidation(detectionResults, ...
    groundTruthData, mfilename, true, boxFormat, varargin{:});

% Hit/miss threshold for IOU (intersection over union) metric.
threshold = 0.5;
if ~isempty(varargin)
    threshold = varargin{1};
end

resultds = iDetectionResultsDatastore(detectionResults,classes);

% Match the detection results with ground truth
s = vision.internal.detector.evaluateDetection(resultds, gtds, threshold, classes);

numClasses = numel(classes);
aos        = zeros(numClasses, 1);
ap         = zeros(numClasses, 1);
precision  = cell(numClasses, 1);
recall     = cell(numClasses, 1);
osim       = cell(numClasses, 1);

% Compute the precision and recall for each class
for c = 1 : numClasses
    
    labels = vertcat(s(:,c).labels);
    scores = vertcat(s(:,c).scores);
    numExpected = sum([s(:,c).NumExpected]);

    thetaDiff = iOrientationDifference({s(:,c).Detections}, {s(:,c).Groundtruth}, {s(:,c).GroundTruthAssignments});
    
    [aos(c), ap(c), osim{c}, precision{c}, recall{c}] = ...
        vision.internal.detector.detectorAOS(labels, thetaDiff, numExpected, scores);
    
end

metrics = table(aos,ap,osim,precision,recall,...
    'VariableNames',{'AOS','AP','OrientationSimilarity','Precision','Recall'},...
    'RowNames',classes);
end

%--------------------------------------------------------------------------
function thetaDiff = iOrientationDifference(detections, truth, assignments)
ads = vision.internal.cnn.datastore.InMemoryDatastore(reshape(assignments,[],1));
resultds = vision.internal.cnn.datastore.InMemoryDatastore(reshape(detections,[],1));
gtds = vision.internal.cnn.datastore.InMemoryDatastore(reshape(truth,[],1));

cds = combine(resultds,gtds,ads);
tds = transform(cds,@iThetaDiff);
thetaDiff = readall(tds);
thetaDiff = vertcat(thetaDiff{:});
end

%--------------------------------------------------------------------------
function thetaDiff = iThetaDiff(data)

[bbox,gtbbox,assignments] = deal(data{:});

if isempty(bbox)
    thetaDiff = {zeros(0,1)};
    return
end

thetaDiff = zeros(size(bbox,1),1);

if isempty(gtbbox)
    % All detections are false positives.
    thetaDiff = {thetaDiff};
    return
end

% detections that were assigned to ground truth;
idx = assignments > 0; 

% ground truth box that was assigned.
gidx = assignments(idx);

thetaDiff(idx) = bbox(idx,5) - gtbbox(gidx,5);

thetaDiff = {thetaDiff};
end

%--------------------------------------------------------------------------
function ds = iDetectionResultsDatastore(results,classname)
if size(results,2) == 2
    % Results has bbox and scores. Add labels. This standardizes the
    % datastore read output to [bbox, scores, labels] and make downstream
    % computation simpler.
    addScores = false;
    addLabels = true;
else
    addScores = false;
    addLabels = false;
end
ds = vision.internal.detector.detectionResultsTableToDatastore(results,addScores,addLabels,classname);
end
