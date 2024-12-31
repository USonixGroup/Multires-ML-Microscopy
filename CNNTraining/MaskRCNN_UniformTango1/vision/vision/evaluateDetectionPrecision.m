function [averagePrecision, recall, precision] = evaluateDetectionPrecision(...
    detectionResults, groundTruthData, varargin)

% Copyright 2016-2023 The MathWorks, Inc.

narginchk(2, 3);

% Validate user inputs
boxFormat = 4; % axis-aligned
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

numClasses       = numel(classes);
averagePrecision = zeros(numClasses, 1);
precision        = cell(numClasses, 1);
recall           = cell(numClasses, 1);

% Compute the precision and recall for each class
for c = 1 : numClasses
    
    labels = vertcat(s(:,c).labels);
    scores = vertcat(s(:,c).scores);
    numExpected = sum([s(:,c).NumExpected]);
    
    [ap, p, r] = vision.internal.detector.detectorPrecisionRecall(labels, numExpected, scores);
    
    averagePrecision(c) = ap;
    precision{c} = p;
    recall{c}    = r;
end

if numClasses == 1
    precision = precision{1};
    recall    = recall{1};
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
