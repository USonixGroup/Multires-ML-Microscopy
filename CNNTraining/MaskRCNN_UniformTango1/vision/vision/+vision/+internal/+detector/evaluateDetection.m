function s = evaluateDetection(resultds,gtds,threshold,classes,checkScore)
% Return per image detection evaluation result.
% detectionResults is a table of one/two/three columns: boxes, scores, labels
% groundTruth is a table of boxes, one column for each class
% threshold is the intersection-over-union (IOU) threshold. 
% checkScore is true if the detectionResults has scores in its second
% column

% Copyright 2016-2019 The MathWorks, Inc.

if nargin == 4
    checkScore = true;
end

cds = combine(resultds,gtds);
tds = transform(cds, @(data)iForEach(@nAssignDetectionsToGroundTruth,data));
data = readall(tds);

% vertcat results over all images for each class. Results holds a
% 1xnumClasses struct array. After concat s is numImages-by-numClasses
% struct array.
s = vertcat(data{:});
  
    %----------------------------------------------------------------------
    function results = nAssignDetectionsToGroundTruth(data)
        
        [bbox,scores,labels,gtbbox,gtlabels] = deal(data{:});
        
        numClasses = numel(classes);
        results = iPreallocatePerClassResultsStruct(numClasses);
        for c = 1:numClasses
                       
            gtidx = iFindEqual(gtlabels,classes(c));
            expectedBoxesForClass = gtbbox(gtidx, :);
            
            idx = iFindEqual(labels,classes(c));
            
            bboxesPerClass = bbox(idx, :);
            
            if checkScore
                scoresPerClass = scores(idx,:);
            else
                scoresPerClass = [];
            end
            
            [ypred, falseNegative, assignments] = ...
                vision.internal.detector.assignDetectionsToGroundTruth(bboxesPerClass, ...
                expectedBoxesForClass, threshold, scoresPerClass);
           
            % per class per image results
            results(c).labels = ypred;
            results(c).scores = scoresPerClass;
            results(c).Detections = bboxesPerClass;
            results(c).Groundtruth = expectedBoxesForClass;
            results(c).FalseNegative = falseNegative;
            results(c).GroundTruthAssignments = assignments;
            results(c).NumExpected = size(expectedBoxesForClass,1);
        end
        results = {results};
    end

end

%--------------------------------------------------------------------------
function idx = iFindEqual(labels,classes)
if isempty(labels)
    idx = [];
else
    idx = labels == classes;
end
end

%--------------------------------------------------------------------------
function out = iForEach(fcn,data)
out = cell(size(data,1));
for i = 1:size(data,1)
    out(i) = fcn(data(i,:));
end
end

%--------------------------------------------------------------------------
function s = iPreallocatePerClassResultsStruct(numClasses)
s(numClasses) = struct(...
    'labels', [], ...
    'scores', [], ...
    'Detections', [], ...
    'FalseNegative', [], ...
    'GroundTruthAssignments', [], ...
    'NumExpected', []);
end