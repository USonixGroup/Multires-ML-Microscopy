function ds = detectionResultsTableToDatastore(results,addScore,addLabel,label)
% convert detection results table into an in-memory datastore. Add scores
% and labels if required. This is used in evaluateDetectionPrecision,
% evaluateDetectionMissRate, evaluateDetectionAOS, and bboxPrecisionRecall
% to standardize the detection result data into [bbox, scores, labels].

% Copyright 2019 The Mathworks, Inc.

arguments
    results  table 
    addScore (1,1) logical = false
    addLabel (1,1) logical = false
    label     = []
end

results = table2cell(results);

ds = vision.internal.cnn.datastore.InMemoryDatastore(results);

if addScore || addLabel
    
   if addScore && addLabel
       % results only has bounding boxes.
       label = categorical(label,label);
       ds = transform(ds,@(data)iAddScoreAndLabel(data,label));
       
   elseif addScore
       % results has boxes and labels.
       ds = transform(ds,@iAddScores);
       
   elseif addLabel
       % results has boxes and scores.
       label = categorical(label,label);
       ds = transform(ds,@(data)iAddLabel(data,label));
       
   end
    
end

end

%--------------------------------------------------------------------------
function data = iAddScores(data)
M = size(data,1);
scores = cell(M,1);
for i = 1:M
    if isempty(data{i,1})
        scores{i} = [];
    else
        numelem = size(data{i,1},1);
        scores{i} = ones(numelem,1);
    end
end
data = [data(:,1) scores data(:,2)];
end

%--------------------------------------------------------------------------
function data = iAddLabel(data,label)
assert(iscell(data))
M = size(data,1);
labels = cell(M,1);
for i = 1:M
    if isempty(data{i,1})
        labels{i} = label([],1);
    else
        numelem = size(data{i,1},1);
        labels{i} = label(ones(numelem,1));
    end
end
data = [data labels];
end

%--------------------------------------------------------------------------
function data = iAddScoreAndLabel(data,label)
M = size(data,1);
labels = cell(M,1);
scores = cell(M,1);
for i = 1:M
    if isempty(data{i,1})
        labels{i} = label([],1);
        scores{i} = [];
    else
        numelem = size(data{i,1},1);
        labels{i} = label(ones(numelem,1));
        scores{i} = ones(numelem,1);
    end
end
data = [data scores labels];
end
