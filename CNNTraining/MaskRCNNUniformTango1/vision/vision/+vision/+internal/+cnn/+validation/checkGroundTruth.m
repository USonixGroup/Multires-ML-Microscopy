function checkGroundTruth(gt, name, acceptedWidth)
%

%   Copyright 2016-2020 The MathWorks, Inc.

validateattributes(gt, {'table'},{'nonempty'}, name, 'trainingData',1);

if nargin < 3
    acceptedWidth = 2;
end

if width(gt) < acceptedWidth
    error(message('vision:ObjectDetector:trainingDataTableWidthLessThanTwo'));
end

% First column must contain filenames as cell array of char or strings.
for i = 1:height(gt)
    if ~iscellstr(gt{i,1}) && ~isstring(gt{i,1}) 
       error(message('vision:ObjectDetector:trainingDataFirstColumnNotFilename'));
    end
end

% error if all columns have no boxes.
for i = 2:width(gt)
    if classHasNoBoxes(gt(:,i))        
        cls = gt.Properties.VariableNames{i};
        error(message('vision:ObjectDetector:classHasNoBoxes', cls));
    end
end

%--------------------------------------------------------------------------
function TF = classHasNoBoxes(tblCol)
boxes = tblCol{:,1};
if iscell(boxes)
    boxes = vertcat(boxes{:});
    TF = isempty(boxes);
else
    TF = isempty(boxes);
end
