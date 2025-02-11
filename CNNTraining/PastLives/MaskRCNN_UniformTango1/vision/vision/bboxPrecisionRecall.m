function [precision, recall] = bboxPrecisionRecall(boundingBoxes, groundTruthData, varargin)
%bboxPrecisionRecall Compute bounding box precision and recall against ground truth.
%   [precision, recall] = bboxPrecisionRecall(boundingBoxes,groundTruthData) 
%   returns precision and recall to measure the accuracy of bounding box
%   overlap. If the bounding box is associated with a class label,
%   precision and recall are vectors consisting of metrics for each class.
%   The class order follows the same column order as the groundTruthData
%   table. If the bounding box is also associated with a confidence score
%   for ranking, use evaluateDetectionPrecision function.
% 
%   [...] = bboxPrecisionRecall(..., threshold) specifies the overlap
%   threshold for assigning a given box to a ground truth box. The overlap
%   ratio is computed as the intersection over union. The default value is
%   0.5.
%
%   Single-class
%   ------------
%   boundingBoxes   - An M-by-4 matrix of [x, y, width, height] bounding
%                     boxes specifying box locations, or a one-column table
%                     with multiple rows of bounding boxes.
%
%   groundTruthData - An M-by-4 matrix of [x, y, width, height] bounding
%                     boxes specifying ground truth box locations, or a
%                     one-column table with multiple rows of bounding
%                     boxes.
%
%   Multi-class
%   -----------
%   boundingBoxes   - A two-column table, with the first column specifying
%                     multiple rows of [x, y, width, height] bounding boxes
%                     and the second column specifying predicted labels as
%                     categorical type with categories defined by class
%                     names.
%
%   groundTruthData - A multi-column table, with each column containing
%                     ground truth box locations for a particular class,
%                     each specified as an M-by-4 matrix of [x, y, width,
%                     height] locations. The variable name for each column
%                     must specify the class name.
%
%  
%   Example : Evaluate bounding box overlap accuracy
%   ------------------------------------------------
%   % Create two ground truth boxes
%   groundTruthData = [2 2 10 20; 80 80 30 40];
%
%   % Create three boxes for evaluation
%   boundingBoxes = [4 4 10 20; 50 50 30 10; 90 90 40 50];
%
%   % Plot the boxes
%   figure
%   hold on
%   for i = 1 : 2
%       rectangle('Position', groundTruthData(i, :), 'EdgeColor', 'r');
%   end
%   for i = 1 : 3
%       rectangle('Position', boundingBoxes(i, :), 'EdgeColor', 'b');
%   end
%   
%   % Evaluate the overlap accuracy against the ground truth data
%   [precision, recall] = bboxPrecisionRecall(boundingBoxes, groundTruthData)
%
%
%   Example : Evaluate bounding box overlap for 3 classes
%   -----------------------------------------------------
%   % Define class names
%   classNames = ["A","B","C"];
%
%   % Create bounding boxes for evaluation
%   predictedLabels = {...
%       categorical("A", classNames); ...
%       categorical(["C";"B"], classNames)};
%   bboxes = {...
%       [10 10 20 30]; ...
%       [60 18 20 10; 120 120 5 10]};
%   boundingBoxes = table(bboxes, predictedLabels, 'VariableNames',...
%       {'PredictedBoxes', 'PredictedLabels'});
%
%   % Create ground truth boxes
%   A = {[10 10 20 28]; []};
%   B = {[]; [118 120 5 10]};
%   C = {[]; [59 19 20 10]};
%   groundTruthData = table(A, B, C);
%
%   % Evaluate overlap accuracy against ground truth data
%   [precision, recall] = bboxPrecisionRecall(boundingBoxes, groundTruthData)
%   
%
% See also bboxOverlapRatio, evaluateDetectionPrecision.

% Copyright 2017-2018 The MathWorks, Inc.
%
% References
% ----------
%   C. D. Manning, P. Raghavan, and H. Schutze. An Introduction to
%   Information Retrieval. Cambridge University Press, 2008.

narginchk(2, 3);

% Validate user inputs
if isa(boundingBoxes, 'table')
    if ~isa(groundTruthData, 'table')
        validateNonTableInput(groundTruthData, 'groundTruthData');
        groundTruthData = table({groundTruthData});
    end
    boxFormat = 4;
    vision.internal.detector.evaluationInputValidation(boundingBoxes, ...
        groundTruthData, mfilename, false, boxFormat, varargin{:});
else
    validateNonTableInput(boundingBoxes, 'boundingBoxes');
    validateNonTableInput(groundTruthData, 'groundTruthData');
    
    % Convert to a simple table
    boundingBoxes = table({boundingBoxes});
    groundTruthData = table({groundTruthData}); 
end

% Hit/miss threshold for IOU (intersection over union) metric
threshold = 0.5;
if ~isempty(varargin)
    threshold = varargin{1};
end

classes = groundTruthData.Properties.VariableNames;

% Wrap table data into datastore.
resultds = iDetectionResultsDatastore(boundingBoxes,classes);
gtds = boxLabelDatastore(groundTruthData);

% Match the bounding boxes with ground truth
checkScores = false;
s = vision.internal.detector.evaluateDetection(resultds, gtds, threshold, classes, checkScores);

numClasses = width(groundTruthData);
precision  = zeros(numClasses, 1);
recall     = zeros(numClasses, 1);
    
% Compute the precision and recall for each class
for c = 1 : numClasses
    
    labels = vertcat(s(:,c).labels);
    numExpected = sum([s(:,c).NumExpected]);

    [~, p, r] = vision.internal.detector.detectorPrecisionRecall(labels, numExpected);  
    
    precision(c) = p;
    recall(c)    = r;
end

%--------------------------------------------------------------------------
function validateNonTableInput(bbox,arg)

validateattributes(bbox, {'numeric'},...
    {'finite','real','nonsparse','2d', 'size', [NaN, 4]},...
    mfilename,arg);
if (any(bbox(:,3)<=0) || any(bbox(:,4)<=0))
    error(message('vision:visionlib:invalidBboxHeightWidth'));
end

%--------------------------------------------------------------------------
function ds = iDetectionResultsDatastore(results,classname)
switch size(results,2) 
    case 1
        % Results has bbox. Add scores and labels. This standardizes the
        % datastore read output to [bbox, score, labels] and make downstream
        % computation simpler.
        addScores = true;
        addLabels = true;
    case 2
        % Results has bbox and labels. Add scores.
       addScores = true;     
       addLabels = false;
end
ds = vision.internal.detector.detectionResultsTableToDatastore(results,addScores,addLabels,classname);
