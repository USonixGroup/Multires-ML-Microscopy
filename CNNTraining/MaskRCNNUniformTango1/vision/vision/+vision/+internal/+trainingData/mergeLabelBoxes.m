function data = mergeLabelBoxes(data, uniqueLabels, outputType)
%MERGELABELBOXES Merge multi-category bounding-box annotations
%
%   The ground-truth labels may contain multi-category bounding boxes: e.g.
%   separate labels for 'vehicleFront', 'vehicleRear', 'vehicleRightSide',
%   'vehicleLeftSide', etc.
%
%   This helper function merges multi-category object labels into a single
%   category.
%
%   Input:
%   ------
%
%   groundTruthTable    An M-by-N table in the format for object detector
%                       training data.
%                       Each column corresponds to an object category that
%                       is present in the label data.
%
%                       | classA | classB | classC |...
%
%                       Each row of the table has the Mx4 bounding boxes
%                       for M objects of a specific category.
%
%   Outputs:
%   --------
%   A table or a cell based on the OutputType input argument, with first
%   column being the bounding boxes and the second column being the label
%   names as a categorical vector. Each categorical vector will contain the
%   categories across all of the data provided in the input.
%
%   BoundingBoxes     - Cell array of bounding boxes, each cell
%                       corresponding to an image. For M objects in that
%                       image, the bounding box will be an M-by-4 matrix of
%                       [x y width height].
%
%   Labels            - A cell array of categorical vectors. Each element of the
%                       cell array is an M-by-1 categorical vector, each value
%                       corresponding to the class label of a bounding box.
%
%   See also boxLabelDatastore, imageDatastore, objectDetectorTrainingData,
%            groundTruth, trainFasterRCNNObjectDetector, groundTruthLabeler,
%            videoLabeler, imageLabeler.

% Copyright 2019 The MathWorks, Inc.
    narginchk(1,3);
    switch nargin
        case 1
            uniqueLabels = data.VariableNames;
            outputType = 'table';
        case 2
            outputType = 'table';
    end
    outputType = convertStringsToChars(outputType);
    outputType = validatestring(outputType, {'cell','table'});
    [bboxes, labels] = iMergeBBoxAnnots(data, uniqueLabels);
    switch outputType
        case 'table'
            data = table(bboxes, labels, 'VariableNames', {'Boxes','Labels'});
        case 'cell'
            data = cell(numel(bboxes), 2);
            data(:,1) = bboxes;
            data(:,2) = labels;
    end
end

function [bboxes, labels] = iMergeBBoxAnnots(T, uniqueLabels)
%MERGEBBOXANNOTS Merge multi-category bounding-box annotations
%
%   The ground-truth labels may contain multi-category bounding boxes: e.g.
%   separate labels for 'vehicleFront', 'vehicleRear', 'vehicleRightSide',
%   'vehicleLeftSide', etc.
%
%   This helper function merges multi-category object labels into a single
%   category.
%
%   Input:
%   ------
%
%   groundTruthTable    Table in format for object detector training data.
%                       The first column corresponds to image filenames.
%                       Subsequent columns each correspond to an object
%                       category that is present in the dataset.
%
%                       | imageFiles | classA | classB | classC |...
%
%                       Each row of the table has the path to an image,
%                       stored under the first column. The subsequent
%                       columns store Mx4 bounding boxes for M objects of a
%                       specific category present in that image.
%
%   Outputs:
%   --------
%   imageFilenames      Cell array of image filenames.
%
%   bboxes              Cell array of bounding boxes, each cell
%                       corresponding to an image in 'imageFilenames'. For
%                       M objects in that image, the bounding box will be
%                       an Mx4 matrix of [x y width height].
%
%   labels              A cell array of cell arrays. Each element of the
%                       nested cell array corresponds to the class label of
%                       a bounding box.


bboxes = cell(height(T), 1);
labels = cell(height(T), 1);

tableVarNames = T.Properties.VariableNames; % other cols are classes

for ii = 1:height(T)
    % for every row, aggregate the bboxes and/or labels
    bboxRow = T{ii, :};
    [x, y] = iAggregateBboxesAndLabels(bboxRow, tableVarNames);

    % sanity checks
    assert(isequal(numel(y), size(x,1)));

    bboxes{ii} = x;
    catLabels = categorical(y, uniqueLabels);
    labels{ii} = catLabels(:);

end

assert(~isempty(bboxes));

end

% -------------------------------------------------------------------------
function [x, y] = iAggregateBboxesAndLabels(labeledBoxes, tableVarNames)
%   Aggregate annotations in a single row of a table

% find the empty annotations - that class is not present in image
emptyIdx = cellfun(@isempty, labeledBoxes, 'un', 0);
emptyIdx = cell2mat(emptyIdx);

% merge non-empty bboxes
x = labeledBoxes(~emptyIdx);
x = vertcat(x{:});

boxcounts = cellfun(@(x)(size(x,1)), labeledBoxes, 'un', 0);
boxlabels = cellfun(@(a,b)(repmat({a},[1 b])), tableVarNames, boxcounts, 'un', 0);
y = cat(2, boxlabels{:});
end
