function [cleanBoxes, noBoxesRemoved, hasNoBoxes] = hasValidTrainingData(imageSize,oneTrainingDataRow, isRotatedRectangle)
% Scans one training data row and removes invalid boxes,
% returning a clean row of boxes.

%   Copyright 2019-2023 The MathWorks, Inc.

narginchk(2,3)

if nargin == 2
    isRotatedRectangle = false;
end

noBoxesRemoved = true;
cleanBoxes = rowfun(@(varargin)nSanitizeTrainingDataRow(isRotatedRectangle, varargin{:}),...
    oneTrainingDataRow, ...
    'OutputFormat','table', ...
    'ExtractCellContents', true, ...
    'NumOutputs', width(oneTrainingDataRow), ...
    'OutputVariableNames', oneTrainingDataRow.Properties.VariableNames);

allBoxes = rowfun(@(varargin)vertcat(varargin{:}),...
    cleanBoxes(1,2:end),'OutputFormat','cell',...
    'ExtractCellContents',true);
hasNoBoxes = isempty(allBoxes{1});

    function varargout = nSanitizeTrainingDataRow(isRotatedRectangle, varargin)
       % Iterate through boxes and remove invalid ones. Ground
       % truth boxes with fractional values are rounded to the
       % nearest pixel center.
       varargout    = cell(size(varargin));
       varargout(1) = {varargin(1)};
   
       for k = 2:numel(varargin)
           boxes = varargin{k};
   
           boxes = vision.internal.cnn.utils.hasValidTrainingBoxes(imageSize, boxes, isRotatedRectangle);

           varargout{k}  = {boxes};
           noBoxesRemoved = noBoxesRemoved && size(varargout{k}{1},1) == size(varargin{k},1);
       end
   
    end
end
