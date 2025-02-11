function index = indexImages(imgSet, varargin)
%
%   This function is used to implement code generation support for the
%   indexImages function.

% Copyright 2022 The MathWorks, Inc.
%#codegen

[inpBag, saveFeatureLocations] = parseInputsCodegen(imgSet, varargin{:});

if isempty(inpBag)
    imgStr.Images = imgSet.Images;
    if isfield(imgSet, 'Labels')
        imgStr.Labels = imgSet.Labels;
    end

    bag = bagOfFeatures(imgStr, 'TreeProperties', [1 20000], ...
        'PointSelection', 'Detector', ...
        'Upright', false);
else
    bag = inpBag;
end

index = invertedImageIndex(bag, 'SaveFeatureLocations', saveFeatureLocations);

newImg.Images = imgSet.Images;
newImg.Location = imgSet.Location;
addImages(index, newImg);

% -------------------------------------------------------------------------
function [bag, saveFeatureLocation] = parseInputsCodegen(imgSet, varargin)

coder.inline('always')
coder.internal.prefer_const(varargin{:});

validateattributes(imgSet, {'struct'}, {'nonempty'}, mfilename, 'imgSet');

coder.internal.errorIf(~numel(imgSet.Images),'vision:invertedImageIndex:emptyImageSet');

len = length(varargin);

if (len==1)||(len==3)
    bag = varargin{1};
    checkBag(bag);
    pvPairStartIdx = 2;
else
    bag = [];
    pvPairStartIdx = 1;
end

% Define parser mapping struct
pvPairs = struct(...
    'SaveFeatureLocations', uint32(0));

% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

% Parse PV pairs
pstruct = coder.internal.parseParameterInputs(pvPairs, ...
    poptions, varargin{pvPairStartIdx:end});

saveFeatureLocation = coder.internal.getParameterValue(pstruct.SaveFeatureLocations, coder.internal.const(true), varargin{pvPairStartIdx:end});

vision.internal.inputValidation.validateLogical(saveFeatureLocation,'SaveFeatureLocations');

% -------------------------------------------------------------------------
function checkBag(bag)

validateattributes(bag, {'bagOfFeatures'},{},mfilename,'bag');