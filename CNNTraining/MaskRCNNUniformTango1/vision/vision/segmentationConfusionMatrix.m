function confusionMatrix = segmentationConfusionMatrix(predictedLabels,groundTruthLabels, varargin)

narginchk(2,4);
matlab.images.internal.errorIfgpuArray(predictedLabels,groundTruthLabels);

validateInput = @(x,name,pos) validateattributes(x, ...
    {'logical','double','categorical'}, ...
    {'3d','real','nonempty','nonsparse'}, ...
    mfilename,name,pos);

validateInput(predictedLabels,'predictedLabels',1);
validateInput(groundTruthLabels,'groundTruthLabels',2);

iAssertHaveSameSize(predictedLabels,groundTruthLabels);
iAssertSameDataType(predictedLabels,groundTruthLabels);

% Additional validation
if iscategorical(predictedLabels)
    % Categorical matrices
    iAssertCategoricalsHaveSameCategories(predictedLabels,groundTruthLabels);
        
elseif isa(predictedLabels,'double')
    % Label matrices
    iValidateLabelMatrices(predictedLabels,groundTruthLabels);
end

classes = iParseOptionalArguments(varargin{:});
classes = iValidateClasses(classes, class(predictedLabels));
if strcmpi(classes,'auto')
    classes = iDefineAutoBehavior(predictedLabels, groundTruthLabels);
end

% Mask ground truth pixels that are not specified in classes.
BW1 = images.internal.segmentation.convertToCellOfLogicals(predictedLabels,classes);
BW2 = images.internal.segmentation.convertToCellOfLogicals(groundTruthLabels,classes);

if iscategorical(predictedLabels)
    % Masks pixels marked undefined in the ground truth so that they
    % are not counted in the predicted labels.
    BW1 = semanticSegmentationMetrics.maskUnlabeledPixels(BW1, groundTruthLabels);
end

confusionMatrix = images.internal.segmentation.bwconfmat(BW1,BW2);

end

%--------------------------------------------------------------------------
function iAssertCategoricalsHaveSameCategories(predictedLabels,groundTruthLabels)
if ~isequal(categories(predictedLabels),categories(groundTruthLabels))
    error(message('vision:segmentationConfusionMatrix:mustHaveSameClasses', ...
        'predictedLabels','groundTruthLabels'))
end
end

%--------------------------------------------------------------------------
function iAssertHaveSameSize(predictedLabels,groundTruthLabels)
if ~isequal(size(predictedLabels),size(groundTruthLabels))
    error(message('vision:segmentationConfusionMatrix:mustHaveSameSize',...
        'predictedLabels','groundTruthLabels'));
end
end

%--------------------------------------------------------------------------
function iAssertSameDataType(predictedLabels,groundTruthLabels)
if ~isa(predictedLabels,class(groundTruthLabels))
    error(message('images:validate:differentClassMatrices','predictedLabels','groundTruthLabels'))
end
end

%--------------------------------------------------------------------------
function iValidateLabelMatrices(predictedLabels,groundTruthLabels)
validateLabelMatrix = @(x,name,pos) validateattributes(x, ...
    {'double'}, ...
    {'finite','nonnegative','integer'}, ...
    mfilename,name,pos);
validateLabelMatrix(predictedLabels,'predictedLabels',1);
validateLabelMatrix(groundTruthLabels,'groundTruthLabels',2);
end

%--------------------------------------------------------------------------
function classes = iParseOptionalArguments(varargin)
p = inputParser;

p.addParameter('Classes','auto');

p.parse(varargin{:});

results = p.Results;

classes = results.Classes;

end

%--------------------------------------------------------------------------
function classes = iValidateClasses(classes, inputType)

if ischar(classes)
   classes = {classes}; 
end

if any(strcmpi(classes,'auto'))
    if numel(classes)>1
        error(message('vision:segmentationConfusionMatrix:invalidClassAuto'));
    end
else
    iValidateClassesType(classes,inputType);
end

if ~isequal(unique(classes,'stable'), classes)
    error(message('vision:segmentationConfusionMatrix:classesMustBeUnique'));
end
end

%--------------------------------------------------------------------------
function classes = iDefineAutoBehavior(predictedLabels,groundTruthLabels)

inputType = class(predictedLabels);

switch inputType
    case 'logical'
        classes = [false true];
    case 'double'
        classes = (1:max(max(predictedLabels(:)),max(groundTruthLabels(:))))';
    case 'categorical'
        classes = categories(predictedLabels);
end

end

%--------------------------------------------------------------------------
function iValidateClassesType(classes,inputType)

switch inputType
    case 'logical'
        validateattributes(classes, {'logical'}, {'vector','nonempty'}, mfilename, 'Classes');
    case 'double'
        validateattributes(classes, {'double'}, {'vector','nonempty','integer','increasing','finite','nonnegative'}, mfilename, 'Classes');
    case 'categorical'
        validateattributes(classes, {'string', 'char', 'cell'}, {'nonempty'}, mfilename, 'Classes');
        if iscell(classes)
            for idx = 1:numel(classes)
                validateattributes(classes{idx},...
                    {'char'}, {'nonempty'}, mfilename, 'Classes')
            end
        end
end
end

%   Copyright 2020-2023 The MathWorks, Inc.