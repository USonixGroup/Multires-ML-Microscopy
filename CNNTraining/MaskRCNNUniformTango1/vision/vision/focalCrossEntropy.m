function X = focalCrossEntropy(X, T, NameValueArgs)
  
%   Copyright 2020-2023 The MathWorks, Inc.

arguments
    X {validateattributes(X,{'dlarray','numeric'},{'nonempty'}, ...
        'focalCrossEntropy','X')}
    T {validateattributes(T,{'dlarray','numeric'},{'nonempty'}, ...
        'focalCrossEntropy','T')}
    NameValueArgs.Gamma {validateattributes(NameValueArgs.Gamma, ...
        {'numeric'},{'finite','scalar','real','nonnegative','nonsparse'}, ...
        'focalCrossEntropy','Gamma')} = 2
    NameValueArgs.Alpha {validateattributes(NameValueArgs.Alpha, ...
        {'numeric'},{'finite','scalar','real','positive','nonsparse'}, ...
        'focalCrossEntropy','Alpha')} = 0.25
    NameValueArgs.DataFormat
    NameValueArgs.ClassificationMode string
    NameValueArgs.TargetCategories string
    NameValueArgs.Reduction string = "mean"
end

% Handle input DataFormat.
labelString = iGetDimensionLabels(NameValueArgs);
[X,perm] = deep.internal.dlarray.validateDataFormatArg(X, labelString);

% Validate true values.
T = validateTrueValues(X, T, labelString);

% Validate the optional type argument.
isMultilabelClassification = validateClassificationTask(NameValueArgs);

% Validate the optional reduction argument.
reduceVal = validateReduction(NameValueArgs.Reduction);
    
% Loss operation.
X = lossOp(X, T, NameValueArgs.Gamma, NameValueArgs.Alpha, ...
    isMultilabelClassification);

if reduceVal % mean Reduction
    
    labels = dims(T);
    channelDim = find(labels == 'C');
    
    % Count non-zero observations along the channel dimension of T. 
    if ~isempty(channelDim)
        N = numValidObservations( sum(T ~= 0, channelDim) ); 
    else
        N = numValidObservations( T );
    end
    
    if N ~= 0
        X = sum(X,'all')/N;        
    else
        X = N;
    end
    
    % Remove dimensions from the dlarray
    X = stripdims(X);
    
else % no Reduction
    
    % Remove dimensions from the dlarray if it was passed as unformatted.
    if isfield(NameValueArgs,'DataFormat')
        X = stripdims(X);

        % Permutes back in same order as DataFormat.
        X = ipermute(X, perm);
    end
end

end

%--------------------------------------------------------------------------
function isMultilabelClassification = validateClassificationTask(args)
isMultilabelClassification = false;
userSpecifiedClassificationMode = isfield(args, "ClassificationMode");
userSpecifiedTargetCategories = isfield(args, "TargetCategories");
if userSpecifiedClassificationMode && userSpecifiedTargetCategories
    error(message("deep:dlarray:ClassificationModeTargetCategoriesClash"));
elseif userSpecifiedTargetCategories
    isMultilabelClassification = validateTargetCategories(args.TargetCategories);
elseif userSpecifiedClassificationMode
    isMultilabelClassification = validateClassificationMode(args.ClassificationMode);
end
end

%--------------------------------------------------------------------------
function isMultilabelClassification = validateTargetCategories(value)
value = validatestring(value,{'exclusive','independent'}, ...
    'focalCrossEntropy','TargetCategories');
if strcmp(value,'exclusive')
    isMultilabelClassification = false; % categorical focal cross-entropy
else
    isMultilabelClassification = true; % binary focal cross-entropy
end
end

%--------------------------------------------------------------------------
function isMultilabelClassification = validateClassificationMode(value)
value = validatestring(value,{'single-label','multilabel'}, ...
    'focalCrossEntropy','ClassificationMode');
if strcmp(value,'single-label')
    isMultilabelClassification = false; % categorical focal cross-entropy
else
    isMultilabelClassification = true; % binary focal cross-entropy
end
end

%--------------------------------------------------------------------------
function reduceVal = validateReduction(value)
value = validatestring(value,{'mean','none'}, 'focalCrossEntropy','Reduction');
if strcmp(value,'mean')
    reduceVal = true; 
else
    reduceVal = false; 
end
end

%--------------------------------------------------------------------------
function T = validateTrueValues(X, T, dataFormat)
% validateTrueValues Verify the true values label and size match X data.

% Find how the arguments are formatted.
firstArgumentIsLabeled = isempty(dataFormat);
secondArgumentIsLabeled = isa(T, 'dlarray') && ~isempty(dims(T));

% Make sure dataFormat contains either the value of DataFormat or the
% labels of X.
labelsX = dims(X);
if firstArgumentIsLabeled
    dataFormat = labelsX;
end

% Verify that there are enough labels for T.
numLabels = numel(dataFormat);
if numLabels > 1 && ndims(T) > numLabels
    % T has too many dimensions compared to X. Throw the size mismatch
    % error.
    error(message('deep:dlarray:LossSizeMismatch',""));
end

if secondArgumentIsLabeled
    % Verify the ordered formats are identical.
    if ~isequal(dims(T), labelsX)
        error(message('deep:dlarray:LossDescriptorMismatch'));
    end
else
    % Create a dlarrary with the data format of the input X.
    T = dlarray(T, dataFormat);
end

% Verify the sizes match after dlarray construction.
if ~isequal(size(T), size(X))
    if ~firstArgumentIsLabeled && secondArgumentIsLabeled
        % Throw a more general error message as size vectors are allowed to
        % be different for this case.
        error(message('deep:dlarray:LossLabeledTargetMismatchSize'));
    else
        error(message('deep:dlarray:LossSizeMismatch',""));
    end
end

% The dlarray methods should not accept logical input.
if islogical(T)
    error(message('deep:dlarray:LogicalsNotSupported'));
end
if ~isreal(X)
    error(message('deep:dlarray:ComplexNotSupported'));
end
if ~isreal(T)
    error(message('deep:dlarray:ComplexNotSupported'));
end
end

%--------------------------------------------------------------------------
function Z = lossOp(X,T,gamma,alpha,hasIndependentTargetCategories)

% Explicitly casting means that derivatives are guaranteed to be real.
X = real(X);
T = real(T);
gamma = real(gamma);
alpha = real(alpha);

P = boundAwayFromZero(X);
OneMinusP = boundAwayFromZero(1-X);

% Categorical focal cross-entropy.
Z = -T.*alpha.*(OneMinusP.^gamma).*log(P);

if hasIndependentTargetCategories
    % Binary focal cross-entropy.
    Z = Z - (1-T).*(1-alpha).*(P.^gamma).*log(OneMinusP);
end

% Explicitly casting means that derivatives are guaranteed to be real.
Z = real(Z);

end

%--------------------------------------------------------------------------
function X = boundAwayFromZero(X)
classX = classUnderlyingOfDlarray(X);
X(X < eps(classX)) = eps(classX);
end

%--------------------------------------------------------------------------
function labelString = iGetDimensionLabels(NameValueArgs)

if isfield(NameValueArgs,'DataFormat')
    labelString = NameValueArgs.DataFormat;
else
    labelString = [];
end
end

%--------------------------------------------------------------------------
function c = classUnderlyingOfDlarray(X)
    c = class(gather(extractdata(X([]))));
end

%--------------------------------------------------------------------------
function n = numValidObservations(X)
n = sum(X~=0, 'all');
end
