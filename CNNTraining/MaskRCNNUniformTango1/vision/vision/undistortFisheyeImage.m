function [J, camIntrinsics] = undistortFisheyeImage(I, intrinsics, varargin)

% Copyright 2018-2023 The MathWorks, Inc.

%#codegen
narginchk(2, 9);
if isSimMode()
    if nargin > 2
        [varargin{:}] = convertStringsToChars(varargin{:});
    end
end

validateattributes(intrinsics, {'fisheyeIntrinsics'}, {'scalar'}, ...
    mfilename, 'intrinsics');

[interp, outputView, scaleFactor, fillValues, method] = parseInputs(I, varargin{:});

imageSize = intrinsics.ImageSize;
coder.internal.errorIf(isempty(imageSize), 'vision:calibrate:emptyImageSize');
coder.internal.errorIf(~isequal([size(I,1),size(I,2)], imageSize), 'vision:calibrate:inconsistentImageSize');

originalClass = class(I);
if ~(isa(I,'double') || isa(I,'single') || isa(I,'uint8'))
    imageInput = single(I);
    fillValuesUpdated = cast(fillValues, 'like', imageInput);
else
    imageInput = I;
    fillValuesUpdated = fillValues;
end

f = min(imageSize) / 2;
focalLength = f .* scaleFactor(:)';

[J, camIntrinsics] = undistortImageImpl(intrinsics, imageInput, interp, ...
    outputView, focalLength, fillValuesUpdated, method);

J = cast(J, originalClass);


%--------------------------------------------------------------------------
function [interp, outputView, scaleFactor, fillValues, method] = parseInputs(I, varargin)
vision.internal.inputValidation.validateImage(I);

if isSimMode()
    [interp, outputView, scaleFactor, fillValues, method] = parseInputsSim(I, varargin{:});
else
    [interp, outputView, scaleFactor, fillValues, method] = parseInputsCodegen(I, varargin{:});
end

fillValues = vision.internal.inputValidation.scalarExpandFillValues(...
    fillValues, I);

%--------------------------------------------------------------------------
function [interp, outputView, scaleFactor, fillValues, method] = ...
                                            parseInputsSim(I, varargin)
defaults = getDefaults();

persistent parser;

if isempty(parser)
    parser = inputParser();
    parser.addOptional('interp', defaults.Interp, @validateInterpMethod);
    parser.addParameter('OutputView', defaults.OutputView, @validateOutputView);
    parser.addParameter('ScaleFactor', 1, @validateScaleFactor);
    parser.addParameter('FillValues', defaults.FillValues);
    parser.addParameter('Method', defaults.Method);
end

parser.parse(varargin{:});
interp = parser.Results.interp;
if ~strcmp(interp, defaults.Interp)
    interp = vision.internal.inputValidation.validateInterp(interp);
end

fillValues = parser.Results.FillValues;
outputView = verifyFillValuesAndOutputView(I, fillValues, parser.Results.OutputView);

scaleFactor = parser.Results.ScaleFactor;
if isscalar(scaleFactor)
    scaleFactor = [scaleFactor, scaleFactor];
end

method = validateMethod(parser.Results.Method);

%--------------------------------------------------------------------------
function [interp, outputView, scaleFactor, fillValues, method] = ...
                                            parseInputsCodegen(I, varargin)
% Parse PV pairs
if numel(varargin) > 0
    param1 = varargin{1};
    if ~(strcmpi(param1, 'OutputView') || strcmpi(param1, 'ScaleFactor') || strcmpi(param1, 'FillValues'))
        interp = vision.internal.inputValidation.validateInterp(param1);
        [outputView, scaleFactor, fillValues, method] = parseInputsCodegenParams(I, varargin{2:end});
    else
        interp = 'bilinear';
        [outputView, scaleFactor, fillValues, method] = parseInputsCodegenParams(I, varargin{:});
    end
else
    interp = 'bilinear';
    [outputView, scaleFactor, fillValues, method] = parseInputsCodegenParams(I, varargin{:});
end

%--------------------------------------------------------------------------
function [outputView, scaleFactor, fillValues, method] = ...
                                            parseInputsCodegenParams(I, varargin)
defaults = getDefaults();
pvPairs = struct( ...
    'OutputView',      uint32(0), ...
    'ScaleFactor',      uint32(0), ...
    'FillValues',   uint32(0));

% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);
pstruct = coder.internal.parseParameterInputs(pvPairs, ...
        poptions, varargin{:});
outputView = coder.internal.getParameterValue(pstruct.OutputView, defaults.OutputView, varargin{:});
scaleFactorVal = double(coder.internal.getParameterValue(pstruct.ScaleFactor, defaults.ScaleFactor, varargin{:}));
validateScaleFactor(scaleFactorVal);
if isscalar(scaleFactorVal)
    scaleFactor = [scaleFactorVal, scaleFactorVal];
else
    scaleFactor = scaleFactorVal;
end
fillValues = coder.internal.getParameterValue(pstruct.FillValues, defaults.FillValues, varargin{:});
outputView = verifyFillValuesAndOutputView(I, fillValues, outputView);
method = defaults.Method;

%--------------------------------------------------------------------------
function defaultValues = getDefaults()
defaultValues.OutputView = 'same';
defaultValues.Interp = 'bilinear';
defaultValues.Method = 'approximate';
defaultValues.FillValues = 0;
defaultValues.ScaleFactor = 1;

%--------------------------------------------------------------------------
function tf = validateOutputView(outputView)
validateattributes(outputView, {'char'}, {'vector'}, mfilename, 'OutputView');
tf = true;
        
%--------------------------------------------------------------------------
function tf = validateInterpMethod(method)
vision.internal.inputValidation.validateInterp(method);
tf = true;

%--------------------------------------------------------------------------
function outputView = validateOutputViewPartial(outputView)
outputView = ...
   validatestring(outputView, {'full', 'valid', 'same'}, mfilename, 'OutputView');

%--------------------------------------------------------------------------
function tf = validateScaleFactor(scaleFactor)
if ~isscalar(scaleFactor)
    validateattributes(scaleFactor, {'single','double'}, ...
        {'vector', 'nonsparse', 'real', 'numel', 2, 'positive'}, mfilename, 'ScaleFactor');
else
    validateattributes(scaleFactor, {'single','double'}, ...
        {'nonsparse', 'real', 'scalar', 'positive'}, mfilename, 'ScaleFactor');
end
tf = true;

%--------------------------------------------------------------------------
function method = validateMethod(value)
method = validatestring(value, {'exact','approximate'}, mfilename, 'Method');

%--------------------------------------------------------------------------
function tf = isSimMode()
tf = isempty(coder.target);

%--------------------------------------------------------------------------
function outputViewOut = verifyFillValuesAndOutputView(I, fillValues, outputView)
if ~(isscalar(fillValues) && fillValues == 0)
    vision.internal.inputValidation.validateFillValues(fillValues, I);
end
if ~strcmp(outputView, 'same')
    outputViewOut = validateOutputViewPartial(outputView);
else
    outputViewOut = outputView;
end