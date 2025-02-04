function txt = ocr(im, varargin)

% Copyright 2018-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

if isa(im, "matlab.io.Datastore") || isa(im, "matlab.io.datastore.Datastore")

    isGtruthTextRequired = false;
    vision.internal.ocr.validateOCRDataStoreInput(im, isGtruthTextRequired);
    imds = transform(im, @(data)vision.internal.ocr.validateOCRDataStoreContents(data, isGtruthTextRequired));

    txt = {};
    while hasdata(imds)
        data = read(imds);
        [I, roi, ~] = deal(data{:});
        txt{end+1}  = ocrImpl(I, roi, varargin{:}, Workflow="evaluation");
    end
    txt = txt';
else
    txt = ocrImpl(im, varargin{:});
end

% -------------------------------------------------------------------------
function txt = ocrImpl(I, varargin)

if isempty(coder.target)
    if nargin > 1
    [varargin{:}] = convertStringsToChars(varargin{:});
    end
end
[roi, hasROI, params] = parseInputs(I,varargin{:});


if islogical(I) && ~params.PreprocessBinaryImage  
    % Process binary images as-is if the PreprocessBinaryImage is false.
    % This by-passes tesseract's binarization stage.
    [rawtext, metadata] = tesseract(params, I, roi, hasROI);    
else    
    Iu8 = im2uint8(I);    
    img = vision.internal.ocr.convertRGBToGray(Iu8);
    
    [rawtext, metadata] = tesseract(params, img, roi, hasROI);    
end

txt = ocrText.create(rawtext, metadata, params.TextLayout);

% -------------------------------------------------------------------------
% Invoke Tesseract
% -------------------------------------------------------------------------
function [txt, ocrMetadata] = tesseract(params, Iu8, roi, hasROI)

[isSet, prefix] = unsetTessDataPrefix();

resetParameters = hasLanguageChanged(params.Language);
 
if vision.internal.ocr.isCodegen()
    tessOpts = codegenParseParams(params);
    if coder.internal.preferMATLABHostCompiledLibraries()
        [txt, ocrMetadata] = vision.internal.buildable.OCRBuildable.tesseract(tessOpts, Iu8, hasROI, resetParameters);
    else
        [txt, ocrMetadata] = vision.internal.buildable.OCRBuildablePortable.tesseract(tessOpts, Iu8, hasROI, resetParameters);
    end
else    
    tessOpts = parseParams(params);
	[txt, ocrMetadata] = tesseractWrapper(tessOpts, Iu8, hasROI, roi, resetParameters);
end

coder.extrinsic('setenv')
if isSimOrMex()
    if isSet
        setenv('TESSDATA_PREFIX',prefix)
    end
end

% -------------------------------------------------------------------------
% Return true if the input language does not match the cached language.
% -------------------------------------------------------------------------
function tf = hasLanguageChanged(language)
persistent cachedLanguage

language = convertToCacheableValue(language);

% used fixed size language string to support codegen
% with max length of 4096.
n = min(4096, numel(language));
if isempty(cachedLanguage)
    cachedLanguage = zeros(1,4096,'uint8');
    cachedLanguage(1:n) = cast(language(1:n), 'uint8');
end

if isequal(language(1:n), cachedLanguage(1:n))
    tf = false;
else
    % language has changed. update cached value.
    cachedLanguage(1:n) = cast(language(1:n), 'uint8');
    tf = true;
end

% -------------------------------------------------------------------------
function lang = convertToCacheableValue(lang)
% cached multiple languages as concatenated string
if isempty(coder.target) && iscell(lang)
    lang = [lang{:}];
end

% -------------------------------------------------------------------------
% Parse inputs.
% -------------------------------------------------------------------------
function [roi, hasROI, params] = parseInputs(I, varargin)

sz = size(I);
vision.internal.inputValidation.validateImage(I);

if mod(nargin-1,2) == 1
    hasROI = true;
    roi = int32(round(varargin{1}));
    checkROI(roi,sz(1:2));
else
    hasROI = false;
    roi = ones(0,4,'int32');
end

if vision.internal.ocr.isCodegen()
    if hasROI
        userInput = codegenParseInputs(varargin{2:end});
    else
        userInput = codegenParseInputs(varargin{:});
    end
else
    p = getInputParser();
    parse(p, varargin{:});
    userInput = p.Results;
    
    userInput.UsingCharacterSet = isempty(regexp([p.UsingDefaults{:} ''],...
        'CharacterSet','once'));
    
end

validTextLayout = checkLayout(userInput.LayoutAnalysis, 'LayoutAnalysis');

modelInput = userInput.Model;
parameterName = 'Model';
canUseFastModel = coder.const(true);
doAcceptMultipleModels = coder.const(true);
[validLanguage, isCustomLanguage] = vision.internal.ocr.checkModel(...
    modelInput, parameterName, doAcceptMultipleModels, canUseFastModel);

validWorkflow = checkWorkflow(userInput.Workflow);

if userInput.UsingCharacterSet    
    checkCharacterSet(userInput.CharacterSet);
end

checkPreprocessBinaryImage(userInput.PreprocessBinaryImage);

params = setParams(userInput, validLanguage, validTextLayout, ...
    isCustomLanguage, validWorkflow);

% -------------------------------------------------------------------------
% Parse inputs during codegen.
% -------------------------------------------------------------------------
function results = codegenParseInputs(varargin)
pvPairs = struct( ...
    'CharacterSet', uint32(0),...
    'PreprocessBinaryImage',  uint32(0),...
    'Workflow',       uint32(0), ...
    'Model',          uint32(0),...
    'LayoutAnalysis', uint32(0));

popt = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand'   , true, ...
    'PartialMatching', true);

defaults = getParamDefaults();

optarg = eml_parse_parameter_inputs(pvPairs, popt, varargin{:});

results.CharacterSet = eml_get_parameter_value(optarg.CharacterSet, ...
    defaults.CharacterSet, varargin{:});

results.PreprocessBinaryImage  = eml_get_parameter_value(optarg.PreprocessBinaryImage, ...
    defaults.PreprocessBinaryImage, varargin{:});

results.Workflow = eml_get_parameter_value(optarg.Workflow, defaults.Workflow, varargin{:});
results.Model  = eml_get_parameter_value(optarg.Model, defaults.Model, varargin{:});
results.LayoutAnalysis  = eml_get_parameter_value(optarg.LayoutAnalysis, ...
    defaults.LayoutAnalysis, varargin{:});

% UsingCharacterSet true if the user supplied one
results.UsingCharacterSet = logical(optarg.CharacterSet);

% Warning if a non-English language or custom language is specified.
% This warning is not applicable to seven-segemnt as it contains
% ASCII characterset.
if ~(strcmpi(results.Model,'english')||strcmpi(results.Model,'seven-segment'))    
    coder.internal.compileWarning('vision:ocr:codegenASCIIOnly');    
end

% -------------------------------------------------------------------------
function checkROI(roi,imageSize)

for i = 1:size(roi,1)
    vision.internal.detector.checkROI(roi(i,:),imageSize);
end

% -------------------------------------------------------------------------
function checkCharacterSet(list)

validateattributes(list, {'char'},{},mfilename,'CharacterSet'); % allow empty ''

if ~isempty(list)
    % make sure it's a vector
    validateattributes(list, {'char'},{'vector'},mfilename,'CharacterSet');
end

% -------------------------------------------------------------------------
function str = checkWorkflow(workflow)

str = validatestring(workflow,{'evaluation','inference'},mfilename,'Workflow');

% -------------------------------------------------------------------------
function str = checkLayout(layout, paramName)

str = validatestring(layout,{'auto','block','line','word','character','page', 'none'},...
    mfilename,paramName);

% -------------------------------------------------------------------------
function checkPreprocessBinaryImage(value)

validateattributes(value, {'numeric','logical'}, ...
    {'nonnan', 'scalar', 'real','nonsparse'}, mfilename, 'PreprocessBinaryImage');

% -------------------------------------------------------------------------
function defaults = getParamDefaults()

defaults.CharacterSet = coder.internal.const('');
defaults.PreprocessBinaryImage = true;

defaults.Workflow = coder.internal.const('inference');
defaults.Model  = coder.internal.const('english');
defaults.LayoutAnalysis = coder.internal.const('auto');

% -------------------------------------------------------------------------
function params = setParams(userInput, language, textLayout, ...
                            isCustomLanguage, workflow)

params.TextLayout        = textLayout;
params.Language          = language;
params.CharacterSet      = userInput.CharacterSet;
params.UsingCharacterSet = userInput.UsingCharacterSet;
params.IsCustomLanguage  = coder.internal.const(isCustomLanguage);
params.PreprocessBinaryImage = logical(userInput.PreprocessBinaryImage);
params.Workflow = workflow;

% -------------------------------------------------------------------------
% Parse tesseract parameters
% -------------------------------------------------------------------------
function tessOpts = parseParams(params)

% Specify tesseract variable names as the fields of setVariable. The
% variable values should be specified as strings.
tessOpts.setVariable.tessedit_pageseg_mode = getTextLayout(params);

if params.UsingCharacterSet
    tessOpts.setVariable.tessedit_char_whitelist = params.CharacterSet;
end

[tessdata, lang] = vision.internal.ocr.getModelInfo(params.Language, ...
    params.IsCustomLanguage);

tessOpts.tessdata     = tessdata;
tessOpts.lang         = lang;

% Specify tesseract initialization variables names as the fields of
% initVariable. The variable values should be specified as strings.
tessOpts.initVariable = [];

% OEM_TESSERACT_ONLY = 0; OEM_LSTM_ONLY = 1; 
% OEM_TESSERACT_LSTM_COMBINED = 2; OEM_DEFAULT = 3;
tessOpts.ocrEngineMode = 1;

% -------------------------------------------------------------------------
% codegen: Parse tesseract parameters
% -------------------------------------------------------------------------
function tessOpts = codegenParseParams(params)

textLayout = getTextLayout(params); 

if params.UsingCharacterSet
    charSet = params.CharacterSet;
else
    charSet = '';
end    

[tessdata, lang] = vision.internal.ocr.getModelInfo(params.Language, ...
    params.IsCustomLanguage);

tessOpts.textLayout   = textLayout;
tessOpts.characterSet = charSet;
tessOpts.tessdata     = tessdata;
tessOpts.lang         = lang;

% OEM_TESSERACT_ONLY = 0; OEM_LSTM_ONLY = 1; 
% OEM_TESSERACT_LSTM_COMBINED = 2; OEM_DEFAULT = 3;
tessOpts.ocrEngineMode = 1;

% -------------------------------------------------------------------------
% Return the parameter value used by tesseract to set the page segmentation
% mode (PSM). Setting other values for the page segmentation mode is not
% recommended.
% -------------------------------------------------------------------------
function textLayout = getTextLayout(params)

layout = getAutoTextLayout(params);

switch layout
    case 'page'
        textLayout = '3';
    case 'block'
        textLayout = '6';
    case 'line'
        textLayout = '7';
    case 'word'
        textLayout = '8';
    case 'character'
        textLayout = '10';
    case 'none'
        textLayout = '13';
    otherwise
        textLayout = '';  % codegen requires assignments for all paths               
end

% -------------------------------------------------------------------------
% Return text layout based on auto behavior.
% -------------------------------------------------------------------------
function layout = getAutoTextLayout(params)

if strcmpi(params.TextLayout, 'auto')
    if strcmpi(params.Workflow, 'evaluation')
        layout = 'none';
    elseif strcmpi(params.Language,'seven-segment')
        layout = 'block';
    else
        layout = 'page';
    end
else
    layout = params.TextLayout;
end

% -------------------------------------------------------------------------
% Return the inputParser used for parameter parsing. The inputParser is
% created once and stored in a persistent variable to improve performance.
% -------------------------------------------------------------------------
function parser = getInputParser()
persistent p;
if isempty(p)   
    defaults = getParamDefaults();
    p = inputParser();      
    addOptional(p, 'ROI', []);
    addParameter(p, 'CharacterSet', defaults.CharacterSet);
    addParameter(p, 'PreprocessBinaryImage', defaults.PreprocessBinaryImage);
    addParameter(p, 'Workflow', defaults.Workflow);
    addParameter(p, 'Model', defaults.Model);
    addParameter(p, 'LayoutAnalysis', defaults.LayoutAnalysis);
    
    parser = p;
else
    parser = p;
end

% -------------------------------------------------------------------------
% Clear the TESSDATA_PREFIX environment variable if it is set. This enables
% ocr to use the tessdata files specified using file paths instead of
% defaulting to the location of TESSDATA_PREFIX.
% -------------------------------------------------------------------------
function [isSet, prefix] = unsetTessDataPrefix()
coder.extrinsic('setenv','getenv')

if isSimOrMex()
    prefix = getenv('TESSDATA_PREFIX');
    if isempty(prefix)
        isSet = false;
    else
        setenv('TESSDATA_PREFIX','');
        isSet = true;
    end
else
    isSet  = false;
    prefix = '';
end

% -------------------------------------------------------------------------
% Check whether we are in sim or mex mode
% -------------------------------------------------------------------------
function tf = isSimOrMex()

tf = isempty(coder.target) || coder.target('MEX');
