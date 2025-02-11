function detector = peopleDetectorACF(varargin)

% Copyright 2017-2023 The MathWorks, Inc.

if nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end

narginchk(0, 1);

if (isempty(varargin))
    name = 'inria-100x41';
else
    % validate user input
    name = checkModel(varargin{1});
end

% Get model and extract additional parameters
[model, ~, name] = loadModel(name);

params.ModelName = name;

% Channel related parameters
pPyramid = model.opts.pPyramid;
params.ModelSize          = round(model.opts.modelDs);
params.ModelSizePadded    = model.opts.modelDsPad;
params.Shrink             = pPyramid.pChns.shrink;
params.ChannelPadding     = pPyramid.pad;
params.Lambdas            = pPyramid.lambdas;
params.SmoothChannels     = pPyramid.smooth;
params.PreSmoothColor     = pPyramid.pChns.pColor.smooth;
params.NumUpscaledOctaves = pPyramid.nOctUp;
params.NumApprox          = pPyramid.nApprox;

% Parameters for gradient computation
params.gradient.FullOrientation       = model.opts.pPyramid.pChns.pGradMag.full;
params.gradient.NormalizationRadius   = model.opts.pPyramid.pChns.pGradMag.normRad;
params.gradient.NormalizationConstant = model.opts.pPyramid.pChns.pGradMag.normConst;

% Parameters for HOG computation
pGradHist = model.opts.pPyramid.pChns.pGradHist;
params.hog.NumBins   = pGradHist.nOrients;
params.hog.Normalize = pGradHist.useHog;

if isempty(pGradHist.binSize)
    params.hog.CellSize = params.Shrink;
else
    params.hog.CellSize = pGradHist.binSize;
end

switch pGradHist.softBin
    case 0
        % only interpolate orientation
        params.hog.Interpolation = 'Orientation';
    case 1
        % spatial and orientation interpolation.
        params.hog.Interpolation = 'Both';
end

params.hog.FullOrientation = params.gradient.FullOrientation;

% Training parameters
params.NumStages        = numel(model.opts.nWeak);
params.NegativeSamplesFactor = 10;
params.MaxWeakLearners  = model.opts.nWeak(end);

c = rmfield(model.clf, {'errs', 'losses'});
detector = acfObjectDetector(c, params);

%--------------------------------------------------------------------------
function [model, id, name] = loadModel(name)

modelLocation = fullfile(toolboxdir('vision'), 'visionutilities', 'classifierdata','acf');

[name, id] = getModelNameAndID(name);

if id == 1
    modelFile = fullfile(modelLocation, 'AcfInriaDetector.mat');
else
    modelFile = fullfile(modelLocation, 'AcfCaltech+Detector.mat');
end
data  = load(modelFile);
model = data.detector;

%--------------------------------------------------------------------------
function [name, id] = getModelNameAndID(name)

switch lower(name)
    case 'inria-100x41'
        name = 'inria-100x41';
        id   = 1;
    
    case 'caltech-50x21'
        name = 'caltech-50x21';
        id   = 2;    
end

%--------------------------------------------------------------------------
function valid = checkModel(model)
valid = validatestring(model,{'inria-100x41','caltech-50x21'},...
    mfilename, 'modelName');
