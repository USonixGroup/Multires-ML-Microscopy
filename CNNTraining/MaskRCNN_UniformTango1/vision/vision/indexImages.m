function index = indexImages(imgSet, varargin)

% Copyright 2017-2023 The MathWorks, Inc.

%#codegen

if isSimMode
    if nargin > 1
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    [bag, params] = parseInputs(imgSet, varargin{:});

    printer = vision.internal.MessagePrinter.configure(params.Verbose);

    printer.printMessage('vision:indexImages:startIndexing');
    printer.print('-------------------------------------------------------\n');

    if isempty(bag)

        % Disable vocab reduction warning. imgSet may be small.
        prevState    = warning('off','vision:bagOfFeatures:reducingVocabSize');
        resetWarning = onCleanup(@()warning(prevState));

        bag = bagOfFeatures(imgSet, 'TreeProperties', [1 20000], ...
            'PointSelection', 'Detector', ...
            'Upright', false, 'Verbose', params.Verbose, 'UseParallel', params.UseParallel);
    end

    index = invertedImageIndex(bag, 'SaveFeatureLocations', params.SaveFeatureLocations);
    addImages(index, imgSet, 'UseParallel', params.UseParallel,'Verbose', params.Verbose);

    printer.printMessage('vision:indexImages:indexingDone');
else
    index = vision.internal.codegen.indexImages(imgSet, varargin{:});
end

% -------------------------------------------------------------------------
function [bag, params] = parseInputs(imgSet, varargin)

validateattributes(imgSet, ...
    {'matlab.io.datastore.ImageDatastore'}, ...
    {'scalar'}, mfilename, 'imds',1);

if numel([imgSet.Files]) == 0
    error(message('vision:invertedImageIndex:emptyImageSet'));
end

d = getDefaultParameterValues();

p = inputParser();

p.addOptional('Bag', d.Bag, @checkBag);

p.addParameter('Verbose', d.Verbose, ...
    @(x)vision.internal.inputValidation.validateLogical(x,'Verbose'));

p.addParameter('UseParallel', d.UseParallel, ...
    @(x)vision.internal.inputValidation.validateLogical(x,'UseParallel'));

p.addParameter('SaveFeatureLocations', d.SaveFeatureLocations, ...
    @(x)vision.internal.inputValidation.validateLogical(x,'SaveFeatureLocations'));

parse(p,varargin{:});

bag = p.Results.Bag;

params.Verbose              = logical(p.Results.Verbose);
params.UseParallel          = logical(p.Results.UseParallel);
params.SaveFeatureLocations = logical(p.Results.SaveFeatureLocations);

% -------------------------------------------------------------------------
function checkBag(bag)

validateattributes(bag, {'bagOfFeatures'},{},mfilename,'bag');

% -------------------------------------------------------------------------
function defaults = getDefaultParameterValues()
defaults.Bag                  = [];
defaults.Verbose              = true;
defaults.UseParallel          = vision.internal.useParallelPreference();
defaults.SaveFeatureLocations = true;

%--------------------------------------------------------------------------
function mode = isSimMode()
mode = isempty(coder.target);