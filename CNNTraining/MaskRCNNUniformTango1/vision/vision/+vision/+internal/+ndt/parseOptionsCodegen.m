%vision.internal.ndt.parseOptionsCodegen Parse NDT parameters for code generation

% Copyright 2020-2022 The MathWorks, Inc.

%#codegen
function [stepSize, outlierRatio, maxIterations, tolerance, initialTransform, verbose] = ...
    parseOptionsCodegen(isPcregisterndt, translation, varargin)

coder.internal.prefer_const(isPcregisterndt, translation, varargin{:});

% Get defaults
defaults = coder.internal.const(feval('vision.internal.ndt.parseOptionsSim'));

% Define parser mapping struct
if isPcregisterndt
    pvPairs = struct( ...
        'StepSize',         uint32(0), ...
        'OutlierRatio',     uint32(0), ...
        'MaxIterations',    uint32(0), ...
        'Tolerance',        uint32(0), ...
        'InitialTransform', uint32(0), ...
        'Verbose',          uint32(0));
else
    pvPairs = struct( ...
        'StepSize',         uint32(0), ...
        'OutlierRatio',     uint32(0), ...
        'MaxIterations',    uint32(0), ...
        'Tolerance',        uint32(0), ...
        'Verbose',          uint32(0));
end

% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

% Parse PV pairs
pstruct = coder.internal.parseParameterInputs(pvPairs, ...
    poptions, varargin{:});

% Extract inputs
stepSize          = coder.internal.getParameterValue(pstruct.StepSize, defaults.StepSize, varargin{:});
outlierRatio      = coder.internal.getParameterValue(pstruct.OutlierRatio, defaults.OutlierRatio, varargin{:});
maxIterations     = coder.internal.getParameterValue(pstruct.MaxIterations, defaults.MaxIterations, varargin{:});
tolerance         = coder.internal.getParameterValue(pstruct.Tolerance, defaults.Tolerance, varargin{:});
verbose           = coder.internal.getParameterValue(pstruct.Verbose, defaults.Verbose, varargin{:});

vision.internal.ndt.validateStepSize(stepSize);
vision.internal.ndt.validateOutlierRatio(outlierRatio);
vision.internal.ndt.validateMaxIterations(maxIterations);
vision.internal.ndt.validateTolerance(tolerance);
vision.internal.ndt.validateLogical(verbose);

% If the source is pcregisterndt, validate 'InitialTransform', else return
% a default rigidtform3d object.
if isPcregisterndt
    defaultTform = rigidtform3d(eye(3),translation);
    initialTransform = coder.internal.getParameterValue(pstruct.InitialTransform, defaultTform, varargin{:});
    vision.internal.ndt.validateTform(initialTransform, false);
else
    initialTransform = rigidtform3d;
end
end