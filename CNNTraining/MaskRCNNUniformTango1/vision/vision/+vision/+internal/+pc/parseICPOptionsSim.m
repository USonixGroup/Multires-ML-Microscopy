%vision.internal.pc.parseICPOptionsSim Parse ICP algorithm options for simulation

% Copyright 2020-2023 The MathWorks, Inc.
function [metric, doExtrapolate, inlierRatio, inlierDistance, maxIterations, ...
    tolerance, initTform, verbose, useDegree] = parseICPOptionsSim(fcnName, args)

arguments
    fcnName
    args.Metric                                                        = "pointToPoint";
    args.Extrapolate                                                   = [];
    args.InlierRatio {iValidateInlierRatio(args.InlierRatio, fcnName)} = [];
    args.MaxIterations                                                 = [];
    args.Tolerance {iValidateTolerance(args.Tolerance, fcnName)}       = [0.01 .5];
    args.InitialTransform {iValidateTransform(args.InitialTransform, fcnName)} = [];
    args.Verbose(1,1) {iValidateLogical(args.Verbose, fcnName)}        = false;
    args.UseDegree(1,1) {iValidateLogical(args.UseDegree, fcnName)}    = true;
    args.InlierDistance {iValidateInlierDistance(args.InlierDistance, fcnName)} = [];
end

metric         = iValidateMetric(args.Metric, fcnName);
maxIterations  = iValidateMaxIterations(args.MaxIterations, fcnName);
inlierRatio    = args.InlierRatio;
inlierDistance = args.InlierDistance;
tolerance      = args.Tolerance;
initTform      = args.InitialTransform;
verbose        = args.Verbose;
useDegree      = args.UseDegree;

doExtrapolate = iValidateExtrapolate(args.Extrapolate, metric, fcnName);

if isempty(inlierRatio) && isempty(inlierDistance)
    inlierRatio = 1.0;
elseif ~isempty(inlierRatio) && ~isempty(inlierDistance)
    error(message('vision:pointcloud:inlierRatioOrDistance'))
end

end

%--------------------------------------------------------------------------
function metric = iValidateMetric(x, fcnName)

validateattributes(x, {'char','string'}, {'scalartext'});
validMetrics = ["pointToPoint", "pointToPlane", "planeToPlane", "pointToPlaneWithColor", "planeToPlaneWithColor"];
metric = validatestring(x, validMetrics, fcnName, 'Metric');
end

%--------------------------------------------------------------------------
function doExtrapolate = iValidateExtrapolate(extrapolateValue, metric, fcnName)

if isempty(extrapolateValue) % Extrapolate is not specified by the user.
    doExtrapolate = false;
else % Extrapolate is specified by the user.
    warning(message('vision:pointcloud:extrapolateDeprecation'));
    
    validateattributes(extrapolateValue, {'logical'}, {'scalar', 'binary'}, fcnName, ...
    'Extrapolate');

    unsupportedMetrics = ["planeToPlane", "pointToPlaneWithColor", "planeToPlaneWithColor"];
    if extrapolateValue && any(strcmpi(metric, unsupportedMetrics))
        error(message('vision:pointcloud:extrapolateNotSupported'));
    end

    doExtrapolate = extrapolateValue;
end
end

%--------------------------------------------------------------------------
function iValidateLogical(x, fcnName)

validateattributes(x, {'logical'}, {'scalar', 'binary'}, fcnName);
end

%--------------------------------------------------------------------------
function iValidateInlierRatio(x, fcnName)
if ~isempty(x)
    validateattributes(x, {'single', 'double'}, {'real','scalar','>',0,'<=',1}, ...
        fcnName, 'InlierRatio');
end
end

%--------------------------------------------------------------------------
function iValidateInlierDistance(x, fcnName)
    if ~isempty(x)
        validateattributes(x, {'single', 'double'}, ...
            {'scalar', 'real', 'positive', 'finite'}, ...
            fcnName, 'InlierDistance');
    end
end

%--------------------------------------------------------------------------
function x = iValidateMaxIterations(x, fcnName)

if ~isempty(x)
    validateattributes(x, {'single', 'double'}, ...
        {'real','scalar','integer','positive'}, fcnName, 'MaxIterations')
elseif strcmp(fcnName, 'pcregrigid')
    x = 20;
else
    x = 30;
end
end

%--------------------------------------------------------------------------
function iValidateTolerance(x, fcnName)

validateattributes(x, {'single', 'double'}, {'real','nonnegative','numel', 2}, ...
    fcnName, 'Tolerance');
end

%--------------------------------------------------------------------------
function iValidateTransform(x, fcnName)
if ~isempty(x)
    validateattributes(x, {'rigidtform3d', 'rigid3d', 'affine3d'}, ...
        {'scalar'}, fcnName, 'InitialTransform');
end
end
