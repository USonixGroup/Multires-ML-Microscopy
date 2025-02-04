function descriptor = scanContextDescriptor(ptCloud, varargin)

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen

isSimMode = isempty(coder.target);

if isSimMode
    params = vision.internal.pc.parseInputsScanContextDescriptorSim(ptCloud, varargin{:});
    descriptor = vision.internal.scanContextDescriptor(...
        ptCloud.Location, params.NumBins(1), params.NumBins(2), ...
        params.MinPointsPerBin, params.SensorOrigin, params.RadialRange);
else
    params = parseInputsCodegen(ptCloud, varargin{:});
    descriptor = vision.internal.buildable.scanContextDescriptorBuildable. ...
        scanContextDescriptor(ptCloud.Location, params);
end

end

%--------------------------------------------------------------------------
function params = parseInputsCodegen(ptCloud, varargin)
narginchk(1, 9);
% Set input parser
defaults = struct(...
    'NumBins',         [20 60], ...
    'MinPointsPerBin', 5,...
    'SensorOrigin',    [0 0],...
    'RadialRange',     [0 Inf]);

% Define parser mapping struct
pvPairs = struct( ...
    'NumBins',         uint32(0), ...
    'MinPointsPerBin', uint32(0), ...
    'SensorOrigin',    uint32(0), ...
    'RadialRange',     uint32(0));

% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

% Parse PV pairs
pstruct = coder.internal.parseParameterInputs(pvPairs, ...
    poptions, varargin{:});
% Extract inputs
numBins         = coder.internal.getParameterValue(pstruct.NumBins, defaults.NumBins, varargin{:});
minPointsPerBin = coder.internal.getParameterValue(pstruct.MinPointsPerBin, defaults.MinPointsPerBin, varargin{:});
sensorOrigin    = coder.internal.getParameterValue(pstruct.SensorOrigin, defaults.SensorOrigin, varargin{:});
radialRange     = coder.internal.getParameterValue(pstruct.RadialRange, defaults.RadialRange, varargin{:});

validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloud');
validateattributes(numBins, {'numeric'}, {'real', 'positive', 'integer', 'numel', 2}, mfilename, 'NumBins');
validateattributes(minPointsPerBin, {'numeric'}, {'scalar', 'real','positive','integer'}, mfilename, 'MinPointsPerBin');
validateattributes(sensorOrigin, {'numeric'}, {'finite', 'real', 'numel', 2}, mfilename, 'SensorOrigin');
vision.internal.pc.validateRange(radialRange);

params.NumBins          = numBins;
params.MinPointsPerBin  = minPointsPerBin;

% Cast sensor origin and radial range to the type of points
params.SensorOrigin = cast(sensorOrigin, 'like', ptCloud.Location);
params.RadialRange  = cast(radialRange, 'like', ptCloud.Location);
end
