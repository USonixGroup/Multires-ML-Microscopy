function [labels, numClusters] = segmentLidarData(ptCloud, distThreshold, varargin)
%

% Copyright 2017-2024 The MathWorks, Inc.

%#codegen

narginchk(2, 5);

% Validate the 1st argument
validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloud');
coder.internal.errorIf(ismatrix(ptCloud.Location), ...
    'vision:pointcloud:organizedPtCloudOnly');

% Validate the 2nd argument
validateattributes(distThreshold, {'single','double'}, ...
    {'scalar', 'real', 'nonnegative','nonnan','nonsparse'}, mfilename, 'distThreshold');

distThreshold = double(distThreshold);

% Validate the optional 3rd argument
[angleThreshold, firstArg] = parseOptionalArg(varargin{:});

% Populate range data
if isempty(ptCloud.RangeData)
    ptCloud.RangeData = vision.internal.convertFromCartesianToSphericalCoordinate(ptCloud.Location);
end

isSim = isempty(coder.target);

[minPoints, maxPoints] = parseInputs(isSim, varargin{firstArg:end});

if isSim
    [labels, numClusters] = visionLabelRangeData(ptCloud.Location, ...
        ptCloud.RangeData, distThreshold, angleThreshold, minPoints, maxPoints);
else
    [labels, numClusters] = visionLabelRangeDataCodegen(ptCloud, distThreshold, angleThreshold, minPoints, maxPoints);
end

end

%--------------------------------------------------------------------------
function [minPoints, maxPoints] = parseInputs(isSim, varargin)

if isSim
    numClusterPoints = parseNameValueInputsSim(varargin{:});
else
    numClusterPoints = parseNameValueInputsCodegen(varargin{:});
end

[minPoints, maxPoints] = validateNumClusterPoints(numClusterPoints);

end

%--------------------------------------------------------------------------
function numClusterPoints = parseNameValueInputsSim(params)

arguments                                %#ok
    params.NumClusterPoints = [1, Inf];
end

numClusterPoints = params.NumClusterPoints;

end

%--------------------------------------------------------------------------
function numClusterPoints = parseNameValueInputsCodegen(varargin)

params = struct('NumClusterPoints', [uint32(0), uint32(0)]);

popt = struct('CaseSensitivity', false, 'StructExpand', true, 'PartialMatching', false);

optarg = eml_parse_parameter_inputs(params, popt, varargin{:});
numClusterPoints = eml_get_parameter_value(optarg.NumClusterPoints, [1, Inf], varargin{:});

end

%--------------------------------------------------------------------------
function [angleThreshold, firstArg] = parseOptionalArg(varargin)

if isempty(varargin) || isstring(varargin{1}) || ischar(varargin{1})
    angleThreshold = 0.0873;
    firstArg = 1;
else
    angleThreshDegrees = varargin{1};
    validateattributes(angleThreshDegrees, {'single','double'}, ...
        {'scalar', 'real', 'nonnegative', '<=', 180,'nonsparse'}, ...
        mfilename, 'angleThreshold');
    % Convert to radians
    angleThreshold = double(angleThreshDegrees) * pi / 180;
    firstArg = 2;
end

end

%--------------------------------------------------------------------------
function [minPoints, maxPoints] = validateNumClusterPoints(numClusterPoints)

if isscalar(numClusterPoints)
    validateattributes(numClusterPoints, {'numeric'}, ...
        {'nonnan', 'nonsparse', 'real', 'positive', 'integer'}, ...
        mfilename, 'NumClusterPoints');
    minPoints = uint32(numClusterPoints);
    maxPoints = intmax('uint32');
else
    validateattributes(numClusterPoints, {'numeric'}, ...
        {'nonnan', 'nonsparse', 'real', 'positive', 'increasing', ...
        'numel', 2}, mfilename, 'NumClusterPoints');

    minPoints = numClusterPoints(1);
    maxPoints = numClusterPoints(2);

    % Verify that the inputs are integers or Inf for maxPoints
    isValid = (minPoints == floor(minPoints)) && ...
        (isinf(maxPoints) || maxPoints == floor(maxPoints));
    if ~isValid
        coder.internal.error('vision:pointcloud:mustBeIntegerNumClusterPoints');
    end
    minPoints = uint32(minPoints);
    maxPoints = uint32(maxPoints);
end

end

%--------------------------------------------------------------------------
function [labels, numClusters] = visionLabelRangeDataCodegen(ptCloud, distThreshold, angleThreshold, minPoints, maxPoints)
coder.inline("always");

if coder.internal.preferMATLABHostCompiledLibraries()
    [labels, numClusters] = vision.internal.buildable.visionLabelRangeDataBuildable.visionLabelRangeDataCore(ptCloud.Location, ...
        ptCloud.RangeData, distThreshold, angleThreshold, minPoints, maxPoints);
else % Use portable code generation
    [labels, numClusters] = vision.internal.codegen.pc.visionLabelRangeDataCore(ptCloud.Location, ...
        ptCloud.RangeData, distThreshold, angleThreshold, minPoints, maxPoints);
end
end
