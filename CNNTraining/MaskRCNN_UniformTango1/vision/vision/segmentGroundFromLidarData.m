function groundPtsIdx = segmentGroundFromLidarData(ptCloud, varargin)

% Copyright 2018-2023 The MathWorks, Inc.

%#codegen

validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloud');
coder.internal.errorIf(ismatrix(ptCloud.Location), 'vision:pointcloud:organizedPtCloudOnly'); 

% This algorithm requires at least two rows in an organized point cloud. The
% elevation angle is calculated using the difference value between range
% and pitch values from two rows in the point cloud.
coder.internal.errorIf(size(ptCloud.Location, 1) <= 1, 'vision:pointcloud:rowSizeMustBeGreaterThanOne');

[deltaAngle, seedAngle] = parseOptInputs(varargin{:});

deltaAngle = deltaAngle * pi / 180;
seedAngle  = seedAngle * pi / 180;

if isempty(ptCloud.RangeData) && ~isGPUTarget()
    ptCloud.RangeData = vision.internal.convertFromCartesianToSphericalCoordinate(ptCloud.Location);
end

repairDepthThresh = 1; % meters
if isSimMode()
    groundPtsIdx = visionLabelRangeDataGroundRemoval(ptCloud.RangeData, ...
        seedAngle, deltaAngle, repairDepthThresh);
% Check if GPU is enabled.  
elseif isGPUTarget()
    % Calling GPU specific implementation for segmentGroundFromLidarData.
    groundPtsIdx = vision.internal.codegen.gpu.segmentGroundFromLidarDataImpl...
        (ptCloud.Location,ptCloud.RangeData, seedAngle, deltaAngle, repairDepthThresh);

else
    groundPtsIdx = vision.internal.buildable.labelRangeDataGroundRemovalBuildable. ....
        labelRangeDataGroundRemoval(ptCloud.RangeData, ...
        seedAngle, deltaAngle, repairDepthThresh);
end

end

%==========================================================================
function [deltaAngle, seedAngle] = parseOptInputs(varargin)

% Set input parser
defaults = struct(...
    'ElevationAngleDelta',        5, ...
    'InitialElevationAngle',     30);

if isSimMode    
    parser = inputParser;
    parser.CaseSensitive = false;
    parser.addParameter('ElevationAngleDelta', defaults.ElevationAngleDelta);
    parser.addParameter('InitialElevationAngle', defaults.InitialElevationAngle);
    
    parser.parse(varargin{:});
    
    deltaAngle = parser.Results.ElevationAngleDelta;
    seedAngle  = parser.Results.InitialElevationAngle;
else
    % Define parser mapping struct
    pvPairs = struct( ...
        'ElevationAngleDelta',         uint32(0), ...
        'InitialElevationAngle',     uint32(0));
    
    % Specify parser options
    poptions = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', true);
    
    % Parse PV pairs
    pstruct = coder.internal.parseParameterInputs(pvPairs, ...
        poptions, varargin{:});
    % Extract inputs
    deltaAngle     = coder.internal.getParameterValue(pstruct.ElevationAngleDelta, defaults.ElevationAngleDelta, varargin{:});
    seedAngle      = coder.internal.getParameterValue(pstruct.InitialElevationAngle, defaults.InitialElevationAngle, varargin{:});
    
end
validateattributes(deltaAngle, {'single', 'double'}, ...
    {'scalar', 'real', 'nonnegative', 'nonnan', 'nonsparse'});
validateattributes(seedAngle, {'single', 'double'}, ...
    {'scalar', 'real', 'nonnegative', 'nonnan', 'nonsparse'});
end

function flag = isSimMode()

flag = isempty(coder.target);
end

% GPU codegen support flag
function flag = isGPUTarget()
    flag = coder.gpu.internal.isGpuEnabled;
end
