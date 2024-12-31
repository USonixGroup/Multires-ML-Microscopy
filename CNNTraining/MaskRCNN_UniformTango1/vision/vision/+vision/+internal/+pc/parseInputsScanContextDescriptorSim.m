%vision.internal.pc.parseInputsScanContextDescriptorSim Parse scanContextDescriptor options for simulation

% Copyright 2020-2022 The MathWorks, Inc.

function params = parseInputsScanContextDescriptorSim(ptCloud, params)

arguments
    ptCloud {validatePtCloud}

    params.NumBins {validateNumBins} = [20 60];
    params.MinPointsPerBin {validateMinPointsPerBin} = 5;
    params.SensorOrigin {validateSensorOrigin} = [0 0];
    params.RadialRange {vision.internal.pc.validateRange} = [0 Inf];
end

% Cast sensor origin and radial range to the type of points
params.SensorOrigin = cast(params.SensorOrigin, 'like', ptCloud.Location);
params.RadialRange = cast(params.RadialRange, 'like', ptCloud.Location);
end

function validatePtCloud(in)
validateattributes(in, {'pointCloud'}, {'scalar'}, 'scanContextDescriptor', 'ptCloud');
end

function validateNumBins(in)
validateattributes(in, {'numeric'}, {'real', 'positive', 'integer', 'numel', 2}, ...
    'scanContextDescriptor', 'NumBins');
end

function validateMinPointsPerBin(in)
validateattributes(in, {'numeric'}, {'scalar', 'real','positive','integer'}, ...
    'scanContextDescriptor', 'MinPointsPerBin');
end

function validateSensorOrigin(in)
validateattributes(in, {'numeric'}, {'finite', 'real', 'numel', 2}, ...
    'scanContextDescriptor', 'SensorOrigin');
end
