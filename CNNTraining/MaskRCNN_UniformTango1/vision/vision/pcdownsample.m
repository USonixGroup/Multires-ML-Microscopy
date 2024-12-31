function [ptCloudOut, indices] = pcdownsample(ptCloudIn, varargin)
%

% Copyright 2014-2024 The MathWorks, Inc.

%#codegen

if nargin > 1
    varargin_ = cell(size(varargin));
    [varargin_{:}] = convertStringsToChars(varargin{:});
end

narginchk(3, 5);

% Validate the first argument
validateattributes(ptCloudIn, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloudIn');

% Validate the method argument
strMethod = validatestring(varargin_{1}, ...
    {'random','gridAverage','gridNearest','nonuniformGridSample'}, mfilename);

% Validate name-value pair
if nargin > 2
    organizedOutput = parseOptionalInputs(varargin{3:end});

    if organizedOutput && ~ismatrix(ptCloudIn.Location)
        outputSize = 'full';
        if isGPUTarget
            error(message('vision:pointcloud:unsupportedPreserveStructureGPU'))
        end
    else
        outputSize = 'selected';
    end
else
    outputSize = 'selected';
end

coder.gpu.kernelfun;
if strncmpi(strMethod, 'random', 1)
    % Validate the third argument
    percentage = varargin_{2};
    validateattributes(percentage, {'single','double'}, {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', '>=', 0, '<=', 1}, mfilename, 'percentage');

    K = round(ptCloudIn.Count*percentage);

    if ~isGPUTarget
        indices = vision.internal.samplingWithoutReplacement(ptCloudIn.Count, K);
        [points, color, normal, intensity, rangeData] = subsetImpl(ptCloudIn, indices, outputSize);
    else
        % This is the GPU implementation for random sampling of the points
        % without replacement. The first API uses NVIDIA's cuRand library
        % to generate random numbers and the second API uses the sequence
        % to select the numbers. Note: The output of the GPU impl of
        % 'random' may not be the same as simulation output.
        indices = vision.internal.codegen.gpu.samplingWithoutReplacement(ptCloudIn.Count,K);
        [points, color, normal, intensity, rangeData] = vision.internal.codegen.gpu.indexBasedSelection(ptCloudIn.Location, ...
            ptCloudIn.Color, ...
            ptCloudIn.Normal, ...
            ptCloudIn.Intensity, ...
            ptCloudIn.RangeData, ...
            indices);
    end
elseif strncmpi(strMethod, 'gridAverage', 5)
    nargoutchk(0, 1)

    % Validate the third argument
    gridStep = varargin_{2};
    validateattributes(gridStep, {'single','double'}, {'real', 'nonsparse', 'scalar', 'nonnan', 'positive'}, mfilename, 'gridStep');

    % Apply grid filter to each property.
    if isSimMode()
        % Remove invalid points to determine a bounded volume
        pc = removeInvalidPoints(ptCloudIn);
        if pc.Count == 0
            ptCloudOut = pc;
            return;
        end
        rangeLimits = [pc.XLimits, pc.YLimits, pc.ZLimits];
        [points, color, normal, intensity, rangeData] = visionVoxelGridFilter(pc.Location, ...
            pc.Color, ...
            pc.Normal, ...
            pc.Intensity, ...
            pc.RangeData, ...
            gridStep, rangeLimits);
    else  % Code generation path.
        [points, color, normal, intensity, rangeData] = voxelGridFilterCodegen(ptCloudIn, gridStep);
    end
elseif strncmpi(strMethod, 'gridNearest', 5)
    % Validate the third argument
    gridStep = varargin_{2};
    validateattributes(gridStep, {'single','double'}, {'real', 'nonsparse', 'scalar', 'nonnan', 'positive'}, mfilename, 'gridStep');
    
    % Apply grid filter.
    if isSimMode()
        % Remove invalid points to determine a bounded volume
        [pc, validIndices] = removeInvalidPoints(ptCloudIn);
        if pc.Count == 0
            ptCloudOut = pc;
            indices = zeros(0,1);
            return;
        end
        rangeLimits = [pc.XLimits, pc.YLimits, pc.ZLimits];
        selectedIndices = voxelGridNearest(pc.Location, gridStep, rangeLimits);
        indices = validIndices(selectedIndices);
        [points, color, normal, intensity, rangeData] = subsetImpl(ptCloudIn, indices, outputSize);
    else
        coder.internal.error('vision:pointcloud:gridNearestCodegenNotSupported')
    end
else
    % Validate the third argument
    maxNumPoints = varargin_{2};
    % The maximum number of points needs to be at least 6 because we need
    % to compute normals using PCA.
    validateattributes(maxNumPoints, {'single','double'}, {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'integer', '>=', 6}, mfilename, 'maxNumPoints');

    % Remove invalid points to determine a bounded volume
    [pc, selectedPts] = removeInvalidPoints(ptCloudIn);

    % Apply nonuniformGridSample filter to location property and generate
    % normal if it does not exist
    if isSimMode()
        if ~isempty(pc.Normal)
            selectedIndices = visionNonUniformVoxelGridFilter(pc.Location, maxNumPoints);
            indices = selectedPts(selectedIndices);
            [points, color, tempNormal, intensity, rangeData] = subsetImpl(ptCloudIn, indices, outputSize);
        else
            [selectedIndices, tempNormal] = visionNonUniformVoxelGridFilter(pc.Location, maxNumPoints);
            indices = selectedPts(selectedIndices);
            [points, color, ~, intensity, rangeData] = subsetImpl(ptCloudIn, indices, outputSize);
        end
    else  % codegen mode
        if ~isempty(pc.Normal)
            [~, selectedIndices] = nonUniformVoxelGridFilterCodegen(pc, maxNumPoints);
            indices = selectedPts(selectedIndices);
            [points, color, tempNormal, intensity, rangeData] = subsetImpl(ptCloudIn, indices, outputSize);
        else
            [tempNormal, selectedIndices] = nonUniformVoxelGridFilterCodegen(pc, maxNumPoints);
            indices = selectedPts(selectedIndices);
            [points, color, ~, intensity, rangeData] = subsetImpl(ptCloudIn, indices, outputSize);
        end
    end

    if strcmpi(outputSize, 'full') && isempty(pc.Normal)
        pointsSize = size(points);
        normal = nan(pointsSize, 'like', tempNormal);

        numPoints = prod(pointsSize(1:2));

        % Assign the three dimensions of normals.
        normal(indices) = tempNormal(:,1);
        normal(indices+numPoints) = tempNormal(:,2);
        normal(indices+2*numPoints) = tempNormal(:,3);
    else
        normal = tempNormal;
    end
end

ptCloudOut = pointCloud(points, 'Color', color, 'Normal', normal, 'Intensity', intensity);
ptCloudOut.RangeData = rangeData;
end

%==========================================================================
% Codegen support flag
%==========================================================================
function flag = isSimMode()
flag = isempty(coder.target);
end

%==========================================================================
% GPU codegen support flag
%==========================================================================
function flag = isGPUTarget()
flag = coder.gpu.internal.isGpuEnabled;
end

%==========================================================================
% Codegen function for gridAverage method
%==========================================================================
function [points, color, normal, intensity, rangeData] = voxelGridFilterCodegen(ptCloudIn, gridStep)
coder.inline("always");

% MATLAB Coder code generation path
if ~isGPUTarget
    % Remove invalid points to determine a bounded volume.
    pc = removeInvalidPoints(ptCloudIn);
    if pc.Count == 0
        points = pc.Location; color = pc.Color; normal = pc.Normal; intensity = pc.Intensity; rangeData = pc.RangeData;
        return;
    end
    if coder.internal.preferMATLABHostCompiledLibraries() % Shared Library codegen
        rangeLimits = [pc.XLimits, pc.YLimits, pc.ZLimits];
        [points, color, normal, intensity, rangeData] = vision.internal.buildable.voxelGridFilterBuildable.voxelGridFilter(pc.Location, ...
            pc.Color, ...
            pc.Normal, ...
            pc.Intensity, ...
            pc.RangeData, ...
            gridStep, rangeLimits);
    else % Portable codegen M impl
        [points, color, normal, intensity, rangeData] = vision.internal.codegen.pc.voxelGridFilter(pc.Location, ...
            pc.Color, ...
            pc.Normal, ...
            pc.Intensity, ...
            pc.RangeData, ...
            gridStep, []);
    end

    % GPU Code generation path
else
    % This is the call for the GPU impl of voxel grid filter.
    % The implementation handles the invalid points by not
    % considering them, therefore the call to 'removeInvalidPoints'
    % is skipped for GPU implementation.
    [points, color, normal, intensity, rangeData] = vision.internal.codegen.gpu.voxelGridFilter(ptCloudIn.Location, ...
        ptCloudIn.Color, ...
        ptCloudIn.Normal, ...
        ptCloudIn.Intensity, ...
        ptCloudIn.RangeData, ...
        gridStep);
end
end

%==========================================================================
% Codegen function for nonUniformGridSample method
%==========================================================================
function [normal, selectedIndices] = nonUniformVoxelGridFilterCodegen(ptCloud, maxNumPoints)
coder.inline("always");
% Display warning if nonUniformGridSample method is selected for GPU codegen.
if isGPUTarget
    coder.gpu.internal.diagnostic('gpucoder:diagnostic:PcdownsampleUnsupportedMethod');
end

% MATLAB Coder Shared Library code generation path
if coder.internal.preferMATLABHostCompiledLibraries()
    if ~isempty(ptCloud.Normal)
        selectedIndices = vision.internal.buildable.nonUniformVoxelGridFilterBuildable.nonUniformVoxelGridFilter(ptCloud.Location, maxNumPoints);
        normal = nan(1, 'like', ptCloud.Normal);
    else
        [selectedIndices, normal] = vision.internal.buildable.nonUniformVoxelGridFilterBuildable.nonUniformVoxelGridFilter(ptCloud.Location, maxNumPoints);
    end
else % MATLAB Coder Portable code generation path
    if ~isempty(ptCloud.Normal)
        selectedIndices = vision.internal.codegen.pc.nonUniformVoxelGridFilter(ptCloud.Location, maxNumPoints);
        normal = nan(1, 'like', ptCloud.Normal);
    else
        [selectedIndices, normal] = vision.internal.codegen.pc.nonUniformVoxelGridFilter(ptCloud.Location, maxNumPoints);
    end
end
end

%==========================================================================
% Optional Input Validation
%==========================================================================
function organizedOutput = parseOptionalInputs(varargin)

if coder.target()
    organizedOutput = parseInputsCodegen(varargin{:});
else
    organizedOutput = parseInputsSim(varargin{:});
end

vision.internal.inputValidation.validateLogical(organizedOutput,...
    'PreserveStructure');
end


%--------------------------------------------------------------------------
function organizedOutput = parseInputsSim(args)

arguments                               %#ok
    args.PreserveStructure = false;
end

organizedOutput = args.PreserveStructure;
end

%--------------------------------------------------------------------------
function organizedOutput = parseInputsCodegen(varargin)

% Set input parser
defaults = struct('PreserveStructure', false);
% Define parser mapping struct
pvPairs = struct('PreserveStructure', uint32(0));
% Specify parser options
poptions = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', true);

% Parse PV pairs
pstruct = coder.internal.parseParameterInputs(pvPairs, ...
    poptions, varargin{:});
% Extract input
organizedOutput = coder.internal.getParameterValue(pstruct.PreserveStructure,...
    defaults.PreserveStructure, varargin{:});

vision.internal.inputValidation.validateLogical(organizedOutput, 'PreserveStructure');

eml_invariant(eml_is_const(organizedOutput),...
    eml_message('vision:pointcloud:mustBeConstant', "'PreserveStructure'"));
end
