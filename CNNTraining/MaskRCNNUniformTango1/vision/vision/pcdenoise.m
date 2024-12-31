function [ptCloudOut, inlierIndices, outlierIndices] = pcdenoise(ptCloudIn, varargin)

% Copyright 2014-2023 The MathWorks, Inc.

%#codegen

narginchk(1, 7);

% Parse the inputs
[numNeighbors, threshold, organizedOutput] = validateAndParseInputs(ptCloudIn, varargin{:});

isOrganized = ~ismatrix(ptCloudIn.Location);
if organizedOutput && isOrganized
    outputSize = 'full';
    if isGPUCodegen
        error(message('vision:pointcloud:unsupportedPreserveStructureGPU'))
    end
else
    outputSize = 'selected';
end

if ~isGPUCodegen()
    if ~ismatrix(ptCloudIn.Location)
        points = reshape(ptCloudIn.Location, [], 3);
    else
        points = ptCloudIn.Location;
    end
    
    % Compute the mean distance to neighbors for every point, and exclude the
    % query point itself
    [~, dists, valids] = multiQueryKNNSearchImpl(ptCloudIn, points, numNeighbors+1);
    % This multi-query KNN search uses exact search so every query should
    % return the same number of neighbors if the query is a valid point
    actualNums = double(max(valids(:))-1);
    meanDist = sum(dists(2:actualNums+1, :),1)/actualNums;
    
    isValidPoints = isfinite(points);
    isValidPoints = sum(isValidPoints,2)==3;
    meanDist(~isValidPoints) = NaN;
    
    % Compute the threshold
    finiteMeanDist = meanDist(isfinite(meanDist));
    
    meanD = mean(finiteMeanDist);
    stdD = std(finiteMeanDist);
    distThreshold = meanD+threshold*stdD;

    % Select the inliers
    tf = meanDist <= distThreshold;
    inlierIndices = uint32(find(tf));
    
    [loc, color, nv, intensity, rangeData] = subsetImpl(ptCloudIn, inlierIndices, outputSize);
else
    % GPU codegen implementation for pcdenoise

    % Assign inputs
    inpLocations = ptCloudIn.Location;
    inpColor = ptCloudIn.Color;
    inpIntensity = ptCloudIn.Intensity;
    inpNormals = ptCloudIn.Normal;
    inpRangeData = ptCloudIn.RangeData;
    
    [loc,color,intensity,nv,rangeData,inlierIndices,outlierIndicesGPU] = ...
        vision.internal.codegen.gpu.pcdenoiseGpuImpl(inpLocations,inpColor,inpIntensity,...
        inpNormals,inpRangeData,numNeighbors,threshold);
end

ptCloudOut = pointCloud(loc, 'Color', color, 'Normal', nv, 'Intensity', intensity);
ptCloudOut.RangeData = rangeData;

if nargout == 3
    if ~isGPUCodegen()
        outlierIndices = uint32(find(~tf));
    else
        outlierIndices = outlierIndicesGPU;
    end
end

end

function [numNeighbors, threshold, organizedOutput] = validateAndParseInputs( ...
    ptCloudIn, varargin)

% Validate the first argument

coder.internal.errorIf(~isa(ptCloudIn, 'pointCloud') || ~isscalar(ptCloudIn), ...
    'vision:pointcloud:notPointCloudObject', 'ptCloudIn');

% Validate and parse optional inputs

if isSimMode()
    parser = inputParser;
    parser.CaseSensitive = false;
    parser.FunctionName  = mfilename;
    
    parser.addParameter('NumNeighbors', 4, @(x)validateattributes(x, ...
        {'single', 'double'}, {'scalar','integer','positive'}));
    parser.addParameter('Threshold', 1.0, @(x)validateattributes(x, ...
        {'single', 'double'}, {'scalar','real','finite','positive'}));
    parser.addParameter('PreserveStructure', false, ...
        @(x)vision.internal.inputValidation.validateLogical(x, ...
        'PreserveStructure'));
    
    parser.parse(varargin{:});
    
    numNeighbors = parser.Results.NumNeighbors;
    threshold = parser.Results.Threshold;
    organizedOutput = parser.Results.PreserveStructure;
else
    pvPairs = struct(...
        'numNeighbors', uint32(0), ...
        'threshold', uint32(0), ...
        'PreserveStructure', uint32(0));

    properties = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', false);
    optarg = eml_parse_parameter_inputs(pvPairs, properties, varargin{:});
    
    numNeighbors = eml_get_parameter_value(optarg.numNeighbors, 4, ...
        varargin{:});
    threshold = eml_get_parameter_value(optarg.threshold, 1.0, ...
        varargin{:});
    organizedOutput = eml_get_parameter_value(optarg.PreserveStructure, false, ...
        varargin{:}); 
    
    validateattributes(numNeighbors, {'single', 'double'}, ...
        {'scalar','integer','positive','nonsparse'});
    validateattributes(threshold, {'single', 'double'}, ...
        {'scalar','real','finite','nonsparse','positive'});
    vision.internal.inputValidation.validateLogical(organizedOutput, 'PreserveStructure');

    eml_invariant(eml_is_const(organizedOutput),...
        eml_message('vision:pointcloud:mustBeConstant', "'PreserveStructure'"));
end
end

function flag = isSimMode()
flag = isempty(coder.target);
end

function flag = isGPUCodegen()
flag = coder.gpu.internal.isGpuEnabled;
end