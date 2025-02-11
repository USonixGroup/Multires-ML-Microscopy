function [bins, binLocations] = pcbin(ptCloud, numBins, varargin)

% Copyright 2019-2024 The MathWorks, Inc.

%#codegen

narginchk(2, 5);

% This is needed since the BinOutput needs to be a constant in codegen
coder.internal.prefer_const( varargin{:} );

[ptCloud, numBins, spatialLimits, binOutput] = parseInputs(ptCloud, numBins, varargin{:});

% This is needed since the BinOutput needs to be a constant in codegen
if nargin >= 4
    coder.internal.assert(coder.internal.isConst(varargin{2}), 'vision:pointcloud:binOutputConstant');
end

if ~isGPUTarget()
    [XEdges, YEdges, ZEdges] = computeBinEdges(numBins, spatialLimits);

    unorganizedPointCloud = ismatrix(ptCloud.Location);

    if unorganizedPointCloud

        rowBins = discretize(ptCloud.Location(:,1), XEdges);
        colBins = discretize(ptCloud.Location(:,2), YEdges);
        heightBins = discretize(ptCloud.Location(:,3), ZEdges);

        idx = subscriptsToIndices(numBins, rowBins, colBins, heightBins);

        if binOutput
            bins = repmat({[]}, numBins);
            coder.varsize('bins{:}');
            for c = 1:length(idx)
                if ~isnan(idx(c))
                    M = bins{idx(c)};
                    bins{idx(c)} = vertcat(M, c);
                end
            end
        else
            % Assign the indices in the case of BinOutput = false
            bins = idx;
        end        

    else

        sizeOrganizedGridRow = size(ptCloud.Location,1);
        sizeOrganizedGridCol = size(ptCloud.Location,2);

        rowBins = discretize(ptCloud.Location(:,:,1), XEdges);
        colBins = discretize(ptCloud.Location(:,:,2), YEdges);
        heightBins = discretize(ptCloud.Location(:,:,3), ZEdges);

        idx = subscriptsToIndices(numBins, rowBins, colBins, heightBins);

        if binOutput
            bins = repmat({[]}, numBins);
            coder.varsize('bins{:}');
            [m,n] = ind2sub([sizeOrganizedGridRow sizeOrganizedGridCol], 1:numel(idx));

            for c = 1:numel(idx)
                if ~isnan(idx(c))
                    M = bins{idx(c)};
                    bins{idx(c)} = vertcat(M, [m(c), n(c)]);
                end
            end        
        else
            % Assign the indices in the case of BinOutput = false
            bins = idx;
        end

    end

    if nargout > 1
        if binOutput
            binLocations = repmat({[]}, numBins);
            coder.varsize('binLocations{:}');
            for z = 1:numel(ZEdges)-1
                for y = 1:numel(YEdges)-1
                    for x = 1:numel(XEdges)-1
                        binLocations{x,y,z} = [XEdges(x),XEdges(x+1); YEdges(y),YEdges(y+1); ZEdges(z), ZEdges(z+1)];
                    end
                end
            end
        else
            if unorganizedPointCloud
                binLocations = nan(ptCloud.Count, 6);
                numPoints = ptCloud.Count;
            else
                binLocations = nan(size(rowBins,1), size(rowBins,2), 6);
                numPoints = sizeOrganizedGridCol * sizeOrganizedGridRow;
            end

            nonNanLogical = ~isnan(rowBins) & ~isnan(colBins) & ~isnan(heightBins);
            nonNanIndices = find(nonNanLogical); 
            binLocations(nonNanIndices)                 = XEdges(rowBins(nonNanLogical));
            binLocations(nonNanIndices + numPoints)     = XEdges(rowBins(nonNanLogical)+1);
            binLocations(nonNanIndices + 2 * numPoints) = YEdges(colBins(nonNanLogical));
            binLocations(nonNanIndices + 3 * numPoints) = YEdges(colBins(nonNanLogical)+1);
            binLocations(nonNanIndices + 4 * numPoints) = ZEdges(heightBins(nonNanLogical));
            binLocations(nonNanIndices + 5 * numPoints) = ZEdges(heightBins(nonNanLogical)+1);        

        end
    end
else
    % GPU implementation of pcbin
    inpLocations = ptCloud.Location;
    [bins, binLocations] = vision.internal.codegen.gpu.pcbinImpl(double(inpLocations), numBins, spatialLimits, binOutput); 
end
end

function [XEdges, YEdges, ZEdges] = computeBinEdges(numBins, spatialLimits)

XEdges = linspace(spatialLimits(1,1), spatialLimits(1,2), numBins(1)+1);
YEdges = linspace(spatialLimits(2,1), spatialLimits(2,2), numBins(2)+1);
ZEdges = linspace(spatialLimits(3,1), spatialLimits(3,2), numBins(3)+1);

end

function [ptCloud, numBins,spatialLimits, binOutput] = parseInputs(ptCloud, numBins, varargin)

if  isSimMode()
    [ptCloud, numBins, spatialLimits, binOutput] = parseInputsSim(ptCloud, numBins, varargin{:});
else
    [ptCloud, numBins, spatialLimits, binOutput] = parseInputsCodegen(ptCloud, numBins, varargin{:});
end

numBins = double(numBins);
spatialLimits = double(spatialLimits);
end

function [ptCloud, numBins,spatialLimits, binOutput] = parseInputsSim(ptCloud,numBins,spatialLimits, pvpairs)
arguments
    ptCloud   {validatePointCloud(ptCloud)}
    numBins   {validateNumBins(numBins)}
    spatialLimits {validateSpatialLimits(spatialLimits)} = defineDefaultLimits(ptCloud);
    pvpairs.BinOutput {validateBinOutput(pvpairs.BinOutput)} = true;
end

binOutput = pvpairs.BinOutput;
end

function [ptCloud,numBins, spatialLimits, binOutput] =  parseInputsCodegen(ptCloud, numBins, varargin)

validatePointCloud(ptCloud);
validateNumBins(numBins);

optionalArgs = {'spatialLimits'};
pvPairNames = {'BinOutput'};
poptions = struct( ...
    'CaseSensitivity',false, ...
    'PartialMatching','unique', ...
    'StructExpand',false, ...
    'IgnoreNulls',false, ...
    'SupportOverrides',false);

defaults = struct(...
    'spatialLimits', defineDefaultLimits(ptCloud),...
    'BinOutput', true);

pstruct = coder.internal.parseInputs(optionalArgs,pvPairNames, poptions,varargin{:});

spatialLimits = coder.internal.getParameterValue(pstruct.spatialLimits,...
    defaults.spatialLimits,varargin{:});
binOutput = coder.internal.getParameterValue(pstruct.BinOutput,...
    defaults.BinOutput,varargin{:});

validateSpatialLimits(spatialLimits);
validateBinOutput(binOutput);
end

function validatePointCloud(ptCloud)
validateattributes(ptCloud, {'pointCloud'}, {'size', [1 1]}, mfilename, 'ptCloud');
end

function validateNumBins(numBins)
validateattributes(numBins, {'numeric'}, {'size', [1 3], 'real', 'positive',...
    'nonnan', 'integer'}, mfilename, 'numBins');
end

function validateSpatialLimits(spatialLimits)
% Not validating attributes if GPU codegen is enabled and user has not 
% specified spatial limits
if ~(isGPUTarget() && isempty(spatialLimits))
    validateattributes(spatialLimits, {'numeric'}, {'size', [3 2], 'real', 'finite'}, ...
        'pcbin', 'spatialLimits'); 

    diffVal = (diff(spatialLimits, 1, 2) >= 0);
    flag = ~isequal(diffVal, true(3,1));
    coder.internal.errorIf(flag, 'vision:pointcloud:limitsMustBeIncreasing');
end
end

function validateBinOutput(binOutput)
validateattributes(binOutput, {'logical', 'numeric'},...
    {'scalar','finite', 'real', 'nonsparse'},...
     mfilename, 'BinOutput');
end

function limits = defineDefaultLimits(pcIn)

% Limits will be computed in GPU implementation of pcbin
if isGPUTarget() 
    limits = [];
else
    limits = [pcIn.XLimits; pcIn.YLimits; pcIn.ZLimits];
    if ~all(all(isfinite(limits)))
        % Remove invalid points if any of the limits is Inf.
        pc = removeInvalidPoints(pcIn);
        limits = [pc.XLimits; pc.YLimits; pc.ZLimits];
    end
    if isempty(limits)
        % The XYZLimits properties of pointCloud are all empty when there
        % are no valid points in the point cloud.
        limits = cast(zeros(3,2), 'like', pcIn.XLimits) ;
        coder.internal.error('vision:pointcloud:allInvalidPoints', 'pcbin');
    end
end
end

function flag = isSimMode()

flag = isempty(coder.target);
end

function idx = subscriptsToIndices(binSize, rowBins, colBins, heightBins)
% Custom function to convert from subscripts to linear indices since
% sub2ind does not support NaN values.

s1 = binSize(1) * binSize(2);
lenCol = binSize(1);
idx = rowBins + (colBins - 1) * lenCol + (heightBins - 1) * s1;
end

function flag = isGPUTarget()
flag = coder.gpu.internal.isGpuEnabled;
end
