classdef voxelGridFilterBuildable < coder.ExternalDependency
    % voxelGridFilterBuildable - encapsulate voxelGridFilter core implementation

    % Copyright 2018-2022 The MathWorks, Inc.
    %#codegen
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'voxelGridFilter';
        end
        
        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end
        
        function updateBuildInfo(buildInfo, context)
                vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                    'voxelGridFilter', ...
                    {'use_tbb'});
        end
        
        function [filteredLocation, filteredColor, filteredNormal, filteredIntensity, filteredRangeData, varargout] = voxelGridFilter(location, color, normal, intensity, rangeData, voxelSize, range, varargin)
            coder.inline('always');
            coder.cinclude('cvstCG_voxelGridFilter.h');
            
            ptrIndexVector = coder.opaquePtr('void', coder.internal.null);
            
            % call function
            numOut = uint32(0);
            
            minVoxelPoints = 1;
            if nargin >= 8
                minVoxelPoints = varargin{1};
            end
            
            needCovariance = nargout >= 6;
            
            needCount = nargout >= 7;

            if ismatrix(location)
                numPoints = uint32(size(location, 1));
            else
                numPoints = uint32(size(location, 1) * size(location, 2));
            end
            
            inverseVoxelSize = 1 / voxelSize;
            
            xmin = range(1); xmax = range(2);
            ymin = range(3); ymax = range(4);
            zmin = range(5); zmax = range(6);
            
            % Check that the voxel size is not too small, given the size of the data
            dx = uint64((xmax - xmin) * inverseVoxelSize + 1);
            dy = uint64((ymax - ymin) * inverseVoxelSize + 1);
            dz = uint64((zmax - zmin) * inverseVoxelSize + 1);
            
            dxy = dx*dy;  dxyz = dxy*dz;
            
            if (dx == 0 || dy == 0 || dz == 0 || dxy == 0)
                coder.internal.error('vision:pointcloud:voxelSizeTooSmall');
            end
            
            if (dx ~= dxy / dy || dz ~= dxyz / dxy)
                coder.internal.error('vision:pointcloud:voxelSizeTooSmall');
            end
            
            fcnName = ['populateIndexVector_' class(location)];
            numOut = coder.ceval('-col',fcnName, ...
                coder.rref(location), ...
                numPoints, ...
                voxelSize, ...
                coder.rref(range), ...
                coder.ref(ptrIndexVector), ...
                minVoxelPoints);
            
            needColorFlag     = ~isempty(color);
            needNormalFlag    = ~isempty(normal);
            needIntensityFlag = ~isempty(intensity);
            needRangeFlag     = ~isempty(rangeData);
            
            coder.varsize('filteredLocation', [inf 3], [1 0]);
            coder.varsize('filteredColor', [inf 3], [1 0]);
            coder.varsize('filteredNormal', [inf 3], [1 0]);
            coder.varsize('filteredIntensity', [inf 1], [1 0]);
            coder.varsize('filteredRangeData', [inf 3], [1 0]);
            coder.varsize('covariance', [3 3 Inf], [ 0 0 1]);
            
            filteredLocation = coder.nullcopy(zeros(numOut, 3, 'like', location));
            
            if needColorFlag
                filteredColor = coder.nullcopy(zeros(numOut, 3, 'like', color));
            else
                filteredColor = coder.nullcopy(zeros(0, 3, 'like', color));
            end
            
            if needNormalFlag
                filteredNormal = coder.nullcopy(zeros(numOut, 3, 'like', location));
            else
                filteredNormal = coder.nullcopy(zeros(0, 3, 'like', location));
            end
            
            if needIntensityFlag
                filteredIntensity = coder.nullcopy(zeros(numOut, 1, 'like', intensity));
            else
                filteredIntensity = coder.nullcopy(zeros(0, 1, 'like', intensity));
            end
            
            if needRangeFlag
                filteredRangeData = coder.nullcopy(zeros(numOut, 3, 'like', location));
            else
                filteredRangeData = coder.nullcopy(zeros(0, 3, 'like', location));
            end
            
            if needCovariance
                covariance = coder.nullcopy(zeros(3, 3, numOut,'like', location));
            else
                covariance = coder.nullcopy(zeros(3, 3, 0, 'like', location));
            end
            
            if needCount
                voxelCounts = coder.nullcopy(zeros(numOut, 1, 'uint32'));
            else
                voxelCounts = coder.nullcopy(zeros(0, 1, 'uint32'));
            end

            functionName = getCppFunctionName(location, color, intensity);
            
            coder.ceval('-col', functionName, ...
                coder.rref(location), ...
                numPoints, ...
                coder.rref(color), ...
                coder.rref(normal), ...
                coder.rref(intensity), ...
                coder.rref(rangeData), ...
                coder.rref(range), ...
                coder.ref(filteredLocation), ...
                coder.ref(filteredColor), ...
                coder.ref(filteredNormal), ...
                coder.ref(filteredIntensity), ...
                coder.ref(filteredRangeData), ...
                coder.ref(covariance), ...
                coder.ref(voxelCounts), ...
                ptrIndexVector, ...
                numOut, ...
                needColorFlag, ...
                needNormalFlag, ...
                needIntensityFlag, ...
                needRangeFlag, ...
                needCovariance, ...
                needCount);

            if nargout >= 6
                varargout{1} = covariance;
                if nargout >= 7
                    varargout{2} = voxelCounts;
                end
            end
            
        end
    end
end

function name = getCppFunctionName(location, color, intensity)

if isa(intensity, 'uint8')
    if isa(color, 'uint16')
        name = ['voxelGridAlgImpl_' class(location) '_uint16Color_uint8Intensity'];
    else
        name = ['voxelGridAlgImpl_' class(location) '_uint8Color_uint8Intensity'];
    end
elseif isa(intensity, 'uint16')
    if isa(color, 'uint16')
        name = ['voxelGridAlgImpl_' class(location) '_uint16Color_uint16Intensity'];
    else
        name = ['voxelGridAlgImpl_' class(location) '_uint8Color_uint16Intensity'];
    end
elseif isa(intensity, 'single')
    if isa(color, 'uint16')
        name = ['voxelGridAlgImpl_' class(location) '_uint16Color_singleIntensity'];
    else
        name = ['voxelGridAlgImpl_' class(location) '_uint8Color_singleIntensity'];
    end
elseif isa(intensity, 'double')
    if isa(color, 'uint16')
        name = ['voxelGridAlgImpl_' class(location) '_uint16Color_doubleIntensity'];
    else
        name = ['voxelGridAlgImpl_' class(location) '_uint8Color_doubleIntensity'];
    end
else
    coder.internal.assert(false, 'vision:pointcloud:invalidIntensityType');
end
end
