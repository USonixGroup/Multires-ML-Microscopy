classdef labelRangeDataGroundRemovalBuildable < coder.ExternalDependency %#codegen
    % labelRangeDataGroundRemovalBuildable - encapsulate labelRangeDataGroundRemovalBuildable implementation library
    
    % Copyright 2019 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'labelRangeDataGroundRemovalBuildable';
        end
        
        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end
        
        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'),...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','export','include','vision')});
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'labelRangeDataGroundRemovalUtilsCore.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'labelRangeDataGroundRemovalUtils.hpp', 'labelRangeDataGroundRemovalUtilsCore_api.hpp' });
        end
        
        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [groundPtsIdx] = labelRangeDataGroundRemoval(rangeData, ...
                seedAngle, deltaAngle, repairDepthThresh)
            coder.inline('always');
            coder.cinclude('labelRangeDataGroundRemovalUtilsCore_api.hpp');
            
            %Getting the required inputs
            nRows = int32(size(rangeData, 1));
            nCols = int32(size(rangeData, 2));
            dataType = class(rangeData);
            
            % Initialising the groundPointIndices matrix
            groundPtsIdx = zeros(nRows, nCols, 'logical');
            
            % Getting the function name w.r.t dataType
            fcnName = ['preprocessAndSegmentGround_' dataType];
            
            % Calling the core C++ function for segmentGroundFromLidarData
            if coder.isColumnMajor
                coder.ceval('-col', fcnName, coder.ref(rangeData), nRows, nCols,...
                    coder.ref(groundPtsIdx), cast(seedAngle, dataType), cast(deltaAngle, dataType), ...
                    cast(repairDepthThresh, dataType));
            else
                coder.ceval('-row', fcnName, rangeData(:), nRows, nCols,...
                    coder.ref(groundPtsIdx), cast(seedAngle, dataType), cast(deltaAngle, dataType), ...
                    cast(repairDepthThresh, dataType));
                groundPtsIdx = reshape(groundPtsIdx', size(groundPtsIdx));
            end
        end
    end
    
end