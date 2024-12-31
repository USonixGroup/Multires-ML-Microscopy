classdef visionLabelRangeDataBuildable < coder.ExternalDependency %#codegen
    % visionLabelRangeDataBuildable - encapsulate visionLabelRangeData implementation

    % Copyright 2018-2020 The MathWorks, Inc.
    
    methods(Static)
        function name = getDescriptiveName(~)
            name = 'visionLabelRangeDataBuildable';
        end
        
        function b = isSupportedContext(~)
            b = true; % supports non-host target
        end
        
        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include')});
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'visionLabelRangeDataCore.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h',...
                'visionLabelRangeDataCore_api.hpp',...
                'visionLabelRangeData.hpp'});
            
        end
        
        function [labels, numClusters] =  visionLabelRangeDataCore(...
                location, range, threshold, angle_threshold,...
                minClusterPoints, maxClusterPoints)
            coder.inline('always');
            % add '#include "visionLabelRangeDataCore_api.hpp"' in <myfcn>.c
            coder.cinclude('visionLabelRangeDataCore_api.hpp');
            
            % call function
            nRows = size(location, 1);
            nCols = size(location, 2);
            numClusters = cast(0, 'like', location);
            dataType = class(location);
            fcName = ['visionLabelRangeData_',dataType];
            if coder.isColumnMajor
                labels = zeros(nRows, nCols, 'uint32');
                coder.ceval('-col', fcName, coder.ref(location), coder.ref(range), int32(nRows), ...
                    int32(nCols), cast(threshold,'like',location), cast(angle_threshold,'like',location),...
                    minClusterPoints, maxClusterPoints, coder.ref(labels), coder.ref(numClusters));
            else
                tempLabels = zeros(nCols, nRows, 'uint32');
                location_ = reshape(location,[nRows*nCols,3]);
                range_ = reshape(range,[nRows*nCols,3]);
                coder.ceval('-row', fcName, location_', range_', int32(nRows), int32(nCols), ...
                    cast(threshold,'like',location), cast(angle_threshold,'like',location),...
                    minClusterPoints, maxClusterPoints, coder.ref(tempLabels), coder.ref(numClusters));
                labels = tempLabels';
            end
        end
    end
end
