classdef searchOrganizedPointCloudBuildablePortable < coder.ExternalDependency
    % searchOrganizedPointCloudBuildablePortable - encapsulate searchOrganizedPointCloud core implementataion

    % Copyright 2021-2022 The MathWorks, Inc.
    %#codegen
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'searchOrganizedPointCloud_portable';
        end

        function b = isSupportedContext(~)
            b = true;
        end

        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox','vision','builtins','src','shared', ...
                'searchOrganizedPointCloud' , 'export', 'include', 'searchOrganizedPointCloud' ), ...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','export','include','vision')});

            buildInfo.addSourcePaths({fullfile(matlabroot,'toolbox','vision','builtins','src','shared', ...
                'searchOrganizedPointCloud')});

            buildInfo.addIncludeFiles({'searchOrganizedPointCloud_published_c_api.hpp',...
                'searchOrganizedPointCloud_core.hpp',...
                'libmwsearchOrganizedPointCloud_util.hpp'...
                });

            buildInfo.addSourceFiles({'searchOrganizedPointCloud.cpp'});

            buildInfo.addDefines('-D__portable__');
        end

        function [indices, dists] = searchOrganizedPointCloud_core(location, point, kValue, projectionOfPoint, KRKRT, typeOfSearch)
            coder.inline('always');
            coder.cinclude('searchOrganizedPointCloud_published_c_api.hpp');

            ptrIndices = coder.opaquePtr('void', coder.internal.null);
            ptrDists = coder.opaquePtr('void', coder.internal.null);

            % call function
            numOut = uint32(0);

            height  = uint32(size(location,1));
            width = uint32(size(location,2));
            fcnName = ['searchOrganizedPointCloud_' lower(typeOfSearch) '_' class(location)];
            numOut(1) = coder.ceval('-col',fcnName, ...
                coder.ref(location), ...
                height,...
                width, ...
                coder.ref(point),...
                double(kValue),...
                coder.ref(projectionOfPoint),...
                coder.ref(KRKRT),...
                coder.ref(ptrIndices),...
                coder.ref(ptrDists)...
                );
            coder.varsize('indices',[inf 1]);
            coder.varsize('dists',[inf 1]);

            indices = coder.nullcopy(zeros(numOut(1),1, 'uint32'));
            dists = coder.nullcopy(zeros(numOut(1),1,'like',location));

            coder.ceval('-layout:any',['searchOrganizedPointCloudAssignOutputs_' class(location)], ...
                ptrIndices,...
                ptrDists,...
                coder.ref(indices),...
                coder.ref(dists));
        end
    end
end
