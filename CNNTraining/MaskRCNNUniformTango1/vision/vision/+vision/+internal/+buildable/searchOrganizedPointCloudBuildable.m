classdef searchOrganizedPointCloudBuildable < coder.ExternalDependency
    % searchOrganizedPointCloudBuildable - encapsulate searchOrganizedPointCloud core implementataion

    % Copyright 2018-2022 The MathWorks, Inc.
    %#codegen
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'searchOrganizedPointCloud';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo, context)
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                'searchOrganizedPointCloud', {});
        end

        function [indices, dists] = searchOrganizedPointCloud_core(location, point, kValue, projectionOfPoint, KRKRT, typeOfSearch)
            coder.inline('always');

            coder.cinclude('cvstCG_searchOrganizedPointCloud.h');

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
