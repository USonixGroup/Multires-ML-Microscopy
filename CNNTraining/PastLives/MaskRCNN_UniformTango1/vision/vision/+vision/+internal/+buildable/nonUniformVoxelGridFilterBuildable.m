classdef nonUniformVoxelGridFilterBuildable < coder.ExternalDependency
    % nonUniformVoxelGridFilterBuildable - encapsulate nonUniformVoxelGridFilter core implementation

    % Copyright 2018-2022 The MathWorks, Inc.
    %#codegen

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'nonUniformVoxelGridFilter';
        end

        function b = isSupportedContext(~) %supports non-host target
            b = true;
        end

        function updateBuildInfo(buildInfo, ~)

            buildInfo.addIncludePaths({fullfile(matlabroot, 'toolbox', ...
                'vision', 'builtins', 'src', 'vision', 'include'),...
                fullfile(matlabroot, 'toolbox', ...
                'vision', 'builtins', 'src', 'vision', 'export', 'include', 'vision')});

            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'nonUniformVoxelGridFilterCore.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');

            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'nonUniformVoxelGridFilter.hpp', ...
                'normalVector_core.hpp', ...
                'nonUniformVoxelGridFilterCore_api.hpp'});
        end

        function [indices, varargout] = nonUniformVoxelGridFilter(location, maxVoxelPoints)
            coder.inline('always');
            coder.cinclude('nonUniformVoxelGridFilterCore_api.hpp');
            ptrFilterObj = coder.opaquePtr('void', coder.internal.null);

            % call function
            numOut = uint32(0);

            needNormals = false;
            if nargout >= 2
                needNormals = true;
            end

            if ((~needNormals && maxVoxelPoints < 2) || (needNormals && maxVoxelPoints < 3))
                coder.internal.error('vision:ocvShared:invalidInputClass');
            end

            if ismatrix(location)
                numPoints = uint32(size(location, 1));
            else
                numPoints = uint32(size(location, 1) * size(location, 2));
            end

            fcnName = ['nonUniformVoxelGridFilterImpl_' class(location)];
            if coder.isColumnMajor
                numOut = coder.ceval('-col', fcnName, ...
                    coder.ref(ptrFilterObj), ...
                    coder.ref(location), ...
                    numPoints, ...
                    maxVoxelPoints, ...
                    needNormals);
            else
                tempLocation = reshape(location, [], 1);
                numOut = coder.ceval('-row', fcnName, ...
                    coder.ref(ptrFilterObj), ...
                    coder.ref(tempLocation), ...
                    numPoints, ...
                    maxVoxelPoints, ...
                    needNormals);
            end
            coder.varsize('indices', [inf 1], [1 0]);
            coder.varsize('normal', [inf 3], [1 0]);

            indices = coder.nullcopy(zeros(numOut, 1, 'uint32'));

            if coder.isColumnMajor
                if nargout >= 2
                    normal = coder.nullcopy(zeros(numOut, 3, 'like', location));
                else
                    normal = coder.nullcopy(zeros(0, 3, 'like', location));
                end
                coder.ceval('-col', ['nonUniformVoxelGridFilterImplAssignOutputs_' class(location)], ...
                    ptrFilterObj, ...
                    coder.ref(indices),...
                    coder.ref(normal),...
                    needNormals);
            else
                if nargout >= 2
                    tempNormal = coder.nullcopy(zeros(3, numOut,'like',location));
                else
                    tempNormal = coder.nullcopy(zeros(3, 0, 'like', location));
                end
                coder.ceval('-row', ['nonUniformVoxelGridFilterImplAssignOutputs_' class(location)], ...
                    ptrFilterObj, ...
                    coder.ref(indices),...
                    coder.ref(tempNormal),...
                    needNormals);
                normal = tempNormal';
            end

            if nargout >=2
                varargout{1} = normal;
            end
        end
    end
end
