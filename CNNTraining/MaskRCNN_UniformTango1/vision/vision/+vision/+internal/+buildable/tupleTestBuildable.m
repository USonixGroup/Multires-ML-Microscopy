classdef tupleTestBuildable < coder.ExternalDependency %#codegen
    %

    % Copyright 2020 The MathWorks, Inc.

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'tupleTestBuildable';
        end

        function b = isSupportedContext(~)
            b = true; % supports non-host target
        end

        function updateBuildInfo(buildInfo, ~)

            buildInfo.addIncludePaths({fullfile(matlabroot, 'toolbox', ...
                'vision', 'builtins', 'src', 'vision', 'include')} );
            buildInfo.addSourcePaths({fullfile(matlabroot, 'toolbox', ...
                                               'vision', 'builtins', 'src', 'vision')});
            buildInfo.addSourceFiles({'tupleTestCore.cpp'});
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                                'tupleTest.hpp'});
        end

        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        function matchedInds = tupleTestImpl(points1, points2, randInds, scale, numIters)

            coder.inline('always');
            coder.cinclude('tupleTest.hpp');

            numPoints   = uint32(size(points1, 1));
            pointsDim   = uint32(size(points1, 2));

            matchedInds = false(numPoints, 1);

            if (isa(points1, 'double'))
                coder.ceval('findMatchesDouble', coder.ref(matchedInds), ...
                    coder.rref(points1), coder.rref(points2), coder.rref(randInds), ...
                    numPoints, pointsDim, scale, numIters);
            else
                coder.ceval('findMatchesSingle', coder.ref(matchedInds), ...
                    coder.rref(points1), coder.rref(points2), coder.rref(randInds), ...
                    numPoints, pointsDim, scale, numIters);
            end
        end
    end
end
