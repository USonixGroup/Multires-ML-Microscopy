classdef visionSBASolvePointsBuildable < coder.ExternalDependency %#codegen
% visionSBASolvePointsBuildable - encapsulate visionSBASolvePoints implementation

% Copyright 2021 The MathWorks, Inc.
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'visionSBASolvePointsBuildable';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo, context)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'),...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','export','include','vision'),...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include','calibration')});
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'visionSBASolvePointsCore.cpp'}, srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', 'visionSBASolvePointsCore_api.hpp', 'matrixUtils.hpp'});
            
        end

        function Xb = visionSBASolvePoints(Wij, Xa, Vii, ebi, visibility)

            coder.inline('always');
            coder.cinclude('visionSBASolvePointsCore_api.hpp');
            fcnName = 'visionSBASolvePoints';

            numPoints = int32(size(visibility, 1));
            numViews = int32(size(visibility, 2));

            % computing irs, jcs array for sparse array input
            [irsd, ~] = find(visibility);
            irs = int32(irsd-1);
            jcsd = zeros(1, size(visibility, 2) + 1);
            jcsd(1) = 0;
            for i = 2:size(visibility, 2) + 1
                jcsd(i) = nnz(visibility(:, i - 1)) + jcsd(i - 1); 
            end
            jcs = int32(jcsd);
            
            % initializing outputs
            Xb = zeros(3*numPoints, 1);
            mValidJs = zeros(1, numPoints*numViews, 'int32');
            
            if coder.isColumnMajor
                coder.ceval(fcnName, coder.ref(Wij), coder.ref(Xa), ...
                    coder.ref(Vii), coder.ref(ebi), irs, jcs, numPoints, ...
                    numViews, coder.ref(Xb), coder.ref(mValidJs));
            else
                XbRow = zeros(1, 3*numPoints);
                coder.ceval('-row', fcnName, Wij', Xa', ...
                    Vii', ebi', irs', jcs', numPoints, ...
                    numViews, coder.ref(XbRow), mValidJs');
                Xb = XbRow';
            end
        end
    end
end
