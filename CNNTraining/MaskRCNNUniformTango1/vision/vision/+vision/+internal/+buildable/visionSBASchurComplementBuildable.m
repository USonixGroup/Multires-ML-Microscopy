classdef visionSBASchurComplementBuildable < coder.ExternalDependency %#codegen
% visionSBASchurComplementBuildable - encapsulate visionSBASolvePoints implementation

% Copyright 2021 The MathWorks, Inc.
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'visionSBASchurComplementBuildable';
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
            buildInfo.addSourceFiles({'visionSBASchurComplementCore.cpp'}, srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', 'visionSBASchurComplementCore_api.hpp', ...
                'realSymmetricEigenSolver_core.hpp', 'matrixUtils.hpp'});
        end

        function [S, e, Vii] = visionSBASchurComplement(Uj, Vi, Wij, eaj, ebi, visibility)

            coder.inline('always');
            coder.cinclude('visionSBASchurComplementCore_api.hpp');
            fcnName = 'visionSBASchurComplement';

            % computing irs and jcs array for sparse array
            numPoints = int32(size(visibility, 1));
            numViews = int32(size(visibility, 2));
            [irsd, ~] = find(visibility);
            irs = int32(irsd-1);
            jcsd = zeros(1, size(visibility, 2) + 1);
            jcsd(1) = 0;
            for i = 2:size(visibility, 2) + 1
                jcsd(i) = nnz(visibility(:, i - 1)) + jcsd(i - 1); 
            end

            jcs = int32(jcsd);
            nvis = int32(jcs(end));

            % initializing output arrays
            S = zeros(6*numViews);
            e = zeros(6, numViews);
            Vii = zeros(3, 3*numPoints);
            
            if coder.isColumnMajor
                % internal call for column major inputs
                coder.ceval(fcnName, coder.ref(Uj), coder.ref(Vi), ...
                    coder.ref(Wij), coder.ref(eaj), coder.ref(ebi), ...
                    irs, jcs, numPoints, numViews, ...
                    coder.ref(S), coder.ref(e), coder.ref(Vii));
            else
                Srow = zeros(6*numViews);
                erow = zeros(numViews, 6);
                Viirow = zeros(3*numPoints, 3);

                % internal call for row major inputs
                coder.ceval('-row', fcnName, Uj', Vi', ...
                    Wij', eaj', ebi', ...
                    irs', jcs', numPoints, numViews, ...
                    coder.ref(Srow), coder.ref(erow), coder.ref(Viirow));
                S = Srow';
                e = erow';
                Vii = Viirow';
            end
        end
    end
end