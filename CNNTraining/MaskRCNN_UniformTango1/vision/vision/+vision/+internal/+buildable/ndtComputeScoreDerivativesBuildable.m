classdef ndtComputeScoreDerivativesBuildable < coder.ExternalDependency %#codegen
    % ndtComputeScoreDerivativesBuildable - encapsulate ndtComputeScoreDerivativesBuildable implementation library

    % Copyright 2019-2024 The MathWorks, Inc.

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'ndtComputeScoreDerivativesBuildable';
        end

        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end

        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'),...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','export','include','vision'), ...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include', 'calibration')});

            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'ndtUtilsCore.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');

            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'ndtUtils.hpp', 'ndtUtilsCore_api.hpp', 'matrixUtils.hpp' });
        end

        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [score, grad_f, varargout] = ndtComputeScoreDerivatives(pose, args)
            coder.inline('always');
            coder.cinclude('ndtUtilsCore_api.hpp');

            % Getting the dataType
            points = args.ps;
            dataType = class(points);

            % Checking need of hessian matrix
            if(nargout == 3)
                needsHessian = true;
            else
                needsHessian = false;
            end

            % Getting the function name w.r.t dataType
            fcnName = ['computeJaHa_' dataType];

            % Initialising the matrices
            R = zeros(3, 3, dataType);
            Ja = zeros(1, 27, dataType);
            Ha = zeros(1, 81, dataType);
            Jp = zeros(1, 18, dataType);
            Hp = zeros(1, 27, dataType);
            g = zeros(1, 6, dataType);
            h = zeros(1, 36, dataType);
            grad = zeros(1, 6, dataType);
            score = zeros(1, 1, dataType);

            % Calling the core C++ function to calculate the rotation matrix
            if coder.isColumnMajor
                coder.ceval('-col', fcnName, coder.ref(pose),  coder.ref(R), coder.ref(Ja) ,coder.ref(Ha) );
            else
                Ja_row = zeros(27, 1, dataType);
                Ha_row = zeros(81, 1, dataType);
                coder.ceval('-row', fcnName, pose',  coder.ref(R), coder.ref(Ja_row) ,coder.ref(Ha_row) );
                Ja = Ja_row';
                Ha = Ha_row';
                R = R';
            end
            qs = points * R;
            qs(:, 1) = qs(:, 1) + pose(1);
            qs(:, 2) = qs(:, 2) + pose(2);
            qs(:, 3) = qs(:, 3) + pose(3);

            sopts = struct('eps', 0);

            numPoints = size(points,1);
            numMvals = size(args.mvals,1);

            if(needsHessian)
                hessian = zeros(6, 6, dataType);
            else
                if isa(dataType, 'single')
                    hessian = coder.opaquePtr('float', coder.internal.null);
                else
                    hessian = coder.opaquePtr('double', coder.internal.null);
                end
            end

            % Go through individual point and find the score and the corresponding gradient and the hessian.
            for i = 1: numPoints
                p = points(i, :);
                q = qs(i, :);

                %First, find all the nearest mean values for the query point. Then compute
                %the score for each mean and, in the end, simply add them up.
               if coder.internal.preferMATLABHostCompiledLibraries()
                [indices] = vision.internal.buildable.kdtreeBuildable.kdtreeRadiusSearch(args.tree,...
                    class(q), q, args.radius, sopts);
               else
                   [indices] = vision.internal.buildable.kdtreeBuildablePortable.kdtreeRadiusSearch(args.tree,...
                       class(q), q, args.radius, sopts);
               end

                if ~isempty(indices)
                    % Getting the function name w.r.t dataType
                    fcnName = ['computeScoreGradient_' dataType];

                    numIndices = uint16(numel(indices));
                    if coder.isColumnMajor
                        coder.ceval('-col', fcnName, coder.ref((indices)), coder.ref(p), needsHessian, ...
                            coder.ref(Ja), coder.ref(Ha), coder.ref(Jp) ,coder.ref(Hp), coder.ref(args.mvals), ...
                            uint16(numMvals), coder.ref(args.iCov), coder.ref(hessian), args.d1, args.d2, ...
                            coder.ref(score), coder.ref(g), coder.ref(h), coder.ref(q), coder.ref(grad), numIndices);
                    else
                        iCov = args.iCov(:);
                        coder.ceval('-row', fcnName, indices', p', needsHessian, Ja', Ha', coder.ref(Jp),...
                            coder.ref(Hp), args.mvals', uint16(numMvals), iCov, coder.ref(hessian), args.d1,...
                            args.d2, coder.ref(score), coder.ref(g), coder.ref(h), q', coder.ref(grad), numIndices);
                        hessian = hessian';
                    end
                end
            end

            grad_f = grad';
            varargout{1} = hessian;
        end
    end
end
