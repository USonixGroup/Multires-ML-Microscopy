classdef registerICPBuildable < coder.ExternalDependency %#codegen
    % registerICPBuildable - encapsulate pcregistericp core implementataion

    % Copyright 2023 The MathWorks, Inc.

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'registerICPBuildable';
        end

        function b = isSupportedContext(~)
            b = true; % supports non-host target
        end

        function updateBuildInfo(buildInfo, context)

            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'), ...
                fullfile(matlabroot,'toolbox','vision','builtins','src',...
                'visionopen3d','include')});

            srcPaths = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','visionopen3d')}, [4 1]);
            buildInfo.addSourceFiles({'registerICPCore.cpp','registration.cpp',...
                'generalizedICP.cpp', 'coloredICP.cpp'},srcPaths);
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'registerICPUtils.hpp','registration.h','generalizedICP.h', 'coloredICP.h'});

            buildInfo.addIncludeFiles('registerICPCore_api.hpp');

            vision.internal.buildable.portableOpen3DBuildInfo(buildInfo, context);
        end

        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function [outTransform, rmse] = registerICP(obj)

            coder.inline('always');
            coder.cinclude('registerICPCore_api.hpp');

            locationA = obj.moving;
            locationB = obj.fixed;
            numPointsA = int32(size(obj.moving,1));
            numPointsB = int32(size(obj.fixed,1));

            dataType = class(locationA);

            hasNormalsA = obj.HasNormalsA;
            hasNormalsB = obj.HasNormalsB;

            if hasNormalsA
                normalsA = obj.movingNormals;
            else
                normalsA = zeros(coder.ignoreConst(0),3,dataType);
            end

            if hasNormalsB
                normalsB = obj.fixedNormals;
            else
                normalsB = zeros(coder.ignoreConst(0),3,dataType);
            end

            useColors          = obj.UseColors;
            voxelSizes         = obj.VoxelSizes;
            maxIterationVector = obj.MaxIterationsVector;
            if useColors
                movingColors  = obj.movingColors;
                fixedColors  = obj.fixedColors;
            else
                movingColors  = zeros(coder.ignoreConst(0),3);
                fixedColors  = zeros(coder.ignoreConst(0),3);
            end

            initTform = obj.InitialTransform;
            dataTypeTform = [class(initTform) 0];
            maxIteration = int32(obj.MaxIteration);
            relativeRotation = obj.RelativeRotation;
            relativeTranslation = obj.RelativeTranslation;

            useInlierRatio = obj.UseInlierRatio;

            if(useInlierRatio)
                metric = obj.InlierRatio;
            else
                if ~isempty(obj.MaxInlierDistance)
                    metric = obj.MaxInlierDistance;
                else
                    metric = 0;
                end
            end
            method = [obj.Method 0];

            outTransform = zeros(4);
            % Getting the function name w.r.t dataType
            fcnName = ['registerICP_', dataType];

            % Calling the core C++ function
            rmse = 0;
            if coder.isColumnMajor
                rmse = coder.ceval('-col', fcnName, coder.ref(locationA), coder.ref(locationB),...
                    hasNormalsA, coder.ref(normalsA), hasNormalsB, coder.ref(normalsB),...
                    useColors, coder.ref(movingColors), coder.ref(fixedColors),...
                    coder.ref(voxelSizes), coder.ref(maxIterationVector), ...
                    coder.ref(initTform), coder.ref(dataTypeTform),...
                    maxIteration, ...
                    method, ...
                    relativeRotation, ...
                    relativeTranslation, ...
                    metric, ...
                    numPointsA,...
                    numPointsB, ...
                    useInlierRatio,...
                    coder.ref(outTransform));
            else
                outTransformTemp = zeros(4);
                rmse = coder.ceval('-row', fcnName, locationA(:), locationB(:),...
                    hasNormalsA, normalsA(:), hasNormalsB, normalsB(:),...
                    useColors, movingColors(:), fixedColors(:), voxelSizes(:),...
                    maxIterationVector(:), initTform(:), coder.ref(dataTypeTform),...
                    maxIteration, ...
                    method, ...
                    relativeRotation, ...
                    relativeTranslation, ...
                    metric, ...
                    numPointsA,...
                    numPointsB, ...
                    useInlierRatio,...
                    coder.ref(outTransformTemp));
                outTransform = outTransformTemp';

            end
        end

    end
end
