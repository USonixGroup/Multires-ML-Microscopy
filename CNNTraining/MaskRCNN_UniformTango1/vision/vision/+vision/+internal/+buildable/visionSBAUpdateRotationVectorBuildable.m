classdef visionSBAUpdateRotationVectorBuildable < coder.ExternalDependency %#codegen
% visionSBAUpdateRotationVectorBuildable - encapsulate visionSBASolvePoints implementation

% Copyright 2021 The MathWorks, Inc.
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'visionSBAUpdateRotationVectorBuildable';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo, context)
            buildInfo.addIncludePaths({...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','include'),...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','include','calibration'),...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','export','include','vision'),...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','export','include','vision','calibration'),...
                });
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'visionSBAUpdateRotationVectorCore.cpp'}, srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', 'visionSBAUpdateRotationVectorCore_api.hpp', 'quaternion.hpp'});
        end

        function newRot = visionSBAUpdateRotation(quaternionBases, cameraMat)

            coder.inline('always');
            coder.cinclude('visionSBAUpdateRotationVectorCore_api.hpp');
            fcnName = 'visionSBAUpdateRotationVector';

            numViews = int32(size(quaternionBases, 2));
            % initialize new rotatio matrix of size 3xV
            newRot = zeros(3, numViews);

            if coder.isColumnMajor
                coder.ceval(fcnName, coder.ref(quaternionBases), numViews,...
                    coder.ref(cameraMat), coder.ref(newRot));
            else
                outRow = zeros(numViews, 3);
                coder.ceval('-row', fcnName, quaternionBases', numViews,...
                    cameraMat', coder.ref(outRow));
                newRot = outRow';
            end

        end
    end
end
