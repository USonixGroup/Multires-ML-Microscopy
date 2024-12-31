classdef PCANormalBuildable < coder.ExternalDependency
    % PCANormalBuildable - encapsulate PCANormal implementation library

    % Copyright 2018-2020 The MathWorks, Inc.
    %#codegen
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'PCANormalBuildable';
        end
        
        function b = isSupportedContext(~) % supports non-host target
            b = true;
        end
        
        function updateBuildInfo(buildInfo, ~)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'),...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','export','include','vision')});
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'PCANormalCore.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'PCANormalCore_api.hpp', ...
                'PCANormal.hpp', ...
                'realSymmetricEigenSolver_core.hpp', ...
                'normalVector_core.hpp'});
        end
        %------------------------------------------------------------------
        %                  PCA Normal function
        %------------------------------------------------------------------
        function normals = PCANormal_core(points, indices, valid, dataType)
            coder.inline('always');
            coder.cinclude('PCANormalCore_api.hpp');
            
            numPoints = uint32(size(points,1));
            numNeighbors =  uint32(size(indices,1));
            
            fcnName = ['PCANormalImpl_' dataType];
            
            coder.varsize('normals',[inf 3]);
            
            if coder.isColumnMajor
                normals = coder.nullcopy(zeros(size(points),'like',points));
                coder.ceval('-col', fcnName, coder.ref(points), coder.ref(indices), coder.ref(valid), numPoints, numNeighbors, coder.ref(normals));
            else
                tempNormals = coder.nullcopy(zeros(size(points,2),numPoints,'like',points));
                coder.ceval('-row', fcnName, points', indices', valid', numPoints, numNeighbors, coder.ref(tempNormals));
                normals = tempNormals';
            end
        end
    end
end
