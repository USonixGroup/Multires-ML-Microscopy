classdef extractSurfBuildable < coder.ExternalDependency %#codegen
    %CVSTGETAVERAGEBUILDABLE - encapsulate getAverage implementation library
    
    % Copyright 2012-2023 The MathWorks, Inc.
    
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'extractSurfBuildable';
        end
        
        function b = isSupportedContext(~)
            b = true; % supports non-host target
        end
        
        function updateBuildInfo(buildInfo, context)
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv','include'), ...
                fullfile(matlabroot,'toolbox', ...
                'images','opencv','opencvcg', 'include')} );
            
            srcPaths = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv')}, [1 4]);
            buildInfo.addSourceFiles({'extractSurfCore.cpp', ...
                'surfCommon.cpp', ...
                'cgCommon.cpp', ...
                'mwsurf.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'extractSurfCore_api.hpp', ...
                'surfCommon.hpp', ...
                'precomp_mw.hpp', ...
                'features2d_surf_mw.hpp', ...
                'cgCommon.hpp'}); % no need 'rtwtypes.h'
            
            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, ...
                'extractSurf');
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function [outLocation, outScale, outMetric, outSignOfLaplacian, ...
                outOrientation, outFeatures] = ...
                extractSurf_uint8(Iu8T, inLocation, inScale, inMetric, ...
                inSignOfLaplacian, featureWidth, isExtended, isUpright)
            
            coder.inline('always');
            coder.cinclude('extractSurfCore_api.hpp');
            
            ptrKeypoints = coder.opaquePtr('void', coder.internal.null);
            ptrDescriptors = coder.opaquePtr('void', coder.internal.null);
            
            % call function
            out_numel = int32(0);
            numel = int32(size(inLocation, 1));
            numInDims = int32(ndims(Iu8T));
            if coder.isColumnMajor
                nRows = int32(size(Iu8T, 2)); % original (before transpose)
                nCols = int32(size(Iu8T, 1)); % original (before transpose)
                
                out_numel(1)=coder.ceval('-col','extractSurf_compute',...
                    coder.ref(Iu8T), ...
                    nRows, nCols, numInDims, ...
                    inLocation, inScale, inMetric, inSignOfLaplacian, ...
                    numel, isExtended, isUpright, ...
                    coder.ref(ptrKeypoints), coder.ref(ptrDescriptors));
            else
                nRows = int32(size(Iu8T, 1)); % original (before transpose)
                nCols = int32(size(Iu8T, 2)); % original (before transpose)
                
                out_numel(1)=coder.ceval('-row','extractSurf_computeRM',...
                    coder.ref(Iu8T), ...
                    nRows, nCols, numInDims, ...
                    inLocation, inScale, inMetric, inSignOfLaplacian, ...
                    numel, isExtended, isUpright, ...
                    coder.ref(ptrKeypoints), coder.ref(ptrDescriptors));
            end
            
            % copy output to mxArray
            % declare output as variable sized so that _mex file can return differet sized output.
            % allocate output
            coder.internal.prefer_const(featureWidth);
            coder.varsize('outLocation',        [inf, 2]);
            coder.varsize('outScale',           [inf, 1]);
            coder.varsize('outMetric',          [inf, 1]);
            coder.varsize('outSignOfLaplacian', [inf, 1]);
            coder.varsize('outOrientation',     [inf, 1]);
            coder.varsize('outFeatures',        [inf, 128],[1 1]);
            
            % create uninitialized memory using coder.nullcopy
            outLocation = coder.nullcopy(zeros(out_numel,2,'single'));
            outScale    = coder.nullcopy(zeros(out_numel,1,'single'));
            outMetric   = coder.nullcopy(zeros(out_numel,1,'single'));
            outSignOfLaplacian = coder.nullcopy(zeros(out_numel,1,'int8'));
            outOrientation = coder.nullcopy(zeros(out_numel,1,'single'));
            outFeatures = coder.nullcopy(zeros(out_numel,featureWidth,'single'));
            
            if coder.isColumnMajor
                coder.ceval('-col','extractSurf_assignOutput',...
                    ptrKeypoints, ptrDescriptors, ...
                    coder.ref(outLocation), coder.ref(outScale), ...
                    coder.ref(outMetric), coder.ref(outSignOfLaplacian), ...
                    coder.ref(outOrientation), coder.ref(outFeatures));
            else
                coder.ceval('-row','extractSurf_assignOutputRM',...
                    ptrKeypoints, ptrDescriptors, ...
                    coder.ref(outLocation), coder.ref(outScale), ...
                    coder.ref(outMetric), coder.ref(outSignOfLaplacian), ...
                    coder.ref(outOrientation), coder.ref(outFeatures));
            end
            
        end
    end
end
