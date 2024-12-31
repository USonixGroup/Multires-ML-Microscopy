classdef detectKAZEBuildable < coder.ExternalDependency %#codegen
    % detectKAZEBuildable - used by detectKAZEFeatures
    
    % Copyright 2017-2023 The MathWorks, Inc.
    
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'detectKAZEBuildable';
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
                'vision','builtins','src','ocv')}, [1 2]);
            buildInfo.addSourceFiles({'detectKAZECore.cpp', ...
                'cgCommon.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'detectKAZECore_api.hpp', ...
                'cgCommon.hpp'}); % no need 'rtwtypes.h'
            
            vision.internal.buildable.portableOpenCVBuildInfo(...
                buildInfo, context, 'detectKAZE');
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function ptsStruct = detectKAZE_uint8(...
                Iu8, extended, upright, threshold, numOctaves, ...
                numScaleLevels, diffusivity)
            
            coder.inline('always');
            coder.cinclude('detectKAZECore_api.hpp');
            
            % variables to hold core C/C++ function outputs
            pKeypoints = coder.opaquePtr('void', ...
                coder.internal.null); % fpts output ptr
            numPtsOut = int32(0); % number of detected points (to be told)
            
            % additional input parameters to the core C/C++ functions
            nRows = int32(size(Iu8, 1));
            nCols = int32(size(Iu8, 2));
            
            if coder.isColumnMajor
                layout = '-col';
                computeCoreFcnName = 'detectKAZEComputeCM';
                assignOutputFcnName = 'detectKAZEAssignOutputCM';
            else
                layout = '-row';
                computeCoreFcnName = 'detectKAZEComputeRM';
                assignOutputFcnName = 'detectKAZEAssignOutputRM';
            end
            
            % Detect KAZE points
            numPtsOut(1) = coder.ceval(...
                layout, computeCoreFcnName,...
                coder.ref(Iu8), ...
                nRows, nCols, ...
                extended, upright, ...
                threshold, numOctaves, ...
                numScaleLevels, diffusivity, ...
                coder.ref(pKeypoints));
            
            % Declare and initialize output variables. Variables are
            % declared varsize because numPotsOut is not constant
            coder.varsize('location',     [inf, 2]);
            coder.varsize('orientation',  [inf, 1]);
            coder.varsize('metric',       [inf, 1]);
            coder.varsize('scale',        [inf, 1]);
            coder.varsize('misc',         [inf, 1]); % layer id
            location    = coder.nullcopy(zeros(numPtsOut, 2, 'single'));
            metric      = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            scale       = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            orientation = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            misc        = coder.nullcopy(zeros(numPtsOut, 1, 'uint8'));
            
            % copy the output from where the raw output pointer points
            coder.ceval(...
                layout, assignOutputFcnName, ...
                pKeypoints, ...
                coder.ref(location), ...
                coder.ref(orientation), ...
                coder.ref(metric), ...
                coder.ref(scale), ...
                coder.ref(misc));
            
            % prepare return structure
            ptsStruct.Location    = location;
            ptsStruct.Orientation = orientation;
            ptsStruct.Metric      = metric;
            ptsStruct.Scale       = scale;
            ptsStruct.Misc        = misc;
        end
    end
end
