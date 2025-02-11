classdef detectORBBuildable < coder.ExternalDependency %#codegen
    % detectORBBuildable - used by detectORBFeatures
    
    % Copyright 2018-2023 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'detectORBBuildable';
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
            buildInfo.addSourceFiles({'detectORBCore.cpp', ...
                'cgCommon.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'detectORBCore_api.hpp', ...
                'cgCommon.hpp'}); % no need 'rtwtypes.h'
            
            vision.internal.buildable.portableOpenCVBuildInfo(...
                buildInfo, context, 'detectORB');
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function ptsStruct = detectORB_uint8(...
                Iu8, numFeatures, scaleFactor,...
                numLevels, edgeThreshold, firstLevel,...
                samplingPairs, scoreType, patchSize, fastThreshold)
            
            coder.inline('always');
            coder.cinclude('detectORBCore_api.hpp');
            
            % variables to hold core C/C++ function outputs
            pKeypoints = coder.opaquePtr('void', ...
                coder.internal.null); % fpts output ptr
            numPtsOut = int32(0); % number of detected points (to be told)
            
            % additional input parameters to the core C/C++ functions
            nRows = int32(size(Iu8, 1));
            nCols = int32(size(Iu8, 2));
            
            if coder.isColumnMajor
                layout = '-col';
                computeCoreFcnName = 'detectORBComputeCM';
                assignOutputFcnName = 'detectORBAssignOutputCM';
            else
                layout = '-row';
                computeCoreFcnName = 'detectORBComputeRM';
                assignOutputFcnName = 'detectORBAssignOutputRM';
            end
            
            % Detect ORB points
            numPtsOut(1) = coder.ceval(...
                layout, computeCoreFcnName,...
                coder.ref(Iu8), ...
                nRows, nCols, ...
                numFeatures, scaleFactor,...
                numLevels, edgeThreshold,firstLevel,...
                samplingPairs, scoreType, patchSize, fastThreshold,...
                coder.ref(pKeypoints));
            
            % Declare and initialize output variables. Variables are
            % declared varsize because numPotsOut is not constant
            coder.varsize('location',     [inf, 2]);
            coder.varsize('orientation',  [inf, 1]);
            coder.varsize('metric',       [inf, 1]);
            coder.varsize('scale',        [inf, 1]);
            location    = coder.nullcopy(zeros(numPtsOut, 2, 'single'));
            metric      = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            scale       = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            orientation = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            
            
            % copy the output from where the raw output pointer points
            coder.ceval(...
                layout, assignOutputFcnName, ...
                pKeypoints, ...
                coder.ref(location), ...
                coder.ref(orientation), ...
                coder.ref(metric), ...
                coder.ref(scale));
            
            % prepare return structure
            ptsStruct.Location    = location;
            ptsStruct.Orientation = orientation;
            ptsStruct.Metric      = metric;
            ptsStruct.Scale       = scale;
        end
    end
end
