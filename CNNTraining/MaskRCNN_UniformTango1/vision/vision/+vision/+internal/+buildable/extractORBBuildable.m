classdef extractORBBuildable < coder.ExternalDependency %#codegen
    
    % extractORBBuildable - used by extractORBFeatures
    
    % Copyright 2018-2023 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'extractORBBuildable';
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
            buildInfo.addSourceFiles({'extractORBCore.cpp', ...
                'cgCommon.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'extractORBCore_api.hpp', ...
                'cgCommon.hpp'}); % no need 'rtwtypes.h'
            
            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, ...
                'extractORB');
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function [ftrs, vPts] = extractORB_uint8(...
                Iu8, ptsStct,numFeatures, scaleFactor,...
                numLevels, edgeThreshold,firstLevel,...
                samplingPairs, scoreTypeCode, patchSize, fastThreshold)
            
            coder.inline('always');
            coder.cinclude('extractORBCore_api.hpp');
            
            % variables to hold core C/C++ function outputs
            pVPts = coder.opaquePtr('void', ...
                coder.internal.null); % fpts output ptr
            pFtrs = coder.opaquePtr('void', ...
                coder.internal.null); % features ptr
            
            numPtsOut = int32(0); % number of extracted points (to be told)
            
            % additional input parameters to the core C/C++ functions
            nRows = int32(size(Iu8, 1));
            nCols = int32(size(Iu8, 2));
            
            inLoc = ptsStct.Location;
            inOri = ptsStct.Orientation;
            inMet = ptsStct.Metric;
            inScl = ptsStct.Scale;
            
            
            layout = '-col';
            computeCoreFcnName = 'extractORBComputeCM';
            assignOutputFcnName = 'extractORBAssignOutputCM';
            if ~coder.isColumnMajor
                layout = '-row';
                computeCoreFcnName = 'extractORBComputeRM';
                assignOutputFcnName = 'extractORBAssignOutputRM';
            end
            
            % Extract ORB features
            numPtsOut(1) = coder.ceval(...
                layout, computeCoreFcnName,...
                coder.ref(Iu8), ...
                nRows, nCols, ...
                coder.ref(inLoc), coder.ref(inOri), ...
                coder.ref(inMet), coder.ref(inScl),int32(size(inLoc,1)), ...
                numFeatures, scaleFactor,...
                numLevels, edgeThreshold,firstLevel,...
                samplingPairs, scoreTypeCode, patchSize, fastThreshold, ...
                coder.ref(pVPts), ...
                coder.ref(pFtrs));
            % because we need to update the pointers as input/output
            % arguments, we are passing in the pointers to these
            % pointers
            
            % Declare and initialize output variables. Variables are
            % declared varsize because numPotsOut is not constant
            coder.varsize('location',     [inf, 2]);
            coder.varsize('orientation',  [inf, 1]);
            coder.varsize('metric',       [inf, 1]);
            coder.varsize('scale',        [inf, 1]);
            
            vLoc = coder.nullcopy(zeros(numPtsOut, 2, 'single'));
            vMet = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vScl = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vOri = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            % similarly for the output feature matrix
            coder.varsize('features',[inf, inf],[1 1]);
            fSize = 32;
            ftrs = coder.nullcopy(zeros(numPtsOut, fSize, 'uint8'));
            
            % Copy the output from where the raw output pointers point to
            % to the actual storages declared above
            coder.ceval(...
                layout, assignOutputFcnName, ...
                pVPts, pFtrs, ...
                coder.ref(vLoc), ...
                coder.ref(vOri), ...
                coder.ref(vMet), ...
                coder.ref(vScl), ...
                coder.ref(ftrs));
            
            % Tie the point information to the returned point structure
            vPts.Location    = vLoc;
            vPts.Orientation = vOri;
            vPts.Metric      = vMet;
            vPts.Scale       = vScl;
        end
    end
end
