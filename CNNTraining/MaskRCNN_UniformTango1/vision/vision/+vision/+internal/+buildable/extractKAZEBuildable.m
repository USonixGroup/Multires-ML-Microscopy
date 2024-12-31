classdef extractKAZEBuildable < coder.ExternalDependency %#codegen
    % extractKAZEBuildable - used by extractKAZEFeatures
    
    % Copyright 2017-2023 The MathWorks, Inc.
    
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'extractKAZEBuildable';
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
            buildInfo.addSourceFiles({'extractKAZECore.cpp', ...
                'cgCommon.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'extractKAZECore_api.hpp', ...
                'cgCommon.hpp'}); % no need 'rtwtypes.h'
            
            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, ...
                'extractKAZE');
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function [ftrs, vPts] = extractKAZE_uint8(...
                Iu8, ptsStct, ...
                extended, upright, threshold, ...
                noctaves, nscalelevels, ...
                diffusivity)
            
            coder.inline('always');
            coder.cinclude('extractKAZECore_api.hpp');
            
            % variables to hold core C/C++ function outputs
            pVPts = coder.opaquePtr('void', ...
                coder.internal.null); % fpts output ptr
            pFtrs  = coder.opaquePtr('void', ...
                coder.internal.null); % features ptr
            
            numPtsOut = int32(0); % number of extracted points (to be told)
            
            % additional input parameters to the core C/C++ functions
            nRows = int32(size(Iu8, 1));
            nCols = int32(size(Iu8, 2));
            
            inLoc = ptsStct.Location;
            inOri = ptsStct.Orientation;
            inMet = ptsStct.Metric;
            inScl = ptsStct.Scale;
            inMis = ptsStct.Misc;
            
            layout = '-col';
            computeCoreFcnName = 'extractKAZEComputeCM';
            assignOutputFcnName = 'extractKAZEAssignOutputCM';
            if ~coder.isColumnMajor
                layout = '-row';
                computeCoreFcnName = 'extractKAZEComputeRM';
                assignOutputFcnName = 'extractKAZEAssignOutputRM';
            end
            
            % Extract KAZE features
            numPtsOut(1) = coder.ceval(...
                layout, computeCoreFcnName,...
                coder.ref(Iu8), ...
                nRows, nCols, ...
                coder.ref(inLoc), coder.ref(inOri), ...
                coder.ref(inMet), coder.ref(inScl), ...
                coder.ref(inMis), int32(size(inLoc,1)), ...
                extended, upright, threshold, ...
                noctaves, nscalelevels, diffusivity, ...
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
            coder.varsize('misc',         [inf, 1]); % layer id
            vLoc = coder.nullcopy(zeros(numPtsOut, 2, 'single'));
            vMet = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vScl = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vOri = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vMis = coder.nullcopy(zeros(numPtsOut, 1, 'uint8'));
            % similarly for the output feature matrix
            coder.varsize('features',     [inf, inf]);
            fSize = 64;
            if extended % if extended, feature size is 128
                fSize = 128;
            end
            ftrs = coder.nullcopy(zeros(numPtsOut, fSize, 'single'));
            
            % Copy the output from where the raw output pointers point to
            % to the actual storages declared above
            coder.ceval(...
                layout, assignOutputFcnName, ...
                pVPts, pFtrs, ...
                coder.ref(vLoc), ...
                coder.ref(vOri), ...
                coder.ref(vMet), ...
                coder.ref(vScl), ...
                coder.ref(vMis), ...
                coder.ref(ftrs));
            
            % Tie the point information to the returned point structure
            vPts.Location    = vLoc;
            vPts.Orientation = vOri;
            vPts.Metric      = vMet;
            vPts.Scale       = vScl;
            vPts.Misc        = vMis;
        end
    end
end
