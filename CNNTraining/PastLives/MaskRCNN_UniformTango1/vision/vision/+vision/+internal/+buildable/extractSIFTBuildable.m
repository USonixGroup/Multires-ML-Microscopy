classdef extractSIFTBuildable < coder.ExternalDependency %#codegen
  
    % Copyright 2021-2023 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'extractSIFTBuildable';
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
                'vision','builtins','src','ocv')}, [1 3]);
            buildInfo.addSourceFiles({'extractSIFTCore.cpp', ...
                'cgCommon.cpp', ...
                'mwsift.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'extractSIFTCore_api.hpp', ...
                'cgCommon.hpp', ...
                'xfeatures2d_sift_mw.hpp'});
            
            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, ...
                'extractSIFT');
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function [ftrs, vPts] = extractSIFT_uint8(...
                Iu8, ptsStct)
            
            coder.inline('always');
            coder.cinclude('extractSIFTCore_api.hpp');
            
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
            inScl = ptsStct.Scale;
            inMet = ptsStct.Metric;
            inOri = ptsStct.Orientation;
            inOct = ptsStct.Octave;
            inLay = ptsStct.Layer;
            
            if coder.isColumnMajor
                layout = '-col';
                computeCoreFcnName = 'extractSIFTComputeColumnMajor';
                assignOutputFcnName = 'extractSIFTAssignOutputColumnMajor';
            else
                layout = '-row';
                computeCoreFcnName = 'extractSIFTComputeRowMajor';
                assignOutputFcnName = 'extractSIFTAssignOutputRowMajor';
            end
            
            % Extract SIFT features
            numPtsOut(1) = coder.ceval(...
                layout, computeCoreFcnName,...
                coder.ref(Iu8), ...
                nRows, nCols, ...
                coder.ref(inLoc), coder.ref(inScl), ...
                coder.ref(inMet), coder.ref(inOri), ...
                coder.ref(inOct), coder.ref(inLay), ...
                int32(size(inLoc,1)), ...
                coder.ref(pVPts), ...
                coder.ref(pFtrs));
            % because we need to update the pointers as input/output
            % arguments, we are passing in the pointers to these
            % pointers
            
            % Declare and initialize output variables. Variables are
            % declared varsize because numPotsOut is not constant
            coder.varsize('Location',     [inf, 2]);
            coder.varsize('Scale',        [inf, 1]);
            coder.varsize('Metric',       [inf, 1]);
            coder.varsize('Orientation',  [inf, 1]);
            coder.varsize('Octave',       [inf, 1]);
            coder.varsize('Layer',        [inf, 1]);
            vLoc = coder.nullcopy(zeros(numPtsOut, 2, 'single'));
            vScl = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vMet = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vOri = coder.nullcopy(zeros(numPtsOut, 1, 'single'));
            vOct = coder.nullcopy(zeros(numPtsOut, 1, 'int32'));
            vLay = coder.nullcopy(zeros(numPtsOut, 1, 'int32'));
            % similarly for the output feature matrix
            coder.varsize('features',     [inf, inf]);
            ftrs = coder.nullcopy(zeros(numPtsOut, 128, 'single'));
            
            % Copy the output from where the raw output pointers point to
            % to the actual storages declared above
            coder.ceval(...
                layout, assignOutputFcnName, ...
                pVPts, pFtrs, ...
                coder.ref(vLoc), ...
                coder.ref(vScl), ...
                coder.ref(vMet), ...
                coder.ref(vOri), ...
                coder.ref(vOct), ...
                coder.ref(vLay), ...
                coder.ref(ftrs));
            
            % Tie the point information to the returned point structure
            vPts.Location    = vLoc;
            vPts.Scale       = vScl;
            vPts.Metric      = vMet;
            vPts.Orientation = vOri;
            vPts.Octave      = vOct;
            vPts.Layer       = vLay;
        end
    end
end
