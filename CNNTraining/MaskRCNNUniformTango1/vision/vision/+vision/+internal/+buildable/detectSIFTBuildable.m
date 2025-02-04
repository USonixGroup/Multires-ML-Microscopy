classdef detectSIFTBuildable < coder.ExternalDependency %#codegen
    %

    % Copyright 2021-2023 The MathWorks, Inc.
    
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'detectSIFTFeatures';
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
            buildInfo.addSourceFiles({'detectSIFTCore.cpp', ...
                'cgCommon.cpp', ...
                'mwsift.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'detectSIFTCore_api.hpp', ...
                'cgCommon.hpp', ...
                'xfeatures2d_sift_mw.hpp'});
            
            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, ...
                'detectSIFT');
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function points = detectSIFT(Iu8, contrastThreshold, edgeThreshold, ...
                             numLayersInOctave, sigma)
            
            coder.inline('always');
            coder.cinclude('detectSIFTCore_api.hpp');
            
            ptrKeypoints = coder.opaquePtr('void', coder.internal.null);
            
            % call function
            numOut = int32(0);
            
            nRows = int32(size(Iu8,1));
            nCols = int32(size(Iu8,2));
            
            % Detect SIFT Features
            if coder.isColumnMajor
                numOut(1) = coder.ceval('-col', 'detectSIFTComputeColumnMajor',...
                    coder.ref(Iu8), ...
                    nRows, nCols,...
                    contrastThreshold, edgeThreshold, ...
                    numLayersInOctave, sigma,...
                    coder.ref(ptrKeypoints));
            else
                numOut(1) = coder.ceval('-row', 'detectSIFTComputeRowMajor',...
                    coder.ref(Iu8), ...
                    nRows, nCols,...
                    contrastThreshold, edgeThreshold, ...
                    numLayersInOctave, sigma,...
                    coder.ref(ptrKeypoints));
            end
            
            coder.varsize('Location',[inf 2]);
            coder.varsize('Scale',[inf 1]);
            coder.varsize('Metric',[inf 1]);
            coder.varsize('Orientation',[inf 1]);
            coder.varsize('Octave',[inf 1]);
            coder.varsize('Layer',[inf 1]);
            
            location    = coder.nullcopy(zeros(numOut,2,'single'));
            scale       = coder.nullcopy(zeros(numOut,1,'single'));
            metric      = coder.nullcopy(zeros(numOut,1,'single'));
            orientation = coder.nullcopy(zeros(numOut,1,'single'));
            octave      = coder.nullcopy(zeros(numOut,1,'int32'));
            layer       = coder.nullcopy(zeros(numOut,1,'int32'));
            
            % Copy detected SIFT Features to output
            if coder.isColumnMajor
                coder.ceval('-col','detectSIFTAssignOutputsColumnMajor', ptrKeypoints, ...
                    coder.ref(location),...
                    coder.ref(scale),...
                    coder.ref(metric),...
                    coder.ref(orientation),...
                    coder.ref(octave),...
                    coder.ref(layer));
            else
                coder.ceval('-row','detectSIFTAssignOutputsRowMajor', ptrKeypoints, ...
                    coder.ref(location),...
                    coder.ref(scale),...
                    coder.ref(metric),...
                    coder.ref(orientation),...
                    coder.ref(octave),...
                    coder.ref(layer));
            end
            
            points.Location    = location;
            points.Scale       = scale;
            points.Metric      = metric;
            points.Orientation = orientation;
            points.Octave      = octave;
            points.Layer       = layer;
            
        end
    end
end
