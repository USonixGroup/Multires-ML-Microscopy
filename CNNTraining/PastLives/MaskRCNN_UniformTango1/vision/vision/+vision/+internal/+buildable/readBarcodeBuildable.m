classdef readBarcodeBuildable < coder.ExternalDependency %#codegen
    % readBarcodeBuildable - encapsulate readBarcode implementation library

    % Copyright 2021-2022 The MathWorks, Inc.
    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'readBarcodeBuildable';
        end
        
        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end
        
        function updateBuildInfo(buildInfo, context)
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                'readBarcode', {});
            arch            = computer('arch');
            pathBinArch     = fullfile(matlabroot, 'bin', arch, filesep);

            % For all platforms, includes all link libraries.
            %     (win: dll, linux: so, mac: dylib) as nonBuildFiles
            
            % Platform specific link and non-build files
            switch arch
                case {'win64'}
                    nonBuildPath = fullfile(matlabroot, 'bin', arch);
                    nonBuildFiles = strcat(nonBuildPath, filesep, {'barcode.dll', 'libzxing-cpp.dll'});

                case {'glnxa64'}
                    nonBuildPath = pathBinArch;
                    nonBuildFiles = strcat(nonBuildPath, filesep, {'libmwbarcode.so', 'libzxing-cpp.so.1', 'libzxing-cpp.so.1.0.6'});

                case {'maci64','maca64'}
                    nonBuildPath = pathBinArch;
                    nonBuildFiles = strcat(nonBuildPath, filesep, {'libmwbarcode.dylib', 'libzxing-cpp.1.dylib', 'libzxing-cpp.dylib'});
            end
            group = 'BlockModules';
            buildInfo.addNonBuildFiles(nonBuildFiles, '', group);
        end
        
        function [msg, format, loc] = readBarcodeMultiReader(I, formats, tryHarderOneD, tryHarderTwoD)
            
            coder.inline('always');
            coder.cinclude('cvstCG_readBarcode.h');
            
            % Image width and height
            nRows = int32(size(I,1));
            nCols = int32(size(I,2));
            
            if iscellstr(formats) %#ok<ISCLSTR>
                
                % Number of formats input by the user and their lengths
                
                numFormats = int32(size(formats,2));
                formatLength = int32(strlength(formats));
                
                if numFormats == 1
                    formatLength = int32([strlength(formats) 0]);
                end
                
                % combine all the formats into a single character vector,
                % if more than one format is provided.
                % The cellstr does not null terminate strings, 
                % so nullcharacter is explicitly added
                nullCharacter = 0;
                formatsProc = [strjoin(formats,'') nullCharacter];
              
            else        
                % if format is provided as a character vector (single
                % format)
                
                numFormats = int32(1);
                formatLength = int32([strlength(formats) 0]);
                formatsProc = [formats 0];           
            end
            
            % stores the length of the message and format, and size of the
            % location
            
            locSize = int32(0);
            msgLen = int32(0);
            formatLen = int32(0);
            
            % stores the referece to the result object
            
            resultObj = coder.opaquePtr('void', coder.internal.null);
            
            fcnName = 'multiFormatDetectDecode';
            if coder.isColumnMajor
                coder.ceval(fcnName, coder.ref(I), nRows, nCols, ...
                    coder.ref(formatsProc), numFormats, formatLength, ...
                    tryHarderOneD, tryHarderTwoD, coder.ref(resultObj), coder.ref(locSize), ...
                    coder.ref(msgLen), coder.ref(formatLen));
                    
                % allocate memory for message, location and detected format
                % based on the sizes initialized 
                
                msg = blanks(msgLen);
                loc = zeros(locSize,2);
                format = blanks(formatLen);
                
                if msgLen ~= 0 && formatLen ~= 0
                    outputFcn = 'initializeOutput';
                    
                    % initializes the output variables, message, location and
                    % detected format
                    coder.ceval(outputFcn, coder.ref(msg), coder.ref(loc), ...
                        coder.ref(format), resultObj);
                end    
            else         
                % convert image to row-major 
                Irow = I(:);
                
                coder.ceval(fcnName, coder.ref(Irow), nRows, nCols, ...
                    coder.ref(formatsProc), numFormats, formatLength, ...
                    tryHarderOneD, tryHarderTwoD, coder.ref(resultObj), coder.ref(locSize), ...
                    coder.ref(msgLen), coder.ref(formatLen));
                
                % allocate memory for message, location and detected format
                % based on the sizes initialized 
                
                msg = blanks(msgLen);
                loc = zeros(locSize,2);
                format = blanks(formatLen);
                
                if msgLen ~= 0 && formatLen ~= 0
                    outputFcn = 'initializeOutput';
                    
                    % initializes the output variables, message, location and
                    % detected format
                    coder.ceval(outputFcn, coder.ref(msg), coder.ref(loc), coder.ref(format), resultObj);
                end
            end

            % deleting the pointer from memory
             coder.ceval('deleteResultPtr', resultObj);
        end
    end   
end
