classdef readBarcodeBuildablePortable < coder.ExternalDependency %#codegen
    % readBarcodeBuildablePortable - Buildable for portable codegen
    % support for readBarcode
    % Copyright 2024 The MathWorks, Inc.

    methods (Static)
        %------------------------------------------------------------------
        function name = getDescriptiveName(~)
            name = 'readBarcodeBuildablePortable';
        end
        %------------------------------------------------------------------
        function isSupported = isSupportedContext(~)            
            isSupported = true; % Supports non-host target
        end
        %------------------------------------------------------------------
        function updateBuildInfo(buildInfo, ~)

            % Add source files
            srcPaths = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src', 'shared', 'barcode')}, [2 1]);
            buildInfo.addSourceFiles({'readBarCodeCore.cpp', 'ImageLuminanceSource.cpp'}, srcPaths);
            % Add header files
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox','vision','builtins','src',...
                'shared', 'barcode', 'export', 'include', 'barcode')});
            buildInfo.addIncludeFiles({'readBarcode_published_c_api.hpp', ...
                'ImageLuminanceSource.h','vision_defines.h'});

            % For all platforms, includes ALL ZXing header and source files.            
            zxingSourcePath = fullfile(matlabroot,'toolbox','vision',...
                'builtins','src','thirdparty','ZXing','src');
            zxingSourceFiles = dir(fullfile(zxingSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({zxingSourceFiles.name});
            buildInfo.addSourcePaths(zxingSourcePath);

            qrSourcePath = fullfile(zxingSourcePath,'qrcode');
            qrSourceFiles = dir(fullfile(qrSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({qrSourceFiles.name});
            buildInfo.addSourcePaths(qrSourcePath);

            onedSourcePath = fullfile(zxingSourcePath,'oned');
            onedSourceFiles = dir(fullfile(onedSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({onedSourceFiles.name});
            buildInfo.addSourcePaths(onedSourcePath);

            rssSourcePath = fullfile(onedSourcePath,'rss');
            rssSourceFiles = dir(fullfile(rssSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({rssSourceFiles.name});
            buildInfo.addSourcePaths(rssSourcePath);


            aztecSourcePath = fullfile(zxingSourcePath,'aztec');
            aztecSourceFiles = dir(fullfile(aztecSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({aztecSourceFiles.name});
            buildInfo.addSourcePaths(aztecSourcePath);

            datamatrixSourcePath = fullfile(zxingSourcePath,'datamatrix');
            datamatrixSourceFiles = dir(fullfile(datamatrixSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({datamatrixSourceFiles.name});
            buildInfo.addSourcePaths(datamatrixSourcePath);

            maxicodeSourcePath = fullfile(zxingSourcePath,'maxicode');
            maxicodeSourceFiles = dir(fullfile(maxicodeSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({maxicodeSourceFiles.name});
            buildInfo.addSourcePaths(maxicodeSourcePath);

            pdf417SourcePath = fullfile(zxingSourcePath,'pdf417');
            pdf417SourceFiles = dir(fullfile(pdf417SourcePath,'*.cpp'));
            buildInfo.addSourceFiles({pdf417SourceFiles.name});
            buildInfo.addSourcePaths(pdf417SourcePath);

            textcodecSourcePath = fullfile(zxingSourcePath,'textcodec');
            textcodecSourceFiles = dir(fullfile(textcodecSourcePath,'*.cpp'));
            buildInfo.addSourceFiles({textcodecSourceFiles.name});
            buildInfo.addSourcePaths(textcodecSourcePath);

            zxingHeaderPath = fullfile(matlabroot,'toolbox','vision','builtins','src',...
                'shared', 'barcode', 'export', 'include', 'barcode', 'include', 'ZXing');
            zxingHeaderFiles = dir(fullfile(zxingHeaderPath,'**/*.h'));
            buildInfo.addIncludePaths(zxingHeaderPath);
            buildInfo.addIncludeFiles({zxingHeaderFiles.name});
        end
        %------------------------------------------------------------------
        function [msg, format, loc] = readBarcodeMultiReader(I, formats, tryHarderOneD, tryHarderTwoD)
            coder.inline('always');
            coder.cinclude('readBarcode_published_c_api.hpp');

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

                % Combine all the formats into a single character vector,
                % if more than one format is provided.
                % The cellstr does not null terminate strings,
                % so nullcharacter is explicitly added
                nullCharacter = 0;
                formatsProc = [strjoin(formats,'') nullCharacter];

            else
                % If format is provided as a character vector (single
                % format)

                numFormats = int32(1);
                formatLength = int32([strlength(formats) 0]);
                formatsProc = [formats 0];
            end

            % Stores the length of the message and format, and size of the
            % location

            locSize = int32(0);
            msgLen = int32(0);
            formatLen = int32(0);

            % Stores the referece to the result object

            resultObj = coder.opaquePtr('void', coder.internal.null);

            fcnName = 'multiFormatDetectDecode';

            if coder.isColumnMajor
                Iin = I;
            else
                % Convert image for row-major
                Iin = I(:);
            end

            coder.ceval(fcnName, coder.ref(Iin), nRows, nCols, ...
                coder.ref(formatsProc), numFormats, formatLength, ...
                tryHarderOneD, tryHarderTwoD, coder.ref(resultObj), coder.ref(locSize), ...
                coder.ref(msgLen), coder.ref(formatLen));

            % Allocate memory for message, location and detected format
            % based on the sizes initialized
            msg = blanks(msgLen);
            loc = zeros(locSize,2);
            format = blanks(formatLen);

            if msgLen ~= 0 && formatLen ~= 0
                outputFcn = 'initializeOutput';

                % Initializes the output variables, message, location and
                % detected format
                coder.ceval(outputFcn, coder.ref(msg), coder.ref(loc), ...
                    coder.ref(format), resultObj);
            end

            % Deleting the pointer from memory
            coder.ceval('deleteResultPtr', resultObj);
        end
    end
end
