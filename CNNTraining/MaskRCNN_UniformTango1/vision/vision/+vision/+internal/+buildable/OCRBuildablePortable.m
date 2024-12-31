classdef OCRBuildablePortable < coder.ExternalDependency
% OCRBuildablePortable - Buildable for portable version of ocr function

%   Copyright 2024 The MathWorks, Inc.
    %#codegen

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'ocr';
        end

        %------------------------------------------------------------------
        % supports non-host target
        %------------------------------------------------------------------
        function b = isSupportedContext(~)
            b = true;
        end

        %------------------------------------------------------------------
        % Update the buildInfo with all the required headers and libraries
        %------------------------------------------------------------------
        function updateBuildInfo(buildInfo, ~)
            % Add leptonica header path to buildinfo
            leptonicaHeaderPath = fullfile(matlabroot,'toolbox','vision','builtins','src', ...
                'thirdparty', 'leptonica', 'leptonica');
            leptonicaHeaderFiles = dir(fullfile(leptonicaHeaderPath,'*.h'));
            buildInfo.addIncludePaths(leptonicaHeaderPath);
            buildInfo.addIncludeFiles({leptonicaHeaderFiles.name});

            leptonicaHeaderPath1 = fullfile(matlabroot,'toolbox','vision','builtins','src', ...
                'thirdparty', 'leptonica');
            buildInfo.addIncludePaths(leptonicaHeaderPath1);

            % Add tesseract header path to buildinfo
            tesseractHeaderPath1 = fullfile(matlabroot,'toolbox','vision','builtins','src', ...
                'thirdparty', 'tesseract', 'include');
            tesseractHeaderFiles = dir(fullfile(tesseractHeaderPath1,'*.h'));
            buildInfo.addIncludePaths(tesseractHeaderPath1);
            buildInfo.addIncludeFiles({tesseractHeaderFiles.name});

            % Flags to link dependant 3p libraries
            arch            = computer('arch');
            pathBinArch     = fullfile(matlabroot, 'bin', arch);
            group = 'BlockModules';
            linkPriority    = '';
            linkPrecompiled = true;
            linkLinkonly    = true;
            switch arch
                case {'win64'}
                    lib1 = 'tesseract50.lib';
                    lib2 = 'leptonica-1.83.0.lib';
                case {'glnxa64','maci64','maca64'}
                    tesseractLibPrefix = 'libtesseract';
                    leptonicaLibPrefix = 'libleptonica';
                    if strcmpi(arch, 'glnxa64')
                        lib1 = strcat(tesseractLibPrefix, '.so.5.0.1');
                        lib2 = strcat(leptonicaLibPrefix, '.so.5');
                        linkStr1 = strcat(pathBinArch, filesep, tesseractLibPrefix, '.so.5.0.1');
                        linkStr2 = strcat(pathBinArch, filesep, leptonicaLibPrefix, '.so.5');
                        buildInfo.addLinkFlags(linkStr1);
                        buildInfo.addLinkFlags(linkStr2);
                    else
                        lib1 = strcat(tesseractLibPrefix, '.dylib');
                        lib2 = strcat(leptonicaLibPrefix, '.5.dylib');
                    end
            end
            buildInfo.addLinkObjects(lib1, pathBinArch, linkPriority, linkPrecompiled, linkLinkonly, group);
            buildInfo.addLinkObjects(lib2, pathBinArch, linkPriority, linkPrecompiled, linkLinkonly, group);
            srcPaths1 = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src', 'shared', 'ocrutils')}, [5 1]);
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src', 'shared', 'ocrutils', 'export', 'include', 'ocrutils')});
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src', 'shared', 'ocrutils', 'export', 'include')});
            buildInfo.addIncludeFiles({'Metadata.hpp', 'metadata_utils.hpp', 'MetadataCollection.hpp', 'ocrutils_published_c_api.hpp', 'ocrutils_util.hpp', 'pix_utils.hpp'});
            buildInfo.addSourceFiles({'Metadata.cpp', 'metadata_utils.cpp', 'MetadataCollection.cpp', 'ocrutils.cpp', 'pix_utils.cpp'}, srcPaths1);

            buildInfo.addLinkFlags('-lpthread');
            
            % PackNGo Note: The language data file used by tesseract is not
            % added to the buildInfo object because
            %   1) tesseract requires the language data file to be saved in
            %      a folder named 'tessdata'
            %   2) folders cannot be added to the buildInfo.
            %
            % Therefore, the packNGo zip file will not have the language
            % data file included in it.
        end

        %------------------------------------------------------------------
        % Calculate required params and call the algorithm
        %------------------------------------------------------------------
        function [asciiText, ocrMetadata] = tesseract(tessOpts, I, hasROI, resetParameters)

            coder.internal.errorIf(hasROI,'vision:ocr:codegenROIUnsupported');

            coder.cinclude('ocrutils_published_c_api.hpp');

            sizeI  = size(I);
            height = int32(sizeI(1));
            width  = int32(sizeI(2));
            assert(height > 0);
            assert(width  > 0);

            utf8Text  = coder.opaquePtr('char',coder.internal.null);
            txtLength = int32(0);

            textLayout   = vision.internal.codegen.ocr.ocrCodegenUtilities.nullTerminateString(tessOpts.textLayout);
            characterSet = vision.internal.codegen.ocr.ocrCodegenUtilities.nullTerminateString(tessOpts.characterSet);
            tessdata     = vision.internal.codegen.ocr.ocrCodegenUtilities.nullTerminateString(tessOpts.tessdata);
            lang         = vision.internal.codegen.ocr.ocrCodegenUtilities.nullTerminateString(tessOpts.lang);

            tessAPIHandle = coder.opaquePtr('void',coder.internal.null);

            if islogical(I)

                %> Invoke tesseract and return UTF-8 formatted text.
                txtLength = coder.ceval('tesseractRecognizeTextLogical', ...
                    coder.ref(tessAPIHandle),...
                    coder.ref(I),...
                    coder.ref(utf8Text),...
                    width,...
                    height,...
                    textLayout,...
                    coder.rref(characterSet),... % coder.rref usage forces characterSet var to be generated in code
                    tessdata,...
                    lang, ...
                    resetParameters, ...
                    uint8(tessOpts.ocrEngineMode));

            elseif strcmpi(class(I),'uint8')

                %> Invoke tesseract and return UTF-8 formatted text.
                txtLength = coder.ceval('tesseractRecognizeTextUint8', ...
                    coder.ref(tessAPIHandle),...
                    coder.ref(I),...
                    coder.ref(utf8Text),...
                    width,...
                    height,...
                    textLayout,...
                    coder.rref(characterSet),... % coder.rref usage forces characterSet var to be generated in code
                    tessdata,...
                    lang, ...
                    resetParameters, ...
                    uint8(tessOpts.ocrEngineMode));
            end

            if txtLength >= 0

                metadata = coder.opaquePtr('void',coder.internal.null);

                numChars      = coder.nullcopy(int32(0));
                numWords      = coder.nullcopy(int32(0));
                numTextlines  = coder.nullcopy(int32(0));
                numParagraphs = coder.nullcopy(int32(0));
                numBlocks     = coder.nullcopy(int32(0));

                coder.ceval('collectMetadata',...,
                    tessAPIHandle, ...,
                    coder.ref(metadata),...
                    coder.ref(numChars),...
                    coder.ref(numWords),...
                    coder.ref(numTextlines),...
                    coder.ref(numParagraphs),...
                    coder.ref(numBlocks));

                charBBox       = coder.nullcopy(zeros(numChars, 4));
                charWordIndex  = coder.nullcopy(zeros(numChars, 1,'int32'));
                charConfidence = coder.nullcopy(zeros(numChars, 1,'single'));

                wordBBox           = coder.nullcopy(zeros(numWords, 4));
                wordTextLineIndex  = coder.nullcopy(zeros(numWords, 1,'int32'));
                wordConfidence     = coder.nullcopy(zeros(numWords, 1,'single'));
                wordCharacterIndex = coder.nullcopy(zeros(numWords, 2,'int32'));

                textlineBBox           = coder.nullcopy(zeros(numTextlines, 4));
                textlineParagraphIndex = coder.nullcopy(zeros(numTextlines, 1,'int32'));
                textlineConfidence     = coder.nullcopy(zeros(numTextlines, 1,'single'));
                textlineCharacterIndex = coder.nullcopy(zeros(numTextlines, 2,'int32'));

                paragraphBBox           = coder.nullcopy(zeros(numParagraphs, 4));
                paragraphBlockIndex     = coder.nullcopy(zeros(numParagraphs, 1,'int32'));
                paragraphConfidence     = coder.nullcopy(zeros(numParagraphs, 1,'single'));
                paragraphCharacterIndex = coder.nullcopy(zeros(numParagraphs, 2,'int32'));

                blockBBox           = coder.nullcopy(zeros(numBlocks, 4));
                blockPageIndex      = coder.nullcopy(zeros(numBlocks, 1,'int32'));
                blockConfidence     = coder.nullcopy(zeros(numBlocks, 1,'single'));
                blockCharacterIndex = coder.nullcopy(zeros(numBlocks, 2,'int32'));

                coder.ceval('copyMetadata',...
                    metadata, ...
                    coder.ref(charBBox), coder.ref(charWordIndex),  coder.ref(charConfidence),...
                    coder.ref(wordBBox), coder.ref(wordTextLineIndex), coder.ref(wordConfidence), coder.ref(wordCharacterIndex),...
                    coder.ref(textlineBBox), coder.ref(textlineParagraphIndex), coder.ref(textlineConfidence), coder.ref(textlineCharacterIndex),...
                    coder.ref(paragraphBBox), coder.ref(paragraphBlockIndex), coder.ref(paragraphConfidence), coder.ref(paragraphCharacterIndex),...
                    coder.ref(blockBBox), coder.ref(blockPageIndex), coder.ref(blockConfidence), coder.ref(blockCharacterIndex));

                ocrMetadata.CharacterBBox = charBBox;
                ocrMetadata.CharacterWordIndex = charWordIndex;
                ocrMetadata.CharacterConfidence = charConfidence;

                ocrMetadata.WordBBox = wordBBox;
                ocrMetadata.WordTextLineIndex = wordTextLineIndex;
                ocrMetadata.WordConfidence = wordConfidence;
                ocrMetadata.WordCharacterIndex = wordCharacterIndex;

                ocrMetadata.TextLineBBox = textlineBBox;
                ocrMetadata.TextLineParagraphIndex = textlineParagraphIndex;
                ocrMetadata.TextLineConfidence = textlineConfidence;
                ocrMetadata.TextLineCharacterIndex = textlineCharacterIndex;

                ocrMetadata.ParagraphBBox = paragraphBBox;
                ocrMetadata.ParagraphBlockIndex = paragraphBlockIndex;
                ocrMetadata.ParagraphConfidence = paragraphConfidence;
                ocrMetadata.ParagraphCharacterIndex = paragraphCharacterIndex;

                ocrMetadata.BlockBBox = blockBBox;
                ocrMetadata.BlockPageIndex = blockPageIndex;
                ocrMetadata.BlockConfidence = blockConfidence;
                ocrMetadata.BlockCharacterIndex = blockCharacterIndex;

                tmp = coder.nullcopy(zeros(1,txtLength, 'uint8'));

                %> Copy UTF-8 encoded text into temporary buffer and cleanup.
                coder.ceval('copyTextAndCleanup', utf8Text,...
                    coder.ref(tmp),txtLength);

                %> Convert UTF-8 text to ASCII text.
                asciiText = char(vision.internal.codegen.ocr.ocrCodegenUtilities.utf8ToAscii(tmp));

                if numel(asciiText) ~= numChars
                    % Text does not match what is in metadata. Get text
                    % from metadata instead to maintain consistency.

                    metadataText = coder.opaquePtr('char',coder.internal.null);

                    % get text from metadata
                    len = coder.nullcopy(zeros(1,'int32'));
                    len = coder.ceval('getTextFromMetadata', metadata, coder.ref(metadataText));

                    % copy text into output buffer
                    tmpText = coder.nullcopy(zeros(1,len, 'uint8'));
                    coder.ceval('copyTextAndCleanup', metadataText,...
                        coder.ref(tmpText),len);

                    %> Convert UTF-8 text to ASCII text.
                    ocrMetadata.Characters = char(vision.internal.codegen.ocr.ocrCodegenUtilities.utf8ToAscii(tmpText));

                else
                    ocrMetadata.Characters = '';
                end

                coder.ceval('cleanupMetadata', metadata);
                coder.ceval('cleanupTesseract', tessAPIHandle);

            else
                % An error occurred while running tesseract.

                coder.internal.errorIf(txtLength == int32(-1),...
                    'vision:ocr:codegenInitFailure');

                coder.internal.errorIf(txtLength == int32(-2), ...
                    'vision:ocr:codegenMemAllocFailure');

                coder.internal.errorIf(txtLength == int32(-3), ...
                    'vision:ocr:codegenInternalError');

                % codegen requires all execution paths to assign outputs
                asciiText   = char(zeros(1,0,'uint8'));
                ocrMetadata.CharacterBBox       = zeros(0, 4);
                ocrMetadata.CharacterWordIndex  = zeros(0, 1,'int32');
                ocrMetadata.CharacterConfidence = zeros(0, 1,'single');

                ocrMetadata.WordBBox = zeros(0, 4);
                ocrMetadata.WordTextLineIndex = zeros(0, 1,'int32');
                ocrMetadata.WordConfidence = zeros(0, 1,'single');
                ocrMetadata.WordCharacterIndex = zeros(0, 2,'int32');

                ocrMetadata.TextLineBBox = zeros(0, 4);
                ocrMetadata.TextLineParagraphIndex = zeros(0, 1,'int32');
                ocrMetadata.TextLineConfidence = zeros(0, 1,'single');
                ocrMetadata.TextLineCharacterIndex = zeros(0, 2,'int32');

                ocrMetadata.ParagraphBBox = zeros(0, 4);
                ocrMetadata.ParagraphBlockIndex = zeros(0, 1,'int32');
                ocrMetadata.ParagraphConfidence = zeros(0, 1,'single');
                ocrMetadata.ParagraphCharacterIndex = zeros(0, 2,'int32');

                ocrMetadata.BlockBBox = zeros(0, 4);
                ocrMetadata.BlockPageIndex = zeros(0, 1,'int32');
                ocrMetadata.BlockConfidence = zeros(0, 1,'single');
                ocrMetadata.BlockCharacterIndex = zeros(0, 2,'int32');

                ocrMetadata.Characters = '';
            end

        end
    end
end
