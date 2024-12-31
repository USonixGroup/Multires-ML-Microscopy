function quantizedModelFileName = quantizeOCR(inputFileName, quantizedModelName, options)

% Copyright 2022-2023 The MathWorks, Inc.

    arguments
        inputFileName           {mustBeTextScalar, mustBeNonzeroLengthText, validateInputFileName}
        quantizedModelName      {mustBeTextScalar, mustBeNonzeroLengthText}
        options.OutputLocation  {mustBeTextScalar, mustBeNonzeroLengthText, ...
                                 mustBeFolder, vision.internal.inputValidation.checkWritePermissions} = pwd
    end

    % Cast inputs to string.
    inputFileName = string(inputFileName);
    quantizedModelName = string(quantizedModelName);
    options.OutputLocation = string(options.OutputLocation);

    % Find the full path of input file.
    inputModelFileName = iGetfullpath(inputFileName);

    % Get the quantized model file name with .traineddata extension.
    quantizedModelFileName = fullfile(options.OutputLocation, quantizedModelName);
    quantizedModelFileName = quantizedModelFileName + ".traineddata";

    % Copy original file to the output location with the quantized model name.
    iCopyFile(inputModelFileName, quantizedModelFileName);

    % Quantize the OCR model.
    tesseractQuantizer(char(quantizedModelFileName));
end

%--------------------------------------------------------------------------
function validateInputFileName(inputFileName)
    
    [~, ~, ext] = fileparts(inputFileName);
    if ~strcmpi(ext, '.traineddata')
        error(message('vision:ocr:invalidModelFileExtension'))
    end

    if exist(inputFileName,'file') ~= 2
        error(message('vision:ocr:modelFileDoesNotExist', inputFileName))
    end
    
    fid = fopen(inputFileName, 'r');
    if fid < 0
        error(message('vision:ocr:modelFileReadError', inputFileName))
    end
    fclose(fid);

    [~, ~, isQuantizedModel] = tesseractReadTraineddata(char(inputFileName));
    if isQuantizedModel
        error(message('vision:ocr:quantizedModelNotSupported'))
    end
end

%--------------------------------------------------------------------------
function outputFileName = iGetfullpath(inputFileName)

    [filepath, ~, ~] = fileparts(inputFileName);

    if isempty(filepath)
        fullpath = which(inputFileName);
        outputFileName = fullpath;
    else
        outputFileName = inputFileName;
    end
end

%--------------------------------------------------------------------------
% iCopyFile is similar to MATLAB's copyfile function except that it does
% not copy the read-write permission information of the source file to the
% destination file, so that the destination file is always unprotected for
% read and write.
%--------------------------------------------------------------------------
function iCopyFile(src, dest)

    % Read the contents of the source file.
    srcFid = fopen(src, 'r');
    if srcFid < 0
        error(message('vision:ocr:modelFileReadError', src))
    end
    srcContent = fread(srcFid);
    fclose(srcFid);

    % Write the source contents onto the destination file.
    destFid = fopen(dest, 'w');
    if destFid < 0
        error(message('vision:ocr:modelFileWriteError', dest))
    end
    fwrite(destFid, srcContent);
    fclose(destFid);
end