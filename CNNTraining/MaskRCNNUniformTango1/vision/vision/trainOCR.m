function [modelFileName, info] = trainOCR(trainingData, modelName, baseModelName, options)

% Copyright 2022-2023 The MathWorks, Inc.

    arguments
        trainingData   {mustBeNonempty, vision.internal.ocr.validateOCRDataStoreInput}
        
        modelName      {mustBeTextScalar, mustBeNonzeroLengthText, iValidateModelName}

        baseModelName  {mustBeTextScalar, mustBeNonzeroLengthText}

        options        {mustBeNonempty, mustBeScalarOrEmpty, ... % Validation for Scalar-NonEmpty value.
                        mustBeA(options, 'ocrTrainingOptions')}
    end
    
    % Parse base model name to get model full path.
    baseModelFile = iParseAndValidateBaseModelName(baseModelName);

    % Configure verbose printer.
    printer = vision.internal.MessagePrinter.configure(options.Verbose);

    % Print header.
    iPrintHeader(printer, modelName, baseModelName);

    % Configure the OCR trainer.
    trainer = vision.internal.ocr.OCRTrainerImpl(trainingData, modelName, baseModelFile, options);
    
    % Step 1: Generate LSTMF files.
    trainer.generateLSTMFFiles(printer);
    
    % Step 2: Decompose base model into its components.
    trainer.extractBaseModelComponents();

    % Step 3: Create unicharset.
    trainer.createUnicharset();
    
    % Step 4: Create a starter model file.
    starterModelCharacterSet = trainer.createStarterModelFile();

    % Print model character set.
    printCharacterSet(printer, starterModelCharacterSet);
    
    % Step 5: Run training.
    convergenceReason = trainer.runTraining();
    
    % Step 6: Generate the final model.
    [modelFileName, info] = trainer.generateFinalModelFile();
    
    % Print footer.
    iPrintFooter(printer, convergenceReason, modelFileName);

    % Clean up temporary artifacts.
    trainer.cleanUp();
end

%--------------------------------------------------------------------------
% Validator function for model name to check for a valid file name across
% platforms as this will be used to name various files during training.
%--------------------------------------------------------------------------
function iValidateModelName(modelName)
    
    modelName = char(modelName);

    % Check for alpha numeric characters.
    isAlphaNumericCharacter = isstrprop(modelName, "alphanum");
    hasOnlyAlphaNumericCharacters = all(isAlphaNumericCharacter);
    
    if ~hasOnlyAlphaNumericCharacters
        % Check for valid special characters if model name contains
        % non-alpha numeric characters.

        specialCharacters = modelName(~isAlphaNumericCharacter);
        validSpecialCharacters = '_-';
        validSpecialCharactersIndices = regexp(specialCharacters, ['[' validSpecialCharacters ']'], 'all');
        hasOnlyValidSpecialCharacters = length(validSpecialCharactersIndices) == length(specialCharacters);

        if ~hasOnlyValidSpecialCharacters
            % model name contains characters that are not alphanumeric and
            % valid special characters.
            error(message('vision:trainOCR:invalidModelName'));
        end
    end
end

%--------------------------------------------------------------------------
% Helper to parse and validate base model name to get model full path.
%--------------------------------------------------------------------------
function filename = iParseAndValidateBaseModelName(baseModelName)
    
    modelFileExtension = '.traineddata';
    
    isCustomModel = contains(baseModelName, modelFileExtension);
    
    if ~isCustomModel
        if  contains(baseModelName, "-fast")
            % Check for fast models.
            error(message('vision:trainOCR:fastBaseModelsNotSupported'));
        end

        % Perform additional validation on the base model name.
        canUseFastModel = coder.const(false);
        doAcceptMultipleModels = coder.const(false);
        baseModelName = vision.internal.ocr.checkModel(char(baseModelName), ...
            'Model', doAcceptMultipleModels, canUseFastModel);
    
        % Parse model name to get full file path.
        [tessdata, model] = vision.internal.ocr.getModelInfo(baseModelName, ...
            isCustomModel);
        filename = [fullfile(tessdata, model) modelFileExtension];
    else
        filename = baseModelName;
    end

    [~, ~, isQuantizedModel] = tesseractReadTraineddata(char(filename));
    if isQuantizedModel
        error(message('vision:trainOCR:quantizedModelNotSupported'))
    end
end

%--------------------------------------------------------------------------
function iPrintHeader(printer, modelName, baseModelName)

    printer.print('*************************************************************************\n');
    
    checkpointExtension = '.checkpoint.traineddata';
    isTrainingResuming = contains(baseModelName, checkpointExtension);
    if isTrainingResuming
        printer.printMessage("vision:trainOCR:trainingHeaderResume");
    else
        printer.printMessage("vision:trainOCR:trainingHeaderStart");
    end

    printer.linebreak;
    
    printer.printMessage("vision:trainOCR:modelName", modelName);

    % Format blackslash character for printing using sprintf. This is especially 
    % required in windows machines where the path separator is a backslash.
    baseModelName = iFormatBackslash(baseModelName);
    if isTrainingResuming
        printer.printMessage("vision:trainOCR:checkpoint", baseModelName);
    else
        printer.printMessage("vision:trainOCR:baseModelName", baseModelName);
    end
end

%--------------------------------------------------------------------------
function printCharacterSet(printer, characterSet)

    printer.linebreak;
    printer.linebreak;
    
    % Format black slash character for printing using sprintf.
    characterSet = iFormatBackslash(characterSet);

    printer.printMessage("vision:trainOCR:characterSet", characterSet);
    printer.linebreak;
end

%--------------------------------------------------------------------------
function iPrintFooter(printer, convergenceReason, modelFileName)

    printer.linebreak;
    printer.printMessage("vision:trainOCR:trainingComplete");
    convergenceReason = vision.getMessage(convergenceReason);
    printer.printMessage("vision:trainOCR:exitCondition", convergenceReason);
    
    printer.linebreak;

    % Format blackslash character for printing using sprintf. This is especially 
    % required in windows machines where the path separator is a backslash.
    modelFileName = iFormatBackslash(modelFileName);
    
    printer.printMessage("vision:trainOCR:modelFileName", modelFileName);

    printer.print('*************************************************************************\n');
    printer.linebreak;
end

%--------------------------------------------------------------------------
% Helper function to string format backslash characters to avoid sprintf
% misunderstanding it as a start of an escape sequence. The formatting is a
% NO-OP if backslash is not already present in the input text.
%--------------------------------------------------------------------------
function replacedText = iFormatBackslash(inputText)

    replacedText = strrep(inputText, '\', '\\');
end