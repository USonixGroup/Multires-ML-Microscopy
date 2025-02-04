function metrics = evaluateOCR(resultTxt, groundTruthTxt, options)

% Copyright 2022-2023 The MathWorks, Inc.

    arguments
        resultTxt              {mustBeNonempty, iValidateResultInput}
        groundTruthTxt         {mustBeNonempty, vision.internal.ocr.validateOCRDataStoreInput}
        options.Metrics (1,1) string = "all"
        options.Verbose (1,1) logical = true
    end
    
    options.Metrics = iValidateMetricSelection(options.Metrics);

    metrics = ocrMetrics.compute(resultTxt, groundTruthTxt, options);
end

%--------------------------------------------------------------------------
function iValidateResultInput(result)
    try 
        if iscell(result)
            validateattributes(result, {'cell'}, {'column'}, mfilename, "resultTxt")
            iValidateOCRTextArray(result)
        else
            vision.internal.ocr.validateOCRDataStoreInput(result);
        end
    catch ex
        isInputTypeInvalid = strcmpi(ex.identifier, 'MATLAB:invalidType') || ...
                strcmpi(ex.identifier, 'MATLAB:evaluateOCR:invalidType');
        if isInputTypeInvalid
            error(message('vision:ocr:invalidEvalResultsType'));
        else
            throw(ex);
        end
    end
end

%--------------------------------------------------------------------------
function metrics = iValidateMetricSelection(metrics)
    
    validMetrics = ["all", "character-error-rate", "word-error-rate"];
    metrics = validatestring(metrics, validMetrics, mfilename, "Metrics");
end

%--------------------------------------------------------------------------
function iValidateOCRTextArray(result)
    
    validator = @(x) validateattributes(x, {'ocrText'}, {'nonempty'}, mfilename, "resultTxt");
    cellfun(validator, result)
end