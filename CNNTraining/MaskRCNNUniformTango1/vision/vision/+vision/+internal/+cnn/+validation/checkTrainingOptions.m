function checkTrainingOptions(options, name, allowValidationDatastore, isTrainingDataTable)
%

%   Copyright 2016-2023 The MathWorks, Inc.

switch nargin
    case 2
        allowValidationDatastore = false;
        isTrainingDataTable = true;
    case 3
        isTrainingDataTable = true;
end

validateattributes(options, {'nnet.cnn.TrainingOptions'}, {}, name);
assertMiniBatchSolver(options, name)

validateDataFormatsAndMetricsAreEmpty(options, name);

vData = options.ValidationData;
if ~isempty(vData)
    if allowValidationDatastore
        iValidateValidationDatastore(vData,isTrainingDataTable);
    else
        error(message('vision:rcnn:validationDataNotSupported',name));
    end
end

end

function iValidateValidationDatastore(vDatastore, isTrainingDataTable)
    if isTrainingDataTable
        % When training data is a table, validation data cannot be provided.
        error(message('vision:ObjectDetector:validationDataMustBeDatastore'));
    end
    if ~isDatastore(vDatastore)
        error(message('vision:ObjectDetector:invalidValidationData'));
    end
    try
        vision.internal.inputValidation.checkGroundTruthDatastore(vDatastore);
    catch cause 
        msg = message('vision:ObjectDetector:invalidValidationData');
        throw(addCause(MException(msg),cause));
    end
end

function tf = isDatastore(ds)
    tf = isa(ds, 'matlab.io.Datastore') || ...
        isa(ds, 'matlab.io.datastore.Datastore');
end