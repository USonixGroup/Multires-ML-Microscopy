function [tessdata,model] = getBuiltinModelInfo(modelName)
% Return the path to the tessdata folder, the model alias.

% Copyright 2022-2023 The MathWorks, Inc.
%#codegen

    model = vision.internal.ocr.convertLanguageToAlias(modelName);
    
    useFastModel = contains(modelName,'-fast');
    tessdata = vision.internal.ocr.locateTessdataFolder(model, useFastModel);
end