function [tessdata, model] = getModelInfo(modelName, isCustomLanguage)
% Return the path to the tessdata folder and the language string.

% Copyright 2022-2023 The MathWorks, Inc.
%#codegen

    if isCustomLanguage     
        [tessdata, model] = vision.internal.ocr.getCustomModelInfo(modelName);
    else
        [tessdata, model] = vision.internal.ocr.getBuiltinModelInfo(modelName);
    end
end