function lang = validateSupportPackageLanguages(userLang, canUseFastModel)
% Validate languages strings against those in the support package.

%   Copyright 2014-2023 The MathWorks, Inc.

%#codegen

validStrings = vision.internal.ocr.languagesInSupportPackage();
if canUseFastModel
    % Legacy models that does not have faster alternatives.
    unsupported = ismember(validStrings,{'mathequation','tagalog', 'esperantoalternative'});
    
    validStrings = [validStrings strcat(validStrings(~unsupported),'-fast')];
end
lang = validatestring(userLang, validStrings, 'ocr');



