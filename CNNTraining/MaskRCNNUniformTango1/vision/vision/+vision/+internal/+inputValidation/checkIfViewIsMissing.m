function checkIfViewIsMissing(viewsOrViewIds, viewId)
%
%  Copyright 2020-2021 The MathWorks, Inc.
%#codegen

missingViewIdx = ~vision.internal.inputValidation.checkIfHasView(...
    viewsOrViewIds, viewId);

if any(missingViewIdx)
    missingViewIds = viewId(missingViewIdx);
    coder.internal.error('vision:viewSet:missingViewId', ...
    missingViewIds(1));
end
end