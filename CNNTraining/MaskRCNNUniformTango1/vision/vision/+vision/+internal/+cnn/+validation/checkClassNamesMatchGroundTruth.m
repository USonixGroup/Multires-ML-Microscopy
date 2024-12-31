function checkClassNamesMatchGroundTruth(classnames, expectedNames, bgLabel)
% Verify that detector classnames still match those used in ground truth
% when resuming training.

%   Copyright 2016-2020 The MathWorks, Inc.

if istable(expectedNames)
    expectedNames = expectedNames.Properties.VariableNames(2:end);
end
expectedNames = [expectedNames bgLabel]';
if ~isequal(classnames, expectedNames)
    error(message('vision:rcnn:resumeClassNameMismatch'));        
end

