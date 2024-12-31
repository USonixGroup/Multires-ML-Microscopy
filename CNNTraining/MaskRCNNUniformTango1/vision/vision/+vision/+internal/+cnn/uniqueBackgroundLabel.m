function label = uniqueBackgroundLabel(classes)
% Use a background label that is not included in the ground truth.

%   Copyright 2016-2020 The MathWorks, Inc.

label = 'Background';
if istable(classes)
    classes = classes.Properties.VariableNames;
end
while ismember(label, classes)
    label = sprintf('%s_%d', label, randi(9));
end
