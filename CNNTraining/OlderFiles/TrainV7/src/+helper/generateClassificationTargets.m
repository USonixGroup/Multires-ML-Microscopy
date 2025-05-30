function [ assignedLabels ] = generateClassificationTargets (gTruthLabels, assignments,...
                                             positiveIndex, negativeIndex,...
                                             classNames, backgroundClassName)
                                         
    % Copyright 2020 The MathWorks, Inc.
    
    for i=1:size(gTruthLabels,1)
        assignedLabels{i} = helper.boxAssignmentUtils.boxLabelsFromAssignment(...
                                        assignments{i}, gTruthLabels{i}, ...
                                        positiveIndex{i}, negativeIndex{i}, ...
                                        classNames, backgroundClassName);
                                    
    end

% if size(assignedLabels, 1)==1
%     newlabs=vertcat(assignedLabels{1,1});
%     newlabs(isundefined(newlabs))='background';
%     assignedLabels{1,1}=newlabs;
% end




