function labelNames = returnCategoricalLabels(classNames, numBBoxes, labels)
%#codegen
coder.inline('never');
% do not generate CUDA code for this function

% get the class name of each bounding box detected.
labelCells = coder.nullcopy(cell(numBBoxes, 1));
for i=1:numBBoxes
    % transpose to write row vector of char array
    labelCells{i, 1} = nonzeros(classNames(labels(i),:))';
end

% Grow cell array of valueset dynamically.
% Maintaining a varsized labelCells avoids the issue of
% bloating of IR causing slow code-generation for
% large const fixed-sized structs.
% Refer: g2286665
valueset = {};
upperBound = size(classNames,1);
coder.varsize('valueset',[1 upperBound],[0 1]);
for i = 1:upperBound
    valueset{end + 1} = nonzeros(classNames(i,:))';
end

labelNames = categorical(labelCells, valueset);
end