function checkOverlapRatio(range, fname, pname, numStages)
%

%   Copyright 2016-2020 The MathWorks, Inc.

if nargin==3
    numStages = 1;
end

if numStages == 1 || isrow(range) 
    % R-CNN and Fast R-CNN, or Faster R-CNN where all 4 stages use same
    % range values.
    iAssertAndCheckOneRow(range,fname,pname)
else
    % Faster R-CNN (four-stage or end-to-end, which has 2 stages)
    iAssertAndCheckMultipleRows(range,fname,pname,numStages);
end
end

%--------------------------------------------------------------------------
function iAssertAndCheckOneRow(range,fname,pname)

validateattributes(range, {'numeric'}, ...
    {'vector', 'numel', 2, 'real', 'finite', 'nonsparse', 'nonnegative', 'increasing', '<=', 1},...
    fname, pname);
end

%--------------------------------------------------------------------------
function iAssertAndCheckMultipleRows(range,fname,pname,numStages)
transposedRange = range'; % to allow increasing validation
validateattributes(transposedRange, {'numeric'}, ...
    {'size', [2 numStages], 'real', 'finite', 'nonsparse', 'nonnegative', 'increasing', '<=', 1},...
    fname, pname);
end
