%vision.internal.ndt.validateOutlierRatio Validate 'OutlierRatio' parameter

% Copyright 2020 The MathWorks, Inc.

%#codegen
function validateOutlierRatio(in)

validateattributes(in, {'single', 'double'}, ...
    {'real','nonempty','scalar','>=',0,'<',1}, '', 'OutlierRatio');
end