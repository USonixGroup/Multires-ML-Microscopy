function checkNumRegionsToSample(x,fname)
%

%   Copyright 2018-2020 The MathWorks, Inc.

validateattributes(x,{'numeric'},...
    {'positive','integer','finite','nonsparse','real','nonempty'},...
    fname,'NumRegionsToSample');
