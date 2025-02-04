function checkStrongestRegions(N, name)
%

%   Copyright 2016-2020 The MathWorks, Inc.

if isinf(N)
    % OK, use all regions.
else
    validateattributes(N, ...
        {'numeric'},...
        {'scalar', 'real', 'positive', 'integer', 'nonempty', 'finite', 'nonsparse'}, ...
        name, 'NumStrongestRegions');
end
