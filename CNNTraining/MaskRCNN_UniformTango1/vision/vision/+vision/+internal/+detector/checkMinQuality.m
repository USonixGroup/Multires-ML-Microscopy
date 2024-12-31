function tf = checkMinQuality(x)
% validates MinQuality parameter value. 

%   Copyright 2013-2020 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

vision.internal.errorIfNotFixedSize(x,'MinQuality');

validateattributes(x,{'double','single'},...
    {'nonempty', 'nonnan', 'nonsparse', 'real', 'scalar', 'finite', '>=', 0, '<=', 1},...
    'checkMinQuality','MinQuality');

tf = true;
