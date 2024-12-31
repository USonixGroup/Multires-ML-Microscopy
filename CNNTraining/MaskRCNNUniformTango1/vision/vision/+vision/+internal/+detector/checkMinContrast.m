function tf = checkMinContrast(x)
% validates MinContrast parameter value

%   Copyright 2013-2020 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

vision.internal.errorIfNotFixedSize(x, 'MinContrast');

validateattributes(x, {'double', 'single'}, ...
    {'nonempty', 'nonnan', 'nonsparse', 'real', 'scalar', 'finite', '>',0, '<',1},...
    'checkMinContrast', 'MinContrast');
tf = true;
