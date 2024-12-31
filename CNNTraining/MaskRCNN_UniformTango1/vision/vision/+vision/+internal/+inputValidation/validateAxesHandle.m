% -------------------------------------------------------------------------
% Axes handle validation. Returns true if AX is a valid axes handle
% otherwise an error is thrown.
% -------------------------------------------------------------------------
function tf = validateAxesHandle(ax, uiaxesSupport)

%   Copyright 2013-2020 The MathWorks, Inc.

% UIAxes is supported by default. 
if nargin < 2
    uiaxesSupport = true;
end

tf = true;
if ~uiaxesSupport && isa(ax, 'matlab.ui.control.UIAxes')
    error(message('MATLAB:ui:uiaxes:NotSupported'));
end

if ~(isscalar(ax) && ishghandle(ax,'axes'))
    error(message('vision:validation:invalidAxesHandle'));
end
