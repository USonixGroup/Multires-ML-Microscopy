function dataReleased(this, ~, ~)
%DATARELEASED  React to the data source being uninstalled.

%   Copyright 2022 The MathWorks, Inc.

% If we have no data, restore the default, which is Intensity 0x0.
emptyDataString = '';

hUIMgr = this.Application.getGUI;
if isempty(hUIMgr)
    hDims = this.DimsStatus;
    if ~isempty(hDims)
        hDims.Text = emptyDataString;
    end
else
    hDims = hUIMgr.findchild({'StatusBar','StdOpts','vipscopes.PointCloudVisual Dims'});
    if hDims.IsRendered
        hDims.WidgetHandle.Text = emptyDataString;
    else
        hDims.setWidgetPropertyDefault('Text', emptyDataString);
    end
end

% 
% [EOF]