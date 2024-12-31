function dataSourceChanged(this,~,~)
%DATASOURCECHANGED React to a new data source being installed.
%   React to a new data source being installed into the scope application.

%   Copyright 2022-2023 The MathWorks, Inc.

uiservices.setListenerEnable(this.ScalingChangedListener, false);
source = this.Application.DataSource;

% If the source is invalid, simply return.
if ~validateSource(this, source)
    return;
end

nInputs = getNumInputs(source);
maxDims = getMaxDimensions(source);

dataType = getDataTypes(source, 1);

 
if nInputs == 2
    this.ColorProvided = true;
elseif nInputs == 1
    this.ColorProvided = false;
end

if numel(maxDims(1,:)) > 2
    if(maxDims(1,3) ~= 3)
        error('Invalid organized input of point Clouds');
    end
    this.IsOrganized = true;
    s = 'Organized';
    sizeStr = sprintf('%dx%dx%d', maxDims(1,1), maxDims(1,2), maxDims(1,3));
else
    this.IsOrganized = false;
     s = 'Unorganized';
    sizeStr = sprintf('%dx%d', maxDims(1, 1), maxDims(1, 2));
end

this.DataType      = dataType;
this.OldDimensions = maxDims;
this.MaxDimensions = maxDims;

this.PointCloudInfo.update;

% Do not update the status bar if there is no data.
if isDataEmpty(source)
    return;
end

sizeStr = sprintf('%s:%s', s, sizeStr);

hUIMgr = getGUI(this.Application);
if isempty(hUIMgr)
    hDims = this.DimsStatus;
    if ~isempty(hDims)
        hDims.Text = sizeStr;
        hDims.Width = max(hDims.Width, largestuiwidth({sizeStr})+2);
    else
        hDims.Text = sizeStr;
    end
else
    hDims = hUIMgr.findchild({'StatusBar','StdOpts','vipscopes.PointCloudVisual Dims'});
    if hDims.IsRendered
        hDims.WidgetHandle.Text = sizeStr;
        hDims.WidgetHandle.Width = ...
            max(hDims.WidgetHandle.Width, largestuiwidth({sizeStr})+2);
    else
        hDims.setWidgetPropertyDefault('Text', sizeStr);
    end
end

% [EOF]