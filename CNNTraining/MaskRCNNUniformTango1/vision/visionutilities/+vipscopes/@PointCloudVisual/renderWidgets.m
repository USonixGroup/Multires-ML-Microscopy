function renderWidgets(this)
%

% Copyright 2022 The MathWorks, Inc.

if ispc
    w = 80;
else
    w = 104;
end

this.DimsStatus = spcwidgets.Status( ...
    this.Application.Handles.statusBar, ...
    'Tag', [sprintf('%s Dims', class(this)) 'Status'], ...
    'Width', w);
    
setup(this, this.Application.Handles.visualizationPanel);