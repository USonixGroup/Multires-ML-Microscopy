function setup(this, hVisParent)
%SETUP Setup the Visual

%   Copyright 2022-2023 The MathWorks, Inc.

    % Create figure to draw into
    hVisParent.BackgroundColor = [0 0 0];
    hFigure = this.Application.Parent;
    hAxes = newplot(hFigure);
    hAxes.Parent  = hVisParent;
    this.Axes = hAxes;

    hgaddbehavior(hAxes, uiservices.getPlotEditBehavior('select'));

    this.XLimit = [-50 50];
    this.YLimit = [-50 50];
    this.ZLimit = [-5 40];
    this.PcplayerObj = pcplayer(this.XLimit, this.YLimit , this.ZLimit , 'MarkerSize',this.MarkerSize, 'Parent',this.Axes);

    hide(this.PcplayerObj);
    hPrimitive = findall(this.PcplayerObj.Axes, 'Tag', 'pcviewer');
    this.Primitive = hPrimitive;

    hgaddbehavior(hAxes, uiservices.getPlotEditBehavior('select'));

if ~isempty(this.Application.DataSource)
    dataSourceChanged(this);
end
% [EOF]
