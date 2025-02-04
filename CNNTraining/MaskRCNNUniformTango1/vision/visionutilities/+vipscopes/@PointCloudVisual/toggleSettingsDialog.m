function toggleSettingsDialog(this, src, varargin)
%toggleAxesDialog Toggle Triggers dialog visibility

%   Copyright 2022 The MathWorks, Inc.

if ishghandle(src)
  if isa(src,'matlab.ui.container.toolbar.ToggleTool')
    % called from a measurement panel toggle tool
    makeVisible = strcmp(get(src,'State'),'on');
  else    
    % called from a measurement panel context menu (uimenu)
    % (widget does not auto-check before entering)
    makeVisible = ~strcmp(get(src,'Checked'),'on');
  end
elseif ischar(src)
    % source is a string
    makeVisible = strcmp(src,'on');    
else
    % called from the uimgr.menugroup 
    makeVisible = strcmp(src.Checked,'on');
end

if ~ischar(src) || ~strcmp(src,'forcequit')
  % ensure dialog exists
  registerSettingsDialog(this, ...
      'Settings', ...
      @vipscopes.SettingsDialog, ...
      makeVisible);
    setSettingsDialogVisibility( ...
      this.Application.Visual.DialogMgr, ...
      'Settings', ...
      makeVisible);
end

set(this.AxesLimitsButton, 'State', uiservices.logicalToOnOff(makeVisible));
% [EOF]