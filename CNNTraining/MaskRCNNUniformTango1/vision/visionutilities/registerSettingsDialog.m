function dlg = registerSettingsDialog(this, dlgName, dlgConstructor, visible)
%registerMeasurementsDialogPanel create/register dialog with dialog manager

%   Copyright 2022 The MathWorks, Inc.

% Return early if we do not have enough information to make a dialog.
if isempty(dlgName) || isempty(dlgConstructor)
    return;
end

% Get the dialog manager from the visual
dp = this.Application.Visual.DialogMgr;

% create dialog manager if necessary
if isempty(dp)  
  dp =vipscopes.SettingsPanel(this.Application);  
  this.Application.Visual.DialogMgr = dp;
end

% Check to see if the content has already been created
dlg = findobj(dp.Dialogs,'Name', dlgName);

if isempty(dlg)
  % If not, call the constructor of the dialog
  hdlg = dlgConstructor(this, dlgName);
  
  % now create and register it with the dialog manager
  dp.createAndRegisterDialog(hdlg);

  % make this dialog and its associated panel visible.
  if visible
    setSettingsDialogVisibility(dp, dlgName, true);
  end
  
  % finally, return a handle to our dialog
  dlg = findobj(dp.Dialogs, 'Name', dlgName);

end


