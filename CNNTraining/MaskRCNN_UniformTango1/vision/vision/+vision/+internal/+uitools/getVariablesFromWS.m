function [variable_data, var_name, user_canceled] = getVariablesFromWS(var_types,...
    var_disp_names, location)
%

%   Copyright 2016-2023 The MathWorks, Inc.

fig_height = 380;
fig_width  = 320;
fig_size = [fig_width fig_height];

if nargin < 3
    location = getCenteredPosition;
end

title = getString(message('images:privateUIString:importFromWorkspace'));


dlg = images.internal.app.utilities.VariableDialog(location(1:2),title,...
    getString(message('images:segmenter:variables')),var_disp_names{1});
% Overwrite the OK and Cancel Button tags  here so that the webfigures
% tests are not affected
dlg.FigureHandle.Tag = 'cvImportFromWS';
dlg.Ok.Tag = 'okButton';
dlg.Cancel.Tag = 'cancelButton';
dlg.Table.Tag = sprintf('%sList',lower(var_disp_names{1}));

wait(dlg);

if dlg.Canceled
    variable_data = [];
else
    variable_data = evalin('base',dlg.SelectedVariable);
end

user_canceled = dlg.Canceled;
var_name = dlg.SelectedVariable;

%----------------------------------
function pos = getCenteredPosition
    % Returns the position of the import dialog
    % centered on the screen.

    old_units = get(0,'Units');
    set(0,'Units','Pixels');
    screen_size = get(0,'ScreenSize');
    set(0,'Units', old_units);

    lower_left_pos = 0.5 * (screen_size(3:4) - fig_size);
    pos = [lower_left_pos fig_size];
end % getCenteredPosition

end