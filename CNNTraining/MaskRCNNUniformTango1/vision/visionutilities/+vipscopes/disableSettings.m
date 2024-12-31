function disableSettings(this)
%

%   Copyright 2022 The MathWorks, Inc.

hApp = this.Application;


dp = hApp.Visual.DialogMgr;
if ~isempty(dp)
    % Shut down the dialog manager
    closeDialogPanel(dp);
end

hApp.Visual.DialogMgr = [];

