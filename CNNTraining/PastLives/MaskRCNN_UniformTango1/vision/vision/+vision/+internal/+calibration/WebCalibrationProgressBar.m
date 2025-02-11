% CalibrationProgressBar Progress bar dialog for camera calibration

%   Copyright 2014-2022 The MathWorks, Inc.
classdef WebCalibrationProgressBar < handle
    properties(Access=private)
        isEnabled = true;
        hWaitBar = [];
        Messages = {};
        Percentages = [];
        CurrentState = 1;
    end

    methods
        %------------------------------------------------------------------
        function this = WebCalibrationProgressBar(isEnabled, parent, messages, percentages)
            this.isEnabled = isEnabled;
            if this.isEnabled
                this.Messages = messages;
                this.Percentages = percentages;
                
                dlgTitle = vision.getMessage('vision:caltool:CalibrationProgressBarTitle');
                dlgMsg   = vision.getMessage(this.Messages{1});
                this.hWaitBar = uiprogressdlg(parent, 'Title', dlgTitle, ...
                    'Message', dlgMsg);
            end
        end

        %------------------------------------------------------------------
        function update(this)
            if this.isEnabled && this.CurrentState <= numel(this.Messages)
                this.CurrentState = this.CurrentState + 1;
                this.hWaitBar.Value = this.Percentages(this.CurrentState);
                this.hWaitBar.Message = getString(message(this.Messages{this.CurrentState}));
            end
        end

        %------------------------------------------------------------------
        function delete(this)
            % close the progress bar window
            if this.isEnabled && ~isempty(this.hWaitBar) && isvalid(this.hWaitBar)
                close(this.hWaitBar);
            end
        end                
    end
end
