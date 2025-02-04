% OkSkipDlg Dialog with OK and Skip buttons

% Copyright 2021 The MathWorks, Inc.

classdef OkSkipDlg < vision.internal.uitools.AbstractDlg
    properties
       OkButton;
       SkipButton;
       IsSkipped = true;
    end
    
    properties(Access=protected)
        ButtonSize = [60, 20];
        ButtonHalfSpace = 10;
    end
    
    methods
        function this = OkSkipDlg(tool, dlgTitle)
            this = this@vision.internal.uitools.AbstractDlg(...
                tool, dlgTitle);
        end
       
        %------------------------------------------------------------------
        function createDialog(this)
            createDialog@vision.internal.uitools.AbstractDlg(this);
            addOK(this);
            addSkip(this);
        end
    end
    
    methods(Abstract, Access=protected)
        onOK(this, ~, ~);
    end
    
    methods(Access=protected)
       %------------------------------------------------------------------
        function addOK(this)
            x = this.DlgSize(1) / 2 - this.ButtonSize(1) - this.ButtonHalfSpace;
            this.OkButton = uicontrol('Parent', this.Dlg, 'Callback', @this.onOK,...              
                'Position', [x, 10, this.ButtonSize], ...
                'FontUnits', 'normalized', 'FontSize', 0.6,'String',...
                getString(message('MATLAB:uistring:popupdialogs:OK')));
        end
        
        %------------------------------------------------------------------
        function addSkip(this)
            x = this.DlgSize(1) / 2 + this.ButtonHalfSpace;
            this.SkipButton = uicontrol('Parent', this.Dlg, ...
                'Callback', @this.onSkip,...
                'Position',[x, 10, this.ButtonSize], ...
                'FontUnits', 'normalized', 'FontSize', 0.6, 'String',...
                getString(message('vision:imageLabeler:SkipButton')));
        end                
        
        %------------------------------------------------------------------
        function onSkip(this, ~, ~)            
            close(this);
        end
                
        %------------------------------------------------------------------
        function onKeyPress(this, ~, evd)
            switch(evd.Key)
                case {'return','space'}
                    onOK(this);
                case {'escape'}
                    onSkip(this);
            end
        end

    end
end
