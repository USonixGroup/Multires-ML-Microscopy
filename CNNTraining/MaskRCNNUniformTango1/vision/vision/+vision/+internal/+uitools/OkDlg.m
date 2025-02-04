% OkCancelDlg Dialog with an OK button.  
%
% This dialog can only be used to display information to the user. Pressing 
% the OK button simply closes the dialog. No data from the user is
% captured.

% Copyright 2014-2023 The MathWorks, Inc.

classdef OkDlg < vision.internal.uitools.AbstractDlg
    properties
        OkButton;
        ButtonTag = 'btnBoardOk';
    end
    
    properties(Access=private)
        ButtonSize = [60, 20];
    end
    
    methods
        function this = OkDlg(tool, dlgTitle, buttonTag)
            this = this@vision.internal.uitools.AbstractDlg(...
                tool, dlgTitle);
            if nargin > 2
                this.ButtonTag = buttonTag;
            end
        end
       
        %------------------------------------------------------------------
        function createDialog(this)
            createDialog@vision.internal.uitools.AbstractDlg(this);
            addOK(this);
        end
    end 
    
    
    methods(Access=private)
    %------------------------------------------------------------------
        function addOK(this)
            w = round(this.ButtonSize(1) / 2);
            this.OkButton = uibutton('Parent',this.Dlg, ...
                'ButtonPushedFcn', @(~, ~)this.close(),...
                'FontSize', 11, ...
                'Position',[round(this.DlgSize(1)/2)-w 10 2*w 20], 'Text', ...
                 getString(message('MATLAB:uistring:popupdialogs:OK')),...
                'Tag', this.ButtonTag);
        end
    end
    
    methods(Access=protected)        
        %------------------------------------------------------------------
        function onKeyPress(this, ~, evd)
            switch(evd.Key)
                case {'return', 'space', 'escape'}
                    close(this);
            end
        end

        function onWindowKeyPress(this,~,evd)
                switch(evd.Key)
                    case {'return', 'space', 'escape'}
                        close(this);
                end
        end
    end
end