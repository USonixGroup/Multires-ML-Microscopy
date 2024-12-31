% OkCancelDlg Dialog with an OK and Cancel buttons

% Copyright 2014-2023 The MathWorks, Inc.

classdef OkCancelDlg < vision.internal.uitools.AbstractDlg
    properties
       OkButton;
       CancelButton;
       IsCanceled = true;
    end
    
    properties(Access=protected)
        ButtonSize = [60, 20];
        ButtonHalfSpace = 10;
    end
    
    methods
        function this = OkCancelDlg(tool, dlgTitle)
            this = this@vision.internal.uitools.AbstractDlg(...
                tool, dlgTitle);
        end
       
        %------------------------------------------------------------------
        function createDialog(this)
            createDialog@vision.internal.uitools.AbstractDlg(this);
            addOK(this);
            addCancel(this);
        end
    end
    
    methods(Abstract, Access=protected)
        onOK(this, ~, ~);
    end
    
    methods(Access=protected)
       %------------------------------------------------------------------
        function addOK(this)
            x = this.DlgSize(1) / 2 - this.ButtonSize(1) - this.ButtonHalfSpace;
            this.OkButton = uibutton('Parent', this.Dlg, ...
                'ButtonPushedFcn', @this.onOK, ...
                'FontSize', 12,...
                'Position', [x, 10, this.ButtonSize],...
                'Text', getString(message('MATLAB:uistring:popupdialogs:OK')),...
                'Tag','AbstractOkButton');
        end
        
        %------------------------------------------------------------------
        function addCancel(this)
            x = this.DlgSize(1) / 2 + this.ButtonHalfSpace;
            this.CancelButton = uibutton('Parent', this.Dlg,...
                'ButtonPushedFcn', @this.onCancel,....
                'Position', [x, 10, this.ButtonSize], 'FontSize', 12,...
                'Text', getString(message('MATLAB:uistring:popupdialogs:Cancel')),...
                'Tag','AbstractCancelButton');
        end                
        
        %------------------------------------------------------------------
        function onCancel(this, ~, ~)            
            close(this);
        end
                
        %------------------------------------------------------------------
        function onKeyPress(this, ~, evd)
            
            % Validate if KeyPress needs to be triggered.
            if ~validateKeyPressSupport(this,evd)
                return;
            end
            switch(evd.Key)
                case {'return','space'}
                    onOK(this);
                case {'escape'}
                    onCancel(this);
            end
        end

    end
end