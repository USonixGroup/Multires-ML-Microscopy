% CloseDlg Dialog with Close buttons

% Copyright 2019-2024 The MathWorks, Inc.

classdef CloseDlg < vision.internal.uitools.AbstractDlg
    properties
        CloseButton;
    end
    
    properties (Access = protected)
        ButtonSize = [60, 20];
        ButtonHalfSpace = 15;
    end
    
    methods
        function this = CloseDlg(groupName, dlgTitle)
            this = this@vision.internal.uitools.AbstractDlg(...
                groupName, dlgTitle);
        end
            
        %------------------------------------------------------------------
        function createDialog(this)
            % createDialog Create and shows the dialog window
            %   createDialog(dlg) creates and shows the dialog window. dlg is
            %   an AbstractDlg object.
          
            dlgPosition = getInitialDialogPosition(this);
            this.Dlg = uifigure('Name', this.DlgTitle,...
                'WindowStyle','normal',...
                'Position', dlgPosition,...
                'WindowKeyPress', @this.onKeyPress,...
                'Visible','off');   
            matlab.graphics.internal.themes.figureUseDesktopTheme(this.Dlg);
            addClose(this);
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function pos = getInitialDialogPosition(this)
            % Get the dialog to the app center
            pos = imageslib.internal.app.utilities.ScreenUtilities.getModalDialogPos( ...
                this.App, this.DlgSize);
        end
    end

    methods(Access=protected)
        %------------------------------------------------------------------
        function addClose(this)
            x = this.DlgSize(1) - this.ButtonSize(1) - this.ButtonHalfSpace;
            this.CloseButton = uibutton('Parent', this.Dlg, ...
                'ButtonPushedFcn', @this.onClose,...
                'Position',[x, 10, this.ButtonSize], ...
                'FontSize', 11, 'Text',...
                getString(message('vision:labeler:Close')),...
                'Tag','AbstractCloseButton');
        end
        
        %------------------------------------------------------------------
        function onClose(this, ~, ~)
            close(this);
            delete(this);
        end
        
        %------------------------------------------------------------------
        function onKeyPress(this, ~, evd)
                % Validate if KeyPress needs to be triggered.
            if ~validateKeyPressSupport(this,evd)
                return;
            end
            
            switch(evd.Key)
                case {'return','space','escape'}
                    onClose(this);
            end
        end
        
    end
end
