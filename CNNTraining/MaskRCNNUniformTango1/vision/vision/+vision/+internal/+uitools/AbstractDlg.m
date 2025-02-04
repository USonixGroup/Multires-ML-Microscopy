% AbstractDlg Base class for dialogs used with a toolgroup
%
% AbstractDlg class handles dialog creation, positioning, and closing.
%
%   dlg = AbstractDlg(groupName, dlgTitle) creates the AbstractDlg object. 
%   groupName is the name of the toolgroup (toolstrip app) using the
%   dialog. The dialog will be positioned in the center of the tool group
%   window.
%
% AbstractDlg properties:
%
%   Dlg       - Handle to the dialog object
%   DlgSize   - Size of the dialog [width, height] 
%   DlgTitle  - Title of the dialog
%   GroupName - Name of the toolgroup
%
% AbstractDlg methods:
%   
%   createDialog - Create and show the dialog.
%   wait         - Wait for the user to close the dialog.
%   close        - Close the dialog
%   delete       - Close the dialog
%
%   Abstract methods:
%   onKeyPress   - Handle the behaviour of enter, space, and escape keys.
%

% Copyright 2014-2024 The MathWorks, Inc.

classdef AbstractDlg < handle
    properties 
        % Dlg Handle to the dialog object
        Dlg;
        
        % DlgSize Size of the dialog [width, height] 
        DlgSize = [400, 200];
        
        % DlgTitle Title of the dialog as a string
        DlgTitle;
        
        % GroupName Name of the appcontainer. Used for positioning.
        GroupName;
        
        % AppContainerHandle
        App;
        
    end
    
    methods
        %------------------------------------------------------------------
        function this = AbstractDlg(tool, dlgTitle)
            this.DlgTitle = dlgTitle;
            this.App = tool;
            this.GroupName = this.App.Tag;
        end
        
        %------------------------------------------------------------------
        function createDialog(this)
            % createDialog Create and shows the dialog window
            % createDialog(dlg) creates and shows the dialog window. dlg is
            % an AbstractDlg object.
             dlgPosition = getInitialDialogPosition(this);
             this.Dlg = uifigure('WindowStyle', 'modal', 'Name', this.DlgTitle,...
                'Position', dlgPosition, ...
                'Resize','off',...
                'HandleVisibility', 'callback',...
                'WindowKeyPressFcn', @this.onKeyPress);
             matlab.graphics.internal.themes.figureUseDesktopTheme(this.Dlg);
             this.Dlg.ThemeChangedFcn = @(src,evt)this.updateTheme();
        end
        
        %------------------------------------------------------------------
        function updateDialogSize(this, dlgSize)
            this.DlgSize = dlgSize;
            this.Dlg.Position(3) = dlgSize(1);
            this.Dlg.Position(4) = dlgSize(2);
        end

        %------------------------------------------------------------------
        function updateTheme(this)
           matlab.graphics.internal.themes.figureUseDesktopTheme(this.Dlg);
        end
        %------------------------------------------------------------------
        function close(this, ~, ~)
        % CLOSE Close the dialog
        % CLOSE(dlg)
            if ishandle(this.Dlg)
                close(this.Dlg);
            end
        end
        
        %------------------------------------------------------------------
        function wait(this)
        % WAIT Wait for the user to close the dialog.
        %   WAIT(dlg) wait untilt he user closes the dialog. dlg is an
        %   AbstractDlg object.
            uiwait(this.Dlg);
        end
        
        %------------------------------------------------------------------
        function delete(this)
        % DELETE Close the dialog
        %   DELETE(dlg)
            close(this);
        end
    end
    
    %----------------------------------------------------------------------
    methods(Abstract, Access=protected)
        % onKeyPress handle the behavior of enter, space, and escape keys.
        %   onKeyPress(dlg)
        onKeyPress(this)
    end
    %----------------------------------------------------------------------
   
    methods(Access=private)
        %------------------------------------------------------------------
        function pos = getInitialDialogPosition(this)
            % Get the dialog to the screen centre
            pos = imageslib.internal.app.utilities.ScreenUtilities.getModalDialogPos(...
              this.App,this.DlgSize);
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function ctrl = addTextLabel(this, position, string)
            ctrl = uicontrol('Parent', this.Dlg, ...
                'Style', 'text', ...
                'Position', position, ...
                'HorizontalAlignment', 'left', ...
                'String', string);
        end
        
        %------------------------------------------------------------------
        function ctrl = addCheckBox(this, position, state)
            ctrl = uicontrol('Parent', this.Dlg, ...
                'Style', 'checkbox', ...
                'Position', position, ...
                'HorizontalAlignment', 'left', 'Value', state);
        end
        
        %------------------------------------------------------------------
        function ctrl = addTextField(this, position)
            ctrl = uicontrol('Parent', this.Dlg, ...
                'Style', 'edit', 'Position', position, ...
                'HorizontalAlignment', 'left');
        end
                
    end

    methods(Access = protected)
         %--Validate Key Press Support--------------------------------------
        function TF = validateKeyPressSupport(~,evt)
            % Return true if the keyboard press should be honored. This is
            % used to block keyboard shortcuts from happening when users
            % are interacting with objects that receive keyboard input. For
            % example, the enter key press should not close the dialog when
            % the user is currently typing in an editfield.
            TF = ~any(strcmp(class(evt.Source.CurrentObject),{...
                'matlab.ui.control.EditField',...
                'matlab.ui.control.NumericEditField',...
                'matlab.ui.control.DropDown',...
                'matlab.ui.control.DatePicker',...
                'matlab.ui.control.Spinner',...
                'matlab.ui.control.TextArea'}));
        end
    end
            
end
