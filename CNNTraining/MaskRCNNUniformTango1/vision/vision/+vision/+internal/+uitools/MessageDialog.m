% This class is for internal use only and may change in the future.

% MessageDialog creates a message dialog with an OK button.

classdef MessageDialog < vision.internal.uitools.OkDlg

%   Copyright 2018-2021 The MathWorks, Inc.
   
    methods
        function this = MessageDialog(tool, title, msg)
            this  = this@vision.internal.uitools.OkDlg(tool, title);
            
            this.DlgSize = [400 150];            
            
            createDialog(this);           
            
            addMessage(this,msg)   
        end
    end
    
    %----------------------------------------------------------------------
    methods(Access = private)
        function addMessage(this, msg)  
            
            w = this.DlgSize(1);
            h = this.DlgSize(2);
            
            position =  [5 h/4 w-10 70];
            [~] = uicontrol('Parent',this.Dlg,'Style','text',...
                'Position', position,...
                'FontUnits', 'pixels', 'FontSize', 10, ...
                'HorizontalAlignment', 'Left',...
                'String', msg);
            
        end
    end
end
