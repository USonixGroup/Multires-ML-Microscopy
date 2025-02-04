% ExportDlg Dialog for exporting the results of calibration

% Copyright 2014-2023 The MathWorks, Inc.

classdef ExportDlg < images.internal.app.utilities.OkCancelDialog
    properties
        ParamsVarName;
        ErrorsVarName;
        ShouldExportErrors;
        
        CheckBox;
    end
    
    properties(Access=private)
        ParamsPrompt;
        ParamEditBox;
        
        ErrorsPrompt;
        ErrorsEditBox;
        
        PromptX = 10;
        EditBoxX = 207;
    end
    
    methods
        %------------------------------------------------------------------
        function this = ExportDlg(paramsPrompt, ...
                paramsVarName, errorsVarName, shouldExportErrors, location)
            dlgTitle = vision.getMessage('vision:uitools:ExportTitle');
            
            this = this@images.internal.app.utilities.OkCancelDialog(...
                location, dlgTitle);
            
            this.ParamsVarName = paramsVarName;
            this.ParamsPrompt = paramsPrompt;
            
            this.ErrorsVarName = errorsVarName;
            this.ErrorsPrompt = ...
                vision.getMessage('vision:caltool:ErrorsExportPrompt');
            
            this.ShouldExportErrors = shouldExportErrors;
            
            this.Size = [400, 180];
            create(this);
            
            addParamsVarPrompt(this);
            addParamsVarEditBox(this);
            addErrorsVarPrompt(this);
            addErrorsVarEditBox(this);
            addErrorsCheckBox(this);
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function addParamsVarPrompt(this)
            % Prompt
            uilabel('Parent',this.FigureHandle,...
                'Position', [this.PromptX, 128, 200, 20],...
                'HorizontalAlignment', 'Left',...
                'Text', this.ParamsPrompt);
        end
        
        %------------------------------------------------------------------
        function addParamsVarEditBox(this)
            this.ParamEditBox = uieditfield(this.FigureHandle,'Editable','on', ...
                'HorizontalAlignment', 'left', ...
                'Position', [this.EditBoxX, 127, 180, 25],...
                'Value', this.ParamsVarName, ...
                'Tag', 'varEditBox',...
                'Tooltip', ...
                vision.getMessage('vision:caltool:ExportParametersNameToolTip'));
        end
        
        %------------------------------------------------------------------
        function addErrorsVarPrompt(this)
            uilabel('Parent',this.FigureHandle,...
                'Position', [this.PromptX, 48, 200, 20],...
                'HorizontalAlignment', 'Left',...
                'Text', this.ErrorsPrompt);
        end
        
        %------------------------------------------------------------------
        function addErrorsVarEditBox(this)
            this.ErrorsEditBox = uieditfield(this.FigureHandle,'Editable','on', ...
                'HorizontalAlignment', 'left', ...
                'Position', [this.EditBoxX, 47, 180, 25],...
                'Value', this.ErrorsVarName, ...
                'Tooltip', ...
                vision.getMessage('vision:caltool:ExportErrorsNameToolTip'));
        end
        
        %------------------------------------------------------------------
        function addErrorsCheckBox(this)
            % prompt
            uilabel('Parent',this.FigureHandle,...
                'Position', [this.PromptX + 20, 72, 200, 20],...
                'HorizontalAlignment', 'Left',...
                'Text', ...
                vision.getMessage('vision:caltool:ExportErrorsCheckboxLabel'),...
                'Tooltip',...
                vision.getMessage('vision:caltool:EnableExportErrorsToolTip'));
            
            
            this.CheckBox = uicheckbox(this.FigureHandle,...
                'Text','',...
                'Position', [this.PromptX, 74, 20, 20], ...
                'ValueChangedFcn', @checkBoxCallback,...
                'Value', this.ShouldExportErrors);
            
            if ~this.ShouldExportErrors
                disableErrors(this);
            end
            
            %--------------------------------------------------------------
            function checkBoxCallback(h, ~)
                if get(h, 'Value')
                    enableErrors(this);
                else
                    disableErrors(this);
                end
            end
        end
        
        %------------------------------------------------------------------
        function disableErrors(this)
            set(this.ErrorsEditBox, 'Enable', 'off');
        end
        
        %------------------------------------------------------------------
        function enableErrors(this)
            set(this.ErrorsEditBox, 'Enable', 'on');
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function okClicked(this, ~, ~)
            this.ShouldExportErrors = get(this.CheckBox, 'Value');
            this.ParamsVarName = get(this.ParamEditBox, 'Value');
            this.ErrorsVarName = get(this.ErrorsEditBox, 'Value');
            if ~isvarname(this.ParamsVarName)
                uialert(this.FigureHandle, getString(message('vision:uitools:invalidExportVariable')),...
                    getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle')));
            elseif ~isvarname(this.ErrorsVarName)
                uialert(this.FigureHandle, getString(message('vision:uitools:invalidExportVariable')),...
                    getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle')));
            elseif strcmp(this.ParamsVarName, this.ErrorsVarName) && this.CheckBox.Value
                uialert(this.FigureHandle, getString(message('vision:caltool:RepeatedExportVariableNames')),...
                    getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle')));
            else
                this.Canceled = false;
                close(this);
            end
        end
    end
end
