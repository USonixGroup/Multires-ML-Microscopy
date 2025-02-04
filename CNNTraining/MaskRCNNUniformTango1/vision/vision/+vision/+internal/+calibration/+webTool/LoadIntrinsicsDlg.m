% LoadIntrinsicsDlg Dialog for loading fixed intrinsics.

% Copyright 2017-2021 The MathWorks, Inc.

classdef LoadIntrinsicsDlg < images.internal.app.utilities.OkCancelDialog
    properties
        VarName1
        VarName2
    end
    
    properties(Access=private)
        Prompt1Pos = [10, 98, 280, 20];
        
        VariableClass
        VarSelector1
        VarSelector2
    end
    
    methods
        %------------------------------------------------------------------
        function this = LoadIntrinsicsDlg(varClass, intrinsicsName1, intrinsicsName2, location)
            dlgTitle = vision.getMessage('vision:caltool:LoadStereoIntrinsicsTitle');
            
            this = this@images.internal.app.utilities.OkCancelDialog(location, dlgTitle);
            % TODO: should cameraParameters be allowed to be imported too?
            % This would mean conversion from cameraParameters to
            % cameraIntrinsics. Is this 1-1? This would help loading the
            % output of the single camera calibrator App into the stereo
            % App.
            this.VariableClass = varClass;
            this.VarName1 = intrinsicsName1;
            this.VarName2 = intrinsicsName2;
            
            this.Size = [300, 200];
            create(this);
            addVarSelectors(this);
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function addVarSelectors(this)
            import vision.internal.calibration.webTool.*;
            
            this.Prompt1Pos(2) = this.Size(2) - 30;
            this.VarSelector1 = VariableSelector(...
                vision.getMessage('vision:caltool:StereoIntrinsics1Prompt'), ...
                this.Prompt1Pos, this.FigureHandle, this.VariableClass);
            
            selectorPos2 = this.Prompt1Pos;
            selectorPos2(2) = selectorPos2(2) - 80;
            this.VarSelector2 = VariableSelector(...
                vision.getMessage('vision:caltool:StereoIntrinsics2Prompt'), ...
                selectorPos2, this.FigureHandle, this.VariableClass);
            
            % If previously loaded varname is valid, switch to it.
            this.VarSelector1.switchIfValid(this.VarName1);
            this.VarSelector2.switchIfValid(this.VarName2);
            
            if length(this.VarSelector1.validVarNames) == 1
                % No valid variables found in workspace
                this.VarSelector1.showEmptyState();
                this.VarSelector2.showEmptyState();
                this.Ok.Enable = 'off';
            end
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function okClicked(this, ~, ~)
            this.VarName1 = this.VarSelector1.SelectedVar;
            this.VarName2 = this.VarSelector2.SelectedVar;
            
            if areVarsBad(this)
                return;
            end
            
            this.Canceled = false;
            close(this);
        end
        
        %------------------------------------------------------------------
        function TF = areVarsBad(this)
            % Check if variable names are empty: take back to dialog if so.
            if isempty(this.VarName1) || isempty(this.VarName2)
                if strcmp(this.FigureHandle.Visible,'on') % TODO: Remove this
                    uialert(this.FigureHandle, getString(message('vision:caltool:invalidStereoIntrinsicsLoaded')),...
                        'Warning', 'Icon','warning');
                end
                TF = true;
            else
                TF = false;
            end
        end
    end
end

