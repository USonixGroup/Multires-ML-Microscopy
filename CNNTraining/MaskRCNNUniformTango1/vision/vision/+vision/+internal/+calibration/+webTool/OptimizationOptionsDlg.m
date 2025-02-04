%

% Copyright 2014-2021 The MathWorks, Inc.

classdef OptimizationOptionsDlg < images.internal.app.utilities.OkCancelDialog
    
    properties
        OptimizationOptions;
    end
    
    properties(Access=private)
        IntrinsicsCheckBox;
        IntrinsicsTextField;
        DistortionCheckBox;
        DistortionTextField;
    end
    
    
    methods
        %------------------------------------------------------------------
        function this = OptimizationOptionsDlg(optimizationOptions, location)
            dlgTitle = getString(message('vision:caltool:OptimOptionsDlgTitle'));
            
            this = this@images.internal.app.utilities.OkCancelDialog(location, dlgTitle);
            
            if nargin < 1
                optimizationOptions = [];
            end
            
            this.OptimizationOptions = optimizationOptions;
            
            if isempty(this.OptimizationOptions)
                this.OptimizationOptions.InitialIntrinsics = [];
                this.OptimizationOptions.InitialDistortion = [];
            end
            
            this.Size = [400, 240];
            create(this);
            addIntrinsicsWidgets(this);
            addDistortionWidgets(this);
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function addIntrinsicsWidgets(this)
            usingInitialIntrinsics = ...
                ~isempty(this.OptimizationOptions) && ...
                ~isempty(this.OptimizationOptions.InitialIntrinsics);
            
            % Add the checkbox
            xLoc = 10;
            yLoc = 205;
            width = 375;
            location = [xLoc, yLoc, 20, 20];
            this.IntrinsicsCheckBox = uicheckbox( this.FigureHandle, ...
                'Position', location, 'Value', usingInitialIntrinsics, 'Text','');
            this.IntrinsicsCheckBox.ValueChangedFcn = @intrinsicsCheckboxCallback;
            this.IntrinsicsCheckBox.Tooltip = getString(message(...
                'vision:caltool:InitialIntrinsicsToolTip'));
            
            % Add the checkbox label
            location = [xLoc + 20, yLoc - 12, width, 32];
            uilabel(this.FigureHandle,...
                'Position', location, 'Text', ...
                getString(message('vision:caltool:InitialIntrinsicsPrompt')));
            
            % Add the text field
            height = 60;
            location = [xLoc, yLoc - 22 - height, width,  height];
            this.IntrinsicsTextField.Min = 1;
            this.IntrinsicsTextField.Max = 3;
            this.IntrinsicsTextField = uieditfield(this.FigureHandle, ...
                'Position', location, ...
                'HorizontalAlignment', 'left');
            
            if usingInitialIntrinsics
                this.IntrinsicsTextField.Enable = 'on';
                this.IntrinsicsTextField.Value = ...
                    mat2str(this.OptimizationOptions.InitialIntrinsics);
            else
                this.IntrinsicsTextField.Enable = 'off';
                this.IntrinsicsTextField.Value = '';
            end
            
            %--------------------------------------------------------------
            function intrinsicsCheckboxCallback(h, ~)
                if get(h, 'Value')
                    this.IntrinsicsTextField.Enable = 'on';
                else
                    this.IntrinsicsTextField.Enable = 'off';
                end
            end
        end
        
        %------------------------------------------------------------------
        function addDistortionWidgets(this)
            usingInitialDistortion = ...
                ~isempty(this.OptimizationOptions) && ...
                ~isempty(this.OptimizationOptions.InitialDistortion);
            
            xLoc = 10;
            yLoc = 94;
            width = 375;
            location = [xLoc, yLoc, 20, 20];
            this.DistortionCheckBox = uicheckbox(this.FigureHandle, ...
                'Position', location, 'Value', ...
                usingInitialDistortion,'ValueChangedFcn', @distortionCheckboxCallback,'Tooltip', ...
                getString(message('vision:caltool:InitialDistortionToolTip')), 'Text', '');
            
            % Add the checkbox label
            location = [xLoc + 20, yLoc - 12, width, 32];
            uilabel(this.FigureHandle,...
                'Position', location, 'Text', ...
                getString(message('vision:caltool:InitialDistortionPrompt')));
            
            % Add the text field
            height = 20;
            location = [xLoc, yLoc - 20 - height, width, height];
            
            this.DistortionTextField =  uieditfield( this.FigureHandle, ...
                'Position', location, ...
                'HorizontalAlignment', 'left');
            
            if usingInitialDistortion
                this.DistortionTextField.Enable = 'on';
                this.DistortionTextField.Value = ...
                    mat2str(this.OptimizationOptions.InitialDistortion);
            else
                this.DistortionTextField.Enable = 'off';
                this.DistortionTextField.Value = '';
            end
            
            %---------------------------------------------------------------
            function distortionCheckboxCallback(h, ~)
                if get(h, 'Value')
                    this.DistortionTextField.Enable = 'on';
                else
                    this.DistortionTextField.Enable = 'off';
                end
            end
        end
        
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function okClicked(this)
            K = str2num(this.IntrinsicsTextField.Value); %#ok<ST2NM>
            
            if ~isempty(checkBadIntrinsics(K))
                isBadIntrinsics = this.IntrinsicsCheckBox.Value && checkBadIntrinsics(K);
            else
                isBadIntrinsics =  false;
            end
            
            distCoeffs = str2num(this.DistortionTextField.Value);  %#ok<ST2NM>
            numCoeffs = numel(distCoeffs);
            
            isBadDistCoeffs = this.DistortionCheckBox.Value && ...
                ~(isvector(distCoeffs) && isnumeric(distCoeffs) && ...
                (numCoeffs == 2 || numCoeffs == 3));
            
            if isBadIntrinsics
                dlgTitle = vision.getMessage('vision:caltool:GenericErrorTitle');
                dlgContent = vision.getMessage('vision:caltool:InvalidInitialIntrinsics');
                matlab.graphics.internal.themes.figureUseDesktopTheme(this.FigureHandle);
                uialert(this.FigureHandle, dlgContent, dlgTitle);
                return;
                
            elseif isBadDistCoeffs
                dlgTitle = vision.getMessage('vision:caltool:GenericErrorTitle');
                dlgContent = vision.getMessage('vision:caltool:InvalidInitialDistortion');
                matlab.graphics.internal.themes.figureUseDesktopTheme(this.FigureHandle);
                uialert(this.FigureHandle, dlgContent, dlgTitle);
                return;
                
            else
                this.Canceled = false;
                if this.IntrinsicsCheckBox.Value
                    this.OptimizationOptions.InitialIntrinsics = K;
                else
                    this.OptimizationOptions.InitialIntrinsics = [];
                end
                
                if this.DistortionCheckBox.Value
                    this.OptimizationOptions.InitialDistortion = distCoeffs;
                else
                    this.OptimizationOptions.InitialDistortion = [];
                end
                
                close(this);
            end
        end
        
        %--Key Press-------------------------------------------------------
        % Overwriting the key press function to avoid triggering of
        % okClicked callback on pressing on space bar.
        function keyPress(self, evt)
            if strcmp(evt.Key, 'escape')
                cancelClicked(self);
            end
        end
    end
end

function TF = checkBadIntrinsics(intrinsicsMat)
% Check that the intrinsics matrix user provides is good. If bad,
% return true. The matrix needs to conform to a [fx, 0, 0; s, fx, 0; cx, cy 0]
% format. Specific elements must be 0, as requested in the dialog.
TF = false;
if ~(ismatrix(intrinsicsMat) && isequal(size(intrinsicsMat), [3 3]))
    TF = true;
end
if ~TF
    % It is a 3-by-3 matrix. Check values that need to be 0.
    % 1,2 is 0
    if intrinsicsMat(1,2) ~= 0
        TF = true;
    end
    % 1,3 is 0
    if intrinsicsMat(1,3) ~= 0
        TF = true;
    end
    % 2,3 is 0
    if intrinsicsMat(2,3) ~= 0
        TF = true;
    end
end
end
