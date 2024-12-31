% StandardOptionsPanel Encapsulates standard camera model tool strip panel
%
%   This class represents the calibration settings/options panel used in
%   Camera Calibrator App and Stereo Camera Calibrator App.
%
%   panel = StandardOptionsPanel(showInitButton, fun, funInit) creates a
%   tool strip panel containing the standard camera model options controls.
%   showInitButton is a boolean to control button visibility, fun and funInit
%   are callback functions.
%
%   StandardOptionsPanel properties:
%
%       RadialCoeffsButton1          - 2 coefficients radio button
%       RadialCoeffsButton2          - 3 coefficients radio button
%       CheckSkewButton              - estimate skew check box
%       CheckTangentialButton        - estimate tangential check box
%       InitializeOptimizationButton - Optimization options button (non-stereo)
%
%   StandardOptionsPanel methods:
%
%       setAllButtonsEnabled         - enable or disable the panel controls
%       enableNumRadialCoefficients  - enable radio buttons
%       disableNumRadialCoefficients - disable radio buttons

% Copyright 2017-2020 The MathWorks, Inc.

classdef StandardOptionsPanel < handle
    
    properties (Access=private)
        RadialCoeffsButton1
        RadialCoeffsButton2
        CheckSkewButton
        CheckTangentialButton
        InitializeOptimizationButton
        ComputeLabel
        RadialCoefficientsButtonGroup
        RadialDistortionLabel
    end
    
    properties(Access=public)
        PopupList
        Panel
    end
    
    properties(Dependent)
        % CameraModel A struct containing the calibration options
        % corresponding to the current state of the panel. The struct
        % contains the following fields:
        %   NumDistortionCoefficients   - 2 or 3
        %   ComputeSkew                 - true or false
        %   ComputeTangentialDistortion - true or false
        CameraModel
    end

    properties (Access=private)
        RadialCoeffsButton1Listenerhandle
        RadialCoeffsButton2Listenerhandle
    end
    
    methods
        function this = StandardOptionsPanel(showInitButton, fun,funInit)
            this.createPopupList();
            this.createRadialCoeffsButtons();
            this.addComputeButtons();
            this.addCallback(fun);
            
            if showInitButton
                this.addInitializationButton();
                this.addInitCallback(funInit);
            end
        end
        
        %------------------------------------------------------------------
        % Gets the camera model options
        %------------------------------------------------------------------
        function cameraModel = get.CameraModel(this)
            if this.RadialCoeffsButton1.Value
                cameraModel.NumDistortionCoefficients = 2;
            else
                cameraModel.NumDistortionCoefficients = 3;
            end
            
            cameraModel.ComputeSkew = this.CheckSkewButton.Value;
            cameraModel.ComputeTangentialDistortion = ...
                this.CheckTangentialButton.Value;
        end
        
        %------------------------------------------------------------------
        function set.CameraModel(this, cameraModel)
            if isfield(cameraModel, 'NumDistortionCoefficients')
                if cameraModel.NumDistortionCoefficients == 2
                    this.RadialCoeffsButton1.Value = true;
                    this.RadialCoeffsButton2.Value = false;
                else
                    this.RadialCoeffsButton2.Value = true;
                    this.RadialCoeffsButton1.Value = false;
                end
            else
                this.RadialCoeffsButton1.Value = true;
                this.RadialCoeffsButton2.Value = false;
            end
            
            this.CheckSkewButton.Value = cameraModel.ComputeSkew;
            this.CheckTangentialButton.Value = cameraModel.ComputeTangentialDistortion;
        end
        
        %------------------------------------------------------------------
        function setAllButtonsEnabled(this, state)
            % setAllButtonsEnabled Enable or disable all panel controls
            %   setAllButtonsEnabled(panel, state) enables or disables all
            %   controls of the panel. panel is a ToolStripPanel object. state
            %   is a logical scalar.
            this.RadialCoeffsButton1.Enabled = state;
            this.RadialCoeffsButton2.Enabled = state;
            this.CheckSkewButton.Enabled = state;
            this.CheckTangentialButton.Enabled = state;
            this.InitializeOptimizationButton.Enabled = state;
        end
        
        %------------------------------------------------------------------
        function enableNumRadialCoefficients(this)
            this.RadialCoeffsButton1.Enabled = true;
            this.RadialCoeffsButton2.Enabled = true;
            this.RadialCoeffsButton1.Description = ...
                getString(message('vision:caltool:TwoRadialCoeffsToolTip'));
            this.RadialCoeffsButton2.Description = ...
                getString(message('vision:caltool:ThreeRadialCoeffsToolTip'));
        end
        
        %------------------------------------------------------------------
        function disableNumRadialCoefficients(this, numCoeffs)
            this.RadialCoeffsButton1.Enabled = true;
            this.RadialCoeffsButton2.Enabled = true;
            
            if numCoeffs == 2
                this.RadialCoeffsButton1.Value = true;
            else
                this.RadialCoeffsButton2.Value = true;
            end
            
            this.RadialCoeffsButton1.Enabled = false;
            this.RadialCoeffsButton2.Enabled = false;
            
            this.RadialCoeffsButton1.Description = ...
                getString(message('vision:caltool:RadialCoeffsDisabledToolTip'));
            this.RadialCoeffsButton2.Description = ...
                getString(message('vision:caltool:RadialCoeffsDisabledToolTip'));
        end

        function installRadialCoeffsButtonListener(this, fun)
            this.RadialCoeffsButton1Listenerhandle = addButtonCallback(this.RadialCoeffsButton1, fun);
            this.RadialCoeffsButton2Listenerhandle = addButtonCallback(this.RadialCoeffsButton2, fun);
        end

        function removeRadialCoeffsButtonListener(this)
            delete(this.RadialCoeffsButton1Listenerhandle);
            delete(this.RadialCoeffsButton2Listenerhandle);
        end
    end
    
    
    methods(Access = protected)
        
        %------------------------------------------------------------------
        function  createPopupList(this)
            this.PopupList = matlab.ui.internal.toolstrip.PopupList();
            this.Panel = matlab.ui.internal.toolstrip.PopupListPanel();
            tag = 'panelStandardOptions';
            this.PopupList.Tag = tag;
        end
        
        %------------------------------------------------------------------
        function createRadialCoeffsButtons(this)
            import matlab.ui.internal.toolstrip.*
            this.RadialDistortionLabel = ...
                getString(message('vision:caltool:RadialButtonGroupHeading'));
            radiobuttonHeader = matlab.ui.internal.toolstrip.PopupListHeader(this.RadialDistortionLabel);
            add(this.PopupList,radiobuttonHeader,1);
            this.RadialCoefficientsButtonGroup = ButtonGroup();
            group = this.RadialCoefficientsButtonGroup;
            
            titleId = 'vision:caltool:RadialCoeffsButton1';
            this.RadialCoeffsButton1 = ListItemWithRadioButton(group);
            this.RadialCoeffsButton1.Tag = 'btnTwoCoeffs';
            this.RadialCoeffsButton1.Text = getString(message(titleId));
            this.RadialCoeffsButton1.Description = getString(message(...
                'vision:caltool:TwoRadialCoeffsToolTip'));
            add(this.PopupList,this.RadialCoeffsButton1,2);
            
            titleId = 'vision:caltool:RadialCoeffsButton2';
            this.RadialCoeffsButton2 = ListItemWithRadioButton(group);
            this.RadialCoeffsButton2.Tag = 'btnThreeCoeffs';
            this.RadialCoeffsButton2.Text = getString(message(titleId));
            this.RadialCoeffsButton2.Description =  getString(message(...
                'vision:caltool:ThreeRadialCoeffsToolTip'));
            add(this.PopupList,this.RadialCoeffsButton2,3);
            this.PopupList.addSeparator;
        end
        
        %------------------------------------------------------------------
        function addComputeButtons(this)
            this.ComputeLabel =  getString(message(...
                'vision:caltool:ComputeButtonGroupHeading'));
            computeCheckboxHeader = matlab.ui.internal.toolstrip.PopupListHeader(this.ComputeLabel);
            add(this.PopupList,computeCheckboxHeader,5);
            this.createSkewButton();
            this.createTangentialButton();
            add(this.PopupList,this.CheckSkewButton,6);
            add(this.PopupList,this.CheckTangentialButton,7);
        end
        
        %------------------------------------------------------------------
        function createInitializationButton(this)
            titleId = 'vision:caltool:OptimOptionsButton';
            optimizationBtnIcon = matlab.ui.internal.toolstrip.Icon('cameraTools');
            this.InitializeOptimizationButton =  matlab.ui.internal.toolstrip.ListItem(getString(message(titleId)),optimizationBtnIcon);
            this.InitializeOptimizationButton.Description =getString(message( 'vision:caltool:OptimizationOptionsToolTip'));
            this.InitializeOptimizationButton.Tag = 'optimizationOptionsButton';
            add(this.PopupList,this.InitializeOptimizationButton,8);
        end
        
        %------------------------------------------------------------------
        function addInitializationButton(this)
            this.createInitializationButton();
        end
        
        %------------------------------------------------------------------
        function createSkewButton(this)
            titleId = 'vision:caltool:SkewButton';
            toolTipId = 'vision:caltool:CheckSkewToolTip';
            this.CheckSkewButton = matlab.ui.internal.toolstrip.ListItemWithCheckBox(...
                getString(message(titleId)));
            this.CheckSkewButton.Tag = 'btnCheckSkew';
            this.CheckSkewButton.Description = getString(message(toolTipId));
        end
        
        %------------------------------------------------------------------
        function createTangentialButton(this)
            titleId = 'vision:caltool:TangentialButton';
            toolTipId = 'vision:caltool:CheckTangentialToolTip';
            this.CheckTangentialButton = matlab.ui.internal.toolstrip.ListItemWithCheckBox(...
                getString(message(titleId)));
            this.CheckTangentialButton.Tag = 'btnCheckTangential';
            this.CheckTangentialButton.Description = getString(message(toolTipId));
        end
        
        %------------------------------------------------------------------
        function addCallback(this, fun)
            this.RadialCoeffsButton1Listenerhandle = addButtonCallback(this.RadialCoeffsButton1, fun);
            this.RadialCoeffsButton2Listenerhandle = addButtonCallback(this.RadialCoeffsButton2, fun);
            addButtonCallback(this.CheckSkewButton, fun);
            addButtonCallback(this.CheckTangentialButton, fun);
        end
        
        %------------------------------------------------------------------
        function addInitCallback(this, fun)
            addlistener(this.InitializeOptimizationButton, 'ItemPushed', fun);
        end
    end
end

% %--------------------------------------------------------------------------
function listenerHandle = addButtonCallback(button, fun)
listenerHandle = addlistener(button, 'ValueChanged', fun);
end
