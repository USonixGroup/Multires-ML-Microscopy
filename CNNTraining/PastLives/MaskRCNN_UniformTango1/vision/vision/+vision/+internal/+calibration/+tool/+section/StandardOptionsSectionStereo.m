% StandardOptionsSection Encapsulates standard camera model tool strip panel
%
% This class represents the calibration settings/options section used in
% Stereo Camera Calibrator App and used as Tear off pop up panel in
% Camera Calibrator App
%
% StandardOptionsSection properties:
%
%  RadialCoeffsButton1          - 2 coefficients radio button
%  RadialCoeffsButton2          - 3 coefficients radio button
%  CheckSkewButton              - estimate skew check box
%  CheckTangentialButton        - estimate tangential check box
%
% StandardOptionsSection methods:
%
%  setAllButtonsEnabled         - enable or disable the panel controls
%  enableNumRadialCoefficients  - enable radio buttons
%  disableNumRadialCoefficients - disable radio buttons

% Copyright 2019 The MathWorks, Inc.

classdef StandardOptionsSectionStereo < vision.internal.uitools.NewToolStripSection
    properties
        RadialCoeffsButton1
        RadialCoeffsButton2
        RadialDistortionLabel
        ComputeLabel
        CheckSkewButton
        CheckTangentialButton
        
        RadialCoefficientsButtonGroup
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
    
    methods
        function this = StandardOptionsSectionStereo()
            this.createSection();
            this.layoutSection();
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
                    this.RadialCoeffsButton1.Value= true;
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
            
        end
        
        %------------------------------------------------------------------
        function enableNumRadialCoefficients(this)
            this.RadialCoeffsButton1.Enabled = true;
            this.RadialCoeffsButton2.Enabled = true;
            this.setToolTipText(this.RadialCoeffsButton1,...
                'vision:caltool:TwoRadialCoeffsToolTip');
            this.setToolTipText(this.RadialCoeffsButton2,...
                'vision:caltool:ThreeRadialCoeffsToolTip');
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
            
            this.setToolTipText(this.RadialCoeffsButton1,...
                'vision:caltool:RadialCoeffsDisabledToolTip');
            this.setToolTipText(this.RadialCoeffsButton2,...
                'vision:caltool:RadialCoeffsDisabledToolTip');
        end
    end
    
    methods
        %------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:OptionsSection'));
            tag = 'secStandardOptions';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %------------------------------------------------------------------
        function layoutSection(this)
            this.addRadialDistortionButtons();
            this.addComputeButtons();
            
            colRadialDistortion = this.addColumn();
            colRadialDistortion.add(this.RadialDistortionLabel);
            colRadialDistortion.add(this.RadialCoeffsButton1);
            colRadialDistortion.add(this.RadialCoeffsButton2);
            
            colCompute = this.addColumn();
            colCompute.add(this.ComputeLabel);
            colCompute.add(this.CheckSkewButton);
            colCompute.add(this.CheckTangentialButton);
        end
        %------------------------------------------------------------------
        function addRadialDistortionButtons(this)
            this.RadialDistortionLabel = this.createLabel('vision:caltool:RadialButtonGroupHeading');
            this.createRadialCoeffsButtons();
            this.RadialCoefficientsButtonGroup = matlab.ui.internal.toolstrip.ButtonGroup;
        end
        
        %------------------------------------------------------------------
        function createRadialCoeffsButtons(this)
            this.RadialCoefficientsButtonGroup = matlab.ui.internal.toolstrip.ButtonGroup;
            titleId = 'vision:caltool:RadialCoeffsButton1';
            toolTipId = 'vision:caltool:TwoRadialCoeffsToolTip';
            tag = 'btnTwoCoeffs';
            group = this.RadialCoefficientsButtonGroup;
            this.RadialCoeffsButton1 = this.createRadioButton(titleId, ...
                tag, toolTipId, group);
            
            titleId = 'vision:caltool:RadialCoeffsButton2';
            toolTipId = 'vision:caltool:ThreeRadialCoeffsToolTip';
            tag = 'btnThreeCoeffs';
            group = this.RadialCoefficientsButtonGroup;
            this.RadialCoeffsButton2 = this.createRadioButton(titleId,...
                tag , toolTipId, group);
            
        end
        
        %------------------------------------------------------------------
        function addComputeButtons(this)
            this.ComputeLabel = this.createLabel('vision:caltool:ComputeButtonGroupHeading');
            this.createSkewButton();
            this.createTangentialButton();
            
        end
        %------------------------------------------------------------------
        function createSkewButton(this)
            titleId = 'vision:caltool:SkewButton';
            toolTipId = 'vision:caltool:CheckSkewToolTip';
            tag = 'btnCheckSkew';
            this.CheckSkewButton = this.createCheckBox(titleId, tag, toolTipId);
        end
        
        %------------------------------------------------------------------
        function createTangentialButton(this)
            titleId = 'vision:caltool:TangentialButton';
            toolTipId = 'vision:caltool:CheckTangentialToolTip';
            tag = 'btnCheckTangential';
            this.CheckTangentialButton = this.createCheckBox(titleId, tag, toolTipId);
        end
        
    end
end


