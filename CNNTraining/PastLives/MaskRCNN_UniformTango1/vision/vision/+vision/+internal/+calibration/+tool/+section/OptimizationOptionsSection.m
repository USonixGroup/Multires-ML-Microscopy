% OptimizationOptionsSection Encapsulates Optimization tool strip section
%
% This class represents the optimization options section used in Stereo camera
% calibrator App
%
% OptimizationOptionsSection properties:
% 
%  OptimizationOptionsButton  - Button to Specify the initial values for
%                               camera intrinsics and radial ditortion
%                               cofficients(can be 2 or 3 element vector).

% Copyright 2019-2023 The MathWorks, Inc.

classdef OptimizationOptionsSection < vision.internal.uitools.NewToolStripSection
    
    properties
        OptimizationOptionsButton
    end
    methods
        %------------------------------------------------------------------
        function this = OptimizationOptionsSection()
            this.createSection();
            this.layoutSection();
        end
        %------------------------------------------------------------------
        function enableButton(this)
            % enableButton Enable the button
            this.OptimizationOptionsButton.Enabled = true;
        end
        
        %------------------------------------------------------------------
        function disableButton(this)
            % disableButton Disable the button
            this.OptimizationOptionsButton.Enabled = false;
        end
        
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:OptimOptionsSection'));
            tag = 'secOptimOptions';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            this.addOptimizationoptionButton
            
            colDefaultLayout = this.addColumn();
            colDefaultLayout.add(this.OptimizationOptionsButton);
        end
        %-----------------------------------------------------------------------
        function addOptimizationoptionButton(this)
            import matlab.ui.internal.toolstrip.*;
            
            titleId = 'vision:caltool:OptimOptionsButton';
            tag = 'btnOptimOptions';
            toolTipId = 'vision:caltool:OptimOptionsToolTip';
            icon = Icon('cameraTools');
            import vision.internal.calibration.tool.*;
            this.OptimizationOptionsButton = this.createButton(icon, titleId, tag);
            this.setToolTipText(this.OptimizationOptionsButton,toolTipId);
        end
        
    end
end
