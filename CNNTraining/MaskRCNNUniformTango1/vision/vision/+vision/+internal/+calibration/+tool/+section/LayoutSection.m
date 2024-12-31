% LayoutSection Encapsulates the Layout tool strip section
%
% This class represents the Layout section used in both Camera Calibrator
% and Stereo Camera Calibrator App
%
% LayoutSection properties:
% 
%  DefaultLayoutButton - Button to restore default layout of app's
%                        figures and image browser

% Copyright 2019-2023 The MathWorks, Inc.

classdef LayoutSection< vision.internal.uitools.NewToolStripSection
    properties
        DefaultLayoutButton
    end
    properties(Dependent)
        IsButtonEnabled
    end
    
    methods
        function set.IsButtonEnabled(this, IsEnabled)
            this.DefaultLayoutButton.Enabled = IsEnabled;
        end
        
        function IsEnabled = get.IsButtonEnabled(this)
            IsEnabled = this.DefaultLayoutButton.Enabled;
        end
    end
    methods
        %------------------------------------------------------------------
        function this = LayoutSection()
            this.createSection();
            this.layoutSection();
        end
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:uitools:LayoutSection'));
            tag = 'calibratorLayoutSection';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            
            this.addDefaultLayoutButton();
            
            colDefaultLayout = this.addColumn();
            colDefaultLayout.add(this.DefaultLayoutButton);
            
        end
        %---------------------------------------------------------------------
        function addDefaultLayoutButton(this)
            import matlab.ui.internal.toolstrip.*;
            
            % New Session Button
            DefaultLayoutTitleId = 'vision:uitools:LayoutButton';
            DefaultLayoutIcon = Icon('layout');
            DefaultLayoutTag = 'btnLayout';
            this.DefaultLayoutButton = this.createButton(DefaultLayoutIcon, ...
                DefaultLayoutTitleId, DefaultLayoutTag);
            toolTipID = 'vision:caltool:LayoutToolTip';
            this.setToolTipText(this.DefaultLayoutButton, toolTipID);
            
        end
    end
    % Update Button States
    methods
        %------------------------------------------------------------------
        function disableDefaultLayoutButton(this)
            this.DefaultLayoutButton.Enabled = false;
        end
        
        %------------------------------------------------------------------
        function enableDefaultLayoutButton(this)
            this.DefaultLayoutButton.Enabled = true;
        end
        %---------------------------------------------------------------------
        function addListeners(this, callbackFun)
            this.DefaultLayoutListener = ...
                addlistener(this.DefaultLayoutButton, 'ButtonPushed', callbackFun);
        end
    end
end
