% ZoomSection Encapsulates the zoom controls for the new tool strip.
%
%  This class encapsulates a tool strip section containing the zoom controls.
%  You can add this section to a tab of the toolstrip. You can also extend
%  this class to add more buttons to the section.
%
%  zoomSection = ZoomSection() creates a tool strip panel containing the zoom
%  controls.
%
%  ZoomSection properties:
%
%    ZoomInButtonState  - State of the zoom-in button
%    ZoomOutButtonState - State of the zoom-out button
%    PanButtonState     - State of the pan button
%
%  ZoomSection methods:
%
%    addListeners       - Add listeners to the zoom control buttons
%    removeListeners    - Delete the button listeners
%    enableButtons      - Enable all zoom control buttons
%    disableButtons     - Disable (gray out) all zoom control buttons
%    resetButtons       - Un-click all zoom control buttons

% Copyright 2014-2021 The MathWorks, Inc.

classdef ZoomSection < vision.internal.uitools.NewToolStripSection
    
    properties(Access=protected)
        ZoomInButton
        ZoomOutButton
        PanButton
        
        ZoomInListener;
        ZoomOutListener;
        PanListener;
    end
    
    properties(Dependent)
        % ZoomInButtonState A logical scalar representing the state of the
        %   zoom-in button.
        ZoomInButtonState;
        
        % ZoomOutButtonState A logical scalar representing the state of the
        %   zoom-out button.
        ZoomOutButtonState;
        
        % PanButtonState A logical scalar representing the state of the
        %   pan button.
        PanButtonState;
        
        % IsAnabled A logical scalar indicating whether all the buttons are
        %   enabled.
        IsEnabled;
    end
    
    methods
        %------------------------------------------------------------------
        function this = ZoomSection()
            this.createSection();
            this.layoutSection();
        end
        %------------------------------------------------------------------
        function tf = get.IsEnabled(this)
            tf = this.ZoomInButton.Enabled && this.ZoomOutButton.Enabled && ...
                this.PanButton.Enabled;
        end
        
        %------------------------------------------------------------------
        function tf = get.ZoomInButtonState(this)
            tf = this.ZoomInButton.Value;
        end
        
        %------------------------------------------------------------------
        function tf = get.ZoomOutButtonState(this)
            tf = this.ZoomOutButton.Value;
        end
        
        %------------------------------------------------------------------
        function tf = get.PanButtonState(this)
            tf = this.PanButton.Value;
        end
        
        %------------------------------------------------------------------
        function set.ZoomInButtonState(this, tf)
            this.ZoomInButton.Value = tf;
        end
        
        %------------------------------------------------------------------
        function set.ZoomOutButtonState(this, tf)
            this.ZoomOutButton.Value = tf;
        end
        
        %------------------------------------------------------------------
        function set.PanButtonState(this, tf)
            this.PanButton.Value = tf;
        end
        
        function addListeners(this, callbackFun)
            % addListeners Add listeners to the zoom control buttons
            %   addListeners(zoomSection, callbackFun) adds listeners to the zoom
            %   control buttons contained in ZoomSection. zoomSection is a
            %   ZoomSection object.  callbackFun is a handle of a callback function
            %   to be called when one of the buttons is pressed.
            this.ZoomInListener = ...
                addlistener(this.ZoomInButton, 'ValueChanged', callbackFun);
            this.ZoomOutListener = ...
                addlistener(this.ZoomOutButton, 'ValueChanged', callbackFun);
            this.PanListener = ...
                addlistener(this.PanButton, 'ValueChanged', callbackFun);
        end
        
        function removeListeners(this)
            % removeListeners Delete the zoom control button listeners.
            % removeListeners(zoomSection) deletes listeners from the zoom
            % control buttons contained in zoomSection. zoomSection is a
            % ZoomSection object.
            delete(this.ZoomInListener);
            delete(this.ZoomOutListener);
            delete(this.PanListener);
        end
        
        %------------------------------------------------------------------
        function enableButtons(this)
            % enableButtons Enable all zoom control buttons
            % enableButtons(zoomSection) enables all zoom control buttons
            % contained in zoomSection.
            this.ZoomInButton.Enabled  = true;
            this.ZoomOutButton.Enabled = true;
            this.PanButton.Enabled     = true;
        end
        
        %------------------------------------------------------------------
        function disableButtons(this)
            % disableButtons Disable all zoom control buttons
            % disableButtons(zoomSection) disables all zoom control buttons
            % contained in zoomSection. The buttons will appear grayed-out.
            this.ZoomInButton.Enabled  = false;
            this.ZoomOutButton.Enabled = false;
            this.PanButton.Enabled     = false;
        end
        
        %------------------------------------------------------------------
        function resetButtons(this)
            % resetButtons Un-click all zoom control buttons
            % resetButtons(zoomSection) sets the state of all zoom control
            % buttons to false.
            this.ZoomInButton.Value  = false;
            this.ZoomOutButton.Value = false;
            this.PanButton.Value     = false;
            drawnow();
        end
    end
    
    methods (Access = private)
        
        %------------------------------------------------------------------
        function createSection(this)
            title = vision.getMessage('vision:uitools:ZoomSection');
            tag = 'sectionZoom';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        
        %------------------------------------------------------------------
        function layoutSection(this)
            this.addZoomInButton();
            this.addZoomOutButton();
            this.addPanButton();
            
            zoomCol = this.addColumn();
            zoomCol.add(this.ZoomInButton);
            zoomCol.add(this.ZoomOutButton);
            zoomCol.add(this.PanButton);
        end
        
        %------------------------------------------------------------------
        function addZoomInButton(this)
            zoomInIcon = matlab.ui.internal.toolstrip.Icon.ZOOM_IN_16;
            zoomInID   = 'vision:uitools:ZoomInButton';
            zoomInTag  = 'btnZoomIn';
            
            this.ZoomInButton = this.createToggleButton(zoomInIcon, zoomInID, zoomInTag);
            this.setToolTipText(this.ZoomInButton, 'vision:uitools:ZoomInToolTip');
        end
        
        %------------------------------------------------------------------
        function addZoomOutButton(this)
            zoomOutIcon = matlab.ui.internal.toolstrip.Icon.ZOOM_OUT_16;
            zoomOutID   = 'vision:uitools:ZoomOutButton';
            zoomOutTag  = 'btnZoomOut';
            
            this.ZoomOutButton = this.createToggleButton(zoomOutIcon, zoomOutID, zoomOutTag);
            this.setToolTipText(this.ZoomOutButton, 'vision:uitools:ZoomOutToolTip');
        end
        
        %------------------------------------------------------------------
        function addPanButton(this)
            panIcon = matlab.ui.internal.toolstrip.Icon.PAN_16;
            panID   = 'vision:uitools:PanButton';
            panTag  = 'btnPan';
            
            this.PanButton = this.createToggleButton(panIcon, panID, panTag);
            this.setToolTipText(this.PanButton,'vision:uitools:PanToolTip');
        end
    end
    
end