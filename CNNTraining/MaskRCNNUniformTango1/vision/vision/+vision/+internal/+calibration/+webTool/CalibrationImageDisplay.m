classdef CalibrationImageDisplay < vision.internal.uitools.AbstractFigureDocument
    
    % CalibrationImageDisplay Base class for the main image display
    %   in the Camera Calibrator App
    %
    % This class takes care of figure creation and destruction, figure and
    % handle visibility, and zoom controls.
    
    % Copyright 2012-2023 The MathWorks, Inc.
    
    properties(Access=protected)
        ImageAxesTag = 'BoardImageAxes';
        LegendEntries = {};
    end
    
    methods
        function this = CalibrationImageDisplay(options)
            this = this@vision.internal.uitools.AbstractFigureDocument(options);
            setFigureColor(this);
            this.Figure.ThemeChangedFcn = @(src,evt)this.reactToThemeChange(src,evt);
            set(this.Figure, 'AutoResizeChildren', 'off');
            enableLegacyExplorationModes(this.Figure);
        end
        
        %------------------------------------------------------------------
        function drawImage(this, I)
            makeHandleVisible(this);
            
            hAxes = createImageAxes(this);
            
            currWarnState = warning;
            warning('off','MATLAB:callback:PropertyEventError');
            cl = onCleanup(@()warning(currWarnState)); 
            
            imshow(I,'InitialMagnification', 'fit', 'Parent', hAxes, ...
                'Border', 'tight');
            
            set(hAxes, 'NextPlot', 'add');
            set(hAxes.Toolbar,'Visible', 'off');
            
            % imshow resets the tag
            set(hAxes, 'Tag', this.ImageAxesTag);
            
            % this controls the font size of the legend
            set(hAxes, 'FontUnits', 'normalized');
            set(hAxes, 'FontSize', 0.03);
            
            disableDefaultInteractivity(hAxes);
        end
        
        %------------------------------------------------------------------
        function holdOff(this)
            hAxes = getImageAxes(this);
            
            % resets all axes properties to default values
            set(hAxes, 'NextPlot', 'replace');
        end
        
        %------------------------------------------------------------------
        function setTitle(this, titleString)
            hAxes = getImageAxes(this);
            title(hAxes, titleString, 'Interpreter', 'none');
        end
        
        %------------------------------------------------------------------
        function showLegend(this)
            hAxes = getImageAxes(this);
            % Turn off AutoUpdate to avoid unintended items from getting
            % added to the legend.
            legend(hAxes, this.LegendEntries, 'AutoUpdate', 'off');
        end
        
        %------------------------------------------------------------------
        function tf = isAxesValid(this)
            tf = ~isempty(getImageAxes(this));
        end
        
        %------------------------------------------------------------------
        function setZoomInState(this, shouldZoomIn)
            resetZoomState(this);
            hAxes = getImageAxes(this);
            if shouldZoomIn
                zoom(hAxes, 'on');
            else
                zoom(hAxes, 'off');
            end
        end
        
        %------------------------------------------------------------------
        function setZoomOutState(this, shouldZoomOut)
            resetZoomState(this);
            hAxes = getImageAxes(this);
            
            if shouldZoomOut
                h = zoom(hAxes);
                h.Direction = 'out';
                zoom(hAxes, 'on');
            else
                h = zoom(hAxes);
                h.Direction = 'in';
                zoom(hAxes, 'off');
            end
        end
        
        %------------------------------------------------------------------
        function setPanState(this, shouldPan)
            resetZoomState(this);
            hAxes = getImageAxes(this);
            if shouldPan
                pan(hAxes, 'on');
            else
                pan(hAxes, 'off');
            end
        end
        %------------------------------------------------------------------
        function setFigureColor(this)
            this.Figure.Color = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-primary');
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function hAxes = getImageAxes(this)
            hAxes = findobj(this.Figure, 'Type','axes','Tag', this.ImageAxesTag);
        end
        
        %------------------------------------------------------------------
        function hAxes = createImageAxes(this)
            hAxes = getImageAxes(this);
            if isempty(hAxes) % add an axes if needed
                hAxes = axes('Parent', this.Figure, 'Tag', this.ImageAxesTag);
            end
        end
        
        %------------------------------------------------------------------
        function resetZoomState(this)
            hAxes = getImageAxes(this);
            if isempty(hAxes)
                return;
            end
            
            h = zoom(hAxes);
            h.Direction = 'in';
            zoom(hAxes, 'off');
            pan(hAxes, 'off');
        end
    end

    methods
        %------------------------------------------------------------------
        function reactToThemeChange(this,~,~)
            setTheme(this);
            setFigureColor(this);
            setDisplayColors(this);
        end
    end
end
