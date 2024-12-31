% ThumbnailDisplay class for the board thumbnails of Camera Calibrator and
% Stereo Camera Calibrator app

% Copyright 2020-2024 The MathWorks, Inc.

classdef ThumbnailDisplay < handle
    
    % This class takes care of figure creation and destruction, figure and
    % handle visibility, and controls.
    properties
        ThumbnailDisplayGrid
        Fig
        Theme
    end
    
    methods
        %------------------------------------------------------------------
        function this = ThumbnailDisplay(browser)
            title = vision.getMessage('vision:caltool:BoardThumbnailFigure');
            
            % suppress docked-window warning
            warning('off', 'MATLAB:figure:SetResize');
            c = onCleanup(@()warning('on', 'MATLAB:figure:SetResize'));
            
            this.Fig = browser.Figure;
            matlab.graphics.internal.themes.figureUseDesktopTheme(this.Fig);
            setTheme(this);
            setFigColor(this);
            this.Fig.ThemeChangedFcn = @(src,evt)this.updateOnThemeChange(src,evt);
            
            browser.Title = title;
            browser.Tag = 'CalibratorThumbnail';
            pos = this.Fig.Position;
            if pos(1) > 1 || pos(2) > 1
                pos(1) = 1;
                pos(2) = 1;
            end
            % this.Fig.Position = pos;
        end
        
        %------------------------------------------------------------------
        function displayInitialDataBrowserMessage(this, isStereo)
            
            if isStereo
                msg = vision.getMessage( ...
                    'vision:caltool:LoadImagesFirstMsgStereo');
            else
                msg = vision.getMessage( ...
                    'vision:caltool:LoadImagesFirstMsg');
            end

            figureWidth = this.Fig.Position(3);

            if isempty(this.ThumbnailDisplayGrid) || ~isvalid(this.ThumbnailDisplayGrid)
                this.ThumbnailDisplayGrid = uigridlayout(this.Fig, [1,1],...
                    'Padding', 0);
                this.setThumbnailDisplayGridColor();
                
            else
                set(this.ThumbnailDisplayGrid, 'Visible', 'on');
                this.setThumbnailDisplayGridColor();
            end
            
            if ~isempty(this.ThumbnailDisplayGrid.Children)
                delete(this.ThumbnailDisplayGrid.Children);
            end
            
            label = uilabel(this.ThumbnailDisplayGrid,...
                'Text', msg,...
                'HorizontalAlignment', 'left', ...
                'VerticalAlignment', 'top',...
                'Position',[3 1 figureWidth-2 90],...
                'WordWrap','on');
        
            set(this.Fig,'AutoResizeChildren','off');
        end
        
        %------------------------------------------------------------------
        function removeDisplayPanel(this)
            if ~isempty(this.ThumbnailDisplayGrid) && isvalid(this.ThumbnailDisplayGrid)
                delete(this.ThumbnailDisplayGrid);
            end
        end
        
        %------------------------------------------------------------------
        function attachThumbnailCallbacks(this, boardThumbnail)
            % Attach callbacks to Board display
            this.Fig.WindowButtonDownFcn  = ...
                @(varargin)boardThumbnail.mouseButtonDownFcn(varargin{1}, varargin{2});
            this.Fig.WindowScrollWheelFcn = ...
                @(varargin)boardThumbnail.mouseWheelFcn(varargin{1}, varargin{2});
            this.Fig.WindowKeyPressFcn    = ...
                @(varargin)boardThumbnail.keyPressFcn(varargin{1}, varargin{2});
        end
        
        %------------------------------------------------------------------
        function setFocus(this)
            % Get focus on current Display
            figure(this.Fig);
        end
        
        %------------------------------------------------------------------
        function wipeFigure(this)
            %wipeFigure Clear the figure
            if ishandle(this.Fig)
                set(this.Fig,'HandleVisibility','on');
                clf(this.Fig);
                set(this.Fig,'HandleVisibility','callback');
            end
        end
        
        %------------------------------------------------------------------
        function makeFigureInvisible(this)
            %makeFigureVisible Turn figure handle visibility off
            this.Fig.Visible = 'off';
        end

        %------------------------------------------------------------------
        function makeFigureVisible(this)
            %makeFigureVisible Turn figure handle visibility off
            this.Fig.Visible = 'on';
        end
        
        %------------------------------------------------------------------
        function close(this)
            %CLOSE Close the figure
            if ishandle(this.Fig)
                this.makeHandleVisible();
                delete(this.Fig);
            end
        end
        
        %------------------------------------------------------------------
        function makeHandleVisible(this)
            %makeHandleVisible Turn figure handle visibility on
            set(this.Fig,'HandleVisibility','on');
        end

        %------------------------------------------------------------------
        function updateOnThemeChange(this,~,~)
            this.setTheme();
            setFigColor(this);
            if isa(this.ThumbnailDisplayGrid,'matlab.ui.container.GridLayout')
                setThumbnailDisplayGridColor(this);
            end
        end

        %------------------------------------------------------------------
        function setThumbnailDisplayGridColor(this)
            this.setTheme(); 
            this.ThumbnailDisplayGrid.BackgroundColor = matlab.graphics. ...
                internal.themes.getAttributeValue(this.Theme,'--mw-backgroundColor-primary');
        end

        %------------------------------------------------------------------
        function setFigColor(this)
            this.Fig.Color = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-primary');
        end

        %------------------------------------------------------------------
        function setTheme(this)
            if this.useDarkTheme
                this.Theme = matlab.graphics.internal.themes.darkTheme;
            else
                this.Theme = matlab.graphics.internal.themes.lightTheme;
            end
        end

        %------------------------------------------------------------------
        function tf = useDarkTheme(this)
            if ~isempty(this.Fig.Theme)
                theme = this.Fig.Theme.BaseColorStyle;
                tf = strcmpi(theme,'dark');
            else
                s = settings;
                theme = s.matlab.appearance.MATLABTheme.ActiveValue;
                tf = strcmpi(theme,'Dark');
            end
        end
    end
end
