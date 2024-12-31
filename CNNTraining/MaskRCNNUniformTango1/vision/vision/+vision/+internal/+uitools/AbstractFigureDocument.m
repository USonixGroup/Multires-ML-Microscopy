classdef AbstractFigureDocument < matlab.ui.internal.FigureDocument
    % AbstractFigureDocument Base class for figures Documents used in the
    % App-Container.
    %
    %   obj = AbstractFigureDocument(options) returns an Figure Document
    %   object representing a figure that can be dropped into a App
    %   Container. options is a structure containing the figure title, Tag
    %   and DocumentGroupTag.
    %
    %   AbstractFigureDocument methods:
    %     restricUserClose    - Restrict the figure closing by user
    %     wipeFigure          - Clear the figure
    %     makeFigureVisible   - Turn figure visibility on
    %     makeHandleVisible   - Turn figure handle visibility on
    %     makeHandleInvisible - Turn figure handle visibility off
    %     lockFigure          - Make the figure modifiable by callbacks only
    %     showHelperText      - Show helper text
    %     hideHelperText      - Hide helper text
    %     close               - Close the figure
    
    % Copyright 2020-2024 The MathWorks, Inc.
    
    properties
        %TextLabel containing static text displayed at top of the figure
        TextLabel
        % Enable theming for the fig document
        Theme
    end
    
    properties(Dependent)
        % Add alias for Title property to keep it in-sync with "Name"
        % property usage in HG.
        Name
    end
    
    methods
        %------------------------------------------------------------------
        function this = AbstractFigureDocument(options)
            this = this@matlab.ui.internal.FigureDocument(options);
            matlab.graphics.internal.themes.figureUseDesktopTheme(this.Figure);
            setTheme(this);
        end
        
        %------------------------------------------------------------------
        function restricUserClose(this)
            this.Closable = false;
        end
        
        %------------------------------------------------------------------
        function wipeFigure(this)
            %wipeFigure Clear the figure
            if ishandle(this.Figure)
                set(this.Figure,'HandleVisibility','on');
                clf(this.Figure);
                set(this.Figure,'HandleVisibility','callback');
            end
        end
        
        %------------------------------------------------------------------
        function makeFigureVisible(this)
            %makeFigureVisible Turn figure handle visibility on
            this.Figure.Visible = 'on';
        end
        
        %------------------------------------------------------------------
        function makeFigureInvisible(this)
            %makeFigureVisible Turn figure handle visibility off
            this.Figure.Visible = 'off';
        end
        
        %------------------------------------------------------------------
        function makeHandleVisible(this)
            %makeHandleVisible Turn figure handle visibility on
            set(this.Figure,'HandleVisibility','on');
        end
        
        %------------------------------------------------------------------
        function makeHandleInvisible(this)
            %makeHandleInvisible Turn figure handle visibility off
            set(this.Figure,'HandleVisibility','off');
        end
        
        %------------------------------------------------------------------
        function lockFigure(this)
            %lockFigure Make the figure modifiable by callbacks only
            if ishandle(this.Figure)
                set(this.Figure,'HandleVisibility', 'callback');
            end
        end
        
        %------------------------------------------------------------------
        function textLabel = showHelperText(this, textString, pos)
            % Expect pos to be normalized
            
            if isempty(this.TextLabel) || ~isvalid(this.TextLabel)
                figPosition = this.Figure.Position;
                helperTextPosition = [1 0.75*figPosition(4) figPosition(3)*0.8  figPosition(4)*0.2];
                color = setHelperTextColor(this);
                this.TextLabel = uilabel(this.Figure,...
                    'HorizontalAlignment','Left',...
                    'Position',helperTextPosition,...
                    'BackgroundColor',color);
            end
            
            if nargin>1
                set(this.TextLabel, 'Text', textString, 'Visible', 'on')
            end
            
            if nargin>2
                set(this.TextLabel, 'Position', pos);
            end
            
            textLabel = this.TextLabel;
        end
        
        %------------------------------------------------------------------
        function hideHelperText(this)
            % Hide the helper text displayed on the document figure
            if ~isempty(this.TextLabel) && isvalid(this.TextLabel)
                set(this.TextLabel, 'Visible', 'off');
            end
        end
        
        %------------------------------------------------------------------
        function close(this)
            %CLOSE Close the figure
            if ishandle(this.Figure)
                this.makeHandleVisible();
                this.Opened = 0;
            end
        end
        
        %------------------------------------------------------------------
        function name = get.Name(this)
            name = this.Title;
        end
        
        %------------------------------------------------------------------
        function setFigureTitle(this, title)
            % The Title property and Fig 'Name' property must be in sync
            set(this.Figure,'Name',title);
            this.Title = title;
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
        function color = setHelperTextColor(this)
            color = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-input');
        end

        %------------------------------------------------------------------
        function reactToThemeChange(this,~,~)
            setTheme(this);
            setFigureColor(this);
            setHelperTextColor(this)
        end
        
        %------------------------------------------------------------------
        function tf = useDarkTheme(this)
            if ~isempty(this.Figure.Theme)
                theme = this.Figure.Theme.BaseColorStyle;
                tf = strcmpi(theme,'dark');
            else
                s = settings;
                mode = s.matlab.appearance.MATLABTheme.ActiveValue;
                tf = strcmpi(mode,'Dark');
            end
        
        end
    end
end
