classdef AppFig < handle %%% MK: see Component.m in spc frame work
    % AppFig Base class for figures used in a tool strip-based app.
    %
    %   fig = AppFig(title) returns an AppFig object representing a
    %   figure that can be dropped into a tool strip app. title is a string
    %   containing the figure title.
    %
    %   AppFig properties:
    %     Fig       - The figure handle
    %     Title     - The figure title
    %     GroupName - the name of the toolgroup
    %
    %   AppFig methods:
    %     createFigure        - Create the figure
    %     wipeFigure          - Clear the figure
    %     makeFigureVisible   - Turn figure visibility on
    %     makeHandleVisible   - Turn figure handle visibility on
    %     makeHandleInvisible - Turn figure handle visibility off
    %     lockFigure          - Make the figure modifiable by callbacks only
    %     showHelperText      - Show helper text
    %     hideHelperText      - Hide helper text
    %     close               - Close the figure
    
    % Copyright 2014-2024 The MathWorks, Inc.
    
    properties (Access = private)
        FigurePanel

        % Displays that utilize uilabel
        UILabelDisplays
    end
    
    properties
        %Fig The figure handle
        Fig = []
        
        FigureDocument
        
        %Title The figure title
        Title
        
        %GroupName The name of the toolgroup. Useful for creating dialogs
        %          from within an AppFig.
        GroupName
        
        %TextLabel UICONTROL containing static text displayed at top of the
        %          figure
        TextLabel
        
        %MessagePane Optional yellow banner for displaying
        %            helpful/instructive text.
        MessagePane
        
        %MessageText String (one line long) holding help text to display in
        %            yellow banner MessagePane.
        MessageText

        ColorBeige 
        % Dark or light Theme
        Theme
    end
   
    properties(Dependent)
        % Add alias for Title property to keep it in-sync with "Name"
        % property usage in HG.
        Name
    end
    
    methods
        %------------------------------------------------------------------
        function this = AppFig(hFig, title, varargin)
            
            % varargin: assumed to be true
            % If there is varargin, it means: change background color to
            % beige
            this.Fig = hFig;
            this.Fig.Name =  title;
            this.Title = title;
            this.Fig.ToolBarMode = 'auto';
            matlab.graphics.internal.themes.figureUseDesktopTheme(this.Fig);
            this.Theme = this.Fig.Theme;
            this.Fig.ThemeChangedFcn = @(src,evt)this.updateOnThemeChange();
            % 
            if nargin > 2 
                this.ColorBeige = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-primary');
            end
        end
     
        function n = getName(this)
            n = this.Title;
        end
        
        function t = getTag(this)
            t = this.Tag;
        end
        
        function b = isCloseable(~)
            b = false;
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
        function makeFigureVisible(this)
            %makeFigureVisible Turn figure handle visibility on
            set(this.Fig, 'Visible', 'on');
        end
        
        %------------------------------------------------------------------
        function makeFigureInvisible(this)
            %makeFigureVisible Turn figure handle visibility off
            set(this.Fig, 'Visible', 'off');
        end
       
        %------------------------------------------------------------------
        function makeHandleVisible(this)
            %makeHandleVisible Turn figure handle visibility on
            set(this.Fig,'HandleVisibility','on');
        end
        
        %------------------------------------------------------------------
        function makeHandleInvisible(this)
            %makeHandleInvisible Turn figure handle visibility off
            set(this.Fig,'HandleVisibility','off');
        end
        
        %------------------------------------------------------------------
        function lockFigure(this)
            %lockFigure Make the figure modifiable by callbacks only
            if ishandle(this.Fig)
                set(this.Fig,'HandleVisibility', 'callback');
            end
        end
        
        %------------------------------------------------------------------
        function textLabel = showHelperText(this, textString, pos)
            % Expect pos to be normalized
            
            if isempty(this.TextLabel) || ~isvalid(this.TextLabel)
                if ismember(class(this), this.UILabelDisplays)
                    this.TextLabel = uilabel(this.Fig,...
                        Text="",...
                        HorizontalAlignment="left",...
                        VerticalAlignment="top",...
                        WordWrap=false,...
                        Position=this.Fig.Position);
                else
                    this.TextLabel = uicontrol(this.Fig,...
                            Style='text',...
                            HorizontalAlignment='Left',...
                            Units='normalized',...
                            Position=[0.05 0.75 0.8 0.1],...
                            ForegroundColor=[0.45 0.45 0.45]);
                end
            end
            
            if nargin>1
                if ismember(class(this.TextLabel), "matlab.ui.control.Label")
                    set(this.TextLabel, 'Text', textString, 'Visible', 'on');
                else
                    set(this.TextLabel, 'String', textString, 'Visible', 'on');
                end
            end
            
            if nargin>2
                set(this.TextLabel, 'Position', pos);
            end
            
            textLabel = this.TextLabel;
        end
        
        %------------------------------------------------------------------
        function hideHelperText(this)
            
            if ~isempty(this.TextLabel) && isvalid(this.TextLabel)
                set(this.TextLabel, 'Visible', 'off');
            end
        end
        
        %------------------------------------------------------------------
        function updateHelperText(this, textString)
            
            if ~isempty(this.TextLabel) && isvalid(this.TextLabel)
                if ismember(class(this), this.UILabelDisplays)
                    set(this.TextLabel,'Text',textString)
                else
                    set(this.TextLabel, 'String',textString)
                end
            end
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
        function name = get.Name(this)
            name = this.Title;
        end
        
        %------------------------------------------------------------------
        function setFigureTitle(this, title)
            % The Title property and Figure 'Name' property must be in sync
            set(this.Fig,'Name',title);
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
        % 
        %------------------------------------------------------------------
        function updateOnThemeChange(this)
            setTheme(this);
            this.ColorBeige = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-primary');
            this.Fig.Color = this.ColorBeige;

        end
        % 
        % %------------------------------------------------------------------
        function tf = useDarkTheme(this)
           if ~isempty(this.Fig.Theme)
                theme = this.Fig.Theme.BaseColorStyle;
                tf = strcmpi(theme,'Dark');
            else
                s = settings;
                theme = s.matlab.appearance.MATLABTheme.ActiveValue;
                tf = strcmpi(theme,'Dark');
            end
        end
    end    
end


