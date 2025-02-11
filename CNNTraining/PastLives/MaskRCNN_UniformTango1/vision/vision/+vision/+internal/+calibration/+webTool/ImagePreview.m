classdef ImagePreview < matlab.ui.internal.FigureDocument
    % ImagePreview Class for the main image preview display in the Image Capture Tab
    %
    % This class takes care of figure creation and destruction, figure and
    % handle visibility, and zoom controls.
    
    % Copyright 2012-2020 The MathWorks, Inc.
    
    properties
        Fig = [];
        ImHandle = [];
        ScrollPanelHandle = [];
        HAxes
        ImageAxesTag = 'PreviewImageAxes';
        Width;
        Height;
    end
    
    methods
        function this = ImagePreview(options)
            this = this@matlab.ui.internal.FigureDocument(options);
            this.Figure.Scrollable = 'on';
        end
        
        %------------------------------------------------------------------
        function drawImage(this, width, height)
            makeHandleVisible(this);
            this.HAxes = createImageAxes(this, width, height);
            
            % Create the image.
            if isempty(this.ImHandle)
                
                this.ImHandle = imshow(zeros(height, width, 3), 'Parent', this.HAxes, 'InitialMagnification', 'fit', 'Border', 'tight');
                
                this.HAxes.LooseInset(3:4) = 0;
                this.HAxes.Position = [0 0 width height];
                
                positionedImage(this);
            end
            
            set(this.HAxes.Toolbar,'Visible', 'off');
            disableDefaultInteractivity(this.HAxes);
            
            set(this.HAxes, 'Tag', this.ImageAxesTag);
            makeHandleInvisible(this);
            
            set(this.HAxes, 'HitTest', 'off');
            set(this.HAxes, 'PickableParts', 'none');
        end
        
        %------------------------------------------------------------------
        function setTitle(this, titleString)
            hAxes = getImageAxes(this);
            title(hAxes, titleString, 'Interpreter', 'none');
        end
        
        %------------------------------------------------------------------
        function wipeFigure(this)
            if ishandle(this.Figure)
                set(this.Figure,'HandleVisibility','on');
                clf(this.Figure);
            end
        end
        
        %------------------------------------------------------------------
        function makeFigureVisible(this)
            if isvalid(this.Figure)
                set(this.Figure, 'Visible', 'on');
            end
        end
        
        %------------------------------------------------------------------
        function makeFigureInvisible(this)
            if isvalid(this.Figure)
                set(this.Figure, 'Visible', 'off');
            end
        end
        
        %------------------------------------------------------------------
        function makeHandleVisible(this)
            set(this.Figure,'HandleVisibility','on');
        end
        
        %------------------------------------------------------------------
        function makeHandleInvisible(this)
            set(this.Figure,'HandleVisibility','off');
        end
        
        %------------------------------------------------------------------
        function close(this)
            if ishandle(this.Figure)
                this.makeHandleVisible();
                delete(this.Figure);
            end
        end
        
        %------------------------------------------------------------------
        function tf = isAxesValid(this)
            tf = ~isempty(getImageAxes(this));
        end
        
        %------------------------------------------------------------------
        function hAxes = getImageAxes(this)
            hAxes = findobj(this.Figure, 'Type','axes','Tag', this.ImageAxesTag);
        end
        
        %------------------------------------------------------------------
        function hAxes = createImageAxes(this, imageWidth, imageHeight)
            hAxes = getImageAxes(this);
            if isempty(hAxes) % add an axes if needed
                
                hAxes = axes('Parent', this.Figure,...
                    'Tag', this.ImageAxesTag,...
                    'XLim',[.5, (imageWidth + .5)],...
                    'YLim',[.5, (imageHeight + .5)],...
                    'TickDir', 'out', ...
                    'XGrid', 'off', ...
                    'YGrid', 'off', ...
                    'XLimMode','manual',...
                    'YLimMode','manual',...
                    'XTick', [],...
                    'YTick', [],...
                    'Ydir', 'reverse',...
                    'Units','Pixel',...
                    'Position', this.Figure.Position);
            end
        end
        
        %------------------------------------------------------------------
        function replaceImage(this, width, height)
            % Resolution may have changed, replace with new image.
            data = zeros(height, width, 3);
            this.ImHandle.CData = data;
            
            this.HAxes.Position = [0 0 width, height];
            this.HAxes.XLim = [.5, (width + .5)];
            this.HAxes.YLim = [.5, (height + .5)];
            
            positionedImage(this);
        end
        
        %------------------------------------------------------------------
        function clearAxes(this)
            if ~isempty(this.HAxes)
                cla(this.HAxes);
                this.HAxes = [];
            end
        end
        
        %------------------------------------------------------------------
        function positionedImage(this)
            if this.Figure.Position(3) > this.HAxes.Position(3)
                this.HAxes.Position(1) = (this.Figure.Position(3) - this.HAxes.Position(3))/2;
                scrollX = 0;
            else
                this.HAxes.Position(1) = 0;
                scrollX = round((this.HAxes.Position(3) - this.Figure.Position(3))/2);
            end
            
            if this.Figure.Position(4) > this.HAxes.Position(4)
                this.HAxes.Position(2) = (this.Figure.Position(4) - this.HAxes.Position(4))/2;
                scrollY = 0;
            else
                this.HAxes.Position(2) = 0;
                scrollY = round((this.HAxes.Position(4) - this.Figure.Position(4))/2);
            end
            
            % Move slider to center position
            % pause(3)% TODO: remove after g2256618 fix
            scroll(this.Figure, scrollX, scrollY);
        end
    end
end
