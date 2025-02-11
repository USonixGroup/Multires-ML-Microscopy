% AutoPopulateAprilGridPropsDlg Dialog for displaying success or failure
% status of the AprilGrid properties detection.

% Copyright 2024 The MathWorks, Inc.

classdef AutoPopulateAprilGridPropsDlg

    properties
        DlgType
        File = []
        Filename = [];
        Message = "";
        Title
    end

    properties (Access=private)
        DetectorUI
        Figure
        Position
        OkButton
        ThumbnailImage
    end

    % Dialog Presets
    properties (Access=private)
        ButtonsLayout
        DlgSize
        ThumbnailSize = 150;
    end

    methods
        %----------------------------------------------------------------------
        function this = initializeProperties(this, caller, filename, parentDlgLoc, varargin)

            this.DetectorUI = caller;

            this.File = filename;

            [~, filename, ext] = fileparts(filename);
            this.Filename = strcat(filename, ext);
            
            if nargin > 5
                this.DlgType = "SuccessDlg";
                this.ButtonsLayout = {"1x","fit","fit","1x"};
                this.DlgSize = [500 220];
                this.Title = vision.getMessage("vision:caltool:AutoPopulationSuccessTitleAprilGrid");
                props = [num2cell(varargin{1}), varargin{2:end}];
                this.Message = vision.getMessage("vision:caltool:DetectedPropsMsgAprilGrid", props{:});
            else
                this.DlgType = "ErrorDlg";
                this.ButtonsLayout = {"1x","fit","1x"};
                this.DlgSize = [550 220];
                this.Title = vision.getMessage("vision:caltool:AutoPopulationFailedTitleAprilGrid");
                this.Message = vision.getMessage("vision:caltool:AutoPopulationFailedMsgAprilGrid", varargin{:});
            end

            this = initializeDlgPosition(this, parentDlgLoc);
            this = initializeThumbnailImage(this);
        end

        %----------------------------------------------------------------------
        function this = initializeDlgPosition(this, parentDlgLoc)

            import imageslib.internal.app.utilities.ScreenUtilities
            loc = ScreenUtilities.getModalDialogLocation(parentDlgLoc, this.DlgSize);
            this.Position = [loc, this.DlgSize];
        end

        %----------------------------------------------------------------------
        function this = initializeThumbnailImage(this)
            
            img = imread(this.File);
            [height, width] = size(img, 1:2);

            % Resize the largest dimension to the thumbnail size while
            % maintaining image aspect ratio.
            if width > height
    
                this.ThumbnailImage = imresize(img, [NaN this.ThumbnailSize], "bilinear");
            else

                this.ThumbnailImage = imresize(img, [this.ThumbnailSize NaN], "bilinear");
            end
        end
    end
    
    methods
        %----------------------------------------------------------------------
        function this = AutoPopulateAprilGridPropsDlg(parent, filename, parentDlgLoc, varargin)
            
            % Initialize properties.
            this = initializeProperties(this, parent, filename, parentDlgLoc, varargin{:});
            
            % Create dialog.
            this.Figure = uifigure(Name=this.Title, Position = this.Position,...
                Resize="off", WindowStyle="modal");
            this.Figure.CloseRequestFcn = @(~,~) doCancel(this);
            this.Figure.Visible = "off";

            % Configure UI components.
            this = configureUIComponents(this, varargin{:});
            
            % Turn visibility on for the dialog.
            this.Figure.Visible = "on";
            
            % Focus on the ok/auto-populate button.
            focus(this.OkButton)
        end
    end
    
    methods(Access=private)

        %----------------------------------------------------------------------
        function this = configureUIComponents(this, varargin)

            % Create grid layouts for the UI components.
            [statusGridLayout, buttonsGridLayout] = createUIGridLayouts(this);

            % Thumbnail
            addThumbnail(this, statusGridLayout);
            
            if this.DlgType == "ErrorDlg"
                % Error message
                addErrorMsg(this, statusGridLayout);

                % OK button
                text = "   " + vision.getMessage("MATLAB:uistring:popupdialogs:OK") + "   ";
                this = addOKButton(this, text, buttonsGridLayout);
            else
                % Detected properties
                addDetectedProperties(this, statusGridLayout);
                
                % OK button
                text = vision.getMessage("vision:caltool:AutoPopulateButton");
                this = addOKButton(this, text, buttonsGridLayout, varargin{:});
    
                % Cancel button
                addCancelButton(this, buttonsGridLayout)
            end

            matlab.ui.internal.PositionUtils.fitToContent(this.Figure);
        end
    end

    methods(Access=private)
        %----------------------------------------------------------------------
        %  ------------------------
        % |  Caption   |  Filename |
        % |------------------------|
        % |  Text      | Thumbnail |
        % |            |           |
        % |------------------------|
        % |         Buttons        |
        %  ------------------------
        function [statusGridLayout, buttonsGridLayout] = createUIGridLayouts(this)
            
            % Create grid layout for status and buttons.
            gridLayout = uigridlayout(this.Figure,RowSpacing=0);
            gridLayout.RowHeight = {"fit", "fit"};
            gridLayout.ColumnWidth = {"fit"};

            % Create grid layout for thumbnail image and status text.
            totalWidth = this.Position(3)-50;
            statusGridLayout = uigridlayout(gridLayout);
            statusGridLayout.RowHeight = {"fit","fit"};
            statusGridLayout.ColumnWidth = {totalWidth-this.ThumbnailSize, ...
                                            this.ThumbnailSize};
            statusGridLayout.Layout.Row = 1;
            statusGridLayout.Layout.Column = 1;

            % Create grid layout for the buttons.
            buttonsGridLayout = uigridlayout(gridLayout);
            buttonsGridLayout.RowHeight = {"fit"};
            buttonsGridLayout.ColumnWidth = this.ButtonsLayout;
            buttonsGridLayout.Layout.Row = 2;
            buttonsGridLayout.Layout.Column = 1;
        end

        %----------------------------------------------------------------------
        % UI Elements
        %----------------------------------------------------------------------
        function addThumbnail(this, gridLayout)

            % Filename
            imgFilename = uilabel(Parent=gridLayout, WordWrap="on", FontWeight="bold", ...
                Text=vision.getMessage("vision:caltool:InputImageCaption", this.Filename), ...
                HorizontalAlignment="center",VerticalAlignment="top");
            imgFilename.Layout.Row = 1; 
            imgFilename.Layout.Column = 2;

            % Image
            imgLayout = uigridlayout(gridLayout, Padding = [0 0 0 0]);
            padding = 10;
            imgLayout.RowHeight = this.ThumbnailSize-padding;
            imgLayout.ColumnWidth = this.ThumbnailSize-padding;
            imgLayout.Layout.Row = 2; 
            imgLayout.Layout.Column = 2;
            uiimage(imgLayout, ImageSource=this.ThumbnailImage);
        end

        %----------------------------------------------------------------------
        function addErrorMsg(this, gLayout)
                        
            errorMsg = uilabel(Parent=gLayout, WordWrap="on", ...
                Text=this.Message, HorizontalAlignment="left",...
                VerticalAlignment="center");
            errorMsg.Layout.Row = 2;
            errorMsg.Layout.Column = 1;
        end

        %----------------------------------------------------------------------
        function addDetectedProperties(this, gLayout)

            % Detected properties caption/column title.
            caption = uilabel(Parent=gLayout, WordWrap="on", ...
                Text=vision.getMessage("vision:caltool:DetectedPropsCaption"), ...
                HorizontalAlignment="center", VerticalAlignment="top", ...
                FontWeight="bold");
            caption.Layout.Row = 1;
            caption.Layout.Column = 1;

            % Detected properties.
            propsMsg = uilabel(Parent=gLayout, WordWrap="on", ...
                Text=this.Message, HorizontalAlignment="left",...
                VerticalAlignment="center");
            propsMsg.Layout.Row = 2;
            propsMsg.Layout.Column = 1;
        end

        %----------------------------------------------------------------------
        function this = addOKButton(this, text, gridLayout, varargin)

            this.OkButton = uibutton(Parent=gridLayout, ...
                ButtonPushedFcn=@(~,~) doOK(this, varargin{:}),...
                FontSize=12, Text=text, Tag="OK");
            this.OkButton.Layout.Column = 2;
            this.OkButton.Layout.Row = 1;
        end

        %----------------------------------------------------------------------
        function addCancelButton(this, gridLayout)
            cancelButton = uibutton(Parent=gridLayout, FontSize=12, ...
                ButtonPushedFcn=@(~,~) doCancel(this), Tag="Cancel", ...
                Text=vision.getMessage("MATLAB:uistring:popupdialogs:Cancel"));
            cancelButton.Layout.Column = 3;
            cancelButton.Layout.Row = 1;
        end
    end

    %--------------------------------------------------------------------------
    % UI Callbacks
    %--------------------------------------------------------------------------
    methods (Access=private)
        
        %--------------------------------------------------------------------------
        % doCancel: Close the dialog and enable interactions in parent figure.
        %--------------------------------------------------------------------------
        function doCancel(this)
            enableInteractions = true;
            updateDialogState(this.DetectorUI, enableInteractions);
            delete(this.Figure)
        end

        %--------------------------------------------------------------------------
        % doOK: Close the dialog, enable interactions in parent figure and
        % update the parent figure UI elements with the detected properties.
        %----------------------------------------------------------------------
        function doOK(this, varargin)
            enableInteractions = true;
            detectedProperties = varargin;
            updateDialogState(this.DetectorUI, enableInteractions, detectedProperties{:});
            delete(this.Figure)
        end
    end
end