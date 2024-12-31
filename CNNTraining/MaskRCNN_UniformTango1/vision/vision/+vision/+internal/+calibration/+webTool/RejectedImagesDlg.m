% RejectedImagesDlg Dialog for displaying rejected calibration images.

% Copyright 2014-2021 The MathWorks, Inc.

classdef RejectedImagesDlg < images.internal.app.utilities.Dialog
    properties(Access=private)
        RejectedFileNames = {};
        Ok
    end
    
    methods
        function this = RejectedImagesDlg(fileNames, location)
            dlgTitle =  vision.getMessage('vision:caltool:RejectedImagesDialogTitle');
           
            this = this@images.internal.app.utilities.Dialog(location, dlgTitle);
            this.RejectedFileNames = fileNames;
            this.Size = [800, 600];
            create(this);
            this.FigureHandle.Visible = 'on';

            % Create grid layout for Image and button
            parentGridLayout = uigridlayout(this.FigureHandle);
            parentGridLayout.RowHeight = {'1x', 'fit'};
            parentGridLayout.ColumnWidth = {'1x', 'fit', '1x'};

            addImageDisplay(this, parentGridLayout);
            addOK(this, parentGridLayout)
        end
    end
    
    methods(Access=private)
        function addImageDisplay(this, parentGridLayout)
            ax = axes('parent', parentGridLayout);
            ax.Layout.Column = [1 3];

            % Drawnow to complete axes creation before passing image into it.
            drawnow();
            showRejectedImages(this, ax);
            
            set(ax.Toolbar, 'Visible', 'off');
            disableDefaultInteractivity(ax);
        end
        
        %------------------------------------------------------------------
        function showRejectedImages(this, ax)
            if size(this.RejectedFileNames, 1) == 1
                montage(this.RejectedFileNames, 'Parent', ax);
            else
                numPairs = size(this.RejectedFileNames, 2);
                
                images = cell(numPairs,1);
                for i = 1:numPairs
                    I1 = imread(this.RejectedFileNames{1, i});
                    I2 = imread(this.RejectedFileNames{2, i});
                    images{i} = imfuse(I1, I2, 'montage');
                end
                
                montage(images, 'Parent', ax);
            end
        end

        %--Add Ok----------------------------------------------------------
        function addOK(this, parentGridLayout)
            this.Ok =  uibutton('Parent', parentGridLayout, ...
                'ButtonPushedFcn', @(~,~) close(this),...
                'FontSize', 12, ...
                'Text',getString(message('MATLAB:uistring:popupdialogs:OK')),...
                'Tag', 'OK');
            this.Ok.Layout.Row = 2;
            this.Ok.Layout.Column = 2;
        end
    end

    methods (Access = protected)
        
        %--Key Press-------------------------------------------------------
        function keyPress(self, evt)
            
            switch(evt.Key)
                case {'return', 'space', 'escape'}
                    close(self);
            end
            
        end
        
    end
end
