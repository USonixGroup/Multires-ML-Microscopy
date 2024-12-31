% AddImageStatsDlg Class for displaying add image statistics
%
% This object implements a dialog box for displaying the number of added,
% rejected and skipped images.

% Copyright 2014-2021 The MathWorks, Inc.

classdef AddImageStatsDlg < images.internal.app.utilities.OkDialog
    
    properties
        RejectedImagesBtn = [];
    end

    properties(Access=protected)
        DlgHeight;
        TextHeight;
        TextYPosition;
        StatsExtent = -1;
        StatsTextPosition;
        ButtonHeightDenom;
        
        NumRejected;
        StatsString;
    end
    
    properties(Access=protected)
        HeadingString;
        RejectedFileNames = {};
    end

    properties
        RejectedImagesDlg
    end
    
    methods
        function this = AddImageStatsDlg(stats, rejectedFileNames, location)
            
            dlgTitle = vision.getMessage('vision:caltool:CalibrationCompleteTitle');
            
            this = this@images.internal.app.utilities.OkDialog(location, dlgTitle);
            
            this.RejectedFileNames = rejectedFileNames;
            
            if stats.numDuplicates == 0
                initNoDuplicates(this, stats.numProcessed, stats.numAdded, stats.numPartial);
            else
                initWithDuplicates(this, stats.numProcessed, stats.numAdded,...
                    stats.numDuplicates, stats.numPartial);
            end
            
            this.Size = [390, this.DlgHeight];
            create(this);
            addHeading(this);
            addStats(this);
            if ~isempty(this.RejectedFileNames)
                addShowRejectedImagesBtn(this);
            end
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function initNoDuplicates(this, numProcessed, numAdded, numPartial)
            this.DlgHeight  = 120;
            this.TextHeight = 85;
            this.ButtonHeightDenom = 4;
            this.TextYPosition = 30;
            
            this.NumRejected = numProcessed - numAdded;
            setHeadingStringNoDuplicates(this);
            this.StatsString = sprintf('%d\n%d\n%d\n%d',...
                numProcessed, numAdded, numPartial, this.NumRejected);
        end
        
        %------------------------------------------------------------------
        function initWithDuplicates(this, numProcessed, numAdded, numDuplicates, numPartial)
            this.DlgHeight  = 140;
            this.TextHeight = 90;
            this.ButtonHeightDenom = 5;
            this.TextYPosition = 40;
            
            this.NumRejected = numProcessed - numAdded - numDuplicates;
            setHeadingStringWithDuplicates(this);
            this.StatsString = sprintf('%d\n%d\n%d\n%d\n%d',...
                numProcessed, numAdded, numPartial, this.NumRejected, numDuplicates);
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function setHeadingStringNoDuplicates(this)
            this.HeadingString = vision.getMessage(...
                'vision:caltool:NumDetectedBoards');
        end
        
        %------------------------------------------------------------------
        function setHeadingStringWithDuplicates(this)
            this.HeadingString = vision.getMessage(...
                'vision:caltool:AddBoardStatistics');
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function addHeading(this)
            uilabel(this.FigureHandle,...
                'Text', this.HeadingString,...
                'HorizontalAlignment', 'left', ...
                'Position',[20, this.TextYPosition, 270, this.TextHeight]);
        end
        
        %------------------------------------------------------------------
        function addStats(this)
            this.StatsTextPosition = [235, this.TextYPosition, 50, this.TextHeight];
            uilabel(this.FigureHandle,...
                'Text', this.StatsString,...
                'HorizontalAlignment', 'Right', ...
                'Position',this.StatsTextPosition);
            
            this.StatsExtent = hgconvertunits(this.FigureHandle, [0 0 1 1], 'char', 'pixels', this.FigureHandle);
        end
        
        %------------------------------------------------------------------
        function addShowRejectedImagesBtn(this)
            textSize = this.StatsExtent;
            buttonHeight = round(textSize(4)*1.3);
            fudgeOffset = 2; % helps to better align the button with text
            buttonYPosition = this.StatsTextPosition(2) + this.StatsExtent(4) - fudgeOffset;
            
            this.RejectedImagesBtn = uibutton(this.FigureHandle,...
                'Position', [this.StatsTextPosition(1) + 60, ...
                buttonYPosition, 90, buttonHeight], ...
                'Text', vision.getMessage('vision:caltool:SeeImages'),...
                'ButtonPushedFcn', @this.showRejectedImagesDlg,...
                'FontSize', buttonHeight*0.6,...
                'Tag', 'btnSeeImages');
            this.RejectedImagesBtn.Interruptible = 'off';
            this.RejectedImagesBtn.BusyAction = 'cancel';
        end
        
        %------------------------------------------------------------------
        function showRejectedImagesDlg(this, ~, ~)
            persistent rejectedImageDlgState
            
            if isempty(rejectedImageDlgState)
                rejectedImageDlgState = true;
                this.RejectedImagesDlg = vision.internal.calibration.webTool.RejectedImagesDlg(...
                    this.RejectedFileNames, this.Location);
                wait(this.RejectedImagesDlg);
                rejectedImageDlgState = [];
            end
        end
    end
end
