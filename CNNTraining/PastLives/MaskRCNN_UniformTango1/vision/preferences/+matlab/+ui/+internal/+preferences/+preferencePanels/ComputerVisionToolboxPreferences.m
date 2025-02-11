classdef ComputerVisionToolboxPreferences < handle
    %  Java free, MATLAB UI based Computer Vision Toolbox preference panel
   
    %  The source for the preference panel must be located in a directory
    %  that does not check out a CVT license. (Fix for g3003196)

    %  Copyright 2020 - 2023 The MathWorks, Inc.
    properties
        % Panel Components
        UIFigure
        ParentGrid

        UseParallelTitle
        UseParallelDescription
        UseParallelReq
        UseParallelCheckBox

        PCViewersTitle
        PCViewersDescription
        PCViewersButtonGroup
        RotateAroundAxesRadioButton
        RotateAroundPointRadioButton
    end
        
    properties(Constant)
        % Default settings values
        UseParallelSettingsDefaultValue = 0;
        PCViewersSettingsDefaultValue  = 1;
    end

    methods (Access = public)

        function this = ComputerVisionToolboxPreferences()
               
            % Parent grid layout
            this.ParentGrid = uigridlayout(uifigure,[8 2]);
            this.UIFigure = this.ParentGrid.Parent;
            this.UIFigure.Visible = matlab.lang.OnOffSwitchState.off;

            % Specify the row and column heights for each grid component
            panelGrid = this.ParentGrid;
            rowHeight = {20, 35, 20, 20, 10, 15, 30, 80};
            panelGrid.RowHeight = rowHeight;
            panelGrid.ColumnWidth = {'fit'};
            
            % Use Parallel
            title1 = getString(message("vision:preferencesCVT:ParallelComputingSupportTitle"));
            this.UseParallelTitle = uilabel(panelGrid,"Text", title1, "FontWeight", "bold",'Tag','labeluseparallel');
            this.UseParallelTitle.Layout.Row = 1;
            
            text1 = getString(message("vision:preferencesCVT:ParallelComputingSupportDescription"));
            this.UseParallelDescription = uilabel(panelGrid, "Text",text1, "WordWrap","on");
            this.UseParallelDescription.Layout.Row = 2;
            
            text2 = getString(message("vision:preferencesCVT:ParallelRequirement"));
            this.UseParallelReq = uilabel(panelGrid, "Text", text2,"WordWrap", "on");
            this.UseParallelReq.Layout.Row = 3;

            text3 = getString(message('vision:preferencesCVT:Useparallel'));
            this.UseParallelCheckBox = uicheckbox(panelGrid,"Text", text3);
            this.UseParallelCheckBox.Layout.Row  = 4;
            this.UseParallelCheckBox.Value = this.UseParallelSettingsDefaultValue;
           
            % PC Viewers
            title2 = getString(message('vision:preferencesCVT:PointCloudViewers'));
            this.PCViewersTitle = uilabel(panelGrid, "Text", title2, "FontWeight", "bold");
            this.PCViewersTitle.Layout.Row = 6;
            
            text4 = getString(message("vision:preferencesCVT:PointCloudViewersDescription"));
            this.PCViewersDescription = uilabel(panelGrid, "Text", text4, "WordWrap","on");
            this.PCViewersDescription.Layout.Row = 7;
            
            this.PCViewersButtonGroup  = uibuttongroup(panelGrid,"BorderType","none");
            this.PCViewersButtonGroup.Layout.Row = 8;

            buttonGrpHeight = this.PCViewersButtonGroup.Position(4);
            y = buttonGrpHeight - 15;
            this.RotateAroundAxesRadioButton = uiradiobutton(this.PCViewersButtonGroup, 'Position', [1 y 169 15]);
            this.RotateAroundAxesRadioButton.Text = getString(message('vision:preferencesCVT:RotateAroundCenter'));
            
            this.RotateAroundPointRadioButton = uiradiobutton(this.PCViewersButtonGroup, 'Position',[1 y-30 162 15]);
            this.RotateAroundPointRadioButton.Text = getString(message('vision:preferencesCVT:RotateAroundPoint'));
            this.RotateAroundPointRadioButton.Value = this.PCViewersSettingsDefaultValue;

            % Get the settings and assign the values
            getCurrentSettingValues(this);
            this.UIFigure.Visible = matlab.lang.OnOffSwitchState.on;
        end
        
        function getCurrentSettingValues(this)
            s = settings;
            this.UseParallelCheckBox.Value= s.vision.parallelsupport.UseParallel.ActiveValue;
            this.RotateAroundAxesRadioButton.Value = ~s.pointclouds.pcviewers.PCViewerRotation.ActiveValue;
            this.RotateAroundPointRadioButton.Value = s.pointclouds.pcviewers.PCViewerRotation.ActiveValue;
        end
        
        function setCurrentSettingValues(this)
            s = settings;
            s.vision.parallelsupport.UseParallel.PersonalValue = this.UseParallelCheckBox.Value;
            s.pointclouds.pcviewers.PCViewerRotation.PersonalValue = this.RotateAroundAxesRadioButton.Value;
        end
        
        function result = validate(this)
            % TODO when validation is supported
            result = true;
        end

        function result = commit(this)
            try
              % Save the preferences
                setCurrentSettingValues(this)
                result = true;                
            catch ME
                result = false;
            end
        end
         
        function delete(this)
            delete(this.UIFigure);
        end
           
    end
    
end
