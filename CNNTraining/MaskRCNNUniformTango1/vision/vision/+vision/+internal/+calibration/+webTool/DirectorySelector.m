% DirectorySelector A compound UI control for selecting a folder

% Copyright 2014-2023 The MathWorks, Inc.

classdef DirectorySelector < handle
    properties
        Parent;
        TextBox;
        Label;
        BrowseButton;
        IsModifiedUsingBrowse = false;
    end
    
    properties(Dependent)
        SelectedDir;
    end
    
    methods
        function this = DirectorySelector(label, parent, initialDir, tagName)
            % add label
            directoryGrid = uigridlayout(parent, 'Padding', 0);
            directoryGrid.RowHeight = {'fit', 'fit'};
            directoryGrid.ColumnWidth = {'1x', 'fit'};

            this.Label = uilabel('Parent',directoryGrid,...
                'HorizontalAlignment', 'Left',...
                'Text', label);
            
            % add text box
            tag = [tagName, 'TextBox'];
            
            this.TextBox = uieditfield(directoryGrid,'Editable','on', ...
                'HorizontalAlignment', 'left', ...:
                'Tag', tag, ...
                'Value', initialDir);
            this.TextBox.Layout.Row = 2;
            this.TextBox.Layout.Column= 1;
            
            % add "Browse" button
            tag = [tagName, 'TextBox'];
            
            this.BrowseButton = uibutton('Parent',directoryGrid, ...
                'ButtonPushedFcn', @this.doBrowse,...
                'Tag', tag, ...
                'Text', vision.getMessage('vision:caltool:BrowseButton'));
        end
        
        %--------------------------------------------------------------
        function doBrowse(this, varargin)
            selectedDir = uigetdir(get(this.TextBox, 'Value'));
            if selectedDir ~= 0
                set(this.TextBox, 'Value', selectedDir);
                this.IsModifiedUsingBrowse = true;
            end
        end
        
        %------------------------------------------------------------------
        function selectedDir = get.SelectedDir(this)
            selectedDir = get(this.TextBox, 'Value');
        end
        
        %------------------------------------------------------------------
        function updateComponentLocations(this, location)
            this.TextBox.Position(2) = location(2);
            this.BrowseButton.Position(2) = location(2);
            this.Label.Position(2) = location(2) + 30;
        end
    end
end

