% VariableSelector A compound UI control for selecting a variable

% Copyright 2017 The MathWorks, Inc.

classdef VariableSelector < handle
    properties          
        Parent;
        PopupMenu;
        validVarNames;
        allWkspaceVars;
    end
    
    properties(Dependent)
        SelectedVar
    end
    
    methods
        
        %------------------------------------------------------------------
        function this = VariableSelector(label, position, parent, allowedClass)
            
            % add label
            uilabel('Parent',parent,...
                'Position', position,...
                'HorizontalAlignment', 'Left',...
                'Text', label);
            
            % Dropdown to choose variables
            this.allWkspaceVars = evalin('base', 'whos');
            this.validVarNames = {''};
            
            for idx = 1:numel(this.allWkspaceVars)
                if any(strcmp(this.allWkspaceVars(idx).class, allowedClass))
                   this.validVarNames{end + 1} =  this.allWkspaceVars(idx).name;
                end
            end
            
            % add popup menu
            menuPos = position;
            menuPos(1) = menuPos(1);
            menuPos(2) = menuPos(2) - 30;
            menuPos(3) = menuPos(3);
            this.PopupMenu = uidropdown(parent,...
                'Position', menuPos, ...
                'Items', this.validVarNames);
        end
        
        %------------------------------------------------------------------
        function showEmptyState(this)
            this.PopupMenu.Items = {vision.getMessage('vision:caltool:NoVariables')};
            this.PopupMenu.Enable = 'off';
        end
        
        %------------------------------------------------------------------
        function switchIfValid(this, variableName)
            % Dropdown to choose variables
            for idx = 1:numel(this.validVarNames)
                if strcmp(this.validVarNames{idx}, variableName)
                   set(this.PopupMenu, 'Value', this.validVarNames{idx});
                end
            end
        end
        
        %------------------------------------------------------------------
        function selectedVar = get.SelectedVar(this)
            selectedVar = get(this.PopupMenu, 'Value');
        end
    end
end
        