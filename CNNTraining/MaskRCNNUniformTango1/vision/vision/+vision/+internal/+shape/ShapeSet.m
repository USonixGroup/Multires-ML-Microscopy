classdef ShapeSet < handle
    % This class defines the interface for a ShapeSet. A ShapeSet is a 
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020-2021 The MathWorks, Inc.
    
    properties(Access=protected)
        % Shape A cell array of shapes. These are typically instances of
        % the underlying HG object representing the shape. For example,
        % images.roi.Rectangle.
        Shape = {}
    end
    
    properties(Dependent)
        % Count return the number of shapes in this ShapeSet. 
        Count
    end
    
    methods(Abstract)
        % The method used to show shapes on a figure. 
        show(this,position,params)
    end
    
    methods(Static,Abstract)
        % Return an instance of a ShapeSet with zero shapes.
        makeSet()
        
        % Allocate N shapes.  
        allocate(N)
    end
    
    methods
        
        function n = get.Count(this)
            % Return the number of shapes in this set.
            n = numel(this.Shape);
        end
        
        function set = pop(this,N)
            % Pop N shapes from this set and return them.
            if nargin == 1
                % pop all
                N = this.Count;
            end
            set = this.makeSet();
            set.Shape = this.Shape(1:N);
            this.Shape(1:N) = [];
        end
        
        function push(this,set)
            % Push a set of shapes into the set.
            assert(isa(set,class(this)));     
            if set.Count > 0
                this.Shape = [set.Shape this.Shape];
            end
        end
        
        function hide(this)
            % Set the visibility of shapes in this set to 'off'.
            for ii = 1:this.Count
                this.Shape{ii}.Visible = 'off';
            end
        end
        
        function deparent(this)
            % Deparent the shapes in this set from the Parent. This is used
            % to recycle the shapes.
            for ii = 1:this.Count
                this.Shape{ii}.Parent = [];
            end
        end
        
        function removeInvalid(this)
            % Remove deleted shapes from the shape set.
            if this.Count > 0
                valid = isvalid([this.Shape{:}]);
                this.Shape = this.Shape(valid);
            end
        end
    end
    
    methods(Static)
        
        function setCommonParameters(shape,params,idx)
            % Use to set common parameters during show.
            if isempty(shape.Parent)
                shape.Parent = params.Parent;
            end
            
            if isprop(shape,'FaceAlpha')
                shape.FaceAlpha = params.Opacity(idx);
            end
            
            shape.Color          = params.Color(idx,:);
            shape.Label          = params.Label(idx);
            shape.LineWidth      = params.LineWidth(idx);
            shape.EdgeColor      = params.LineColor(idx,:);
            shape.EdgeAlpha      = params.LineOpacity(idx);
            shape.LabelTextColor = params.LabelTextColor(idx,:);
            shape.LabelAlpha     = params.LabelOpacity(idx);
            shape.Font           = params.Font;
            shape.Visible        = 'on';
           
        end
        
        function p = defaultParameters()
            % Return the default parameters to set for all shapes.
            p = {'ContextMenu',[],...
                'Deletable',false,...
                'LabelVisible','on',...
                'InteractionsAllowed','none',...
                'HandleVisibility','off',...
                'Visible','off'
                };
        end
        
        function tag = uniqueTag()
            % Generate a unique tag for every allocated shape. This is
            % used for testing.
            tag = "shape_"+num2str(tic);
        end
    end
end
