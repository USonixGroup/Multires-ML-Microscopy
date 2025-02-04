classdef ShapePool < handle
    % This class represents a pool of shapes and exposes an API to fill,
    % borrow, and return items to/from the pool.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020 The MathWorks, Inc.
    
    properties
        % Shapes A ShapeSet object.
        Shapes
    end
    
    methods
        function this = ShapePool(set)
            % Create the pool given a ShapeSet.
            this.Shapes = set;
        end
        
        function fillPool(this,n)
            % Fill the pool with items of type this.Shapes
            this.Shapes.push(this.Shapes.allocate(n));
        end
        
        function shapes = borrowFromPool(this,n)
            % Borrow n shapes from the pool.
            
            % Sanitize the pool before borrowing shapes in case some shapes
            % become invalid (e.g. close all force).
            removeInvalid(this.Shapes);
            
            % Fill the pool if there are not enough items.
            if this.Shapes.Count < n
                this.fillPool(n - this.Shapes.Count);
            end
            
            shapes = pop(this.Shapes,n);
        end
        
        function returnToPool(this,shapes,shouldDeparent)
            % Return shapes to the pool and optionally deparent them.
            %
            % Upon return to the pool, the shapes are hidden from view and
            % deparented when their parent are deleted.
            
            % Sanitize shapes returned to the pool.
            removeInvalid(shapes);
            
            hide(shapes);
            if shouldDeparent
                deparent(shapes);
            end
            push(this.Shapes,shapes);
        end
        
        function type = shapeType(this)
            % Return the ShapeType held in this pool.
            type = vision.internal.shape.ShapeType.fromShapeSet(this.Shapes);
        end
    end
    
    methods (Static)
        
        function pools = createShapePools(sets)
            % Return a cell array of pools for each type of ShapeSet.
            import vision.internal.shape.*
            
            pools = cell(1,numel(sets));
            for i = 1:numel(pools)
                pools{i} = ShapePool(sets{i});
            end
            
        end
        
    end
end
