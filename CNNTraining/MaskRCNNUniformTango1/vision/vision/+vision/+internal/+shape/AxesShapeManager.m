classdef AxesShapeManager < handle
    % This class is used to manage pools associated with an axes.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020 The MathWorks, Inc.
    
    properties
        % Shapes A cell array of ShapeSet currently being used by the axes.
        % Each shape set contains one type of shape.
        Shapes
        
        % AxesBeingDestroyedListener A cell array of listeners.
        AxesBeingDestroyedListener
        
        % Pool A cell array for each ShapePools associated with each axes.
        Pool 
    
        % ShapeTypeInPool An array of ShapeType indicating which shape is
        % help in each Pool.
        ShapeTypeInPool
    end
    
    methods
              
        function this = AxesShapeManager(axesToManage,shapePool)
            this.Pool = shapePool;
            
            import vision.internal.shape.*
            
            % Fill shapes in use based on ShapeSet in the pool.
            for i = 1:numel(shapePool)
                this.Shapes{i} = shapePool{i}.Shapes.makeSet();
                this.ShapeTypeInPool(i) = shapeType(this.Pool{i});
            end
            
            % Connect manager to axes.
            connectToAxes(this,axesToManage);
        end
        
        function connectToAxes(this,ax)
            % Attach listeners to both figure and axes to handle. We must
            % attach listeners to both the axes and parent figure to ensure
            % proper update of the scene (g2171094).
            this.AxesBeingDestroyedListener{1} = listener(...
                ancestor(ax,'figure'),...
                'ObjectBeingDestroyed',@(h,e) delete(ax) );
            
            this.AxesBeingDestroyedListener{2} = listener(...
                ax,...
                'ObjectBeingDestroyed',@(h,e)this.returnShapes(true,h,e));
            
            % Listen for CLA events as these also delete child objects
            % within HG axes (g2227300) 
            this.AxesBeingDestroyedListener{3} = listener(...
                ax,...
                'ClaReset',@(h,e)this.returnShapes(true,h,e));
            
        end
        
        function shapes = getShapes(this,shapeType,numShapes)
            assert(isa(shapeType,'vision.internal.shape.ShapeType'))
            idx = shapeType == this.ShapeTypeInPool;
            shapes = borrowFromPool(this.Pool{idx},numShapes);
            push(this.Shapes{idx},shapes); 
        end
        
        function returnShapes(this,deparent,varargin)
            for ii = 1:numel(this.Pool)
                % return objects currently used by the axes.
                set = pop(this.Shapes{ii});
                returnToPool(this.Pool{ii},set,deparent)
                
                if deparent
                    % return objects that are in the pool. In this case, we
                    % should return the shapes back to the global pool.
                    n = this.Pool{ii}.Shapes.Count;
                    set = pop(this.Pool{ii}.Shapes, n);
                    returnToPool(this.Pool{ii},set,deparent);
                end
            end
        end
        
        function clearShapes(this)
            deparent = false;
            returnShapes(this,deparent);
        end
        
    end
    
    methods (Static)
        function mngr = axesShapeManagerWithGlobalPool(axesToManage,sets,globalPool)
            import vision.internal.shape.*
            
            pools = cell(1,numel(sets));
            for i = 1:numel(pools)
                pools{i} = AxesShapePool(sets{i},globalPool{i});
            end
            
            mngr = AxesShapeManager(axesToManage,pools);
        end
        
    end
end