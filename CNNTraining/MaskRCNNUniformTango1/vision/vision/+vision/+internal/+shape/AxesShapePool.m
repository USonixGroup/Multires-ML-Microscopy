classdef AxesShapePool < vision.internal.shape.ShapePool
    % This class represents a ShapePool used by individual axes.
    %
    % The AxesShapePool uses another pool (typically a global pool) that
    % enables an axes to have quick access to shapes already parented to
    % the axes and avoid deparenting costs. The AxesShapePool fills its
    % pool using the global pool and returns items to the global pool if
    % the axes associated to the pool is deleted. Otherwise the shape is
    % returned to the local pool.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020 The MathWorks, Inc.
    
    properties
        % Shapes are borrowed from the global pool when needed. Shapes are
        % returned to the this pool when not deparenting and returned to
        % the global pool otherwise.
        GlobalPool
    end
    
    methods
        function this = AxesShapePool(shapeFcn,globalPool)
            % Create an AxesShapePool given a shape creation function and a
            % global pool. globalPool is a cell array of ShapePool.
            this@vision.internal.shape.ShapePool(shapeFcn);
            this.GlobalPool = globalPool;
        end
        
        function fillPool(this,n)
            % Fill the pool using the global pool.
            this.Shapes.push(this.GlobalPool.borrowFromPool(n));
        end
        
        function returnToPool(this,shapes,deparent)
            % Return shapes to the global pool if deparenting. Otherwise,
            % return it to the local pool.
            if deparent
                this.GlobalPool.returnToPool(shapes,deparent);
            else
                returnToPool@vision.internal.shape.ShapePool(this,shapes,deparent);
            end
        end
    end
end
