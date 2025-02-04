classdef ShapeType < double
    % This class defines the shape types and conversion utilities used in
    % showShape.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020-2023 The MathWorks, Inc.
    enumeration
        Rectangle        (1)
        RotatedRectangle (2)
        Cuboid           (3)
        Polygon          (4)
        Line             (5)
        Circle           (6)
        ProjectedCuboid  (7)
        Ellipse          (8)
    end
    
    methods(Static)
        
        function type = fromShapeSet(shapeSet)
            % Return the shape type from the input ShapeSet.
            
            import vision.internal.shape.*
            switch class(shapeSet)
                case 'vision.internal.shape.RectangleSet'
                    type = ShapeType.Rectangle;
                case 'vision.internal.shape.RotatedRectangleSet'
                    type = ShapeType.RotatedRectangle;
                case 'vision.internal.shape.PolygonSet'
                    type = ShapeType.Polygon;
                case 'vision.internal.shape.CuboidSet'
                    type = ShapeType.Cuboid;
                case 'vision.internal.shape.LineSet'
                    type = ShapeType.Line;
                case 'vision.internal.shape.CircleSet'
                    type = ShapeType.Circle;
                case 'vision.internal.shape.ProjectedCubeSet'
                    type = ShapeType.ProjectedCuboid;
                case 'vision.internal.shape.EllipseSet'
                    type = ShapeType.Ellipse;
                    
            end
        end
        
        function type = fromName(name)
            % Return the shape type from the input name.
            import vision.internal.shape.*
            switch lower(name) 
                case "rectangle"
                    type = ShapeType.Rectangle;
                case "rotated-rectangle"
                    type = ShapeType.RotatedRectangle;
                case "cuboid"
                    type = ShapeType.Cuboid;
                case "polygon"
                    type = ShapeType.Polygon;
                case "line"
                    type = ShapeType.Line;
                case "circle"
                    type = ShapeType.Circle;
                case "projected-cuboid"
                    type = ShapeType.ProjectedCuboid;
                case "ellipse"
                    type = ShapeType.Ellipse;

            end
        end
    end
end
