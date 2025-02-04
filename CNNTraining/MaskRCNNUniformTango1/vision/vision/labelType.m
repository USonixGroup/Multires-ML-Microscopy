classdef labelType < uint8
    %labelType Enumeration of supported label types
    %   labelType creates an enumeration specifying the type of label that
    %   can be used to define labels in the groundTruth or
    %   groundTruthMultiSignal object.
    %
    %   labelType enumerations:
    %   Rectangle        - Label marked as a rectangular region of interest (ROI)
    %   RotatedRectangle - Label marked as a rotated rectangular region of interest (ROI)
    %   Line             - Label marked as a polyline or polyline3D region of interest (ROI)
    %   Scene            - Label marked on a frame or interval of frames
    %   Custom           - Custom label type
    %   PixelLabel       - Label marked as a pixel labeled region of interest(ROI)
    %   Cuboid           - Label marked as a cuboidal region of interest (ROI)
    %   ProjectedCuboid  - Label marked as a projected cuboidal region of interest (ROI)
    %   Polygon          - Label marked as a polygon region of interest (ROI)
    %   Point            - Label marked as a point region of interest (ROI)
    %
    %   labelType methods:
    %   isROI            - Determine if label type is an ROI label
    %   isScene          - Determine if label type is a Scene label
    %   isCustom         - Determine if label type is a Custom label
    %   labelDef         - Grab a dialog friendly string for label types
    %
    %
    %   Example: Create label definitions table
    %   ---------------------------------------
    %   % Define label names
    %   Name = {'Car'; 'LeftLaneMarker'; 'RightLaneMarker'; 'Sunny'};
    %
    %   % Define label types
    %   Type = labelType({'Rectangle'; 'Line'; 'Line'; 'Scene'});
    %
    %   % Create label definitions
    %   labelDefs = table(Name, Type)
    %
    %   % List ROI label names
    %   roiLabelIndices = isROI(labelDefs.Type);
    %   roiLabelNames = labelDefs.Name(roiLabelIndices)
    %
    %   % List Scene label names
    %   sceneLabelIndices = isScene(labelDefs.Type);
    %   sceneLabelNames = labelDefs.Name(sceneLabelIndices)
    %
    %
    % See also groundTruth.
    
    % Copyright 2016-2023 The MathWorks, Inc.
    
    enumeration
        %Rectangle Rectangular region of interest label type
        %   Rectangle specifies a label marked as a rectangular region of
        %   interest (ROI).
        Rectangle   (0)

        %   Rotated Rectangle region of interest label type
        %   Rotated Rectangle specifies a label marked as a rectangular 
        %   region of interest (ROI) which can be rotated at any specific 
        %   angle.
        RotatedRectangle(9)
        
        %Line Polyline region of interest label type
        %   Line specifies a label marked as a polyline region of interest
        %   (ROI). A polyline is a continuous line composed of one or more
        %   line segments.
        Line        (1)
        
        %PixelLabel Pixel labeled region of interest
        %   PixelLabel specifies a label marked as a pixel labeled region
        %   of interest (ROI). Pixel labeled ROI provide labels for every
        %   pixel within the ROI and is typically used to label a group of
        %   neighboring pixels that share the same label category.
        PixelLabel  (4)
        
        %Cuboid Cuboidal region of interest label type
        %   Cuboid specifies a label marked as a cuboidal region of
        %   interest (ROI).
        Cuboid      (5)        
        
        %ProjectedCuboid Projected Cuboidal region of interest label type
        %   ProjectedCuboid specifies a label marked as a projected
        %   cuboidal region of interest (ROI).
        ProjectedCuboid      (6)
        
        %Polygon Polygon region of interest label type
        %   Polygon specifies a label marked as a polygon
        %   region of interest (ROI).
        Polygon      (7)

        %Point Point region of interest label type
        %   Point specifies a label marked as a point
        %   region of interest (ROI).
        Point      (8)

        %Scene Scene label type
        %   Scene specifies a label marked on a frame or interval of
        %   frames.
        Scene       (2)
        
        %Custom Custom label type
        %   Custom specifies a custom label type.
        Custom      (3)

    end
    
    methods
        %------------------------------------------------------------------
        function TF = isROI(this)
            %isROI Determine if label type is an ROI label type.
            %   tf = isROI(labelTypes) returns true if labelTypes is an ROI
            %   label type and false otherwise. An ROI label type is either
            %   Rectangle, RotatedRectangle, Line, Cuboid, ProjectedCuboid or PixelLabel.
            
            TF = (this==labelType.Rectangle) ...
                | (this==labelType.RotatedRectangle) ...
                | (this==labelType.Line) ...
                | (this == labelType.PixelLabel) ...
                | (this==labelType.Cuboid) ...
                | (this==labelType.ProjectedCuboid)...
                | (this==labelType.Polygon)...
                | (this==labelType.Point); 
                
        end
        
        %------------------------------------------------------------------
        function TF = isScene(this)
            %isScene Determine if label type is a Scene label type.
            %   tf = isScene(labelTypes) returns true if labelTypes is a
            %   Scene label type and false otherwise.
            
            TF = this==labelType.Scene;
        end
        
        %------------------------------------------------------------------
        function TF = isCustom(this)
            %isCustom Determine if label type is a Custom label type.
            %   tf = isCustom(labelTypes) returns true if labelTypes is a
            %   Custom label type and false otherwise.
            
            TF = this==labelType.Custom;
        end

        %------------------------------------------------------------------
        function label = labelDef(this)
            %labelDef Obtain dialog string for each label type
            %   label = labelDef(labelTypes) returns a dialog friendly 
            %   string for each label type.
            
            switch this
                case labelType.Rectangle
                    label = vision.getMessage('vision:labeler:Rectangle');
                case labelType.RotatedRectangle
                    label = vision.getMessage('vision:labeler:RotatedRectangle');
                case labelType.Line
                    label = vision.getMessage('vision:labeler:Line');
                case labelType.PixelLabel
                    label = vision.getMessage('vision:labeler:Pixel');
                case labelType.Polygon
                    label = vision.getMessage('vision:labeler:Polygon');
                case labelType.ProjectedCuboid
                    label = vision.getMessage('vision:labeler:ProjectedCuboid');
                case labelType.Point
                    label = vision.getMessage('vision:labeler:Point');
                case labelType.Cuboid
                    label = vision.getMessage('vision:labeler:Cuboid');
            end
        end
        
    end

end
