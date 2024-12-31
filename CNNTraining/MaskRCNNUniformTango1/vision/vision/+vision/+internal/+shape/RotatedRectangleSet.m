classdef RotatedRectangleSet < vision.internal.shape.ShapeSet
    % This class is used to show a set of rotated rectangles on a figure.
    %
    % This function is for internal use only and is likely to change in
    % future releases.
    
    % Copyright 2020 The MathWorks, Inc.
    
    methods
        function this = RotatedRectangleSet()
        end
        
        function show(this,bbox,params)
            % Show a set of rotated rectangles. bbox is an M-by-5 matrix of
            % rotated rectangles specified as [xc yc w h yaw]. yaw is a
            % rotation angle in degrees. Rotation is positive in the
            % clockwise direction looking down the axis of rotation in the
            % positive direction. [xc yc w h] are spatial coordinates.
            
            % Negate the rotation angle as images.roi.Rectangle uses the
            % opposite rotation convention.
            theta = -bbox(:,end);
            
            for ii = 1:size(bbox,1)
                this.Shape{ii}.CenteredPosition = bbox(ii,1:4);
                this.Shape{ii}.RotationAngle = theta(ii,end);
                if isfield(params,'ShowOrientation') && params.ShowOrientation == false
                    this.Shape{ii}.Rotatable = false;
                    this.Shape{ii}.RotationStemVisible = false;
                else
                    this.Shape{ii}.Rotatable = true;
                    this.Shape{ii}.RotationStemVisible = true;
                end
                vision.internal.shape.ShapeSet.setCommonParameters(this.Shape{ii},params,ii);
            end
        end
        
    end
    
    methods(Static)
        
        function set = makeSet()
            set = vision.internal.shape.RotatedRectangleSet();
        end
        
        function set = allocate(n)
            obj = cell(1,n);
            defaultParameters = vision.internal.shape.ShapeSet.defaultParameters();
            for ii = 1:n
                obj{ii} = images.roi.Rectangle(...
                    'Rotatable',false,...
                    'FaceSelectable',false,...
                    'Tag',vision.internal.shape.ShapeSet.uniqueTag(),...
                    defaultParameters{:});
            end
            set = vision.internal.shape.RotatedRectangleSet();
            set.Shape = obj;
        end
    end
    
end