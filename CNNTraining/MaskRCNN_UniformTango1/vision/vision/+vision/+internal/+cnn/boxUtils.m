classdef boxUtils
    % A collection of box conversion and scaling utilities.

%   Copyright 2018-2020 The MathWorks, Inc.
    
    methods(Static)
        %------------------------------------------------------------------
        function scaledBoxes = scaleX1X2Y1Y2(boxes, sx, sy)
            % Scale [x1 y1 x2 y2] box in the x-direction by sx and in the
            % y-direction by sy.
            %
            % Scale roi by sx and sy. Input roi is an M-by-4 matrix of M
            % ROIs. Each row defines an ROI using the [xmin ymin xmax ymax]
            % format in pixel coordinates. The scaled ROIs output in the
            % same format. The scaled ROIs are floored and output in pixel
            % coordinates.
            %
            % input  space is u,v output space is x,y sx,sy scale from
            % input to output
            
            % convert to spatial coordinates
            u1 = boxes(:,1) - 0.5;
            u2 = boxes(:,3) + 0.5;
            v1 = boxes(:,2) - 0.5;
            v2 = boxes(:,4) + 0.5;
            
            % scale
            x1 = u1 * sx + (1-sx)/2;
            x2 = u2 * sx + (1-sx)/2;
            y1 = v1 * sy + (1-sy)/2;
            y2 = v2 * sy + (1-sy)/2;
            
            % convert to pixel coordinates
            x1 = x1 + 0.5;
            x2 = x2 - 0.5;
            y1 = y1 + 0.5;
            y2 = y2 - 0.5;
            
            scaledBoxes = floor([x1 y1 x2 y2]);
        end
        
                %------------------------------------------------------------------
        function scaledBoxes = scaleX1X2Y1Y2Float(boxes, sx, sy)
            % Scale [x1 y1 x2 y2] box in the x-direction by sx and in the
            % y-direction by sy.
            %
            % Scale roi by sx and sy. Input roi is an M-by-4 matrix of M
            % ROIs. Each row defines an ROI using the [xmin ymin xmax ymax]
            % format in pixel coordinates. The scaled ROIs output in the
            % same format. The scaled ROIs are output in spatial coordinates.
            %
            % input  space is u,v output space is x,y sx,sy scale from
            % input to output
            
            % convert to spatial coordinates
            u1 = boxes(:,1) - 0.5;
            u2 = boxes(:,3) + 0.5;
            v1 = boxes(:,2) - 0.5;
            v2 = boxes(:,4) + 0.5;
            
            % scale
            x1 = u1 * sx + (1-sx)/2;
            x2 = u2 * sx + (1-sx)/2;
            y1 = v1 * sy + (1-sy)/2;
            y2 = v2 * sy + (1-sy)/2;
            
            % convert to pixel coordinates
            x1 = x1 + 0.5;
            x2 = x2 - 0.5;
            y1 = y1 + 0.5;
            y2 = y2 - 0.5;
            
            scaledBoxes = [x1 y1 x2 y2];
        end
        
        %------------------------------------------------------------------
        function boxes = xywhToX1Y1X2Y2(boxes)
            % Convert [x y w h] box to [x1 y1 x2 y2]. Input and output
            % boxes are in pixel coordinates. boxes is an M-by-4
            % matrix.
            boxes(:,3) = boxes(:,1) + boxes(:,3) - 1;
            boxes(:,4) = boxes(:,2) + boxes(:,4) - 1;
        end
        
        %------------------------------------------------------------------
        function boxes = x1y1x2y2ToXYWH(boxes)
            % Convert [x1 y1 x2 y2] boxes into [x y w h] format. Input and
            % output boxes are in pixel coordinates. boxes is an M-by-4
            % matrix.
            boxes(:,3) = boxes(:,3) - boxes(:,1) + 1;
            boxes(:,4) = boxes(:,4) - boxes(:,2) + 1;
        end
    end
end
