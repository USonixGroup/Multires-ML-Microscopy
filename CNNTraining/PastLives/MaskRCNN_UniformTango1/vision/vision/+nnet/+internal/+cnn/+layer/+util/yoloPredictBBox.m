function xyBbox = yoloPredictBBox(sigmaXY,expWH,anchors,gridSize,downSamplingFactor)
% yoloPredictBBox returns bounding box coordinates with respect to input image
% dimension.
%
% Input sigmaXY are predicted (x, y) bounding box values for each anchor 
% box per grid, with input dimension [numAnchors, coOrd, numGrids] where 
% numAnchors are number of anchors, coord are (x, y) values, numGrids are 
% number of grids.
%
% Input expWH are predicted width, height of bounding boxes of each anchor 
% box per grid, with input dimension [numAnchors, boxDim, numGrids] where 
% numAnchors are number of anchors, boxDim are (width, height) values, 
% numGrids are number of grids.
%
% Input anchors is M-by-2 matrix defined by user. Input anchors are defined
% in [height, width] format.
%
% Input gridSize is the dimension size of activations from 
% transform layer.
%
% Input downSamplingFactor is a 1-by-2 matrix, with first element
% downsampling in y direction and second element downsampling in x
% direction.
%
% Output xyBbox are bounding boxes in [numAnchors, pred, numGrids]
% format where numAnchors corresponds to size(anchors, 1), pred
% correspond to [x1,y1,x2,y2] of predictions with respect to image, numGrids 
% correspond to number of grids per image.

% Copyright 2018-2019 The MathWorks, Inc.

xyBbox = zeros(size(anchors,1),size(sigmaXY,2)+size(expWH,2),size(sigmaXY,3),'like',sigmaXY);
for rowIdx=0:gridSize(1,1)-1
    for colIdx=0:gridSize(1,2)-1
        ind = rowIdx*gridSize(1,2)+colIdx+1;
        for anchorIdx = 1 : size(anchors,1)
            
            % Compute center with respect to image
            cx = (sigmaXY(anchorIdx,1,ind)+colIdx)* downSamplingFactor(1,2);
            cy = (sigmaXY(anchorIdx,2,ind)+rowIdx)* downSamplingFactor(1,1);
            
            % Compute width, height with respect to image.
            bw = expWH(anchorIdx,1,ind)* anchors(anchorIdx,2) * downSamplingFactor(1,2);
            bh = expWH(anchorIdx,2,ind)* anchors(anchorIdx,1) * downSamplingFactor(1,1);
            
            xyBbox(anchorIdx,1,ind) = (cx-bw/2);
            xyBbox(anchorIdx,2,ind) = (cy-bh/2);
            xyBbox(anchorIdx,3,ind) = (cx+bw/2);
            xyBbox(anchorIdx,4,ind) = (cy+bh/2);
        end
    end
end
end
