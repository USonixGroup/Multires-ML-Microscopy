% ImageTransformerKB Object for applying an arbitrary transformation to an
% image using the Kannala-Brandt camera model
%
% transformer = ImageTransformerKB returns an image transformer object.
%
% ImageTransformerKB methods:
% -------------------------
% transformImage - Apply a transformation to an image.
% needToUpdate   - Returns true if the map needs to be updated.
% update         - Recompute the map.

% Copyright 2023 MathWorks, Inc.

classdef ImageTransformerKB < vision.internal.calibration.ImageTransformerBase
    
    methods(Access=public)        
        
        %------------------------------------------------------------------
        function this = ImageTransformerKB()
            
            this = this@vision.internal.calibration.ImageTransformerBase();
        end

        %------------------------------------------------------------------
        % API to recompute the map if the image has changed.
        %------------------------------------------------------------------
        function update(this, I, K, distortionParams, outputView, ...
                xBounds, yBounds, H)

            updateProperties(this, I, outputView, xBounds, yBounds);
            
            if nargin > 7
                computeMap(this, K, distortionParams, H);
            else
                computeMap(this, K, distortionParams);
            end            
        end
    end
    
    methods(Access=private)                                      
        %------------------------------------------------------------------
        % Compute the map for image transformation.
        %------------------------------------------------------------------
        function computeMap(this, K, distortionParams, varargin)

            % Sample map points based on X and Y bounds.
            ptsIn = sampleMapPoints(this, varargin{:});
            
            % Distort the sampled map points.
            intrinsicMatrix = cast(K, 'like', ptsIn);
            distortionParameters = cast(distortionParams, 'like', ptsIn);
            ptsOut = visionDistortPointsKB(ptsIn, intrinsicMatrix, ...
                distortionParameters);
            
            clear ptsIn; % be careful with memory
            
            % Generate the map.
            generateMap(this, ptsOut);
        end
    end
end

