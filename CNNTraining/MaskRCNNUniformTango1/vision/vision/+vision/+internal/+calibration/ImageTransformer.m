% ImageTransformer Object for applying an arbitrary transformation to an image 
% using the extended pinhole camera model.
%
% transformer = ImageTransformer returns an image transformer object.
%
% ImageTransformer methods:
% -------------------------
% transformImage - Apply a transformation to an image.
% needToUpdate   - Returns true if the map needs to be updated.
% update         - Recompute the map.

% Copyright 2012-2023 MathWorks, Inc.

%#codegen

classdef ImageTransformer < vision.internal.calibration.ImageTransformerBase
    
    methods(Access=public)        
        
        %------------------------------------------------------------------
        function this = ImageTransformer()
            
            this = this@vision.internal.calibration.ImageTransformerBase();
        end
        
        %------------------------------------------------------------------
        % API to recompute the map if the image has changed.
        %------------------------------------------------------------------
        function update(this, I, K, radialDist, tangentialDist, outputView, ...
                xBounds, yBounds, H)

            updateProperties(this, I, outputView, xBounds, yBounds);
            
            if nargin > 8
                computeMap(this, K, radialDist, tangentialDist, H);
            else
                computeMap(this, K, radialDist, tangentialDist);
            end            
        end
    end
    
    methods(Access=private)                                      
        %------------------------------------------------------------------
        % Compute the map for image transformation.
        %------------------------------------------------------------------
        function computeMap(this, K, radialDist, tangentialDist, varargin)
            
            % Sample map points based on X and Y bounds.
            ptsIn = sampleMapPoints(this, varargin{:});
            
            % Distort the sampled map points.
            if isempty(coder.target)

                intrinsicMatrix = cast(K, 'like', ptsIn);
                radialDistortion = cast(radialDist, 'like', ptsIn);
                tangentialDistortion = cast(tangentialDist, 'like', ptsIn);
                ptsOut = visionDistortPoints(ptsIn, intrinsicMatrix, ...
                    radialDistortion, tangentialDistortion);
            else
                ptsOut = vision.internal.calibration.distortPoints(ptsIn, ...
                    K, radialDist, tangentialDist);                
            end
            
            if coder.target('MATLAB')
                clear ptsIn; % be careful with memory
            end
            
            % Generate the map.
            generateMap(this, ptsOut);
        end
    end
end

