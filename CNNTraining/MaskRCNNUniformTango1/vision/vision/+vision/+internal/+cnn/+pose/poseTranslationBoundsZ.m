function [lowerBoundZ,upperBoundZ] = poseTranslationBoundsZ(boxes,xyzImg,centroids2D, K)
%POSETRANSLATIONBOUNDSZ Calculate the z-coordinate bounds within which the 
%3D centroid of an object will lie.

% Copyright 2023 The MathWorks, Inc.
    
    numPreds = size(boxes,1);
    lowerBoundZ = zeros(numPreds, 1);
    upperBoundZ = zeros(numPreds, 1);
    boxes = round(boxes);
    centroids2D = round(centroids2D);

    Kinv = pinv(K);

    % The "XYZ image" contains the 3-D world-coordinates for every pixel
    % position in the input image.
    for i = 1:numPreds

        depthPatch = xyzImg(...
            boxes(i,2):boxes(i,2)+boxes(i,4)-1,...
            boxes(i,1):boxes(i,1)+boxes(i,3)-1,...
            3);

        % The LOWER BOUND on Z is estimated as the max(depth map patch) 
        Zmin = min(depthPatch(:));
        % The depth value at the 2D centroid of the object is the Z value 
        % for a point on the surface of the object. This serves as a
        % clipping threshold in case of incorrect raw Zmin measurement
        % (e.g. if another closer object partialy enters the roi box)
        try
            ZSurface = xyzImg(centroids2D(i,2), centroids2D(i,1), 3);
        catch ME
            disp(ME)
        end

        % The UPPER BOUND on Z is first estimated as the max(depth map patch)
        % where we consider crop out a patch in the depth image using the
        % bbox coordinates. When the object is against a uniform surface 
        % like a table-top or a bin then this will give the depth or Z
        % distance of the surface from the camera, and provide a reasonable
        % upper bound on Z.
        Zmax = max(depthPatch(:));

        % We place a max threshold to handle cases when the max depth may 
        % be incorrectly estimated due to part of the depth image patch
        % including surfaces at a greater distance than the resting surface
        % of the object, e.g. the edge of a table or bin is included within  
        % the bounding box region.
        % Convert box corners from image to world-coordinates, assuming
        % they lie on a plane on the surface of the object (Zmin).
        uvzCorners = cat(2,  ZSurface * [boxes(i,1); boxes(i,1)+boxes(i,3)-1 ],...
            ZSurface * [boxes(i,2); boxes(i,1)+boxes(i,4)-1 ],...
            [ZSurface; ZSurface]);
        xyzCorners = (Kinv * uvzCorners')';
        ZmaxDist = sqrt( sum((xyzCorners(1,:) - xyzCorners(2,:)).^2));
        
        % Clip Zmax to not be larger than the diagonal of the bounding box
        Zmax = min(Zmax, ZSurface + ZmaxDist);

        % Clip Zmin to not be smaller (nearer) than 0.5*diagonal from
        % the Z at the surface of the object
        Zmin = max(Zmin, ZSurface - 0.5*ZmaxDist);

        lowerBoundZ(i) = Zmin;
        upperBoundZ(i) = Zmax;
    end
end

