classdef (Abstract, Hidden) cameraIntrinsicsBase < handle
    % cameraIntrinsicsBase Base class with common tools for image
    % undistortion for different extended pinhole camera models.
    
    % Copyright 2023 The MathWorks, Inc.
    %#codegen

    properties (Abstract, Access = protected, Hidden)
        UndistortMap
    end

    methods (Abstract, Hidden)
        updateUndistortMap(this, I, outputView, xBounds, yBounds)
        distortedPoints = distortPoints(this, points, ~)
        undistortedMask = computeUndistortedMask(this, xBoundsBig, yBoundsBig, ...
                imageSize, outputView);
    end

    methods(Hidden)
        %------------------------------------------------------------------
        function [Jout, newOrigin] = undistortImageImpl(this, I, interp, ...
                outputView, fillValues)
            % undistortImageImpl implements the core lens undistortion
            % algorithm for the undistortImage.m function.  See help for
            % undistortImage for further details.
            if needToUpdate(this.UndistortMap, I, outputView)
                [xBounds, yBounds] = computeUndistortBounds(this, ...
                    [size(I, 1), size(I, 2)], outputView);

                updateUndistortMap(this, I, outputView, xBounds, yBounds);
            end
            
            [J, newOrigin] = transformImage(this.UndistortMap, I, interp, fillValues); 
            
            if strcmp(outputView, 'same')
                Jout = coder.nullcopy(zeros(size(I), 'like', I));
                Jout(:,:,:) = J(1:size(I, 1), 1:size(I, 2), 1:size(I,3));
            else
                Jout = J;
            end
        end

        %------------------------------------------------------------------
        function undistortedPoints = undistortPointsImpl(this, points) 
            if isSimMode
                options = optimset('Display', 'off');
                undistortedPoints = ...
                    lscftsh(@this.distortPoints, points, [], points, [], [], options);
            else
                
                options = struct('SolverName', 'lsqcurvefit',...
                    'Algorithm', 'levenberg-marquardt',...
                    'Display', 'off',...
                    'FiniteDifferenceStepSize', -1,...
                    'FiniteDifferenceType', 'forward',...
                    'FunctionTolerance', 1e-6,...
                    'InitDamping', 1e-2,...
                    'MaxFunctionEvaluations', -1,...
                    'MaxIterations', 400,...
                    'NonFiniteSupport', true,...
                    'ScaleProblem','none',...
                    'SpecifyObjectiveGradient', false,...
                    'SpecifyConstraintGradient', false,...
                    'StepTolerance', 1e-6,...
                    'TypicalX', []);

                xdata = zeros(size(points));
                undistortedPoints = ...
                    lscftsh(@(varargin)this.distortPoints(varargin{:}), points, xdata, points, [], [], options);
            end
        end

        %------------------------------------------------------------------
        function [xBounds, yBounds] = ...
                computeUndistortBounds(this, imageSize, outputView)          
            if strcmp(outputView, 'same')
                xBounds = [1, imageSize(2)];
                yBounds = [1, imageSize(1)];
            else
                [undistortedMask, xBoundsBig, yBoundsBig] = ...
                    createUndistortedMask(this, imageSize, outputView);
                
                if strcmp(outputView, 'full')
                    [xBounds, yBounds] = vision.internal.calibration.getFullBounds(undistortedMask, ...
                        xBoundsBig, yBoundsBig);
                else % valid
                    [xBounds, yBounds] = getValidBounds(this, undistortedMask, ...
                        xBoundsBig, yBoundsBig, imageSize);
                end
            end
        end
    end
    
    methods(Access=private)
        %--------------------------------------------------------------------------
        function [undistortedMask, xBoundsBig, yBoundsBig] = ...
                createUndistortedMask(this, imageSize, outputView)
                
            % start guessing the undistorted mask with the same size of the
            % original image
            xBounds = [1 imageSize(2)];
            yBounds = [1 imageSize(1)];
            
            [X, Y] = meshgrid(xBounds(1):xBounds(2),yBounds(1):yBounds(2));
            ptsIn = [X(:) Y(:)];
            
            ptsOut = distortPoints(this, ptsIn);
            
            mask = zeros(imageSize, 'uint8');
            
            % each pixel in undistorted image contributes to four pixels in
            % the original image, due to bilinear interpolation
            allPts = [floor(ptsOut); ...
                      floor(ptsOut(:,1)),ceil(ptsOut(:,2)); ...
                      ceil(ptsOut(:,1)),floor(ptsOut(:,2)); ...
                      ceil(ptsOut)];
            insideImage = (allPts(:,1)>=1 & allPts(:,2)>=1 ...
                & allPts(:,1)<=imageSize(2) & allPts(:,2)<=imageSize(1));
            allPts = allPts(insideImage, :);
            indices = sub2ind(imageSize, allPts(:,2), allPts(:,1));
            mask(indices) = 1;            
            numUnmapped = prod(imageSize) - sum(mask(:));
            
            % Grow the output size until all pixels in the original image
            % have been used, or the attempt to grow the output size has
            % failed 5 times when new pixels do not contribute to the
            % mapping.
            if numUnmapped > 0 
                p1 = [xBounds(1), yBounds(1)];
                p2 = [xBounds(2), yBounds(2)];
                numTrials = 0;
                while (numTrials < 5 && numUnmapped > 0)

                    p1 = p1 - 1;
                    p2 = p2 + 1;                    
                    w = p2(1) - p1(1) + 1;
                    h = p2(2) - p1(2) + 1;
                    lastNumUnmapped = numUnmapped;

                    ptsIn = [(p1(1):p1(1)+w-1)', p1(2)*ones(w, 1);...
                             (p1(1):p1(1)+w-1)', p2(2)*ones(w, 1);...
                              p1(1)*ones(h, 1),(p1(2):p1(2)+h-1)';...
                              p2(1)*ones(h, 1),(p1(2):p1(2)+h-1)'];
            
                    ptsOut = distortPoints(this, ptsIn);
                    
                    newPts = [floor(ptsOut); ...
                              floor(ptsOut(:,1)),ceil(ptsOut(:,2)); ...
                              ceil(ptsOut(:,1)),floor(ptsOut(:,2)); ...
                              ceil(ptsOut)];
                    insideImage = (newPts(:,1)>=1 & newPts(:,2)>=1 ...
                        & newPts(:,1)<=imageSize(2) & newPts(:,2)<=imageSize(1));
                    newPts = newPts(insideImage, :);
                    indices = sub2ind(imageSize, newPts(:,2), newPts(:,1));
                    mask(indices) = 1;
                    numUnmapped = prod(imageSize) - sum(mask(:));
            
                    if lastNumUnmapped == numUnmapped
                        numTrials = numTrials + 1;
                    else
                        numTrials = 0;
                    end
                                        
                    xBounds = [p1(1), p2(1)];
                    yBounds = [p1(2), p2(2)];
                end
            end
            
            % Compute the mapping with the new output size
            xBoundsBig = xBounds;
            yBoundsBig = yBounds;
            undistortedMask = computeUndistortedMask(this, xBoundsBig, yBoundsBig, ...
                imageSize, outputView);
        end
        
        %--------------------------------------------------------------------------
        function [xBounds, yBounds] = getValidBounds(this, undistortedMask, ...
                xBoundsBig, yBoundsBig, imageSize)
            
            % Get the boundary
            boundaryPixel = vision.internal.calibration.getInitialBoundaryPixel(...
                undistortedMask);
            boundaryPixelsUndistorted = bwtraceboundary(undistortedMask, ...
                boundaryPixel, 'W');
            
            % Convert from R-C to x-y
            boundaryPixelsUndistorted = boundaryPixelsUndistorted(:, [2,1]);
            
            % Convert to the coordinate system of the original image
            boundaryPixelsUndistorted(:, 1) = boundaryPixelsUndistorted(:, 1) + xBoundsBig(1);
            boundaryPixelsUndistorted(:, 2) = boundaryPixelsUndistorted(:, 2) + yBoundsBig(1);
            
            % Apply distortion to turn the boundary back into a rectangle
            boundaryPixelsDistorted = distortPoints(this, boundaryPixelsUndistorted);
            
            % Find the pixels that came from the top, bottom, left, and right edges of
            % the original image.
            tolerance = 7;
            minX = max(1, min(boundaryPixelsDistorted(:, 1)));
            maxX = min(imageSize(2), max(boundaryPixelsDistorted(:, 1)));
            minY = max(1, min(boundaryPixelsDistorted(:, 2)));
            maxY = min(imageSize(1), max(boundaryPixelsDistorted(:, 2)));
            topIdx = abs(boundaryPixelsDistorted(:, 2) - minY) < tolerance;
            botIdx = abs(boundaryPixelsDistorted(:, 2) - maxY) < tolerance;
            leftIdx = abs(boundaryPixelsDistorted(:, 1) - minX) < tolerance;
            rightIdx = abs(boundaryPixelsDistorted(:, 1) - maxX) < tolerance;
                        
            % Find the inscribed rectangle.
            topPixels = boundaryPixelsUndistorted(topIdx, 2);
            botPixels = boundaryPixelsUndistorted(botIdx, 2);
            leftPixels = boundaryPixelsUndistorted(leftIdx, 1);
            rightPixels = boundaryPixelsUndistorted(rightIdx, 1);

            % Check if we can compute the valid bounds at all
            coder.internal.errorIf(isempty(topPixels) || isempty(botPixels) || ...
                isempty(leftPixels) || isempty(rightPixels), ...
                'vision:calibrate:cannotComputeValidBounds');
            
            top = max(topPixels);
            bot = min(botPixels);
            left = max(leftPixels);
            right = min(rightPixels);
            
            % Check if the valid bounds cross
            if isSimMode && (left > right || top > bot ...
                    || minX > tolerance || maxX < imageSize(2)-tolerance ...
                    || minY > tolerance || maxY < imageSize(1)-tolerance)
                warning(message('vision:calibrate:badValidUndistortBounds'));
            end
            
            xBounds = sort([ceil(left), floor(right)]);
            yBounds = sort([ceil(top), floor(bot)]);
        end
    end

end

%--------------------------------------------------------------------------
function mode = isSimMode()

    mode = isempty(coder.target);
end