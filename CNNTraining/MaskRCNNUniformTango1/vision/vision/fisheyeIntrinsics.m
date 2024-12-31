
% Copyright 2017-2023 MathWorks, Inc.

%#codegen

classdef fisheyeIntrinsics < vision.internal.EnforceScalarValue
    
    properties (SetAccess='private', GetAccess='public')
        
        %MappingCoefficients Polynomial coefficients in Scaramuzza's Taylor model.
        %   MappingCoefficients is a 4-element vector [a0 a2 a3 a4],
        %   which are coefficients of a polynomial function, f, that maps
        %   the a world point [X Y Z] to a point [u v] in the sensor plane
        %   by the following equation: s * [u v f(w)] = [X Y Z], where
        %      - w = sqrt(u^2+v^2), is the radial euclidean distance from
        %      the camera center.
        %      - f(w) = a0 + a2 * w^2 + a3 * w^3 + a4 * w^4
        %      - s is a scalar
        MappingCoefficients;
        
        %ImageSize Image size produced by the camera.
        %   ImageSize is a vector [mrows, ncols] corresponding to the image
        %   size produced by the camera.
        ImageSize;
        
        %DistortionCenter Center of distortion.
        %   DistortionCenter is a 2-element vector [cx, cy] that specifies
        %   the center of distortion in pixels.
        DistortionCenter;

        %StretchMatrix A 2-by-2 transformation matrix.
        %   StretchMatrix transforms a point from the sensor plane to a
        %   pixel in the camera image plane. The misalignment is caused by
        %   lens not being parallel to sensor and the digitization process.
        %
        StretchMatrix;
        
    end
    
    properties (Access=protected, Hidden)
        UndistortMap;
        MappingCoeffsInternal;
        Version = ver('vision');
    end
    
    
    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = fisheyeIntrinsics(varargin)
            if isempty(coder.target)
                [mappingCoefficients, imageSize, distortionCenter, stretchMatrix] = fisheyeIntrinsics.parseInputsSim(varargin{:});
            else
                [mappingCoefficients, imageSize, distortionCenter, stretchMatrix] = fisheyeIntrinsics.parseInputsCodegen(varargin{:});
            end
            
            % Use double if any of the key properties is double
            if (isa(mappingCoefficients, 'double') || ...
                    isa(stretchMatrix, 'double') || ...
                    isa(distortionCenter, 'double'))
                this.MappingCoefficients = double(mappingCoefficients(:)');
                this.StretchMatrix = double(stretchMatrix);
                this.DistortionCenter = double(distortionCenter(:)');
            else
                this.MappingCoefficients = mappingCoefficients(:)';
                this.StretchMatrix = stretchMatrix;
                this.DistortionCenter = distortionCenter(:)';
            end
            this.ImageSize = imageSize;
            
            % Initialize the undistort map property. It must be done in the
            % constructor in order to successfully cache the undistortion
            % map inside a fisheyeIntrinsics object.
            this.UndistortMap = vision.internal.calibration.FisheyeImageTransformer;
            
            % Add the constant zero a1, so the full set is [a0 a1 a2 a3 a4]
            this.MappingCoeffsInternal = [this.MappingCoefficients(1), ...
                zeros(1, 'like', this.MappingCoefficients(1)), ...
                this.MappingCoefficients(2:4)];
        end
    end
    
    methods (Hidden = true)
        %------------------------------------------------------------------
        % Determine the normalized 3D vector on the unit sphere
        %------------------------------------------------------------------
        function worldPoints = imageToNormalizedVector(this, imagePoints)
            %imageToNormalizedVector Determine the normalized 3D vector on
            %the unit sphere.
            %  worldPoints = imageToNormalizedWorld(intrinsics,imagePoints)
            %  maps image points to the normalized 3D vector emanating from
            %  the single effective viewpoint on the unit sphere.
            %
            %  Inputs:
            %  -------
            %  intrinsics        - fisheyeIntrinsics object.
            %
            %  imagePoints       - M-by-2 matrix containing [x, y]
            %                      coordinates of image points. M is the
            %                      number of points.
            %
            %  Output:
            %  -------
            %  worldPoints       - M-by-3 matrix containing corresponding
            %                      [X,Y,Z] coordinates on the unit sphere.
            
            points = vision.internal.inputValidation.checkAndConvertPoints(...
                imagePoints, 'fisheyeIntrinsics', 'imagePoints');
            
            if isa(points, 'single')
                points = double(points);
            end
            
            center = double(this.DistortionCenter);
            stretch = double(this.StretchMatrix);
            coeffs = double(this.MappingCoeffsInternal);
            
            % Convert image points to sensor coordinates
            points(:, 1) = points(:, 1) - center(1);
            points(:, 2) = points(:, 2) - center(2);
            points = stretch \ points';
            
            rho = sqrt(points(1, :).^2 + points(2, :).^2);
            f = polyval(coeffs(end:-1:1), rho);
            
            % Note, points could be invalid if f < 0
            worldPoints = [points; f]';
            
            nw = sqrt(sum(worldPoints.^2, 2));
            nw(nw == 0) = eps;
            
            worldPoints = worldPoints ./ horzcat(nw,nw, nw);
            worldPoints = cast(worldPoints, class(imagePoints));
        end
        
        %------------------------------------------------------------------
        % Remove distortion and project to a perspective camera
        %------------------------------------------------------------------
        function undistortedPoints = undistortPointsImpl(this, points, camIntrinsics)
            worldPoints = imageToNormalizedVector(this, points);
            IND = find(worldPoints(:, 3) < 0);
            u = worldPoints(:, 1) ./ worldPoints(:, 3);
            v = worldPoints(:, 2) ./ worldPoints(:, 3);
            X = [u, v, ones(length(u), 1)] * camIntrinsics.IntrinsicMatrix;
            undistortedPoints = X(:, 1:2);
            if ~isempty(IND)
                undistortedPoints(IND, :) = NaN;
                coder.internal.warning('vision:calibrate:failToUndistortPoints');
            end
        end
        
        %------------------------------------------------------------------
        % Apply distortion to a set of points and a given perspective
        % camera
        %------------------------------------------------------------------
        function distortedPoints = distortPoints(this, points, camIntrinsics)
            u = (points(:, 1) - camIntrinsics.PrincipalPoint(1)) / camIntrinsics.FocalLength(1);
            v = (points(:, 2) - camIntrinsics.PrincipalPoint(2)) / camIntrinsics.FocalLength(2);
            points3D = [u, v, ones(numel(u), 1)];
            distortedPoints = vision.internal.calibration.computeImageProjection(...
                points3D, this.MappingCoeffsInternal, ...
                this.StretchMatrix, this.DistortionCenter);
        end
        
        %------------------------------------------------------------------
        function [Jout, camIntrinsics] = undistortImageImpl(this, I, interp, ...
                outputView, focalLength, fillValues, method)
            % undistortImageImpl implements the core lens undistortion
            % algorithm for the undistortFisheyeImage.m function.
            if needToUpdate(this.UndistortMap, I, outputView, focalLength, method)
                [xBounds, yBounds, principalPoint] = computeUndistortBounds(this, ...
                    [size(I, 1), size(I, 2)], outputView, focalLength);
                
                this.UndistortMap.update(I, ...
                    this.MappingCoeffsInternal, this.StretchMatrix, ...
                    this.DistortionCenter, outputView, ...
                    xBounds, yBounds, focalLength, principalPoint, method);
            end
            
            J = transformImage(this.UndistortMap, I, interp, fillValues);
            camIntrinsics = this.UndistortMap.Intrinsics;
            
            if strcmp(outputView, 'same')
                Jout = coder.nullcopy(zeros(size(I), 'like', I));
                Jout(:,:,:) = J(1:size(I, 1), 1:size(I, 2), 1:size(I,3));
            else
                Jout = J;
            end
        end
        
        %------------------------------------------------------------------
        function [xBounds, yBounds, principalPoint] = computeUndistortBounds(this, ...
                imageSize, outputView, focalLength)
            if strcmp(outputView, 'same')
                xBounds = [1, imageSize(2)];
                yBounds = [1, imageSize(1)];
                % +0.5 to land in the middle of the image using our
                % coordinate system. It also matches default behavior of
                % undistortFisheyePoints.
                principalPoint = imageSize([2, 1]) / 2 + 0.5;
            else
                top = [(1:imageSize(2))', ones(imageSize(2), 1)];
                bottom = [(1:imageSize(2))', imageSize(1)*ones(imageSize(2), 1)];
                left = [ones(imageSize(1), 1), (1:imageSize(1))'];
                right = [imageSize(2)*ones(imageSize(1), 1), (1:imageSize(1))'];
                points = [top;bottom;left;right];
                
                worldPoints = imageToNormalizedVector(this, points);
                coder.internal.errorIf(any(worldPoints(:, 3) < 0), 'vision:calibrate:failToUndistortFullImage');
                
                u = worldPoints(:, 1) ./ worldPoints(:, 3);
                v = worldPoints(:, 2) ./ worldPoints(:, 3);
                x = u * focalLength(1);
                y = v * focalLength(2);
                
                if strcmpi(outputView, 'full')
                    xmin = min(x);
                    ymin = min(y);
                    xmax = max(x);
                    ymax = max(y);
                else
                    topInd = 1:imageSize(2);
                    bottomInd = imageSize(2)+1:2*imageSize(2);
                    leftInd = 2*imageSize(2)+1:2*imageSize(2)+imageSize(1);
                    rightInd = 2*imageSize(2)+imageSize(1)+1:2*imageSize(2)+2*imageSize(1);
                    topy = y(topInd);
                    bottomy = y(bottomInd);
                    leftx = x(leftInd);
                    rightx = x(rightInd);
                    
                    xmin = max(leftx);
                    ymin = max(topy);
                    xmax = min(rightx);
                    ymax = min(bottomy);
                end
                
                newWidth = ceil(xmax - xmin);
                newHeight = ceil(ymax - ymin);
                
                % The undistorted image is too large to produce
                coder.internal.errorIf(newWidth >= imageSize(1)*5 || newHeight >= imageSize(2)*5, 'vision:calibrate:failToUndistortFullImage');
                % The undistorted image is not correct
                coder.internal.errorIf(newWidth <= 0 || newHeight <= 0, 'vision:calibrate:failToUndistortFullImage');
                
                xBounds = [1 newWidth];
                yBounds = [1 newHeight];
                principalPoint = 0.5 - [xmin, ymin];
                
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Static methods
    %----------------------------------------------------------------------
    methods(Static, Hidden)
        %------------------------------------------------------------------
        function [mappingCoeffs, imageSize, distortionCenter, stretchMatrix] = parseInputsSim(mappingCoeffs, imageSize, distortionCenter, stretchMatrix)
            arguments %#ok
                mappingCoeffs  {fisheyeIntrinsics.checkCoefficients(mappingCoeffs)}
                imageSize   {fisheyeIntrinsics.checkImageSize(imageSize)}
                distortionCenter {fisheyeIntrinsics.checkDistortionCenter(distortionCenter)}
                stretchMatrix {fisheyeIntrinsics.checkStretchMatrix(stretchMatrix)} =  [1 0; 0 1];
            end
        end
        
        %------------------------------------------------------------------
        function [mappingCoefficients, imageSize, distortionCenter, stretchMatrix] = parseInputsCodegen( varargin)
                narginchk(3, 4);
                mappingCoefficients = varargin{1};
                
                imageSize = varargin{2};
                distortionCenter = varargin{3};
                
                fisheyeIntrinsics.checkCoefficients(mappingCoefficients);
                fisheyeIntrinsics.checkImageSize(imageSize);
                fisheyeIntrinsics.checkDistortionCenter(distortionCenter);
                
                if (nargin == 4)
                    stretchMatrix = varargin{4};
                    fisheyeIntrinsics.checkStretchMatrix(stretchMatrix);
                else
                    stretchMatrix = [1 0; 0 1];
                end
        end
        
        %------------------------------------------------------------------
        function checkCoefficients(coefficients)
            validateattributes(coefficients, {'double', 'single'}, ...
                {'nonnan', 'finite', 'vector', 'real', 'nonsparse', 'numel', 4}, ...
                mfilename, 'mappingCoeffs');
        end
        
        %------------------------------------------------------------------
        function checkStretchMatrix(stretch)
            validateattributes(stretch, {'double', 'single'}, ...
                {'nonnan','finite', 'real', 'nonsparse', 'size', [2 2], }, ...
                mfilename, 'stretchMatrix');

            coder.internal.errorIf(stretch(end)~=1, 'vision:calibrate:invalidStretchMatrix');
        end
        
        %------------------------------------------------------------------
        function checkDistortionCenter(center)
            validateattributes(center, {'double', 'single'}, ...
                {'nonnan', 'finite', 'real', 'nonsparse', 'vector', 'numel', 2, }, ...
                mfilename, 'distortionCenter');
        end
        
        %------------------------------------------------------------------
        function checkImageSize(imageSize)
            if isempty(imageSize)
                return;
            end
            validateattributes(imageSize, {'double', 'single'}, ...
                { 'nonnan','finite', 'vector','real', 'nonsparse','numel', 2, 'integer', 'positive'}, ...
                mfilename, 'imageSize');
        end
        
        %------------------------------------------------------------------
        function this = loadobj(that)
            this = fisheyeIntrinsics(...
                that.MappingCoefficients, ...
                that.ImageSize, ...
                that.DistortionCenter, ...
                that.StretchMatrix ...
                );
        end
        
    end
    
    %----------------------------------------------------------------------
    % saveobj is implemented to ensure compatibility across releases by
    % converting the class to a struct prior to saving it. It also contains
    % a version number, which can be used to customize the loading process.
    methods (Hidden)
        
        function that = saveobj(this)
            that.MappingCoefficients  = this.MappingCoefficients;
            that.StretchMatrix        = this.StretchMatrix;
            that.DistortionCenter     = this.DistortionCenter;
            that.ImageSize            = this.ImageSize;
            that.Version              = this.Version;
        end
        
    end
    
end
