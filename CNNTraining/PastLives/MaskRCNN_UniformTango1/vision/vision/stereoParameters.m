classdef stereoParameters < vision.internal.calibration.StereoParametersImpl & matlab.mixin.CustomDisplay 
    
    % Copyright 2014-2024 MathWorks, Inc.
        
    methods(Access=public, Static)
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.calibration.StereoParametersImpl';
        end
    end
    
    methods
        function this = stereoParameters(varargin)                        
            this = this@vision.internal.calibration.StereoParametersImpl(...
                varargin{:});            
        end        
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        % Group properties into meaningful categories for display
        %------------------------------------------------------------------
        function group = getPropertyGroups(~)
            group1 = 'Parameters of Two Cameras';
            list1 = {'CameraParameters1', 'CameraParameters2'};
            
            group2 = 'Inter-camera Geometry';
            list2 = {'PoseCamera2', 'FundamentalMatrix', 'EssentialMatrix'};
            
            group3 = 'Accuracy of Estimation';
            list3 = {'MeanReprojectionError'};
            
            group4 = 'Calibration Settings';
            list4 = {'NumPatterns', 'WorldPoints', 'WorldUnits'};
            
            group(1) = matlab.mixin.util.PropertyGroup(list1, group1);
            group(2) = matlab.mixin.util.PropertyGroup(list2, group2);
            group(3) = matlab.mixin.util.PropertyGroup(list3, group3);
            group(4) = matlab.mixin.util.PropertyGroup(list4, group4);
        end
    end
    
    methods(Hidden)       
        %------------------------------------------------------------------
        % This method is invoked by the showReprojectionErrors function,
        % which does all the parameter parsing and validation.
        %------------------------------------------------------------------
        function hAxes = showReprojectionErrorsImpl(this, ~, hAxes, highlightIndex)
            
            hAxes = newplot(hAxes);
            
            [meanError1, meanErrorsPerImage1] = ...
                computeMeanError(this.CameraParameters1);
            [meanError2, meanErrorsPerImage2] = ...
                computeMeanError(this.CameraParameters2);
            allErrors = [meanErrorsPerImage1, meanErrorsPerImage2];
            meanError = mean([meanError1, meanError2]);
            
            % Record the current 'hold' state so that we can restore it later
            holdState = get(hAxes,'NextPlot');
            
            % Plot the errors
            hBar = bar(hAxes, allErrors);
            set(hBar(1), 'FaceColor', [0, 0.7, 1]);
            set(hBar(2), 'FaceColor', [242, 197, 148] / 255);
            set(hBar, 'Tag', 'errorBars');
            
            set(hAxes, 'NextPlot', 'add'); % hold on
            hErrorLine = line(get(hAxes, 'XLim'), [meanError, meanError],...
                'LineStyle', '--', 'Parent', hAxes);
            
            % Set AutoUpdate to off to prevent other items from appearing
            % automatically in the legend.
            legend([hBar, hErrorLine], 'Camera 1', 'Camera 2', ...
                getString(message(...
                'vision:calibrate:overallMeanError', ...
                sprintf('%.2f', meanError))), ...
                'Location', 'SouthEast', ...
                'AutoUpdate', 'off');
            
            % Plot highlighted errors
            highlightedErrors = allErrors;
            highlightedErrors(~highlightIndex, :) = 0;
            hHighlightedBar = bar(hAxes, highlightedErrors);
            set(hHighlightedBar(1), 'FaceColor', [0 0 1]);
            set(hHighlightedBar(2), 'FaceColor', [190, 101, 1] ./ 255);
            set(hHighlightedBar, 'Tag', 'highlightedBars');
            
            set(hAxes, 'NextPlot', holdState); % restore the hold state
            
            title(hAxes, getString(message('vision:calibrate:barGraphTitle')));
            xlabel(hAxes, getString(message('vision:calibrate:barGraphXLabelStereo')));
            ylabel(hAxes, getString(message('vision:calibrate:barGraphYLabel')));
        end
        
        %------------------------------------------------------------------
        function errors = refine(this, imagePointsLeft, imagePointsRight, ...
                shouldComputeErrors, isIntrinsicsFixed)
            if nargin < 5
                isIntrinsicsFixed = false;
            end
            x0 = serialize(this, isIntrinsicsFixed);
            numImages = this.CameraParameters1.NumPatterns;
            xdata = repmat(this.CameraParameters2.WorldPoints, [2 * numImages, 1])';
            xdata = xdata(:);
            % data is arranged to be [x1, y1, x2, y2, ...]
            ydataLeft = arrangeImagePointsIntoMatrix(imagePointsLeft)';
            ydataRight = arrangeImagePointsIntoMatrix(imagePointsRight)';

            % Get valid imagePoints (all NaN coordinates are invalid)
            validImagePointsIdxLeft = reshape(~isnan(ydataLeft(1,:)), [size(imagePointsLeft,1), numImages]);
            validImagePointsIdxRight = reshape(~isnan(ydataRight(1,:)), [size(imagePointsRight,1), numImages]);

            % Use only common valid points and drop valid points found in only one camera.
            validImagePointsIdx = validImagePointsIdxLeft & validImagePointsIdxRight;

            % Remove invalid imagePoints from ydata
            ydataLeft(:, ~validImagePointsIdx(:)) = [];
            ydataRight(:, ~validImagePointsIdx(:)) = [];
            ydata = [ydataLeft; ydataRight];
            ydata = ydata(:);
            
            options = optimset('Display', 'off', 'Jacobian', 'on');            
            
            worldPoints = this.WorldPoints;
            worldPointsXYZ = [worldPoints, zeros(size(worldPoints, 1), 1)]';
            numPatterns = this.NumPatterns;
            
            if shouldComputeErrors
                [x, ~, residual, ~, ~, ~, jacobian] = ...
                    lscftsh(@reprojectWrapper, x0, xdata, ydata, [], [], options);
            
                warningstate = warning('off','MATLAB:singularMatrix');
                standardError = ...
                    vision.internal.calibration.computeStandardError(jacobian, ...
                    residual);
                warning(warningstate);
                
                % be careful with memory
                clear jacobian;
                
                errors = deserializeErrors(this, standardError, isIntrinsicsFixed);
            else
                x = ...
                    lscftsh(@reprojectWrapper, x0, xdata, ydata, [], [], options);
                errors = [];
            end
            
            deserialize(this, x, isIntrinsicsFixed);
            
            computeReprojectionErrors(...
                this.CameraParameters1, imagePointsLeft);
            computeReprojectionErrors(...
                this.CameraParameters2, imagePointsRight);
            
            % Resetting LeftSerializedLength to make isequal() behave
            % reasonably. If we do not do that an object returned by
            % estimateCameraParameters and an identical object returned by the
            % constructor will not be equal according to isequal.
            this.LeftSerializedLength = [];
            
            %----------------------------------------------------------------------
            function [reprojectedPoints, jacobian] = reprojectWrapper(paramsVector, ~)
                
                [r0, t0, camera1, camera2] = unpackSerializedParams(this, paramsVector);
                camera1 = unpackSerializedParams(this.CameraParameters1, camera1, isIntrinsicsFixed);
                camera2 = unpackSerializedParams(this.CameraParameters2, camera2, isIntrinsicsFixed);

                % Reproject only valid world points.
                numValidImagePoints = nnz(validImagePointsIdx);
                reprojectedPoints1 = zeros(numValidImagePoints, 2);
                reprojectedPoints2 = zeros(numValidImagePoints, 2);
                jacobian = zeros(numel(reprojectedPoints1)*2, numel(paramsVector));
                
                ptCounter = 0;
                for i = 1:numPatterns
                    % Get world points for all the detected keypoints
                    validWorldPoints = worldPointsXYZ(:,validImagePointsIdx(:,i));
                    numValidWorldPoints = size(validWorldPoints,2);

                    J1 = zeros(2*numValidWorldPoints, numel(paramsVector));
                    J2 = J1;
                    
                    % x = [fx; fy; cx; cy; skew; radial; tangential; rvecs; tvecs];
                    % Note, the internal reprojection function uses a
                    % different definition of skew factor, 
                    % i.e., s = S / fc(1)
                    [Xp, dXpdr, dXpdt, dXpdf, dXpdc, dXpdkr, dXpdkt, dXpds] = ...
                        visionReprojectPointToSingleCamera(validWorldPoints, ...
                                camera1.rotationVectors(i, :), ...
                                camera1.translationVectors(i, :), ...
                                camera1.focalLength, ...
                                camera1.principalPoint, ...
                                camera1.radialDistortion, ...
                                camera1.tangentialDistortion, ...
                                camera1.skew/camera1.focalLength(1));
                    
                    start = ptCounter + 1;
                    stop  = ptCounter + size(Xp, 2);
                    reprojectedPoints1(start:stop,:) = Xp';
     
                    col = 1;
                    
                    if ~isIntrinsicsFixed
                        J1(:, col:col+1) = dXpdf';
                        J1(:, col+2:col+3) = dXpdc';
                        col = col + 4;
                    
                        if this.CameraParameters1.EstimateSkew
                            J1(:, col) = dXpds' / camera1.focalLength(1);
                            col = col + 1;
                        end

                        numRadialDistortionCoeffs = numel(camera1.radialDistortion);
                        J1(:, col:col+numRadialDistortionCoeffs-1) = dXpdkr';
                        col = col + numRadialDistortionCoeffs;

                        if this.CameraParameters1.EstimateTangentialDistortion
                            J1(:, col:col+1) = dXpdkt';
                            col = col + 2;
                        end
                    end
                    
                    J1(:, col+(i-1)*6:col+(i-1)*6+2) = dXpdr';                    
                    J1(:, col+(i-1)*6+3:col+(i-1)*6+5) = dXpdt';                                        
                    
                    % compute Jacobian for the second camera
                    [r2,t2,dr2dr1,dr2dt1,dr2dr0,dr2dt0,dt2dr1,dt2dt1,dt2dr0,dt2dt0] = ...
                        vision.internal.calibration.composeMotion(...
                        camera1.rotationVectors(i, :), ...
                        camera1.translationVectors(i, :), ...
                        r0, t0);
                    
                    [Xp2, dXp2dr2, dXp2dt2, dXp2df2, dXp2dc2, dXp2dkr2, dXp2dkt2, dXp2ds2] = ...
                        visionReprojectPointToSingleCamera(validWorldPoints, ...
                                r2, ...
                                t2, ...
                                camera2.focalLength, ...
                                camera2.principalPoint, ...
                                camera2.radialDistortion, ...
                                camera2.tangentialDistortion, ...
                                camera2.skew/camera2.focalLength(1));
                    
                    reprojectedPoints2(start:stop,:) = Xp2';
                    
                    dXp2dr0 = dXp2dr2' * dr2dr0 + dXp2dt2' * dt2dr0;
                    dXp2dt0 = dXp2dr2' * dr2dt0 + dXp2dt2' * dt2dt0;

                    dXp2dr1 = dXp2dr2' * dr2dr1 + dXp2dt2' * dt2dr1;
                    dXp2dt1 = dXp2dr2' * dr2dt1 + dXp2dt2' * dt2dt1;
                    
                    J2(:, col+(i-1)*6:col+(i-1)*6+2) = dXp2dr1;
                    J2(:, col+(i-1)*6+3:col+(i-1)*6+5) = dXp2dt1; 
                    col = col + numPatterns * 6;
                                                   
                    if ~isIntrinsicsFixed
                        J2(:, col:col+1) = dXp2df2';
                        J2(:, col+2:col+3) = dXp2dc2';
                        col = col + 4;

                        if this.CameraParameters2.EstimateSkew
                            J2(:, col) = dXp2ds2' / camera2.focalLength(1);
                            col = col + 1;
                        end

                        numRadialDistortionCoeffs = numel(camera2.radialDistortion);
                        J2(:, col:col+numRadialDistortionCoeffs-1) = dXp2dkr2';
                        col = col + numRadialDistortionCoeffs;

                        if this.CameraParameters2.EstimateTangentialDistortion
                            J2(:, col:col+1) = dXp2dkt2';
                            col = col + 2;
                        end
                    end
                    
                    J2(:, col:col+2) = dXp2dr0;
                    J2(:, col+3:col+5) = dXp2dt0;
                    
                    ind1x = (1:4:4*numValidWorldPoints);
                    ind1y = (2:4:4*numValidWorldPoints);
                    ind2x = (3:4:4*numValidWorldPoints);
                    ind2y = (4:4:4*numValidWorldPoints);

                    jacobian(4*ptCounter+ind1x, :) = J1(1:2:end, :);
                    jacobian(4*ptCounter+ind1y, :) = J1(2:2:end, :);
                    jacobian(4*ptCounter+ind2x, :) = J2(1:2:end, :);
                    jacobian(4*ptCounter+ind2y, :) = J2(2:2:end, :);

                    ptCounter = stop;
                end

                reprojectedPoints = cat(2, reprojectedPoints1, reprojectedPoints2);
                reprojectedPoints = reprojectedPoints';
                reprojectedPoints = reprojectedPoints(:);
            end
            
            %----------------------------------------------------------------------
            function pointMatrix = arrangeImagePointsIntoMatrix(imagePoints)
                pointMatrix = reshape(permute(imagePoints, [2, 1, 3]), ...
                    [size(imagePoints, 2), size(imagePoints, 1) * size(imagePoints, 3)])';
            end
        end
        
        %------------------------------------------------------------------
        function x = serialize(this, isIntrinsicsFixed)
            if nargin < 2
                isIntrinsicsFixed = false;
            end
            
            rvec = vision.internal.calibration.rodriguesMatrixToVector(...
                this.PoseCamera2.R);
            x = [rvec(:); this.PoseCamera2.Translation(:)];
            x1 = serialize(this.CameraParameters1, isIntrinsicsFixed);
            this.LeftSerializedLength = numel(x1);
            x2 = serialize(this.CameraParameters2, isIntrinsicsFixed);
            % extrinsics of the second camera is not needed
            numExtrinsicElements = 2 * 3 * this.CameraParameters2.NumPatterns;
            x = [x1; x2(1:end-numExtrinsicElements); x];
        end
        
        %------------------------------------------------------------------
        function deserialize(this, x, isIntrinsicsFixed)
            if nargin < 3
                isIntrinsicsFixed = false;
            end
            
            [r, t, camera1, camera2] = unpackSerializedParams(this, x);
            
            % Rotation of camera 2: 3 elements
            this.RotationOfCamera2 = vision.internal.calibration.rodriguesVectorToMatrix(r)';
            
            % Translation of camera 2: 3 elements
            this.TranslationOfCamera2 = t;
            
            deserialize(this.CameraParameters1, camera1, isIntrinsicsFixed);
            
            % CameraParameters2
            deserialize(this.CameraParameters2, camera2, isIntrinsicsFixed);
            [rvecsR, tvecsR] = this.computeRightExtrinsics;
            this.CameraParameters2.setExtrinsics(rvecsR, tvecsR);
        end         
    end
        
     methods(Access=private)
         %------------------------------------------------------------------
         function [rvecs, tvecs] = computeRightExtrinsics(this)
            numImages = this.CameraParameters1.NumPatterns;
            rvecs = zeros(numImages, 3);
            tvecs = zeros(numImages, 3);
            for i = 1:numImages
                currR = this.RotationOfCamera2' * ...
                    this.CameraParameters1.RotationMatrices(:, :, i)';
                rvecs(i, :) = vision.internal.calibration.rodriguesMatrixToVector(currR);
                tvecs(i, :) = (this.TranslationOfCamera2' + ...
                    this.RotationOfCamera2' * ...
                    this.CameraParameters1.TranslationVectors(i, :)')';
            end
        end
        
        %------------------------------------------------------------------
        function [r, t, camera1, camera2] = unpackSerializedParams(this, x)
            r = x(end-5:end-3)';
            t = x(end-2:end)';
            
            leftLength = this.LeftSerializedLength;
            camera1 = x(1:leftLength);
            camera2 = x(leftLength+1:end-6);
        end
        
        %------------------------------------------------------------------
        function errors = deserializeErrors(this, x, isIntrinsicsFixed)
            if nargin < 3
                isIntrinsicsFixed = false;
            end
            [r, t, camera1, camera2] = unpackSerializedParams(this, x);
            errorStruct.r = r;
            errorStruct.t = t;
            errorStruct.camera1 = unpackSerializedParams(...
                this.CameraParameters1, camera1, isIntrinsicsFixed);            
            errorStruct.camera2 = unpackSerializedParams(...
                this.CameraParameters2, camera2, isIntrinsicsFixed);
            
            if isIntrinsicsFixed
                errorStruct.camera1.skew = 0;
                errorStruct.camera1.focalLength = [0 0];
                errorStruct.camera1.principalPoint = [0 0];
                errorStruct.camera1.radialDistortion = zeros(1, numel(errorStruct.camera1.radialDistortion));
                errorStruct.camera1.tangentialDistortion = zeros(1, numel(errorStruct.camera1.tangentialDistortion));

                errorStruct.camera2.skew = 0;
                errorStruct.camera2.focalLength = [0 0];
                errorStruct.camera2.principalPoint = [0 0];
                errorStruct.camera2.radialDistortion = zeros(1, numel(errorStruct.camera2.radialDistortion));
                errorStruct.camera2.tangentialDistortion = zeros(1, numel(errorStruct.camera2.tangentialDistortion));
            end
            errors = stereoCalibrationErrors(errorStruct);
        end
     end
     
    %----------------------------------------------------------------------
    % saveobj and loadobj are implemented to ensure compatibility across
    % releases even if architecture of stereoParameters class changes
    methods(Hidden)
        function that = saveobj(this)
            that.CameraParameters1    = this.CameraParameters1;
            that.CameraParameters2    = this.CameraParameters2;
            that.PoseCamera2          = this.PoseCamera2;
            that.RotationOfCamera2    = this.RotationOfCamera2;
            that.TranslationOfCamera2 = this.TranslationOfCamera2;
            
            that.RectificationParams = this.RectificationParams;
            that.Version = this.Version;
        end
    end
    
    %----------------------------------------------------------------------
    methods (Static, Hidden)
        
        function this = loadobj(that)
            % Ensure backwards compatibility (PoseCamera2 introduced in 22b)
            if ~isfield(that, 'PoseCamera2')
                this.PoseCamera2 = rigidtform3d(that.RotationOfCamera2', ...
                    that.TranslationOfCamera2);
            else
                this.PoseCamera2 = that.PoseCamera2;
            end
            this = stereoParameters(that.CameraParameters1, ...
                that.CameraParameters2, this.PoseCamera2);
            this.RectificationParams = that.RectificationParams;
        end
    end
end