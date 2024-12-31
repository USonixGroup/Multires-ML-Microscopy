
%   Copyright 2013-2023 The MathWorks, Inc.

%#codegen

classdef cameraParameters < vision.internal.calibration.CameraParametersImpl & vision.internal.CustomDisplay

    methods(Access=public, Static)
        function name = matlabCodegenRedirect(~)
            % self-redirection is required here to avoid conflict
            % between the public static method matlabCodegenRedirect in both
            % the base classes.
            name = 'cameraParameters';
        end
    end
   
    methods(Access=protected)
        %------------------------------------------------------------------
        % Group properties into meaningful categories for display
        %------------------------------------------------------------------
        function group = getPropertyGroups(~)
            group1 = 'Camera Intrinsics';
            list1 = {'Intrinsics'};
                        
            group2 = 'Camera Extrinsics';
            list2 = {'PatternExtrinsics'};
            
            group3 = 'Accuracy of Estimation';
            list3 = {'MeanReprojectionError', 'ReprojectionErrors', ...
                'ReprojectedPoints'};
            
            group4 = 'Calibration Settings';
            list4 = {'NumPatterns', 'DetectedKeypoints', 'WorldPoints', 'WorldUnits', ...
                'EstimateSkew', 'NumRadialDistortionCoefficients', ...
                'EstimateTangentialDistortion'};
            
            cameraIntgroup = matlab.mixin.util.PropertyGroup(list1, group1);
            cameraExtgroup = matlab.mixin.util.PropertyGroup(list2, group2);
            accExtgroup = matlab.mixin.util.PropertyGroup(list3, group3);
            calibSetgroup = matlab.mixin.util.PropertyGroup(list4, group4);
            group = [cameraIntgroup, cameraExtgroup, accExtgroup, calibSetgroup];
        end
    end
    
    methods
        %----------------------------------------------------------------------
        function this = cameraParameters(varargin)                         
            this@vision.internal.calibration.CameraParametersImpl(varargin{:});                        
        end               
    end
       
    methods(Hidden)
        %------------------------------------------------------------------        
        function hAxes = showReprojectionErrorsImpl(this, view, hAxes, highlightIndex)
            % showReprojectionErrors Visualize calibration errors.
            %   showReprojectionErrors(cameraParams) displays a bar 
            %   graph that represents the accuracy of camera calibration. 
            %   The bar graph displays the mean reprojection error per image. 
            %   The cameraParams input is returned from the 
            %   estimateCameraParameters function or from the Camera 
            %   Calibrator app. 
            % 
            %   showReprojectionErrors(cameraParams, view) displays the 
            %   errors using the visualization style specified by the view 
            %   input. 
            %   Valid values of view:
            %   'BarGraph':    Displays mean error per image as a bar graph.
            %
            %   'ScatterPlot': Displays the error for each point as a scatter plot.
            %
            %   ax = showReprojectionErrors(...) returns the plot's axes handle.
            %
            %   showReprojectionErrors(...,Name,Value) specifies additional 
            %   name-value pair arguments described below:
            %
            %   'HighlightIndex' Indices of selected images, specified as a 
            %   vector of integers. For the 'BarGraph' view, bars corresponding 
            %   to the selected images are highlighted. For 'ScatterPlot' view, 
            %   points corresponding to the selected images are displayed with 
            %   circle markers.
            %
            %   Default: []
            %
            %   'Parent'         Axes for displaying plot.
            %
            %   Class Support
            %   -------------
            %   cameraParams must be a cameraParameters object.
            %
            
            if isempty(this.ReprojectionErrors)
                error(message('vision:calibrate:cannotShowEmptyErrors'));
            end
            hAxes = newplot(hAxes);
            
            if strcmpi(view, 'bargraph')
                % compute mean errors per image
                [meanError, meanErrors] = computeMeanError(this);
                vision.internal.calibration.plotMeanErrorPerImage(...
                    hAxes, meanError, meanErrors, highlightIndex);
            else % 'scatterPlot'
                errors = this.ReprojectionErrors;                                  
                vision.internal.calibration.plotAllErrors(...
                    hAxes, errors, highlightIndex);
            end
        end        
    end
    

    
    methods (Hidden=true, Access=public)
        function errors = refine(this, imagePoints, shouldComputeErrors, isIntrinsicsFixed)
            % refine Estimate camera parameters numerically.
            %
            % params = refine(this, imagePoints) 
            %   numerically refines an initial guess of camera parameter values.
            %   accounting for lens distortion.
            %
            % this is a cameraParameters object containing the 
            % initial "guess" of the parameter values. These initial values
            % can be computed in closed form by assuming zero lens distortion.
            %
            % imagePoints is an M x 2 x P array containing the [x y] 
            % coordinates of the points detected in images, in pixels. M is
            % the number of points in the pattern and P is the number of images.
            if nargin < 4
                isIntrinsicsFixed = false;
            end
            
            x0 = serialize(this, isIntrinsicsFixed);
            numImages = size(this.RotationVectors, 1);
            xdata = repmat(this.WorldPoints, [numImages, 1])';  
            xdata = xdata(:);
            % data is arranged to be [x1, y1, x2, y2, ...]
            ydata = arrangeImagePointsIntoMatrix(imagePoints)';
            
            % Get valid imagePoints (all NaN coordinates are invalid)
            validImagePointsIdx = reshape(~isnan(ydata(1,:)), [size(imagePoints,1), numImages]);
            
            % Remove invalid imagePoints from ydata
            ydata(:, ~validImagePointsIdx(:)) = [];
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
                
                errorsStruct = unpackSerializedParams(this, standardError, isIntrinsicsFixed);
                if isIntrinsicsFixed
                    errorsStruct.skew = 0;
                    errorsStruct.focalLength = [0 0];
                    errorsStruct.principalPoint = [0 0];
                    errorsStruct.radialDistortion = zeros(1, numel(errorsStruct.radialDistortion));
                    errorsStruct.tangentialDistortion = zeros(1, numel(errorsStruct.tangentialDistortion));
                end
                errors = cameraCalibrationErrors(errorsStruct);
            else
                x = ...
                    lscftsh(@reprojectWrapper, x0, xdata, ydata, [], [], options);
                errors = [];
            end
            
            deserialize(this, x, isIntrinsicsFixed);
            computeReprojectionErrors(this, imagePoints);

            %----------------------------------------------------------------------
            function [reprojectedPoints, jacobian] = reprojectWrapper(paramsVector, ~)
                paramStruct = unpackSerializedParams(this, paramsVector, isIntrinsicsFixed);

                % Reproject valid world points
                reprojectedPoints = zeros(nnz(validImagePointsIdx), 2);
                jacobian = zeros(numel(reprojectedPoints), numel(x0));
                numRadialDistortionCoeffs = numel(paramStruct.radialDistortion);
                
                ptCounter = 0; % Point counter
                for i = 1:numPatterns
                    % x = [fx; fy; cx; cy; skew; radial; tangential; rvecs; tvecs];
                    % Note, the internal reprojection function uses a
                    % different definition of skew factor, 
                    % i.e., s = S / fc(1)
                    
                    % Get world points for all the detected keypoints
                    validWorldPoints = worldPointsXYZ(:,validImagePointsIdx(:,i));
                    
                    [Xp, dXpdr, dXpdt, dXpdf, dXpdc, dXpdkr, dXpdkt, dXpds] = ...
                        visionReprojectPointToSingleCamera(validWorldPoints, ...
                                paramStruct.rotationVectors(i, :), ...
                                paramStruct.translationVectors(i, :), ...
                                paramStruct.focalLength, ...
                                paramStruct.principalPoint, ...
                                paramStruct.radialDistortion, ...
                                paramStruct.tangentialDistortion, ...
                                paramStruct.skew/paramStruct.focalLength(1));
                    % Xp: reprojections, 2xN matrix
                    %
                    % dXpdr: Derivative of xp w.r.t rotation vector, 3x(2N)
                    % matrix
                    %
                    % dXpdt: Derivative of xp w.r.t translation vector,
                    % 3x(2N) matrix
                    %
                    % dXpdf: Derivative of xp w.r.t focal length, 2x(2N)
                    % matrix
                    %
                    % dXpdc: Derivative of xp w.r.t principal points,
                    % 2x(2N) matrix
                    %
                    % dXpdkr: Derivative of xp w.r.t radial distortion,
                    % 3x(2N) matrix if there are three coefficients
                    %
                    % dXpdkt: Derivative of xp w.r.t tangential distortion,
                    % 2x(2N) matrix
                    %
                    % dXpds: Derivative of xp w.r.t skew, (2N)x1 vector
                    
                    start = ptCounter + 1;
                    stop  = ptCounter + size(Xp, 2);
                    reprojectedPoints(start:stop,:) = Xp';

                    % Row indices for jacobian
                    ind = start*2 - 1:stop*2;
                    
                    col = 1;
                    if ~isIntrinsicsFixed
                        jacobian(ind, col:col+1) = dXpdf';
                        jacobian(ind, col+2:col+3) = dXpdc';
                        col = col + 4;
                        
                        if this.EstimateSkew
                            jacobian(ind, col) = dXpds' / paramStruct.focalLength(1);
                            col = col + 1;
                        end

                        jacobian(ind, col:col+numRadialDistortionCoeffs-1) = dXpdkr';
                        col = col + numRadialDistortionCoeffs;

                        if this.EstimateTangentialDistortion
                            jacobian(ind, col:col+1) = dXpdkt';
                            col = col + 2;
                        end
                    end
                    
                    jacobian(ind, col+(i-1)*6:col+(i-1)*6+2) = dXpdr';
                    
                    jacobian(ind, col+(i-1)*6+3:col+(i-1)*6+5) = dXpdt';
                    
                    ptCounter = stop;
                end
                
                reprojectedPoints = reprojectedPoints';
                reprojectedPoints = reprojectedPoints(:);
            end           
            
            %----------------------------------------------------------------------
            function pointMatrix = arrangeImagePointsIntoMatrix(imagePoints)
                pointMatrix = reshape(permute(imagePoints, [2, 1, 3]), ...
                    [2, size(imagePoints, 1) * size(imagePoints, 3)])';
            end
        end
        
        %------------------------------------------------------------------
        % Convert the parameter object into a flat parameter vector
        % to be used in optimization.
        %------------------------------------------------------------------
        function x = serialize(this, isIntrinsicsFixed)
            if nargin < 2
                isIntrinsicsFixed = false;
            end
            % x = [fx; fy; cx; cy; skew; radial; tangential; rvecs; tvecs];
            if ~isIntrinsicsFixed
                x = [this.K(1,1); this.K(2,2); this.K(1,3); this.K(2,3)];
            
                if this.EstimateSkew
                    x = [x; this.K(1,2)];
                end
            
                x = [x; this.RadialDistortion(1:this.NumRadialDistortionCoefficients)'];
            
                if this.EstimateTangentialDistortion
                    x = [x; this.TangentialDistortion'];
                end
            else
                x = [];
            end
            
            rt = [this.RotationVectors, this.TranslationVectors]';
            x = [x; rt(:)]; % r1, t1, r2, t2, ...
        end
        
        
        %------------------------------------------------------------------
        % Initialize the parameter object from a flat parameter vector
        %------------------------------------------------------------------
        function deserialize(this, x, isIntrinsicsFixed)
            if nargin < 3
                isIntrinsicsFixed = false;
            end
            paramStruct = unpackSerializedParams(this, x, isIntrinsicsFixed);
            this.K = vision.internal.calibration.constructIntrinsicMatrix(...
                paramStruct.focalLength(1), paramStruct.focalLength(2), ...
                paramStruct.principalPoint(1), paramStruct.principalPoint(2), ...
                paramStruct.skew);
            
            this.RadialDistortion = paramStruct.radialDistortion;
            this.TangentialDistortion = paramStruct.tangentialDistortion;
            
            this.RotationVectors = paramStruct.rotationVectors;
            this.TranslationVectors = paramStruct.translationVectors;
        end
        
    end
    
    methods(Hidden)
        function paramStruct = unpackSerializedParams(this, x, isIntrinsicsFixed)
            if nargin < 3
                isIntrinsicsFixed = false;
            end
            
            if ~isIntrinsicsFixed
                if this.EstimateSkew
                    paramStruct.skew = x(5);
                    numIntrinsicMatrixEntries = 5;
                else
                    paramStruct.skew = 0;
                    numIntrinsicMatrixEntries = 4;
                end

                paramStruct.focalLength(1) = x(1);
                paramStruct.focalLength(2) = x(2);
                paramStruct.principalPoint(1) = x(3);
                paramStruct.principalPoint(2) = x(4);

                x = x(numIntrinsicMatrixEntries+1:end);

                numRadialCoeffs = this.NumRadialDistortionCoefficients;
                paramStruct.radialDistortion = x(1:numRadialCoeffs)';

                if this.EstimateTangentialDistortion
                    paramStruct.tangentialDistortion = ...
                        x(numRadialCoeffs+1:numRadialCoeffs+2)';
                    numDistortionCoeffs = numRadialCoeffs + 2;
                else
                    paramStruct.tangentialDistortion = [0,0];
                    numDistortionCoeffs = numRadialCoeffs;
                end

                x = x(numDistortionCoeffs+1:end);
            else
                paramStruct.skew = this.Skew;
                paramStruct.focalLength = this.FocalLength;
                paramStruct.principalPoint = this.PrincipalPoint;
                paramStruct.radialDistortion = this.RadialDistortion;
                paramStruct.tangentialDistortion = this.TangentialDistortion;
            end
            
            if isempty(x)
                paramStruct.rotationVectors = [];
                paramStruct.translationVectors = [];
            else
                x = reshape(x, 6, []);
                paramStruct.rotationVectors = x(1:3, :)';
                paramStruct.translationVectors = x(4:6, :)';
            end
        end        
    end
    
    %----------------------------------------------------------------------
    % saveobj and loadobj are implemented to ensure compatibility across
    % releases even if architecture of the class changes
    methods (Hidden)
       
        function that = saveobj(this)
            that.RadialDistortion     = this.RadialDistortion;     
            that.TangentialDistortion = this.TangentialDistortion;
            that.ImageSize            = this.ImageSize;
            that.WorldPoints          = this.WorldPoints;
            that.WorldUnits           = this.WorldUnits;  
            that.EstimateSkew         = this.EstimateSkew;
            that.NumRadialDistortionCoefficients = this.NumRadialDistortionCoefficients;
            that.EstimateTangentialDistortion    = this.EstimateTangentialDistortion;
            that.RotationVectors      = this.RotationVectors;
            that.TranslationVectors   = this.TranslationVectors;
            that.ReprojectionErrors   = this.ReprojectionErrors;
            that.K                    = this.K;
            that.DetectedKeypoints    = this.DetectedKeypoints; 
            that.Version              = this.Version;
        end
        
    end
    
    
    %----------------------------------------------------------------------    
    methods (Static, Hidden)
        
        function this = loadobj(that)
            if isempty(that.ReprojectionErrors)
                reprojErrors = zeros(0, 2, 0);
            else
                reprojErrors = that.ReprojectionErrors;
            end
            
            if ~isfield(that, 'ImageSize')
                imageSize = zeros(0,2);
            else
                imageSize = that.ImageSize;
            end
            
            % Ensure backward compatibility (DetectedKeypoints introduced in 21a)
            if ~isfield(that, 'DetectedKeypoints')
                if isempty(that.RotationVectors)
                    detectedKeypoints = zeros(0,2);
                else
                    numPoints = size(that.WorldPoints, 1);
                    numImages = size(that.RotationVectors, 1);
                    detectedKeypoints = true(numPoints, numImages);
                end
            else
                detectedKeypoints = that.DetectedKeypoints;
            end

            % Ensure backwards compatibility (K introduced in 22b)
            if ~isfield(that, 'K')
                K = that.IntrinsicMatrix';
            else
                K = that.K;
            end
            
            this = cameraParameters(...
                'K',                    K,...
                'RadialDistortion',     that.RadialDistortion,...
                'TangentialDistortion', that.TangentialDistortion,...
                'ImageSize',            imageSize,...
                'WorldPoints',          that.WorldPoints,...
                'WorldUnits',           that.WorldUnits,...
                'EstimateSkew',         that.EstimateSkew,...
                'NumRadialDistortionCoefficients', that.NumRadialDistortionCoefficients,...
                'EstimateTangentialDistortion',    that.EstimateTangentialDistortion,...
                'RotationVectors',      that.RotationVectors,...
                'TranslationVectors',   that.TranslationVectors, ...
                'ReprojectionErrors',   reprojErrors, ...
                'DetectedKeypoints',    detectedKeypoints);
        end
    end
end

function mode = isSimMode()
mode = isempty(coder.target);
end