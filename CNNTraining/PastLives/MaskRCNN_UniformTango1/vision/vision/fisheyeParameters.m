classdef fisheyeParameters < vision.internal.calibration.FisheyeParametersImpl &  matlab.mixin.Copyable & matlab.mixin.CustomDisplay

% Copyright 2017-2023 The MathWorks, Inc.

    methods(Access=protected)
        %------------------------------------------------------------------
        % Group properties into meaningful categories for display
        %------------------------------------------------------------------
        function group = getPropertyGroups(~)
            group1 = 'Camera Intrinsics';
            list1 = {'Intrinsics'};
            
            group2 = 'Camera Extrinsics';
            list2 = {'RotationMatrices', 'TranslationVectors'};
            
            group3 = 'Accuracy of Estimation';
            list3 = {'MeanReprojectionError', 'ReprojectionErrors', ...
                'ReprojectedPoints'};
            
            group4 = 'Calibration Settings';
            list4 = {'NumPatterns', 'DetectedKeypoints', 'WorldPoints', 'WorldUnits', ...
                'EstimateAlignment'};

            group(1) = matlab.mixin.util.PropertyGroup(list1, group1);
            group(2) = matlab.mixin.util.PropertyGroup(list2, group2);
            group(3) = matlab.mixin.util.PropertyGroup(list3, group3);
            group(4) = matlab.mixin.util.PropertyGroup(list4, group4);
        end
    end
    
    methods
        %----------------------------------------------------------------------
        function this = fisheyeParameters(varargin)
             this@vision.internal.calibration.FisheyeParametersImpl(varargin{:});                        
        end               
    end
        
    methods(Hidden)
        %------------------------------------------------------------------
        function hAxes = showReprojectionErrorsImpl(this, view, hAxes, highlightIndex)
            % showReprojectionErrors Visualize calibration errors.
            %   showReprojectionErrors(fisheyeParams) displays a bar
            %   graph that represents the accuracy of camera calibration.
            %   The bar graph displays the mean reprojection error per image.
            %   The fisheyeParams input is returned from the
            %   estimateFisheyeParameters function.
            %
            %   showReprojectionErrors(fisheyeParams, view) displays the
            %   errors using the visualization style specified by the view
            %   input.
            %   'BarGraph'    - Displays mean error per image as a bar graph.
            %
            %   'ScatterPlot' - Displays the error for each point as a scatter plot.
            %
            %   ax = showReprojectionErrors(...) returns the plot's axes handle.
            %
            %   showReprojectionErrors(...,Name,Value) specifies additional
            %   name-value pair arguments described below:
            %
            %   'HighlightIndex'  - Indices of selected images, specified
            %                       as a vector of integers. For the
            %                       'BarGraph' view, bars corresponding to
            %                       the selected images are highlighted.
            %                       For 'ScatterPlot' view, points
            %                       corresponding to the selected images
            %                       are displayed with circle markers.
            %
            %                       Default: []
            %
            %   'Parent'          - Axes for displaying plot.
            %
            %   Class Support
            %   -------------
            %   fisheyeParams must be a fisheyeParameters object.
            %
            
            coder.internal.errorIf(isempty(this.ReprojectionErrors), 'vision:calibrate:cannotShowEmptyErrors');
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
        function errors = refine(this, imagePoints, shouldComputeErrors)
            % refine Estimate fisheye camera parameters.
            %
            % params = refine(this, imagePoints) numerically refines an
            % initial estimates of fisheye camera parameter values.
            %
            % this is a fisheyeParameters object containing the initial
            % estimates of the parameter values.
            %
            % imagePoints is an M x 2 x P array containing the [x y] 
            % coordinates of the points detected in images, in pixels. M is
            % the number of points in the pattern and P is the number of images.

            x0 = serialize(this);            
            numImages = size(this.RotationVectors, 1);
            xdata = repmat(this.WorldPoints, [numImages, 1]);            
            ydata = arrangeImagePointsIntoMatrix(imagePoints); 
            
            % Get valid imagePoints (in partial mode, all zero-valued
            % coordinates are invalid)
            validImagePointsIdx = reshape(~isnan(ydata(:,1)), [size(imagePoints,1), numImages]);
            
            % Remove invalid imagePoints from ydata
            ydata(~validImagePointsIdx(:),:) = [];
            
            options = optimset('Display', 'off', ...
                               'TolX', 1e-5, ...
                               'TolFun', 1e-4, ...
                               'MaxIter', 100);
            
            worldPointsXYZ = [this.WorldPoints, zeros(size(this.WorldPoints, 1), 1)];
                        
            if shouldComputeErrors
                [x, ~, residual, ~, ~, ~, jacobian] = ...
                    lscftsh(@reprojectWrapper, x0, xdata, ydata, [], [], options);

                % Remove the column corresponding to the constant variable
                jacobian(:, 7) = [];
                standardError = ...
                    vision.internal.calibration.computeStandardError(jacobian, ...
                    residual, false);

                % Be careful with memory
                clear jacobian;
                
                standardError = [standardError(1:6); 0; standardError(7:end)];
                errorsStruct = unpackSerializedParams(this, standardError);
                if ~this.EstimateAlignment
                    errorsStruct.stretchMatrix = [0 0 0];
                end
                errors = fisheyeCalibrationErrors(errorsStruct);
            else
                x = lscftsh(@reprojectWrapper, x0, xdata, ydata, [], [], options);
                errors = [];
            end
            
            deserialize(this, x);
            computeReprojectionErrors(this, imagePoints); 
                                                
            %----------------------------------------------------------------------
            function reprojectedPoints = reprojectWrapper(paramsVector, ~)
                paramStruct = unpackSerializedParams(this, paramsVector);

                % Reproject valid world points
                reprojectedPoints = zeros(nnz(validImagePointsIdx),2);
                
                % add the constant zero a1, so the full set is [a0 a1 a2 a3 a4]
                coeffs = [paramStruct.mappingCoefficients(1), ...
                          zeros(1, 'like', paramStruct.mappingCoefficients), ...
                          paramStruct.mappingCoefficients(2:end)];

                ptCounter = 0; % Point counter
                for i = 1:this.NumPatterns
                    tvec = paramStruct.translationVectors(i, :);
                    rvec = paramStruct.rotationVectors(i, :);
                    R = vision.internal.calibration.rodriguesVectorToMatrix(rvec)';
                    
                    % Get worldPoints to reproject based on the valid
                    % imagePoints in the calibration pattern
                    points = worldPointsXYZ(validImagePointsIdx(:,i),:);
                    
                    points = points * R;
                    points(:, 1) = points(:, 1) + tvec(1);
                    points(:, 2) = points(:, 2) + tvec(2);
                    points(:, 3) = points(:, 3) + tvec(3);
                    
                    currReprojectedPoints = vision.internal.calibration.computeImageProjection(...
                        points, coeffs, paramStruct.stretchMatrix, paramStruct.distortionCenter);
                    
                    start = ptCounter + 1;
                    stop  = ptCounter + size(currReprojectedPoints,1);
                    reprojectedPoints(start:stop,:) = currReprojectedPoints;
                    
                    ptCounter = stop;
                end
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
        function x = serialize(this)
            if this.EstimateAlignment
                x = [1; 1; this.Intrinsics.StretchMatrix(1:3)'];
            else
                x = [1; 1];
            end
            % Adding a constant 'dummy' variable a1 may speed up the
            % interior point algorithm here.
            x = [x; ones(length(this.Intrinsics.MappingCoefficients)+1,1)];
            for i = 1:size(this.RotationVectors,1)
                x = [x; this.RotationVectors(i, :)'];
                x = [x; this.TranslationVectors(i, :)'];
            end
        end
        
        
        %------------------------------------------------------------------
        % Initialize the parameter object from a flat parameter vector
        %------------------------------------------------------------------
        function deserialize(this, x)
            paramStruct = unpackSerializedParams(this, x);
            this.Intrinsics = fisheyeIntrinsics(...
                paramStruct.mappingCoefficients, ...
                this.Intrinsics.ImageSize, ...
                paramStruct.distortionCenter, ...
                paramStruct.stretchMatrix);            
            this.RotationVectors = paramStruct.rotationVectors;
            this.TranslationVectors = paramStruct.translationVectors;
        end
        
    end
    
    methods(Hidden)
        function paramStruct = unpackSerializedParams(this, x)

            paramStruct.distortionCenter = this.Intrinsics.DistortionCenter(:) .* x(1:2);
            if this.EstimateAlignment
                numAlignmentEntries = 5;
                paramStruct.stretchMatrix = [x(3) x(5); ...
                                             x(4)  1];
            else
                numAlignmentEntries = 2;
                paramStruct.stretchMatrix = this.Intrinsics.StretchMatrix;
            end            
            x = x(numAlignmentEntries+1:end);
            
            % Adding a constant 'dummy' variable a1 may speed up the
            % interior point algorithm here.
            numCoefficients = length(this.Intrinsics.MappingCoefficients)+1;
            paramStruct.mappingCoefficients = ...
                ([this.Intrinsics.MappingCoefficients(1);...
                  0; ...
                  this.Intrinsics.MappingCoefficients(2:4)'].*x(1:numCoefficients))';
            paramStruct.mappingCoefficients(2) = [];
            
            x = x(numCoefficients+1:end);
            
            if isempty(x)
                paramStruct.rotationVectors = [];
                paramStruct.translationVectors = [];
            else
                sizeVecs = length(x) / 2;
                numImages = sizeVecs / 3;
                rvecs = zeros(numImages, 3);
                tvecs = rvecs;
                for i = 1:numImages
                    a = x(i*6-5 : i*6);
                    rvecs(i, :) = a(1:3);
                    tvecs(i, :) = a(4:6);
                end
                paramStruct.rotationVectors = rvecs;
                paramStruct.translationVectors = tvecs;
            end
        end
    end
  %----------------------------------------------------------------------
    % saveobj and loadobj are implemented to ensure compatibility across
    % releases even if architecture of fisheyeParameters class changes
    methods (Hidden)
       
        function that = saveobj(this)
            that.Intrinsics           = this.Intrinsics;
            that.WorldPoints          = this.WorldPoints;
            that.WorldUnits           = this.WorldUnits;  
            that.RotationVectors      = this.RotationVectors;
            that.TranslationVectors   = this.TranslationVectors;
            that.ReprojectionErrors   = this.ReprojectionErrors;
            that.EstimateAlignment    = this.EstimateAlignment;
            that.DetectedKeypoints    = this.DetectedKeypoints; 
            that.Version              = this.Version;
        end
        
    end
    
    
    %--------------------------------------------------------------------------
    
    methods (Static, Hidden)
        
        function this = loadobj(that)
            
            if isempty(that.ReprojectionErrors)
                reprojErrors = zeros([size(that.WorldPoints), size(that.RotationVectors, 1)]);
            else
                reprojErrors = that.ReprojectionErrors;
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
                        
            this = fisheyeParameters(that.Intrinsics,...
                'WorldPoints',           that.WorldPoints,...
                'WorldUnits',            that.WorldUnits,...
                'RotationVectors',       that.RotationVectors,...
                'TranslationVectors',    that.TranslationVectors,...
                'ReprojectionErrors',    reprojErrors, ...
                'EstimateAlignment',     that.EstimateAlignment, ...
                'DetectedKeypoints',     detectedKeypoints);
        end
        
    end
    
    methods(Access=public, Static, Hidden)
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.codegen.calibration.fisheyeParameters';
        end
    end
end

