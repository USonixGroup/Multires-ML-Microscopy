classdef (Abstract) CalibrationPatternDetector < handle

% Copyright 2024 The MathWorks, Inc.

    properties
        Images1
        Images2
        IsStereo
        DetectorName
    end

    properties (Abstract)
        ShowProgressBar
        ProgressBarParent
    end

    methods (Abstract)
        [imagePoints, isFound] = detectInOneImage(this, I, varargin)
        waitBar = createProgressBar(this, numImages);
    end

    methods
        %-----------------------------------------------------------------------
        % Constructor
        %-----------------------------------------------------------------------
        function this = CalibrationPatternDetector(detectorName, input1, input2)
            this.DetectorName = detectorName;
            this.IsStereo = identifyCameraType(input1, input2);
            this.Images1 = this.validateImages(input1);
            if this.IsStereo
                this.Images2 = this.validateImages(input2);
                this.validateStereoImages;
            end
        end

        %-----------------------------------------------------------------------
        % Detector Public API
        %-----------------------------------------------------------------------
        function [imagePoints, imagesUsed, userCanceled] = detectKeyPoints(this)
            if ~this.IsStereo
                % Single camera
                [imagePoints, imagesUsed, userCanceled] = this.detectMono;
            else
                % Stereo camera
                [imagePoints, imagesUsed, userCanceled] = this.detectStereo;
            end   
        end
    end

    methods (Access = private)
        %-----------------------------------------------------------------------
        function [imagePoints, imagesUsed, userCanceled] = detectMono(this, varargin)
            
            if nargin == 1
                images = this.Images1;
            else
                images = varargin{1};
            end
            
            if ischar(images) || (isnumeric(images) && size(images, 4) == 1)
                % Detect in a single image.
                [imagePoints, imagesUsed] = this.detectInOneImage(images);
                userCanceled = false;
            else % iscell(images) || (isnumeric(images) && size(images, 4) > 1)
                % Detect in a set of images specified by file names (images is a cell) or 
                % detect in a stack of images(images is a matrix).
                [imagePoints, imagesUsed, userCanceled] = this.detectInMultipleImages(images);
            end
        end

        %-------------------------------------------------------------------------------
        % Helper to run the detector on multiple images & update progress bar in the app.
        %-------------------------------------------------------------------------------
        function [imagePoints, imagesUsed, userCanceled] = detectInMultipleImages(this, input)
        
            [numImages, refSize] = this.imageDims(input);
            
            % Create progress bar dialog.
            userCanceled = false;
            if this.ShowProgressBar
                waitBar = this.createProgressBar(numImages);
            end

            points = cell(numImages, 1);
            imagesUsed = false(numImages, 1);
            for i = 1:numImages        
                if this.ShowProgressBar && waitBar.Canceled
                    imagePoints  = [];
                    imagesUsed   = [];
                    userCanceled = true;
                    return
                end
                
                I = this.imageRead(input, i, refSize);
                [points{i}, imagesUsed(i)] = this.detectInOneImage(I);
                
                if this.ShowProgressBar
                    waitBar.update();
                end
            end

            points(~imagesUsed) = [];
            imagePoints = cat(3, points{:});
        end

        %-----------------------------------------------------------------------
        function [imagePoints, pairsUsed, userCanceled] = detectStereo(this)
            
            if ischar(this.Images1) || (isnumeric(this.Images1) && size(this.Images1, 4) == 1)
                % Detect in a pair of single images.
                [imagePoints, pairsUsed] = this.detectInOneImagePair(this.Images1, this.Images2);
                userCanceled = false;
            else % iscell(this.Images1) || (isnumeric(this.Images1) && size(this.Images1, 4) > 1)
                % Detect in pairs of multiple images.
                [imagePoints, pairsUsed, userCanceled] = ...
                    this.detectInMultipleImagePairs(this.Images1, this.Images2);
            end
        end

        %-------------------------------------------------------------------------------
        % Helper to run the detector on multiple images & update progress bar in the app.
        %-------------------------------------------------------------------------------
        function [imagePoints, pairsUsed, userCanceled] = ...
                detectInMultipleImagePairs(this, input1, input2)

            % Concatenate the two sets of images into one.
            images = concatenateImages(input1, input2);

            [imagePoints, pairsUsed, userCanceled] = this.detectMono(images);

            % Separate the points from images1 and images2.
            [imagePoints, pairsUsed] = vision.internal.calibration.separatePoints(imagePoints, pairsUsed);

            %---------------------------------------------------------------------------
            % Helper to concatenate images of a stereo pair.
            %---------------------------------------------------------------------------
            function images = concatenateImages(images1, images2)

                if iscell(images1)
                    images = {images1{:}, images2{:}}; %#ok
                elseif ischar(images1)
                    images = {images1, images2};
                else
                    images = cat(4, images1, images2);
                end
            end
        end

        %-----------------------------------------------------------------------
        function [imagePoints, isFound] = detectInOneImagePair(this, I1, I2)
            
            if ischar(I1) && ischar(I2)
                I1 = imread(I1);
                I2 = imread(I2);
            end
            
            vision.internal.errorIf(any(size(I1)~=size(I2)), 'vision:caltool:imageSizeInconsistent')

            % Detect in a pair of single images.
            cameraIdx = 1;
            [points1, imagesUsed1] = this.detectInOneImage(I1, cameraIdx);
            cameraIdx = 2;
            [points2, imagesUsed2] = this.detectInOneImage(I2, cameraIdx);
            
            isFound = imagesUsed1 && imagesUsed2;
            if isFound
                imagePoints = cat(4, points1, points2);
            else
                imagePoints = zeros(0, 2);
            end
        end
    end

    methods (Access=protected)
        %-------------------------------------------------------------------
        function [numImages, refSize] = imageDims(~, input)
            if iscell(input)
                % input1 is a cell array of filenames.
                numImages = numel(input);
                I = imread(input{1});
                refSize = size(I,1:2);
            else
                % input1 is an array of in-memory images.
                numImages = size(input, 4);
                refSize = size(input,1:2);
            end
        end

        %-------------------------------------------------------------------
        function I = imageRead(~, input, i, refSize)
            % Read images and check for consistent dimensions.
            if iscell(input)
                I = imread(input{i});
                vision.internal.errorIf(any(refSize ~= size(I,1:2)),...
                    'vision:caltool:imageSizeInconsistent');
            else
                I = input(:,:,:,i);
            end
        end
    end

    methods (Access=private)
        %-----------------------------------------------------------------------
        function I = validateImages(this, images)
            
            validClasses = {'double', 'single', 'uint8', 'int16', 'uint16'};
            validAttributes = {'nonempty', 'real', 'nonsparse'};
        
            I = convertStringsToChars(images);
            if iscell(I)
                validateattributes(I, {'cell'}, {'nonempty', 'vector'}, ...
                    this.DetectorName, 'imageFileNames');
                
                % String vector or cell array of character vectors.
                for j = 1:length(I)
                    validateImageFileName(this, I{j});
                end
            elseif ischar(I)
                % String or character vector.
                validateImageFileName(this, I);            
            else
                % Image is in-memory.
                validateattributes(I, validClasses, validAttributes, this.DetectorName, 'images')            
                vision.internal.errorIf(size(I, 3) ~= 1 && size(I, 3) ~= 3,...
                    'vision:dims:imageNot2DorRGB');
                if ~isa(I, 'uint8')
                    I = uint8(I);
                end
            end
        end
        
        %-----------------------------------------------------------------------
        function validateImageFileName(this, fileName)
            
            varname = vision.getMessage('vision:calibrate:elementsOfImageFileNames');
            validateattributes(fileName, {'char', 'string'}, {'nonempty'},...
                this.DetectorName, varname);
            
            try 
                state = warning('off','imageio:tifftagsread:badTagValueDivisionByZero');
                imfinfo(fileName);
            catch e
                warning(state);
                throwAsCaller(e);
            end
            warning(state);
        end

        %-----------------------------------------------------------------------
        function validateStereoImages(this)
            vision.internal.errorIf(strcmp(class(this.Images1), class(this.Images2)) == 0,...
                'vision:calibrate:stereoImagesMustBeSameClass');
            
            vision.internal.errorIf(~ischar(this.Images1) && any(size(this.Images1) ~= size(this.Images2)),...
                'vision:calibrate:stereoImagesMustBeSameSize');
        end
    end
end

%-------------------------------------------------------------------------------
function isStereo = identifyCameraType(input1, input2)
    % identifyCameraType identifies intended camera type based on the syntax
    % used. If second argument is patternDims, set isStereo to false.
    
    if isnumeric(input2)
        % if second argument is numeric, it can be an in-memory image-stack
        % or patternDims which is a two-element vector.
        isStereo = isnumeric(input1) && numel(input2) ~= 2;
    else 
        % if second argument is not numeric, it cannot be patternDims.
        isStereo = true;
    end
end