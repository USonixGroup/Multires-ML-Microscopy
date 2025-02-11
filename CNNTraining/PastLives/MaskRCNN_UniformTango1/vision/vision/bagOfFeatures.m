classdef bagOfFeatures < vision.internal.EnforceScalarHandle & matlab.mixin.CustomDisplay

% Copyright 2014-2023 MathWorks, Inc.

%#codegen

    properties(GetAccess = public, SetAccess = private)

        % CustomExtractor - A function handle to a custom feature extraction function
        CustomExtractor
        % NumVisualWords - The number of visual words in the bag
        NumVisualWords
        % TreeProperties - The vocabulary tree properties is a two element
        %                  vector, [numLevels branchingFactor], that
        %                  defines the number of levels in the vocabulary
        %                  tree and the branching factor at each level of
        %                  the tree.
        TreeProperties
        % StrongestFeatures - Fraction of strongest features to use from each label
        StrongestFeatures
        % PointSelection - Method used to define point locations for feature extraction
        PointSelection
        % GridStep - Step in X and Y directions defining the grid spacing
        GridStep
        % BlockWidth - Patch sizes from which SURF descriptor is extracted
        BlockWidth
        % Upright - Whether or not to extract upright SURF descriptors
        Upright
    end

    properties (Hidden, GetAccess = public, SetAccess = protected, Dependent)
        % VocabularySize - This property is no longer recommended. Use
        % NumVisualWords.
        VocabularySize
    end

    properties(Hidden, Access = protected)
        UsingCustomExtractor
        CustomFeatureLength
        CustomFeatureType
        ExtractorOutputsLocations
    end

    properties(Hidden, Transient, Access = protected)
        % Function handle to either default or custom extractor.
        Extractor

        % Validation function for custom feature extractor.
        ExtractorValidationFcn
    end

    properties(Hidden, Constant, Access = protected)
        ValidPointSelectionOptions = {'Grid','Detector'}
    end

    properties (Access = protected, Transient)
        % VocabularyBuilder  Vocabulary builder strategy for learning the
        %                    visual word vocabulary.
        VocabularyBuilder
    end

    properties (Access = protected)
        % Encoder  Encoder strategy for encoding image features using the
        %          learned vocabulary.
        Encoder
    end

    properties (Dependent, Access = protected)
        % VocabularyLevels - The number of levels in the vocabulary tree.
        VocabularyLevels

        % VocabularyBranches - The branching factor of the vocabulary tree.
        VocabularyBranches
    end

    %----------------------------------------------------------------------
    % Set and get methods for dependent properties.
    %----------------------------------------------------------------------
    methods
        function v = get.VocabularySize(this)
            v = this.NumVisualWords;
        end

        function set.VocabularySize(this, v)
            this.NumVisualWords = v;
        end

        function v = get.VocabularyLevels(this)
            v = this.TreeProperties(1);
        end

        function v = get.VocabularyBranches(this)
            v = this.TreeProperties(2);
        end

        function set.VocabularyBranches(this, v)
            this.VocabularyBuilder.BranchingFactor = v;
        end
    end

    %----------------------------------------------------------------------
    methods (Access = public)

        %------------------------------------------------------------------
        % Constructor
        function this = bagOfFeatures(varargin)

            if nargin == 0

                % Just return empty object with all default configuration
                % settings. This required to serialize/deserialize objects
                % for parallel processing and testing.
                d = bagOfFeatures.getDefaultSettings;
                this = this.setParams(d);
                this.NumVisualWords = 0; % this is actually an empty bag

            else
                % parse the inputs
                [imgSets, params] = bagOfFeatures.parseInputs(varargin{:});

                this = this.setParams(params);

                printer = vision.internal.MessagePrinter.configure(params.Verbose);

                initializeVocabularyBuilder(this, params);

                [descriptors, scores] = this.extractDescriptors(imgSets, params, printer);

                % If needed, remove some of the descriptors
                descriptorSet = this.trimDescriptors(descriptors, scores, printer);

                vocabulary = createVocabulary(this, descriptorSet);

                initializeEncoder(this, vocabulary);

                printer.printMessage('vision:bagOfFeatures:finishedBoF').linebreak;
            end

        end % end of Constructor

        %------------------------------------------------------------------
        function [featureVector, varargout] = encode(this, in, varargin)
        %encode Create a feature vector, a histogram of visual word occurrences
        %  featureVector = encode(bag, I) returns a feature vector,
        %  that is a histogram of visual word occurrences in I.  I can
        %  be grayscale or truecolor. bag is the bagOfFeatures object.
        %  featureVector's length is bag.NumVisualWords.
        %
        %  [..., words] = encode(bag, I) optionally returns the visual
        %  words as a visualWords object. A visualWords object stores
        %  the visual words that occur in I and the locations of those
        %  words.
        %
        %  featureVector = encode(bag, imds) returns a feature vector,
        %  that is a histogram of visual word occurrences in imds.
        %  imds is an ImageDatastore object. featureVector is
        %  M-by-bag.NumVisualWords, where M is the total number of
        %  images in imds, numel(imds.Files).
        %
        %  [..., words] = encode(bag, imds) optionally returns an
        %  array of visualWords objects for each image in imds, an
        %  ImageDatastore object. A visualWords object stores the
        %  visual words and the locations of those words within an
        %  image.
        %
        %  [...] = encode(..., Name, Value) specifies additional
        %  name-value pairs described below:
        %
        %  'Normalization' Specify the type of normalization applied
        %                  to the feature vector. Set to either 'L2' or
        %                  'none'.
        %
        %                  Default: 'L2'
        %
        %  'SparseOutput'  True or false. Set to true to return visual
        %                  word histograms as sparse matrices. This
        %                  reduces memory consumption for large visual
        %                  vocabularies where the visual word
        %                  histograms contain many zero elements.
        %
        %                  Default: false
        %
        %  'Verbose'       Set true to display progress information.
        %
        %                  Default: true
        %  Example
        %  -------
        %  % Load a set of images
        %  setDir  = fullfile(toolboxdir('vision'),'visiondata','imageSets');
        %  imds = imageDatastore(setDir, 'IncludeSubfolders',true, 'LabelSource', 'foldernames');
        %
        %  trainingSet = splitEachLabel(imds, 2); % pick first 2 images from each label
        %  bag = bagOfFeatures(trainingSet);
        %
        %  % encode one of the images into a feature vector
        %  img = readimage(trainingSet, 1);
        %  featureVector = encode(bag, img);

            nargoutchk(0,2);

            % Check if extractor supports visual word output
            if nargout == 2 && ~this.ExtractorOutputsLocations
                error(message('vision:bagOfFeatures:customNoLocations',...
                              func2str(this.CustomExtractor)));
            end

            params = bagOfFeatures.parseEncodeInputs(in, varargin{:});

            % Configure Encoder properties.
            this.Encoder.Normalization = params.Normalization;
            this.Encoder.SparseOutput  = params.SparseOutput;

            numVarargout = nargout-1;

            printer = vision.internal.MessagePrinter.configure(params.Verbose);
            printer.linebreak;
            printer.printMessage('vision:bagOfFeatures:encodeTitle');
            printer.print('--------------------------------------\n');

            if ~isnumeric(in)
                % input is a imageDatastore
                bagOfFeatures.printImageSetDescription(printer, in);

                printer.printMessageNoReturn('vision:bagOfFeatures:encodeStartDS', numel(in.Files));

                [featureVector, varargout{1:numVarargout}] = ...
                    this.encodeScalarImageSet(in, params);

                printer.printMessage('vision:bagOfFeatures:encodeDone');

            else
                printer.printMessageNoReturn('vision:bagOfFeatures:encodeStartOneImage');

                [featureVector, varargout{1:numVarargout}] = this.encodeSingleImage(in);

                printer.printMessage('vision:bagOfFeatures:encodeDone');
            end

        end

        %------------------------------------------------------------------
        function s = saveobj(this)
        % save properties into struct
            s.NumVisualWords            = this.NumVisualWords;
            s.TreeProperties            = this.TreeProperties;
            s.StrongestFeatures         = this.StrongestFeatures;
            s.GridStep                  = this.GridStep;
            s.BlockWidth                = this.BlockWidth;
            s.PointSelection            = this.PointSelection;
            s.CustomExtractor           = this.CustomExtractor;
            s.CustomFeatureType         = this.CustomFeatureType;
            s.UsingCustomExtractor      = this.UsingCustomExtractor;
            s.CustomFeatureLength       = this.CustomFeatureLength;
            s.Upright                   = this.Upright;
            s.ExtractorOutputsLocations = this.ExtractorOutputsLocations;
            s.Encoder                   = saveobj(this.Encoder);
            s.Version                   = 1;
        end
    end % end public methods

    %======================================================================
    methods (Hidden, Access = {?invertedImageIndex})
        %------------------------------------------------------------------
        % Compute a histogram of occurences for given image features from
        % a single image. It turns the input into a feature vector
        %------------------------------------------------------------------
        function featureVector = encodeFeatures(this, features)

            vision.internal.inputValidation.checkFeatures(features, mfilename, 'features');

            % Configure Encoder properties.
            this.Encoder.Normalization = 'None';
            this.Encoder.SparseOutput  = true;

            % check if input feature type matches the one cached in BoF
            if isa(features, 'binaryFeatures')
                if ~strcmp(this.CustomFeatureType,'binaryFeatures')
                    error(message('vision:bagOfFeatures:encodeInvalidBinaryFeatureType'));
                end

                if this.CustomFeatureLength ~= features.NumBits
                    error(message('vision:bagOfFeatures:encodeInvalidFeatureLength', ...
                                  this.CustomFeatureLength, features.NumBits));
                end

                features = features.Features;
            else
                if this.UsingCustomExtractor
                    if strcmp(this.CustomFeatureType,'binaryFeatures')
                        error(message('vision:bagOfFeatures:encodeInvalidNumericFeatureType'));
                    end

                    featureLength = size(features,2);
                    if this.CustomFeatureLength ~= featureLength
                        error(message('vision:bagOfFeatures:encodeInvalidFeatureLength', ...
                                      this.CustomFeatureLength, featureLength));
                    end
                end
            end

            featureVector = this.Encoder.encode(features);

        end
    end
    %======================================================================
    methods (Hidden, Access = protected)

        %------------------------------------------------------------------
        % This method may trim the number of descriptors depending on the
        % circumstances. Here are the conditions:
        % * when imgSet set is a scalar, i.e. it contains one set of images,
        %   the number of features is reduced to (num all
        %   features)*strongestFeaturesFraction
        % * when imgSet is an array, the number of features is reduced to
        %   min(num features per set)*strongestFeaturesFraction in order to
        %   balance relative "strength" of each set
        %------------------------------------------------------------------
        function trimmedDescriptors = trimDescriptors(this, ...
                                                      descriptorCell, scores, printer)

            numFeatures = cellfun(@length,scores);

            % Determine smallest number of features in any of the sets.
            % That will determine the maximum number of features to use
            % while forming the vocabulary
            [maxNumFeatures, setIdx] = min(numFeatures);

            % Based on percentage, figure out how many features to keep
            numToKeep = round(maxNumFeatures*this.StrongestFeatures);

            printer.printMessage('vision:bagOfFeatures:trimDescriptors', ...
                                 100 * this.StrongestFeatures).linebreak;

            this.printFeatureBalancingMessage(printer, ...
                                              numFeatures, setIdx, numToKeep);

            numSets = length(descriptorCell);
            trimmedDescriptors = zeros(numSets*numToKeep, ...
                                       size(descriptorCell{1}, 2), 'like', descriptorCell{1});

            insertIdx = 1:numToKeep;
            for i=1:numSets
                setScores      = scores{i};
                setDescriptors = descriptorCell{i};

                % limit the features to only the strong ones
                [~, sortIdx] = sort(setScores ,'descend');

                topScoresIdx = sortIdx(1:numToKeep,:);

                setDescriptors = setDescriptors(topScoresIdx,:);
                trimmedDescriptors(insertIdx,:) = setDescriptors;
                insertIdx = insertIdx+numToKeep;
            end

        end

        %------------------------------------------------------------------
        function [descriptors, scores] = extractDescriptorsFromImageSet(this, imgSets, params)
            printer = vision.internal.MessagePrinter.configure(params.Verbose);

            numSets = numel(imgSets);

            % Extract descriptors that will be used to create visual vocabulary
            descriptors = cell(1, numSets);
            scores      = cell(1, numSets);

            for categoryIndex=1:numSets

                printer.printMessageNoReturn('vision:bagOfFeatures:extractingFeatures',imgSets(categoryIndex).Count, categoryIndex);

                [descriptors{categoryIndex}, ...
                 scores{categoryIndex}] = this.extractDescriptorsFromSet(imgSets(categoryIndex), params);

                printer.printMessage('vision:bagOfFeatures:extractingFeaturesDone',...
                                     size(descriptors{categoryIndex},1));
            end
            printer.linebreak;
        end

        %------------------------------------------------------------------
        function [groupedDescriptors, groupedScores] = extractDescriptorsFromImageDS(this, imds, params)

            printer = vision.internal.MessagePrinter.configure(params.Verbose);

            numImages = numel(imds.Files);

            descriptors = cell(numImages,1);
            scores = cell(numImages,1);

            if isempty(imds.Labels)
                categoryIndices = repelem(1,numImages);
            else

                % Use categorical to get category indices for grouping
                % descriptors and scores.
                c = categorical(imds.Labels);

                % check if any of the labels were left undefined, which can
                % happen if files are added incrementally to the image
                % datastore.
                undefined = isundefined(c);
                if all(undefined)
                    % manually set category index
                    categoryIndices = repelem(1,numImages);

                elseif any(undefined)
                    % Disallow partially undefined labels
                    error(message('vision:bagOfFeatures:labelsUndefined'))

                else
                    categoryIndices = double(c);
                end

            end

            printer.printMessageNoReturn('vision:bagOfFeatures:extractingFeaturesDS', numImages);

            if params.UseParallel
                parfor i = 1:numImages
                    img = imds.readimage(i); %#ok<PFBNS>

                    [descriptors{i}, scores{i}] = ...
                        this.Extractor(img); %#ok<PFBNS>
                end
            else
                for i = 1:numImages
                    img = imds.readimage(i); % read in an image from the set

                    [descriptors{i}, scores{i}] = ...
                        this.Extractor(img);
                end
            end

            numFeatures = sum(cellfun(@(x)size(x,1),descriptors));

            printer.printMessage(...
                'vision:bagOfFeatures:extractingFeaturesDone', numFeatures);
            printer.linebreak;

            numCategories      = bagOfFeatures.getNumCategories(imds);
            groupedDescriptors = cell(numCategories,1);
            groupedScores      = cell(numCategories,1);

            % Group descriptors by labels.
            for i = 1:numImages
                groupedDescriptors{categoryIndices(i)} = [groupedDescriptors{categoryIndices(i)}; descriptors{i}];
                groupedScores{categoryIndices(i)} = [groupedScores{categoryIndices(i)}; scores{i}];

                % clear memory to avoid extra memory usage
                descriptors{i} = [];
                scores{i} = [];
            end

        end

        %------------------------------------------------------------------
        function [descriptors, scores] = extractDescriptors(this, imgSets, params, printer)

            printer.linebreak;
            printer.printMessage('vision:bagOfFeatures:createBagTitle');
            printer.print('-------------------------\n');

            bagOfFeatures.printImageSetDescription(printer,imgSets);

            this.printPointSelectionInfo(printer);

            [descriptors, scores] = extractDescriptorsFromImageDS(this, imgSets, params);
        end

        %------------------------------------------------------------------
        % Encode a scalar image set. Use parfor if requested.
        %------------------------------------------------------------------
        function [features, varargout] = encodeScalarImageSet(this, imgSet, params)

            validateattributes(imgSet,{'imageSet','matlab.io.datastore.ImageDatastore'},{'scalar'},mfilename);

            numImages = numel(imgSet.Files);

            features = bagOfFeatures.allocateFeatureVector(numImages, this.NumVisualWords, params.SparseOutput);
            words    = bagOfFeatures.allocateVisualWords(numImages);

            numVarargout = nargout-1;

            if params.UseParallel
                if numVarargout == 1
                    % Invoke 2 output syntax because of parfor limitations
                    % with varargout indexing.

                    parfor j = 1:numImages
                        img = imgSet.readimage(j); %#ok<PFBNS>
                        [features(j,:), words(j)]  = this.encodeSingleImage(img); %#ok<PFBNS>
                    end

                    varargout{1} = words;
                else
                    parfor j = 1:numImages
                        img = imgSet.readimage(j); %#ok<PFBNS>
                        features(j,:)  = this.encodeSingleImage(img); %#ok<PFBNS>
                    end
                end
            else % do not use parfor
                if numVarargout == 1

                    for j = 1:numImages
                        img = imgSet.readimage(j);
                        [features(j,:), words(j)]  = this.encodeSingleImage(img);
                    end

                    varargout{1} = words;
                else
                    for j = 1:numImages
                        img = imgSet.readimage(j);
                        features(j,:)  = this.encodeSingleImage(img);
                    end
                end
            end

        end
        %------------------------------------------------------------------
        % This routine computes a histogram of word occurrences for a given
        % input image.  It turns the input image into a feature vector
        %------------------------------------------------------------------
        function [featureVector, varargout] = encodeSingleImage(this, img)

            if nargout == 2
                [features,~,locations] = this.Extractor(img);
                [featureVector, varargout{1}] = this.Encoder.encode(features, locations);
            else
                features = this.Extractor(img);
                [featureVector] = this.Encoder.encode(features);
            end

        end

        %------------------------------------------------------------------
        function vocabulary = createVocabulary(this, descriptors)

            numDescriptors = size(descriptors, 1);

            desiredVocabularySize = this.VocabularyBranches^this.VocabularyLevels;

            if this.VocabularyLevels == 1
                % Check whether we have enough data to build the desired
                % vocabulary. For a vocabulary hierarchy with one level, we
                % can adjust the user's desired vocabulary size to one that
                % fits the data.
                K = min(numDescriptors, desiredVocabularySize);

                if K == 0
                    error(message('vision:bagOfFeatures:notEnoughFeatures',...
                                  string(desiredVocabularySize)))
                end

                if numDescriptors < desiredVocabularySize
                    warning(message('vision:bagOfFeatures:reducingVocabSize', ...
                                    K, string(desiredVocabularySize)));

                    % Update the branching factor in the TreeProperties.
                    this.VocabularyBuilder.BranchingFactor = K;

                end

            else % More than one level.

                if numDescriptors < desiredVocabularySize
                    % Not enough data to build the a vocabulary of the
                    % desired size. For a hierarchical tree, the number of
                    % levels or the branching factor needs to be reduced.
                    % Issue an error to let the user decide.
                    error(message('vision:bagOfFeatures:notEnoughFeatures',...
                                  string(desiredVocabularySize)));
                end

            end

            % Build the vocabulary.
            vocabulary = this.VocabularyBuilder.create(descriptors);

            % Update the vocabulary size based on what we able to learn
            % from the data.
            this.NumVisualWords = numVisualWords(vocabulary);

            if this.VocabularyLevels > 1 && this.NumVisualWords < desiredVocabularySize
                warning(message('vision:bagOfFeatures:reducingVocabTreeSize',...
                                string(this.NumVisualWords)));
            end

            % Update the number of vocabulary levels and branching factor.
            % Given the potential to have unbalanced trees, the hierarchy
            % is reported using maximum values.
            this.TreeProperties = [maxNumLevels(vocabulary) ...
                                   maxBranchingFactor(vocabulary)];
        end

        %------------------------------------------------------------------
        % This function grabs all descriptors from the images contained in
        % imageSet. Note that detection could be done first, followed by
        % selecting strongest detections and finally extraction of
        % descriptors. That would reduce number of extractions, but would
        % require to read the images twice. Which is less expensive?
        %------------------------------------------------------------------
        function [descriptors, scores] = extractDescriptorsFromSet(this, imgSet, params)

            descriptors = [];
            scores      = [];
            numImages   = imgSet.Count;
            if params.UseParallel
                parfor i = 1:numImages
                    img = imgSet.readimage(i); %#ok<PFBNS>

                    [tempDescriptors, tempScores] = ...
                        this.Extractor(img); %#ok<PFBNS>
                    descriptors = [descriptors; tempDescriptors];
                    scores = [scores; tempScores];
                end
            else
                for i = 1:numImages
                    img = imgSet.readimage(i); % read in an image from the set

                    [tempDescriptors, tempScores] = ...
                        this.Extractor(img);
                    descriptors = [descriptors; tempDescriptors]; %#ok<AGROW>
                    scores = [scores; tempScores]; %#ok<AGROW>
                end
            end

        end

        %------------------------------------------------------------------
        function points = determineExtractionPoints(this,grayImage)

            if strcmpi(this.PointSelection, 'Grid')
                points = this.createSURFPointsGrid(grayImage);
            else
                points = detectSURFFeatures(grayImage, 'MetricThreshold', 800);
            end
        end

        %------------------------------------------------------------------
        function points = createSURFPointsGrid(this, grayImage)

            startIdx = round(this.GridStep/2);
            stopIdx  = fliplr(size(grayImage));

            rangeX = startIdx(1):this.GridStep(1):stopIdx(1);
            rangeY = startIdx(2):this.GridStep(2):stopIdx(2);

            if isempty(rangeX) || isempty(rangeY)

                warning(message('vision:bagOfFeatures:noGridPoints'));
                points = SURFPoints(zeros(0,2));
            else

                [X, Y] = meshgrid(rangeX, rangeY);

                % SURFPoint scale values, specified as multiples of 1.6
                scales = single(1.6 * this.BlockWidth/32);

                % create SURFPoints object for the grid
                locations = repmat([X(:) Y(:)], numel(scales),1);
                scales    = repmat(scales, numel(X),1);

                points = SURFPoints(locations, 'Scale', scales(:));

            end

        end

        %------------------------------------------------------------------
        % Return the image descriptors and their scores.
        %
        % When the PointSelection method is 'Grid', the scores are computed
        % using the variance of the SURF descriptors. For more details,
        % refer to figure 3 in:
        %
        %   Herbert Bay, Andreas Ess, Tinne Tuytelaars, Luc Van Gool "SURF:
        %   Speeded Up Robust Features", Computer Vision and Image
        %   Understanding (CVIU), Vol. 110, No. 3, pp. 346-359, 2008
        %
        %------------------------------------------------------------------
        function [descriptors, scores, locations] = extractDescriptorsFromImage(this, image)

            if ismatrix(image) % convert color images to grayscale
                grayImage = image;
            else
                grayImage = rgb2gray(image);
            end

            points = this.determineExtractionPoints(grayImage);

            % Use upright SURF features.
            descriptors = extractFeatures(grayImage, points, 'Upright', this.Upright);

            if strcmpi(this.PointSelection,'Grid')
                scores = var(descriptors,[],2);
            else
                scores = points.Metric;
            end

            locations = points.Location;
        end

        %------------------------------------------------------------------
        function this = setParams(this, params)

            this.NumVisualWords      = params.NumVisualWords;
            this.TreeProperties      = params.TreeProperties;
            this.StrongestFeatures   = params.StrongestFeatures;
            this.GridStep            = params.GridStep;
            this.BlockWidth          = params.BlockWidth;
            this.PointSelection      = params.PointSelection;
            this.Upright             = params.Upright;

            this.UsingCustomExtractor      = params.UsingCustomExtractor;
            this.CustomExtractor           = params.CustomExtractor;
            this.ExtractorOutputsLocations = params.ExtractorOutputsLocations;

            if this.UsingCustomExtractor
                % Use the custom extractor.
                this.CustomFeatureLength = params.CustomFeatureLength;
                this.CustomFeatureType   = params.CustomFeatureType;

                % Create validation function based on feature type.
                this.ExtractorValidationFcn = vision.internal.bof.bagOfFeaturesValidation.makeExtractorValidationFcn(params.CustomFeatureType);

                if strcmp(this.CustomFeatureType,'binaryFeatures')
                    this.Extractor = @(img)this.invokeAndExtractUnderlyingBinaryFeatures(img, params.CustomExtractor);
                else
                    this.Extractor = @(img)this.invokeCustomExtractor(img, params.CustomExtractor);
                end
            else
                % Use the default SURF extractor
                this.Extractor = @(img)extractDescriptorsFromImage(this, img);
                this.CustomFeatureType = 'double';
            end
        end

        %------------------------------------------------------------------
        % Invoke the custom extractor.
        %------------------------------------------------------------------
        function [features, featureMetrics, varargout] = invokeCustomExtractor(this, img, extractor)

            [features, featureMetrics, varargout{1:nargout-2}] = extractor(img);

            featureLength = this.ExtractorValidationFcn(features, featureMetrics, varargout{:});

            % Error if the feature length changes from the one we have
            % cached.
            if this.CustomFeatureLength ~= featureLength
                error(message('vision:bagOfFeatures:customInvalidFeatureLength', ...
                              this.CustomFeatureLength, featureLength));
            end

            % cast custom features and metric to single for clustering.
            if ~strcmp(this.CustomFeatureType,'binaryFeatures')
                features = single(features);
            end

            featureMetrics = single(featureMetrics);

            if nargout > 2
                % cast location data to single
                varargout{1} = single(varargout{1});
            end
        end

        %------------------------------------------------------------------
        % Invoke custom extractor and return underlying binary features as
        % a numeric matrix. This simplifies collecting features across
        % images.
        %------------------------------------------------------------------
        function  [features, featureMetrics, varargout] = ...
                invokeAndExtractUnderlyingBinaryFeatures(this, img, extractor)
            [features, featureMetrics, varargout{1:nargout-2}] = this.invokeCustomExtractor(img,extractor);
            features = features.Features;
        end

        %------------------------------------------------------------------
        function printPointSelectionInfo(this, printer)

            if this.UsingCustomExtractor
                str = func2str(this.CustomExtractor);
                if exist(str,'file')
                    % create hyperlink when extractor can be opened in
                    % editor. Otherwise print as normal string.
                    str = printer.makeHyperlink(str,sprintf('edit %s',str));
                end
                printer.printMessage('vision:bagOfFeatures:customExtractionInfo', str);
            else
                printer.printMessage('vision:bagOfFeatures:extractionInfo1', this.PointSelection);
                printer.printMessage('vision:bagOfFeatures:extractionInfo2');

                if strcmpi(this.PointSelection,'Grid')
                    gs = deblank(sprintf('%d ', this.GridStep));
                    bw = deblank(sprintf('%d ', this.BlockWidth));

                    printer.printMessage('vision:bagOfFeatures:gridInfo',gs,bw);
                else
                    cmd = 'doc(''detectSURFFeatures'')';
                    cmdstr = printer.makeHyperlink('detectSURFFeatures',cmd);

                    printer.printMessage('vision:bagOfFeatures:detectorInfo',...
                                         cmdstr);
                end
            end
            printer.linebreak;
        end

        %------------------------------------------------------------------
        function printFeatureBalancingMessage(this, printer, numFeatures, setIdx, numToKeep)
        % For the Grid method, print balancing message when the number
        % of features across the image sets different. Always print the
        % message for the Detector method.

            setsHaveSameNumFeatures = all(numFeatures(1) == numFeatures);

            if ~(strcmpi(this.PointSelection, 'Grid') && setsHaveSameNumFeatures)
                printer.printMessage('vision:bagOfFeatures:balanceNumFeatures');
                printer.printMessage('vision:bagOfFeatures:minNumFeatures',...
                                     setIdx, numToKeep);
                printer.printMessage('vision:bagOfFeatures:minNumFeaturesInOthers',...
                                     numToKeep).linebreak;
            end

        end

        %------------------------------------------------------------------
        function initializeEncoder(this, vocabulary)
            isBinaryFeature = strcmp(this.CustomFeatureType,'binaryFeatures');
            if isBinaryFeature
                encoderFcn = @(x,y)vision.internal.bof.EncoderBinaryFeatures(x,y);
            else
                encoderFcn = @(x,y)vision.internal.bof.EncoderNumericFeaturesKDTree(x,y,rng);
            end
            this.Encoder = vision.internal.bof.EncoderVocabularyTree(...
                vocabulary, encoderFcn);
        end

        %------------------------------------------------------------------
        function initializeVocabularyBuilder(this, params)

        % Choose the a vocabulary build and encoder based on the feature type.
            if strcmp(this.CustomFeatureType,'binaryFeatures')
                vocabBuilder = vision.internal.bof.VocabularyBuilderKMedians;
                createEncoderFcn = @vision.internal.bof.EncoderBinaryFeatures;
            else
                vocabBuilder = vision.internal.bof.VocabularyBuilderApproximateKMeans;
                createEncoderFcn = @(x,y)vision.internal.bof.EncoderNumericFeaturesKDTree(x,y,rng);
            end

            % Set parameters for vocabulary creation approximate
            % K-means. Literal parameter values are tuned for robust
            % results across a variety of data sets.
            vocabBuilder.NumTrials     = 1;
            vocabBuilder.MaxIterations = 100;
            vocabBuilder.Threshold     = 0.0001;
            vocabBuilder.UseParallel   = params.UseParallel;

            % Create a hierarchical vocabulary builder.
            this.VocabularyBuilder = vision.internal.bof.VocabularyBuilderHierarchicalClustering(...
                vocabBuilder, createEncoderFcn, ...
                params.TreeProperties(1), ... % number of levels
                params.TreeProperties(2), ... % branching factor
                params.Verbose);
        end

    end

    %======================================================================
    % Custom display methods
    %======================================================================
    methods (Access = protected)
        function propgrp = getPropertyGroups(this)
        % Properties that are not relevant for custom feature
        % extraction are not displayed.

            propgrp = getPropertyGroups@matlab.mixin.CustomDisplay(this);

            if this.UsingCustomExtractor
                % Hide grid and detector properties for custom extractor
                propgrp.PropertyList = rmfield(propgrp.PropertyList, ...
                                               {'PointSelection','GridStep','BlockWidth','Upright'});
            else
                % Hide custom extractor
                propgrp.PropertyList = rmfield(propgrp.PropertyList, ...
                                               {'CustomExtractor'});

                if strcmpi(this.PointSelection,'Detector')
                    % Hide grid related properties
                    propgrp.PropertyList = rmfield(propgrp.PropertyList, ...
                                                   {'GridStep','BlockWidth'});
                end
            end
        end
    end

    %======================================================================
    methods (Hidden, Static, Access = protected)

        %------------------------------------------------------------------
        % Returns default object settings
        %------------------------------------------------------------------
        function d = getDefaultSettings
            d.NumVisualWords            = 500;
            d.TreeProperties            = [1 500];
            d.StrongestFeatures         = 0.8;
            d.PointSelection            = 'Grid';
            d.GridStep                  = [8 8];
            d.BlockWidth                = [32 64 96 128];
            d.Verbose                   = true;
            d.UseParallel               = vision.internal.useParallelPreference();
            d.CustomExtractor           = @(x)[]; % no-op function
            d.UsingCustomExtractor      = false;
            d.CustomFeatureLength       = 0;
            d.CustomFeatureType         = 'double';
            d.Upright                   = true;
            d.ExtractorOutputsLocations = true;
        end

        %------------------------------------------------------------------
        function checkImageSet(imgSet, name)
            varName = 'imds';

            validateattributes(imgSet,...
                               {'matlab.io.datastore.ImageDatastore'}, ...
                               {'nonempty'},...
                               name, varName);
        end

        %------------------------------------------------------------------
        function checkCustomExtractorOnLoad(func)
        % check if the function is available on load. otherwise
        % issue a warning.
            try
                [~] = nargin(func); % errors if function not found
            catch
                str = func2str(func);
                warning(message('vision:bagOfFeatures:customExtractorMissingOnLoad',...
                                str,str));
            end
        end

        %------------------------------------------------------------------
        function [tf, ps] = checkAndAssignPointSelection(pointSelection)

            ps = validatestring(pointSelection,...
                                bagOfFeatures.ValidPointSelectionOptions, mfilename, 'PointSelection');

            tf = true;
        end

        %------------------------------------------------------------------
        function [features, metrics] = previewCustomExtractorOutput(imgSet, extractor)
        % try the custom extractor on 1 image.
            img = imgSet.readimage(1);

            [features, metrics] = extractor(img);
        end

        %------------------------------------------------------------------
        function n = getNumCategories(imgSetOrDs)
            n = max(1, height(countEachLabel(imgSetOrDs)));
        end

        %------------------------------------------------------------------
        function cats = getCategoryStrings(imgSetOrDs)

            tbl = countEachLabel(imgSetOrDs);
            if isempty(tbl)
                cats = cellstr('');
            else
                cats = tbl{:,1};

                cats = categorical(cats);

                cats = cellstr(cats);
            end

        end

        %------------------------------------------------------------------
        function [imgSets, params] = parseInputs(varargin)

            d = bagOfFeatures.getDefaultSettings;

            parser = inputParser;
            parser.addRequired('imds', @(x)bagOfFeatures.checkImageSet(x,mfilename));

            parser.addParameter('VocabularySize',      [], @vision.internal.bof.bagOfFeaturesValidation.checkVocabularySize);
            parser.addParameter('TreeProperties', [1 500], @vision.internal.bof.bagOfFeaturesValidation.checkTreeProperties);
            parser.addParameter('StrongestFeatures', d.StrongestFeatures, @vision.internal.bof.bagOfFeaturesValidation.checkStrongestFeatures);
            parser.addParameter('PointSelection',    d.PointSelection,    @bagOfFeatures.checkAndAssignPointSelection);
            parser.addParameter('GridStep',          d.GridStep,          @vision.internal.bof.bagOfFeaturesValidation.checkGridStep);
            parser.addParameter('BlockWidth',        d.BlockWidth,        @vision.internal.bof.bagOfFeaturesValidation.checkBlockWidth);
            parser.addParameter('Verbose',           d.Verbose,           @(x)vision.internal.inputValidation.validateLogical(x,'Verbose'));
            parser.addParameter('UseParallel',       vision.internal.useParallelPreference());
            parser.addParameter('CustomExtractor',   d.CustomExtractor);
            parser.addParameter('Upright',           d.Upright,           @(x)vision.internal.inputValidation.validateLogical(x,'Upright'));

            % Parse input
            parser.parse(varargin{:});

            imgSets = parser.Results.imds;

            useParallel = vision.internal.inputValidation.validateUseParallel(parser.Results.UseParallel);

            % Cache whether extractor outputs locations
            params.ExtractorOutputsLocations = vision.internal.bof.bagOfFeaturesValidation.checkCustomExtractor(parser.Results.CustomExtractor);

            % Set property values to the correct type and format.
            params.TreeProperties    = double(parser.Results.TreeProperties);
            params.StrongestFeatures = double(parser.Results.StrongestFeatures);
            params.GridStep          = double(parser.Results.GridStep(:)');
            params.BlockWidth        = double(parser.Results.BlockWidth(:)');
            params.Verbose           = logical(parser.Results.Verbose);
            params.UseParallel       = logical(useParallel);
            params.CustomExtractor   = parser.Results.CustomExtractor;
            params.Upright           = logical(parser.Results.Upright);

            wasSpecified = @(name)(~any(strcmp(parser.UsingDefaults,name)));

            % Manage the transition from VocabularySize to
            % TreeProperties for code that may have set VocabularySize
            % directly.
            if wasSpecified('VocabularySize')
                wasTreePropertiesSpecified = wasSpecified('TreeProperties');
                if wasTreePropertiesSpecified
                    % VocabularySize and TreeProperties cannot be
                    % specified together.
                    error(message('vision:bagOfFeatures:VocabularySizeAndHierarchyIncompatible'));
                else
                    % Update TreeProperties to use one level with a
                    % branching factor equal to VocabularySize.
                    params.TreeProperties = [1 double(parser.Results.VocabularySize);];
                end
            end

            % Set value of NumVisualWords based on TreeProperties. The
            % value set here may change based on the outcome of the
            % clustering step in createVocabulary.
            params.NumVisualWords = params.TreeProperties(2)^params.TreeProperties(1);

            if wasSpecified('CustomExtractor')
                params.UsingCustomExtractor = true;

                [features, metrics] = bagOfFeatures.previewCustomExtractorOutput(imgSets, params.CustomExtractor);

                [featureLength, featureType, featureValFcn] = vision.internal.bof.bagOfFeaturesValidation.validateCustomExtractor(features, metrics);

                % Cache the feature length for size checks while invoking
                % the custom extractor later on.
                params.CustomFeatureLength    = featureLength;
                params.CustomFeatureType      = featureType;
                params.ExtractorValidationFcn = featureValFcn;
            else
                params.UsingCustomExtractor = d.UsingCustomExtractor;
                params.CustomFeatureType = d.CustomFeatureType;
            end

            wasPointSelectionSpecified = wasSpecified('PointSelection');
            wasGridStepSpecified       = wasSpecified('GridStep');
            wasBlockWidthSpecified     = wasSpecified('BlockWidth');
            wasUprightSpecified        = wasSpecified('Upright');

            if params.UsingCustomExtractor
                if wasPointSelectionSpecified || wasGridStepSpecified ...
                        || wasBlockWidthSpecified || wasUprightSpecified
                    warning(message('vision:bagOfFeatures:customInvalidParamCombo'));
                end
            end

            % Scalar expand grid step
            if isscalar(params.GridStep)
                params.GridStep = [params.GridStep, params.GridStep];
            end

            % Handle partial strings
            [~, params.PointSelection] = ...
                bagOfFeatures.checkAndAssignPointSelection(parser.Results.PointSelection);

            % Warn about ignored options
            if strcmp(params.PointSelection, 'Detector')
                if wasGridStepSpecified || wasBlockWidthSpecified
                    warning(message('vision:bagOfFeatures:paramsIgnored'));
                end
            end
        end

        %------------------------------------------------------------------
        function validateEncodeInput(in)
            validateattributes(in, ...
                               {'numeric','imageSet','matlab.io.datastore.ImageDatastore'},...
                               {},'encode');

            if isnumeric(in)
                if ~isempty(in) % allow empty inputs
                    vision.internal.inputValidation.validateImage(in,'I');
                end
            end
        end

        %------------------------------------------------------------------
        function params = parseEncodeInputs(varargin)

            defaults = bagOfFeatures.getDefaultSettings;

            parser = inputParser;
            parser.addRequired('imds');
            parser.addParameter('Normalization', 'L2');
            parser.addParameter('Verbose', defaults.Verbose);
            parser.addParameter('UseParallel', defaults.UseParallel);
            parser.addParameter('SparseOutput', false, @(x)vision.internal.inputValidation.validateLogical(x,'SparseOutput'));

            parser.parse(varargin{:});

            bagOfFeatures.validateEncodeInput(parser.Results.imds);

            % validate params
            str = validatestring(parser.Results.Normalization,{'none','L2'},mfilename);

            vision.internal.inputValidation.validateLogical(parser.Results.Verbose, 'Verbose');

            useParallel = vision.internal.inputValidation.validateUseParallel(parser.Results.UseParallel);

            % assign user data
            params.Normalization = str;
            params.Verbose       = logical(parser.Results.Verbose);
            params.UseParallel   = logical(useParallel);
            params.SparseOutput  = logical(parser.Results.SparseOutput);
        end

        %------------------------------------------------------------------
        function printImageSetDescription(printer,imgSets)
            cats = bagOfFeatures.getCategoryStrings(imgSets);

            if numel(cats) == 1 && isempty(cats{1})
                % Do not print if has single undefined category
                printer.linebreak;
                return;
            end

            for i = 1:numel(cats)
                printer.printMessage('vision:bagOfFeatures:imageSetDescription',i,cats{i});
            end

        end

        %------------------------------------------------------------------
        function words = allocateVisualWords(n)
            if n > 0
                words(n,1) = vision.internal.visualWords();
            else
                words = vision.internal.visualWords.empty(0,1);
            end
        end

        %------------------------------------------------------------------
        function featureVector = allocateFeatureVector(m,n,isSparse)

        % Determine type for allocation
            if isSparse
                prototype = sparse(0);
            else
                prototype = single(0);
            end

            featureVector = zeros(m,n,'like',prototype);
        end

    end % end of Static methods
        %======================================================================
    methods (Static)
        function createExtractorTemplate()

        % Read in template code. Use full path to avoid local versions
        % of the file from being read.
            example = fullfile(toolboxdir('vision'),...
                               'visionutilities','exampleBagOfFeaturesExtractor.m');
            fid = fopen(example);
            contents = fread(fid,'*char');
            fclose(fid);

            % Open template code in an untitled file in the editor
            editorDoc = matlab.desktop.editor.newDocument(contents');

            functionName = editorDoc.Filename;

            % Change the function name to the name of the untitled file
            contents = regexprep(editorDoc.Text,...
                                 'exampleBagOfFeaturesExtractor', functionName,'once');

            editorDoc.Text = contents;
            editorDoc.smartIndentContents;
            editorDoc.goToLine(1);
        end
    end
    %======================================================================
    methods (Hidden, Static)
        % -----------------------------------------------------------------
        function this = loadobj(s)

            this = bagOfFeatures();

            if all(isfield(s,{'CustomExtractor', 'Upright', 'ExtractorOutputsLocations'})) % added in R2015a

                bagOfFeatures.checkCustomExtractorOnLoad(s.CustomExtractor);

            else
                % set to defaults if loading old version of object

                defaults = bagOfFeatures.getDefaultSettings();

                s.UsingCustomExtractor      = defaults.UsingCustomExtractor;
                s.CustomExtractor           = defaults.CustomExtractor;
                s.CustomFeatureLength       = defaults.CustomFeatureLength;
                s.CustomFeatureType         = defaults.CustomFeatureType;
                s.Upright                   = defaults.Upright;
                s.ExtractorOutputsLocations = defaults.ExtractorOutputsLocations;
            end

            % Manage encoder creation for BoF saved in previous releases.
            objectIsSavedBeforeR2021b = ~isfield(s,'Version');
            if objectIsSavedBeforeR2021b

                % Add feature type information. Prior to 21b, only numeric
                % features were supported.
                s.CustomFeatureType = 'double';

                % Add vocabulary tree properties. Prior to 21b, only flat
                % hierarchies were supported.
                s.TreeProperties = [1 s.VocabularySize];
                s.NumVisualWords = s.VocabularySize;

                % Prior to 21b, the encoder was part of bagOfFeatures.
                % As of 21b, the encoder is encapsulated within an object
                % inside of bagOfFeatures using composition. Create the
                % encoder object using information saved from prior
                % releases.

                objectSavedPriorToR2015b = ~isfield(s,'KDTreeIndexState');
                if objectSavedPriorToR2015b
                    % Loading object prior to R2015b. Vocabulary must be
                    % transposed into row-major format.
                    vocabulary = s.Vocabulary';

                    % Prior to 15b the random state was not serialized. Use
                    % the current state.
                    randState = rng;
                else
                    vocabulary = s.Vocabulary;
                    randState  = s.KDTreeIndexState;
                end

                % Create the encoder.
                encoderFcn = @(x,y)vision.internal.bof.EncoderNumericFeaturesKDTree(x,y,0);
                vocabTree = vision.internal.bof.VocabularyTree();
                vocabTree.Words = vocabulary;
                this.Encoder = vision.internal.bof.EncoderVocabularyTree(vocabTree, encoderFcn);
            else
                % Load encoder if not empty. The Encoder may be empty when
                % the bagOfFeatures is getting created using parallel
                % workers.
                if ~isempty(s.Encoder)
                    % Load object using loadobj().
                    this.Encoder = vision.internal.bof.EncoderVocabularyTree.loadobj(s.Encoder);
                end
            end

            % Assign saved property values.
            this = this.setParams(s);
        end

    end
    methods  (Access = public, Static, Hidden)
        %------------------------------------------------------------------
        function codegenClassName = matlabCodegenRedirect(~)
            codegenClassName = 'vision.internal.codegen.bof.bagOfFeatures';
        end
    end
end

% LocalWords:  imds visiondata Subfolders foldernames readimage Csurka Lixin Jutta Willamowski
% LocalWords:  Keypoints ECCV KD grayscale truecolor Ess Tinne Tuytelaars Luc Gool Speeded CVIU func
% LocalWords:  visionutilities Nister Stewenius CVPR
