classdef invertedImageIndex < vision.internal.EnforceScalarHandle   

%   Copyright 2014-2023 The MathWorks, Inc.

%#codegen
    
    % ---------------------------------------------------------------------
    properties(GetAccess = public, SetAccess = protected)                
        % ImageLocation - Cell array defining indexed image locations
        ImageLocation
        
        % ImageWords - A vector of visualWords objects for each indexed image.
        ImageWords
        
        % WordFrequency - A vector containing the percentage of images in
        %                 which each visual word occurs. This is analogous
        %                 to document frequency in text retrieval
        %                 applications.
        WordFrequency
        
        % BagOfFeatures - The bagOfFeatures object used in the index.
        BagOfFeatures
        
        % ImageID - A vector of integers that uniquely identify indexed images.
        ImageID
    end
    
    % ---------------------------------------------------------------------
    properties        
        % MatchThreshold - Specifies the percentage of similar words
        %                  required between a query and a potential image
        %                  match. Lower this threshold to obtain more
        %                  search results at the cost of additional
        %                  computation.
        %
        %                  Default: 0.01
        MatchThreshold        
        
        % WordFrequencyRange   Specify the word frequency range,
        %                      [lower upper], as a percentage. Use the word
        %                      frequency range to ignore words that are
        %                      very common or very rare within the image
        %                      index. These words are often due to
        %                      repeated patterns or outliers and may reduce
        %                      search accuracy.
        %
        %                      Default: [0.01 0.9]
        WordFrequencyRange
    end
        
    properties(Access = private, Dependent, Hidden)
        NumImages   
    end
    
    % ---------------------------------------------------------------------
    properties(Hidden, Access = protected)        
        WordHistograms        
        InverseDocumentFrequency
        NumImagesPerWord
        WordsPerImage   
        SaveLocations
    end

    % ---------------------------------------------------------------------
    methods(Static)
        function props = matlabCodegenNontunableProperties(~)
        % Used for code generation
            props = {'SaveLocations'};
        end
    end

    % =====================================================================
    methods
        
        % -----------------------------------------------------------------
        function this = invertedImageIndex(bag, varargin)
            
            % Set parameter defaults.
            defaults = invertedImageIndex.getParameterDefaults();
            
            this.MatchThreshold     = defaults.MatchThreshold;
            this.WordFrequencyRange = defaults.WordFrequencyRange;
            
            saveFeatureLocations = invertedImageIndex.parseInputs(bag, varargin{:});
            
            this.BagOfFeatures = bag;
            this.SaveLocations = saveFeatureLocations;

            this.ImageID = zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'uint32');

            coder.varsize('location');
            location = coder.nullcopy(cell(coder.ignoreConst(1), 1));
            location = {char(zeros(coder.ignoreConst(0), coder.ignoreConst(0)))};
            this.ImageLocation = location;
            if isSimMode()
                this.WordHistograms = sparse([]);
                this.NumImagesPerWord = sparse([]);
                this.WordFrequency = sparse([]);
            else
                % Initialize properties
                coder.varsize('initialWordsData');
                initialWordsData = sparse(zeros(coder.ignoreConst(0), coder.ignoreConst(0)));
                this.WordHistograms = initialWordsData;
                this.NumImagesPerWord = initialWordsData;
                this.WordFrequency = initialWordsData;

                coder.varsize('iWords');
                iWords = invertedImageIndex.makeEmptyVisualWords();
                this.ImageWords = iWords;
                this.WordsPerImage = zeros(coder.ignoreConst(0), 1);
            end
        end                       
        
        % -----------------------------------------------------------------
        function [imageIds, varargout] = search(this, query, varargin)                                    
            
            nargoutchk(1,3);

            params = invertedImageIndex.parseSearchInputs(query,varargin{:});

            if this.SaveLocations
                if isSimMode()
                    [queryHist, queryVisualWords] = this.BagOfFeatures.encode(query,'Normalization','none','SparseOutput',true, 'Verbose', false);
                else
                    [queryHist, queryVisualWords] = this.BagOfFeatures.encode(query,'Normalization','none','SparseOutput',true);
                end
            else
                if isSimMode()
                    queryHist = this.BagOfFeatures.encode(query,'Normalization','none','SparseOutput',true, 'Verbose', false);
                    queryVisualWords = vision.internal.visualWords.empty();
                else
                    queryHist = this.BagOfFeatures.encode(query,'Normalization','none','SparseOutput',true);
                    queryVisualWords = invertedImageIndex.makeEmptyVisualWords();
                end
            end

            if this.NumImages == 0
                coder.internal.warning('vision:invertedImageIndex:noImages');
                imageIds = zeros(0, 1,'uint32');
                scores   = zeros(0, 1);
                queryVisualWordsOut = queryVisualWords;
            else

                [words, queryVisualWordsOut] = removeStopWords(this, queryHist, queryVisualWords, this.WordFrequencyRange);

                imageIdx = findImagesContainingWords(this, words);

                imageIdx = removeImagesWithLowWordMatches(this, imageIdx, words, this.MatchThreshold);

                scores   = computeMatchMetric(this,imageIdx,words,queryHist, params.Metric);

                [imageIdx, scores] = selectStrongest(this, imageIdx, scores, params.NumResults);

                imageIds = (this.ImageID(1, imageIdx))';
            end

            if nargout > 1
                varargout{1} = scores;
            end

            if nargout > 2
                varargout{2} = queryVisualWordsOut;
            end          
        end                                    
        
        % -----------------------------------------------------------------
        function addImages(this, imgSet, varargin)   
            % addImages(imageIndex, imds) adds the images in imds into the
            % imageIndex. imds is an ImageDatastore object that contains
            % new images to add to an existing index. Duplicate images are
            % not ignored.
            % 
            % addImages(imageIndex, I, imageId) adds an image, I, into 
            % imageIndex object with the specified image identifier, 
            % imageId. imageId is a positive integer. I can be grayscale or
            % true color.
            %
            % addImages(..., Name, Value) specifies additional name-value
            % pair arguments described below:
            %
            % 'Verbose'  Specify whether or not progress information is 
            %            displayed in the command window. Valid values are
            %            'auto' or a logical scalar. When the value is 
            %            'auto', progress information is displayed when 
            %            adding images from an imageDatastore. Otherwise, 
            %            progress information is not displayed.
            %
            %            Default: 'auto'
            %
            % Example : Add an image to imageIndex 
            % ----------------------------------------------  
            % % Define a set of images to search.
            % imageFiles = ... 
            %      {'elephant.jpg', 'cameraman.tif', ... 
            %       'peppers.png', 'saturn.png',... 
            %       'pears.png', 'stapleRemover.jpg', ... 
            %       'football.jpg', 'mandi.tif',... 
            %       'kids.tif', 'liftingbody.png', ... 
            %       'office_5.jpg', 'gantrycrane.png',... 
            %       'moon.tif', 'circuit.tif', ... 
            %       'tape.png'}; 
            % 
            % imds = imageDatastore(imageFiles); 
            % 
            % % Learn the visual vocabulary. 
            % bag = bagOfFeatures(imds);
            % 
            % % Create an image search index. 
            % imageIndex = invertedImageIndex(bag); 
            % 
            % % Load an image. 
            % I = imread('coins.png'); 
            % 
            % % Add the image into imageIndex. 
            % imageId = 1; 
            % addImages(imageIndex, I, imageId);
            %
            % See also invertedImageIndex, bagOfFeatures, imageDatastore.
            
            params  = invertedImageIndex.parseAddImagesInputs(this, imgSet, varargin{:});
            isSingleImage = isnumeric(imgSet);
            
            if isSimMode
                if this.SaveLocations
                    % Index word histograms and word location information
                    [wordHistograms, words] = this.BagOfFeatures.encode(imgSet, ...
                        'Normalization', 'None', ...
                        'SparseOutput',  true, ...
                        'Verbose',       params.Verbose,...
                        'UseParallel',   params.UseParallel);

                else
                    % Just index the word histograms
                    wordHistograms = this.BagOfFeatures.encode(imgSet, ...
                        'Normalization', 'None', ...
                        'SparseOutput',  true, ...
                        'Verbose',       params.Verbose,...
                        'UseParallel',   params.UseParallel);

                    words = [];
                end
            else
                % Check if input is a struct or numeric image
                if isSingleImage
                    img = imgSet;
                else
                    img = imgSet.Images;
                end
                if this.SaveLocations
                    % Index word histograms and word location information
                    [wordHistograms, words] = this.BagOfFeatures.encode(img, ...
                        'Normalization', 'None', ...
                        'SparseOutput',  true);
                else
                    % Just index the word histograms
                    wordHistograms = this.BagOfFeatures.encode(img, ...
                        'Normalization', 'None', ...
                        'SparseOutput',  true);

                    words =  invertedImageIndex.makeEmptyVisualWords();
                end
            end

            numImagesPerWord = sum(spones(wordHistograms), 1);
            wordsPerImage    = full(sum(wordHistograms,2));

            if isempty(this.ImageID)
                % Empty index - initialize everything
                if isSingleImage % input is a numeric image
                    this.ImageLocation  = {char(zeros(0, 0))};
                    this.ImageID        = params.imageID;
                else % imageDatastore
                    if isSimMode()
                        numImages          = numel([imgSet.Files]);
                        this.ImageLocation = imgSet.Files(:);
                    else
                        numImages          = numel(imgSet.Images);
                        this.ImageLocation = invertedImageIndex.updateImageLocation(this, imgSet.Location);
                    end
                    this.ImageID       = uint32(1: numImages);
                end
                varSizeSparseData = sparse(coder.ignoreConst(0), coder.ignoreConst(0));
                this.WordHistograms   = vertcat(varSizeSparseData, wordHistograms);
                this.NumImagesPerWord = vertcat(varSizeSparseData, numImagesPerWord);
                this.WordsPerImage    = wordsPerImage;
                this.ImageWords       = words;

            else
                if isSingleImage % a numeric image
                    this.ImageLocation  = invertedImageIndex.updateImageLocation(this, {char(zeros(0, 0))});
                    this.ImageID       = [this.ImageID, params.imageID];
                else % imageDatastore
                    if isSimMode()
                        numImages          = numel([imgSet.Files]);
                        this.ImageLocation = [this.ImageLocation; imgSet.Files(:)];
                    else
                        numImages = numel(imgSet.Images);
                        this.ImageLocation = invertedImageIndex.updateImageLocation(this, imgSet.Location);
                    end
                    maxImageID         = max(this.ImageID);
                    this.ImageID       = [this.ImageID, ((maxImageID + 1):  (maxImageID + numImages))];
                end
                this.WordHistograms   = vertcat(this.WordHistograms, wordHistograms);
                this.NumImagesPerWord = this.NumImagesPerWord + numImagesPerWord;
                this.WordsPerImage    = [this.WordsPerImage; wordsPerImage];
                this.ImageWords = [this.ImageWords; words];
            end                        

            updateIndexStatistics(this);                                                 
        end
        
        %------------------------------------------------------------------
        function addImageFeatures(this, features, imageId)   
            % addImageFeatures(imageIndex, features, imageId) adds features 
            % of one image to imageIndex with an image ID, specified 
            % as imageId. imageId is a positive integer. features is an 
            % M-by-N matrix from one image that contains M feature 
            % vectors. Each descriptor is of length N. features can also be 
            % a binaryFeatures object. 
            % 
            % Notes: 
            % ------
            % - To add image features to the image index using 
            %   addImageFeatures, you must set 'SaveFeatureLocations' to 
            %   false. 
            % 
            % Example : Add image features of one image to image index 
            % ----------------------------------------------------------
            % % Define a set of images to search.
            % imageFiles = ... 
            %      {'elephant.jpg', 'cameraman.tif', ... 
            %       'peppers.png', 'saturn.png',... 
            %       'pears.png', 'stapleRemover.jpg', ... 
            %       'football.jpg', 'mandi.tif',... 
            %       'kids.tif', 'liftingbody.png', ... 
            %       'office_5.jpg', 'gantrycrane.png',... 
            %       'moon.tif', 'circuit.tif', ... 
            %       'tape.png'}; 
            % 
            % imds = imageDatastore(imageFiles); 
            % 
            % % Learn the visual vocabulary.
            % bag = bagOfFeatures(imds);
            % 
            % % Create an image search index. 
            % imageIndex = invertedImageIndex(bag, 'SaveFeatureLocations', false); 
            % 
            % % Load an image and extract features from it. 
            % I = imread('coins.png');
            %
            % % Detect SURF points from the image.
            % points = detectSURFFeatures(I);
            %
            % % Detect features from the SURF points.
            % features= extractFeatures(I,points); 
            %
            % % Add the image features into imageIndex. 
            % imageId = 1; 
            % addImageFeatures(imageIndex, features, imageId);
            %
            % See also invertedImageIndex, bagOfFeatures, imageDatastore,
            %          extractFeatures.

            narginchk(2,3);
            
            vision.internal.inputValidation.checkFeatures(features, mfilename, 'features');
            
            imageID  = invertedImageIndex.validateImageID(this, imageId);

            coder.internal.errorIf(this.SaveLocations, 'vision:invertedImageIndex:saveFeatureLocationsMustBeFalse');
            % Just index the word histograms
            wordHistograms = this.BagOfFeatures.encodeFeatures(features);
            if isSimMode
                words = [];
            else
                words =  invertedImageIndex.makeEmptyVisualWords();
            end
            numImagesPerWord = sum(spones(double(wordHistograms)), 1);
            wordsPerImage    = full(sum(wordHistograms,2));

            if isempty(this.ImageID)           
                % Empty index - initialize everything
                this.ImageLocation  = {char(zeros(0, 0))};
                if isSimMode()
                    this.WordHistograms   = wordHistograms;
                    this.NumImagesPerWord = numImagesPerWord;
                else
                    varSizeSparseData = sparse(coder.ignoreConst(0), coder.ignoreConst(0));
                    this.WordHistograms   = vertcat(varSizeSparseData, wordHistograms);
                    this.NumImagesPerWord = vertcat(varSizeSparseData, numImagesPerWord);
                end
                this.WordsPerImage    = wordsPerImage;
                this.ImageWords       = words;
                this.ImageID          = imageID;
            else
                if isSimMode()
                    this.ImageLocation    = [this.ImageLocation; {''}];
                else
                this.ImageLocation = invertedImageIndex.updateImageLocation(this, {char(zeros(0, 0))});
                end
                this.WordHistograms   = vertcat(this.WordHistograms, wordHistograms);               
                this.NumImagesPerWord = this.NumImagesPerWord + numImagesPerWord;
                this.WordsPerImage    = [this.WordsPerImage; wordsPerImage];
                this.ImageWords       = [this.ImageWords; words];
                this.ImageID          = [this.ImageID, imageID];
                
            end

            updateIndexStatistics(this);                                                
        end
        
        % -----------------------------------------------------------------
        function removeImages(this, imageIds)
            % removeImages(imageIndex, imageIds) remove images, specified
            % by imageIds, from the image index object. imageIds is a 
            % vector of integers that correspond to the identifiers 
            % within imageIndex.ImageID.
            %
            % Example: Remove Images from image index
            % ----------------------------------------
            % % Create an image datastore.
            % imageFiles = ... 
            %      {'elephant.jpg', 'cameraman.tif', ... 
            %       'peppers.png', 'saturn.png',... 
            %       'pears.png', 'stapleRemover.jpg', ... 
            %       'football.jpg', 'mandi.tif',... 
            %       'kids.tif', 'liftingbody.png', ... 
            %       'office_5.jpg', 'gantrycrane.png',... 
            %       'moon.tif', 'circuit.tif', ... 
            %       'tape.png'}; 
            % imds = imageDatastore(imageFiles);
            %
            % % Index the image set.
            % imageIndex = indexImages(imds)
            %
            % % Remove first and third image with imagIds = [1, 3].
            % removeImages(imageIndex,[1 3]);
            %
            % See also indexImages, bagOfFeatures, imageDatastore.
            
            validateattributes(imageIds, {'numeric'}, {'nonsparse', 'vector', ...
                'integer', 'positive', 'real'}, mfilename, 'imageIds');
            
            coder.internal.errorIf(numel(unique(imageIds)) ~= numel(imageIds), ...
                    'vision:invertedImageIndex:duplicateImageId');
            
            imageIds = uint32(imageIds);
            
            foundId = ismember(imageIds, this.ImageID);
            
            if ~all(foundId)
                notFound = imageIds(~foundId);
                coder.internal.error('vision:invertedImageIndex:noImageId', notFound(1));
            end
            
            indices = ismember(this.ImageID, imageIds);
            
            % Decrement NumImagesPerWord by the counts for the images being removed
            if isSimMode()
                removedNumImagesPerWord = sum(spones(this.WordHistograms(indices,:)),1);
            else
                idIndices = this.getIndicesImagesToKeep(indices);
                removedNumImagesPerWord = sum(spones(this.WordHistograms(idIndices,:)),1);
            end
            this.NumImagesPerWord   = this.NumImagesPerWord - removedNumImagesPerWord;
            this.WordsPerImage(indices)    = [];
            this.ImageID(indices)          = [];
            
            if isSimMode()
                this.ImageLocation(indices)    = [];
                this.WordHistograms(indices,:) = [];
                if this.SaveLocations
                    this.ImageWords(indices) = [];
                end
            else
                this.ImageLocation = invertedImageIndex.removeLocation(this, idIndices);
                this.WordHistograms(idIndices,:) = 0;
                if this.SaveLocations
                    this.ImageWords =  invertedImageIndex.removeVisualWords(this, idIndices);
                end
            end

            updateIndexStatistics(this);
            
        end
                                          
    end
    
    % =====================================================================
    % Set/Get Methods
    % =====================================================================
    methods
        
        % -----------------------------------------------------------------
        function set.MatchThreshold(this,threshold)
            validateattributes(threshold, {'numeric'},...
                {'scalar','nonnegative', '<=',1,'real', 'nonsparse'},...
                mfilename,'MatchThreshold');                        
            
            % stored as double  
            this.MatchThreshold = double(threshold);          
        end
        
        % -----------------------------------------------------------------
        function set.WordFrequencyRange(this,range)
            validateattributes(range, {'numeric'},...
                {'numel',2,'nonnegative', '<=',1,'real','nonsparse','increasing'},...
                mfilename,'WordFrequencyRange');
               
            % stored as double row vector
            if isSimMode()
                this.WordFrequencyRange(1,:) = double(range);
            else
                if isrow(range)
                    this.WordFrequencyRange = double(range);
                else
                    rangeRowFormat = range';
                    this.WordFrequencyRange = double(rangeRowFormat);
                end
            end
        end
        
        % -----------------------------------------------------------------
        function n = get.NumImages(this)
            n = numel(this.ImageID);
        end
    end
    
    % =====================================================================
    methods(Access = protected, Hidden)
        % -----------------------------------------------------------------
        function updateIndexStatistics(this)                                 

            this.WordFrequency = this.NumImagesPerWord ./ this.NumImages;
            
            this.InverseDocumentFrequency = log( this.NumImages ./ (full(this.NumImagesPerWord) + eps) );
        end             
                       
        % -----------------------------------------------------------------               
        function imageIDs = findImagesContainingWords(this, words)
            % Find images that have at least 1 common word with the query
            % image. This reduces the set of image candidates due to the
            % sparse nature of the visual word histograms.
            
            [id,~]   = find(this.WordHistograms(:, words));
            imageIDs = unique(reshape(id,[],1));                        
        end
        
        % -----------------------------------------------------------------        
        function imageIDs = removeImagesWithLowWordMatches(this, imageIDs, words, threshold)
            % Remove images that do not have enough matching words. This
            % helps reduce the number of computations required to find the
            % best matches.
        
            binaryHist   = spones(this.WordHistograms(imageIDs,words));
            numMatches   = sum(binaryHist,2);
            imagesToKeep = numMatches./numel(words) >= threshold;
            if isSimMode()
                imageIDs     = imageIDs(imagesToKeep);
            else
                imKeepIndices = getIndicesImagesToKeep(this, imagesToKeep);
                imageIDs     = imageIDs(imKeepIndices);
            end
        end

        % -----------------------------------------------------------------
        function indices = getIndicesImagesToKeep(~, imagesToKeep)
            coder.varsize('indices', [Inf, 1], [1, 0]);
            indices = zeros(0, 1);
            for i=1:numel(imagesToKeep)
                if imagesToKeep(i) == 1
                    indices = [indices; i];
                end
            end
        end
        
        % -----------------------------------------------------------------
        function [words, queryWordsOut] = removeStopWords(this, queryHist, queryWords, freqRange)
                           
            stopWordFilter = this.WordFrequency >= freqRange(1) ...
                & this.WordFrequency <= freqRange(2);
            
            % remove stop words from query           
            if isSimMode()
                words = find(queryHist & stopWordFilter);
            else
                words = find(queryHist' & stopWordFilter');
            end             
                        
            if this.SaveLocations
                l = zeros(0,2,'like',queryWords.Location);
                w = zeros(0,1,'like',queryWords.WordIndex);
                for i = 1:coder.internal.indexInt(numel(words))
                    idx = queryWords.WordIndex == words(i);
                    l = [l; queryWords.Location(idx,:)]; %#ok<AGROW>
                    w = [w; queryWords.WordIndex(idx)];  %#ok<AGROW>
                end
                queryWordsOut = vision.internal.visualWords(w,l,this.BagOfFeatures.VocabularySize);
            else
                queryWordsOut = queryWords;
            end
        end
        
        % -----------------------------------------------------------------
        function [strongest, scoresSelected] = selectStrongest(~, imageIDs, scores, K)
            
            K = coder.internal.indexInt(min(numel(imageIDs), K));
            
            [sortedScores, idx] = sort(scores,'descend');
            if isSimMode()
                strongest = imageIDs(idx(1:K));
                scoresSelected = sortedScores(1:K);
            else
                scoresClass = class(scores);
                strongest = coder.nullcopy(zeros(K, 1, scoresClass));
                scoresSelected = coder.nullcopy(zeros(K, 1, scoresClass));
                for i=1:K
                    strongest(i) = imageIDs(idx(i));
                    scoresSelected(i) = sortedScores(i);
                end
            end
        end
        
        % -----------------------------------------------------------------
        function scores = computeMatchMetric(this, imageIDs, words, queryFeatures, metric)
            
            % Select index features by image ID.
            indexFeatures = this.WordHistograms(imageIDs, :);
            
            % Select words that are within the WordFrequency range.
            iFeatures = indexFeatures(:, words);
            qFeatures = queryFeatures(words);
            
            % Apply TF-IDF weighting.
            iFeatures = applyWeighting(this, iFeatures);
            qFeatures = applyWeighting(this, qFeatures);
            
            if strcmpi(metric, 'cosine') % Metric is cosine
                iFeatures = invertedImageIndex.l2NormalizeFeatures(iFeatures);
                qFeatures = invertedImageIndex.l2NormalizeFeatures(qFeatures);
                
                % cosine similarity
                scores = full(iFeatures * qFeatures');                
            else % Metric is L1
                iFeatures = invertedImageIndex.l1NormalizeFeatures(iFeatures);
                qFeatures = invertedImageIndex.l1NormalizeFeatures(qFeatures);
                if isSimMode()
                    absDifference = sum(abs(iFeatures - qFeatures), 2);
                else
                    absDifference = sum(sparse(abs(full(iFeatures) - full(qFeatures))), 2);
                end
                
                % Scaled L1 similarity
                scores = 1 - 0.5 * absDifference;
                
                % See equation (2) in reference paper: Gálvez-López, Dorian, 
                % and Juan D. Tardos. "Bags of binary words for fast place 
                % recognition in image sequences." IEEE Transactions on 
                % Robotics 28.5 (2012): 1188-1197.
            
            end
        end
        
        % -----------------------------------------------------------------
        % Apply Term Frequency-Inverse Document Frequency (TF-IDF)
        % weighting. TF and IDF weights are packed into diagonal matrices
        % for efficient computation.
        % -----------------------------------------------------------------
        function tfidf = applyWeighting(this, h)
            
            [M,N]  = size(h);
                        
            % Pack weights into sparse diagonal matrices
            if isSimMode()
                diagTermFreq   = spdiags(1./(sum(h,2)+eps), 0, M, M);
                diagInvDocFreq = spdiags(this.InverseDocumentFrequency(:), 0, N, N);
            else
                if M==0
                    coder.varsize('diagData');
                    diagData = sparse(zeros(coder.ignoreConst(0), coder.ignoreConst(0)));
                    diagTermFreq = diagData;
                else
                    diagTermFreq   = spdiags(1./(sum(h,2)+eps), 0, M, M);
                end
                if N==0
                    coder.varsize('diagInvData');
                    diagInvData = sparse(zeros(coder.ignoreConst(0), coder.ignoreConst(0)));
                    diagInvDocFreq = diagInvData;
                else
                    diagInvDocFreq = spdiags(this.InverseDocumentFrequency(:), 0, N, N);
                end
            end

            % Apply weights
            tf    = diagTermFreq * h;                                                                  
            tfidf = tf  * diagInvDocFreq;           
        end 
       
    end
       
    % =====================================================================
    methods(Static, Access = private)       
        % -----------------------------------------------------------------
        function normalizedFeatures = l2NormalizeFeatures(features)
            M = size(features, 1);
            
            if isSimMode()
                fNorm = sqrt(sum(features.^2,2));
            else
                fun = @(x) x.^2;
                featuresProd  = spfun(fun, features);
                fNorm = sqrt(sum(featuresProd,2));
            end

            fNormInv = 1./(fNorm + eps(class(features)));
            
            if isSimMode()
                fNormInv = spdiags(fNormInv(:), 0, M, M); % sparse diagonal
            else
                if M == 0
                    coder.varsize('fNormInvData');
                    fNormInvData = sparse(zeros(coder.ignoreConst(0), coder.ignoreConst(0)));
                    fNormInv = fNormInvData;
                else
                    fNormInv = spdiags(fNormInv(:), 0, M, M); % sparse diagonal
                end
            end

            normalizedFeatures = fNormInv * features; % normalize row vectors
        end
        
        % -----------------------------------------------------------------
        function normalizedFeatures = l1NormalizeFeatures(features)
            
            M = size(features, 1);
            
            fNorm = sum(abs(features),2);                        
            
            fNormInv = 1./(fNorm + eps(class(features)));
            
            if isSimMode()
                fNormInv = spdiags(fNormInv(:), 0, M, M); % sparse diagonal
            else
                if M == 0
                    coder.varsize('fNormInvData');
                    fNormInvData = sparse(zeros(coder.ignoreConst(0), coder.ignoreConst(0)));
                    fNormInv = fNormInvData;
                else
                    fNormInv = spdiags(fNormInv(:), 0, M, M); % sparse diagonal
                end
            end
            
            normalizedFeatures = fNormInv * features; % normalize row vectors
            
        end
        
        % -----------------------------------------------------------------
        function defaults = getSearchParameterDefaults()            
            defaults.NumResults = 20;
            defaults.Metric     = 'cosine';
        end
        
        % -----------------------------------------------------------------
        function params = parseSearchInputs(I, varargin)
            vision.internal.inputValidation.validateImage(I,'I');
            if isSimMode()
                params = invertedImageIndex.parseSearchInputsSim(varargin{:});
            else
                params = invertedImageIndex.parseSearchInputsCodegen(varargin{:});
            end
        end

        % -----------------------------------------------------------------
        function params = parseSearchInputsSim(paramsIn)
            arguments
                paramsIn.NumResults = 20;
                paramsIn.Metric = 'cosine';
            end
            params.NumResults = paramsIn.NumResults;
            params.Metric = paramsIn.Metric;
        end

        % -----------------------------------------------------------------
        function params = parseSearchInputsCodegen(varargin)
            coder.internal.prefer_const(varargin{:});
            defaults = invertedImageIndex.getSearchParameterDefaults();
            defaultValues = struct(...
                    'NumResults', defaults.NumResults, ...
                    'Metric', defaults.Metric);
                % Define parser mapping struct
                pvPairs = struct( ...
                    'NumResults', uint32(0), ...
                    'Metric', uint32(0));
                % Specify parser options
                poptions = struct( ...
                    'CaseSensitivity', false, ...
                    'StructExpand',    true, ...
                    'PartialMatching', true);
                % Parse PV pairs
                pstruct = coder.internal.parseParameterInputs(pvPairs, poptions, varargin{:});
                numResults = coder.internal.getParameterValue(pstruct.NumResults, ...
                    defaultValues.NumResults, varargin{:});
                params.NumResults = double(numResults);
                params.Metric = coder.internal.getParameterValue(pstruct.Metric, ...
                    defaultValues.Metric, varargin{:});
        end
        
        % -----------------------------------------------------------------
        function saveFeatureLocations = parseInputs(bag, varargin)
            coder.internal.prefer_const(varargin{:});
            
            validateattributes(bag, {'bagOfFeatures'}, {}, mfilename, 'bag');

            coder.internal.errorIf(bag.NumVisualWords == 0, ...
                'vision:bagOfFeatures:emptyBagOfFeatures');

            if isSimMode()
                params = invertedImageIndex.parseInputsSim(varargin{:});
                saveFeatureLocations = params.SaveFeatureLocations;
            else
                % Define parser mapping struct
                pvPairs = struct('SaveFeatureLocations', uint32(0));
                % Specify parser options
                poptions = struct( ...
                    'CaseSensitivity', false, ...
                    'StructExpand',    true, ...
                    'PartialMatching', true);
                % Parse PV pairs
                pstruct = coder.internal.parseParameterInputs(pvPairs, poptions, varargin{:});
                saveFeatureLocations = coder.internal.getParameterValue(pstruct.SaveFeatureLocations, ...
                    coder.internal.const(true), varargin{:});
                vision.internal.inputValidation.validateLogical(saveFeatureLocations,'SaveFeatureLocations')
            end
        end

        % -----------------------------------------------------------------
        function params = parseInputsSim(args)
            arguments
                args.SaveFeatureLocations {vision.internal.inputValidation.validateLogical(args.SaveFeatureLocations,'SaveFeatureLocations')} = true;
            end
            params.SaveFeatureLocations = logical(args.SaveFeatureLocations);
        end
        
        % -----------------------------------------------------------------
        function params = parseAddImagesInputs(imageIndex, img, varargin)

            coder.internal.prefer_const(varargin{:});
            invertedImageIndex.validateImage(img);
            
            if isnumeric(img)
                narginchk(3,7);
                params.imageID     = invertedImageIndex.validateImageID(imageIndex, varargin{1});
                if isSimMode
                    [params.Verbose, params.UseParallel] = invertedImageIndex.parseAddImagesInputsNumeric(varargin{2:end});
                end
            else
                narginchk(2,6);
                if isSimMode
                    [params.Verbose, params.UseParallel] = invertedImageIndex.parseAddImagesInputsImds(varargin{:});
                else
                    params = 0;
                end
            end
        end

        % -----------------------------------------------------------------
        function [verbose, useParallel] = parseAddImagesInputsNumeric(args)
            arguments
                args.Verbose {invertedImageIndex.validateVerbose(args.Verbose, 1)} = 'auto';  %#ok
                args.UseParallel {invertedImageIndex.validateUseParallel(args.UseParallel, 1)} = vision.internal.useParallelPreference();  %#ok
            end
            verbose = invertedImageIndex.validateVerbose(args.Verbose, 1);
            useParallel = args.UseParallel;
        end

        % -----------------------------------------------------------------
        function [verbose, useParallel] = parseAddImagesInputsImds(args)
            arguments
                args.Verbose {invertedImageIndex.validateVerbose(args.Verbose, 0)} = 'auto';  %#ok
                args.UseParallel {invertedImageIndex.validateUseParallel(args.UseParallel, 0)} = vision.internal.useParallelPreference();  %#ok
            end
            verbose = invertedImageIndex.validateVerbose(args.Verbose, 0);
            useParallel = args.UseParallel;
        end

        % -----------------------------------------------------------------
        function updatedLocation = updateImageLocation(this, location)
            % Update location cell array
            newLocation = coder.internal.indexInt(numel(location));
            if isempty(this.ImageID)
                oldLocation = coder.internal.indexInt(0);
            else
                oldLocation = coder.internal.indexInt(numel(this.ImageLocation));
            end
            updatedLocation = coder.nullcopy(cell(oldLocation+newLocation,1));
            for i=1:oldLocation
                updatedLocation{i} = this.ImageLocation{i};
            end
            for i=1:newLocation
                updatedLocation{oldLocation+i} = location{i};
            end
        end

        % -----------------------------------------------------------------
        function imageLocation = removeLocation(this, indices)
            % Remove location data of respectives images removed
            nLocation = coder.internal.indexInt(numel(this.ImageLocation));
            nIndices = coder.internal.indexInt(numel(indices));
            if nLocation <= nIndices
                imageLocation = {char(zeros(coder.ignoreConst(0), coder.ignoreConst(0)))};
            else
                newLocation = nLocation - nIndices;
                imageLocation = coder.nullcopy(cell(newLocation, 1));
                k = 1;
                for i = 1:nLocation
                    if ~ismember(i, indices)
                        imageLocation{k} = this.ImageLocation{i};
                        k = k+1;
                    end
                end
            end
        end

        % -----------------------------------------------------------------
        function imageWords = removeVisualWords(this, indices)
            % Remove visualWords data of respectives images removed
            nWords = coder.internal.indexInt(numel(this.ImageWords));
            nIndices = coder.internal.indexInt(numel(indices));
            if nWords <= nIndices
                imageWords = invertedImageIndex.makeEmptyVisualWords();
            else
                k = 1;
                imageWords = invertedImageIndex.makeEmptyVisualWords();
                for i = 1:nWords
                    if ~ismember(i, indices)
                        if k == 1
                            imageWords = this.ImageWords(i);
                        else
                            imageWords = [imageWords, this.ImageWords(i)];
                        end
                        k = k+1;
                    end
                end
            end
        end
       
        %------------------------------------------------------------------
        function validateImage(img)
            if isnumeric(img)
                vision.internal.inputValidation.validateImage(img,'I');
            else
                if isSimMode
                    validateattributes(img, ...
                        {'matlab.io.datastore.ImageDatastore'},...
                        {'scalar','nonempty'}, mfilename,'imgSet');
                    if numel(img.Files) == 0
                        error(message('vision:invertedImageIndex:emptyImageSet'));
                    end
                else
                    validateattributes(img, {'struct'}, {'nonempty'}, mfilename, 'imgSet');
                    structFields = fieldnames(img);
                    requiredStructFields = {'Images'; 'Location'};
                    % Add error Id for field missing
                    coder.internal.errorIf(~isequal(structFields, requiredStructFields), ...
                        'vision:bagOfFeatures:invalidFields', 'img');
                    images = img.Images;
                    location = img.Location;
                    validateattributes(images, {'cell'}, {'vector'}, 'invertedImageIndex', 'Images');
                    % location and images should be of same size
                    validateattributes(location, {'cell'}, {'vector', 'size', size(images)}, 'invertedImageIndex', 'Location');
                    for i=1:coder.internal.indexInt(numel(images))
                        vision.internal.inputValidation.validateImage(images{i},'images');
                        coder.internal.errorIf(isempty(location{i}), ...
                            'vision:dims:expectedNonemptyElements', 'Location');
                        % change error Id
                        coder.internal.errorIf(~isstring(location{i}) && ~ischar(location{i}), ...
                            'vision:dims:expectedNonemptyElements', 'Location');
                    end
                end
            end
        end
        
        %------------------------------------------------------------------
        function imageID = validateImageID(imageIndex, ID)
            imageID = vision.internal.inputValidation.checkViewIds(ID, true, mfilename, 'imageID' );
            hasView = vision.internal.inputValidation.checkIfHasView(imageIndex.ImageID, ID);
            coder.internal.errorIf(hasView, 'vision:invertedImageIndex:imageIdExists', ID);
        end
        
        %------------------------------------------------------------------
        function isVerbose = validateVerbose(verbose, isImg)
            if strcmpi(verbose, 'auto')
                if isImg
                    isVerbose = false;
                else
                    isVerbose = true;
                end
            else
                validateattributes(verbose, {'logical','numeric'},...
                    {'nonnan', 'scalar', 'real','nonsparse'},mfilename,'Verbose');
                isVerbose = logical(verbose);
            end
        end
        
        %------------------------------------------------------------------
        function isParallel = validateUseParallel(useParallel, isImg)
            vision.internal.inputValidation.validateLogical(useParallel,'UseParallel');
            if isImg
                if useParallel
                    warning(message('vision:imageCategoryClassifier:ignoreParallel'));
                end
                isParallel = false;
            else
                isParallel = logical(useParallel);
            end
            
        end
        
        % -----------------------------------------------------------------
        function defaults = getParameterDefaults()
            defaults.MatchThreshold       = 0.01;
            defaults.WordFrequencyRange   = [0.01 0.9];                 
            defaults.Verbose              = 'auto';
            if isSimMode()
                defaults.UseParallel          = vision.internal.useParallelPreference();
            else
                defaults.UseParallel = false;
            end
            defaults.SaveFeatureLocations = coder.internal.const(true);
        end
        % -----------------------------------------------------------------
        function emptyWords = makeEmptyVisualWords(~)
            wordIndex  = zeros(coder.ignoreConst(0), coder.ignoreConst(1), 'uint32');
            loc = zeros(coder.ignoreConst(0), coder.ignoreConst(2), 'single');
            vocabSize = double(1);

            vWords = vision.internal.visualWords(wordIndex, loc, vocabSize);
            emptyWords = repmat(vWords, 0, 0);
        end
    end
    
    % =====================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.ImageWords               = this.ImageWords;
            s.BagOfFeatures            = saveobj(this.BagOfFeatures);            
            s.ImageLocation            = this.ImageLocation;
            s.WordFrequency            = this.WordFrequency;
            s.MatchThreshold           = this.MatchThreshold;
            s.WordFrequencyRange       = this.WordFrequencyRange;
            s.WordHistograms           = this.WordHistograms;
            s.InverseDocumentFrequency = this.InverseDocumentFrequency;
            s.NumImagesPerWord         = this.NumImagesPerWord;
            s.WordsPerImage            = this.WordsPerImage;
            s.SaveLocations            = this.SaveLocations;
            s.ImageID                  = this.ImageID;
        end
    end
    
    % =====================================================================
    methods(Static,Hidden)
        function this = loadobj(s)                        
            bag  = bagOfFeatures.loadobj(s.BagOfFeatures);            
            this = invertedImageIndex(bag);
            
            objectIsSavedBeforeR2021b = ~isfield(s,'ImageID');
            if objectIsSavedBeforeR2021b
                numImages = numel(s.ImageLocation);
                this.ImageID = uint32(1:numImages);
            else
                this.ImageID = s.ImageID;
            end
            
            this.ImageWords               = s.ImageWords;            
            this.ImageLocation            = s.ImageLocation;
            this.WordFrequency            = s.WordFrequency;
            this.MatchThreshold           = s.MatchThreshold;
            this.WordFrequencyRange       = s.WordFrequencyRange;
            this.WordHistograms           = s.WordHistograms;
            this.InverseDocumentFrequency = s.InverseDocumentFrequency;
            this.NumImagesPerWord         = s.NumImagesPerWord;
            this.WordsPerImage            = s.WordsPerImage;
            this.SaveLocations            = s.SaveLocations;
        end
    end
    
end
function out = isSimMode()
    % check if simulation mode
    out = isempty(coder.target);
end
