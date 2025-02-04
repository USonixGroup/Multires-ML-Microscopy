%semanticSegmentationMetrics Semantic segmentation quality metrics
%
%   Object encapsulating semantic segmentation quality metrics for a data
%   set. Use <a href="matlab:help('evaluateSemanticSegmentation')">evaluateSemanticSegmentation</a> to create this object.
%
%   semanticSegmentationMetrics properties:
%
%      ConfusionMatrix  -  Confusion matrix summarizing the classification
%                          results for all the labeled pixels in the data set.
%
%      NormalizedConfusionMatrix  -  Confusion matrix where the counts of
%                                    predicted pixels for each class are
%                                    divided by the number of pixels known
%                                    to belong to that class.
%
%      DataSetMetrics  -  Table of up to 5 metrics aggregated over the data
%                         set: global accuracy, mean class accuracy, mean
%                         class IoU (Jaccard index), frequency-weighted
%                         mean class IoU, and mean BF score.
%
%      ClassMetrics  -  Table of up to 3 metrics computed for each class:
%                       accuracy, IoU (Jaccard index), and mean image BF
%                       score.
%
%      ImageMetrics  -  Table where each row lists up to 5 metrics for each
%                       image in the data set: global accuracy, mean class
%                       accuracy, mean class IoU (Jaccard index), frequency-
%                       weighted mean class IoU, and mean class BF score.                    
%
%   Note: 
%   -----
%   BF Score is not supported when the input to evaluateSemanticSegmenation
%   is a confusion matrix.
%
%    See also evaluateSemanticSegmentation.

%   Copyright 2017-2023 The MathWorks, Inc.

classdef semanticSegmentationMetrics
    
    properties (SetAccess = protected)
        %ConfusionMatrix Confusion matrix
        %   A square table where element (i,j) is the count of pixels known
        %   to belong to class i but predicted to belong to class j.
        ConfusionMatrix
        
        %NormalizedConfusionMatrix Normalized confusion matrix
        %   A square table where element (i,j) is the share (in [0,1]) of
        %   pixels known to belong to class i but predicted to belong to
        %   class j.
        NormalizedConfusionMatrix
        
        %DataSetMetrics Semantic segmentation metrics aggregated over the data set
        %    A table of up to 5 metrics aggregated over the whole data set,
        %    depending on the value of the "Metrics" parameter used with
        %    <a href="matlab:help('evaluateSemanticSegmentation')">evaluateSemanticSegmentation</a>:
        %
        %      * GlobalAccuracy: Share of correctly classified pixels
        %        regardless of class.
        %      * MeanAccuracy: Mean of the share of correctly classified
        %        pixels for each class. This is the mean of
        %        ClassMetrics.Accuracy.
        %      * MeanIoU: Mean of the intersection over union or IoU (also
        %        known as Jaccard similarity coefficient) for each class.
        %        This is the mean of ClassMetrics.IoU.
        %      * WeightedIoU: Mean of the intersection over union
        %        coefficient (IoU) for each class weighted by the number of
        %        pixels in each class.
        %      * MeanBFScore: Mean over all the images of the mean BF score
        %        for each image. This is the mean of ImageMetrics.BFScore.
        DataSetMetrics
        
        %ClassMetrics Semantic segmentation metrics for each class
        %   A table listing up to 3 metrics for each class, depending on
        %   the value of the "Metrics" parameter used with
        %   <a href="matlab:help('evaluateSemanticSegmentation')">evaluateSemanticSegmentation</a>:
        %
        %     * Accuracy: Share of correctly classified pixels in each
        %       class.
        %     * IoU: intersection over union (IoU, also known as Jaccard
        %       similarity coefficient) for each class.
        %     * MeanBFScore: Mean over all the images of the BF score for
        %       each class.
        ClassMetrics
        
        %ImageMetrics Semantic segmentation metrics for each image in the set
        %   A table where each row lists up to 5 metrics for each image in
        %   the data set, depending on the value of the "Metrics"
        %   parameter used with <a href="matlab:help('evaluateSemanticSegmentation')">evaluateSemanticSegmentation</a>:
        %
        %      * GlobalAccuracy: Share of correctly classified image pixels
        %        regardless of class.
        %      * MeanAccuracy: Mean of the share of correctly classified
        %        image pixels for each class.
        %      * MeanIoU: Mean of the intersection over union (IoU, also
        %        known as Jaccard similarity coefficient) for each class.
        %      * WeightedIoU: Average of the IoU (Jaccard index) for each
        %        class weighted by the number of image pixels in each class.
        %      * MeanBFScore: Mean of the BF score for each class.
        ImageMetrics
    end
    
    properties (Hidden = true, Access = protected)
        ActDatastore      % Cell array of DS to the results image files
        ExpDatastore      % Cell array of DS to the ground truth image files
        NumSources        % Number of data sources
        NumFiles          % Number of images in the data set
        
        UseParallel       % Whether to run the computation in parallel
        
        Verbose           % Whether to print to the Command Window
        Printer           % Message printer
        WaitBar           % Console wait bar
        
        WantGlobalAccuracy% Whether to compute the global accuracy
        WantAccuracy      % Whether to compute the mean accuracy
        WantJaccard       % Whether to compute the mean Jaccard
        WantFreqJaccard   % Whether to compute the freq-weighted Jaccard
        WantBFScore       % Whether to compute the BF score
        
        ClassNames        % Names of the classes
        NumClasses        % Number of classes in the data set
        ConfusionMat      % Confusion matrix for the whole data set
        InterAccumulator  % Accumulator for the intersection in Jaccard for each class in the whole set
        UnionAccumulator  % Accumulator for the union in Jaccard for each class in the whole set
        
        IsInputConfusionMatrix     % Whether the input contains confusion matrices
        NumImages                  % Number of images in the data set when input contains confusion matrices
        CanComputeBlockMetrics     % Whether Block metrics can be computed for the given input
        InputConfMatrices          % Cell array of input confusion matrices of images
        BlockInfo                  % Block spatial information (when available)
        ImageNumber                % Image number corresponding to input confusion matrices
        BlockMetricsVariableNames  % Output block metrics table variable names
    end
    
    methods
        %------------------------------------------------------------------
        function delete(obj)
            %DELETE Class destructor.
            delete(obj.WaitBar);
        end
        
        %------------------------------------------------------------------
        function tf = eq(obj1,obj2)
            %EQ Operator ==.
            tf = false;
            if isa(obj1,'semanticSegmentationMetrics') && ...
                    isa(obj2,'semanticSegmentationMetrics') && ...
                    isequal(obj1.ConfusionMatrix,obj2.ConfusionMatrix) && ...
                    isequal(obj1.NormalizedConfusionMatrix,obj2.NormalizedConfusionMatrix) && ...
                    isequal(obj1.DataSetMetrics,obj2.DataSetMetrics) && ...
                    isequal(obj1.ClassMetrics,obj2.ClassMetrics) && ...
                    isequal(obj1.ImageMetrics,obj2.ImageMetrics)
                tf = true;
            end
        end
        
        %------------------------------------------------------------------
        function tf = ne(obj1,obj2)
            %NE Operator ~=.
            tf = ~eq(obj1,obj2);
        end
        
        %------------------------------------------------------------------
        function tf = isequal(obj1,obj2)
            %ISEQUAL Determine equality.
            tf = eq(obj1,obj2);
        end
    end
    
    methods (Hidden = true, Static = true)
        %------------------------------------------------------------------
        function [obj, blockMetrics, canComputeBlockMetrics] = compute(varargin)
            [obj, blockMetrics] = semanticSegmentationMetrics(varargin{:});
            canComputeBlockMetrics = obj.CanComputeBlockMetrics;
        end
    end
    
    methods (Hidden = true, Access = protected)
        %------------------------------------------------------------------
        function [obj, blockMetrics] = semanticSegmentationMetrics(varargin)
            narginchk(2,6);
            
            if isempty(varargin{1})
                error(message('vision:semanticseg:expectedNonEmpty'));
            end
            if semanticSegmentationMetrics.isInputDatastore(varargin{1})
                obj = parseInputsForDatastores(obj, varargin{:});
            else
                obj = parseInputsForTableOrCellArrays(obj, varargin{:});
            end
            
            obj = obj.parseOptionalInputs(varargin{3:end});
            obj = obj.initializeAndAllocate();
            [obj, blockMetrics] = obj.computeMetrics();
        end
        
        %------------------------------------------------------------------
        function obj = parseOptionalInputs(obj,varargin)
            parser = inputParser();
            parser.FunctionName = mfilename;
            
            % Metrics
            if obj.IsInputConfusionMatrix
                % bfscore is not supported
                validMetrics = {'all','global-accuracy','accuracy', ...
                    'iou','weighted-iou',''};
            else
                validMetrics = {'all','global-accuracy','accuracy', ...
                    'iou','weighted-iou','bfscore',''};
            end
            defaultMetrics = validMetrics{1};
            validateMetrics = @(x) validateattributes(x, ...
                {'char','cell','string'}, ...
                {'vector'}, ...
                mfilename,'Metrics');
            parser.addParameter('Metrics', ...
                defaultMetrics, ...
                validateMetrics);
            
            % Verbose
            defaultVerbose = true;
            validateVerbose = @(x) vision.internal.inputValidation.validateLogical(x,'Verbose');
            parser.addParameter('Verbose', ...
                defaultVerbose, ...
                validateVerbose);
            
            % UseParallel
            defaultUseParallel = vision.internal.useParallelPreference();
            validateUseParallel = @vision.internal.inputValidation.validateUseParallel;
            parser.addParameter('UseParallel', ...
                defaultUseParallel);
            
            parser.parse(varargin{:});
            inputs = parser.Results;
            
            obj.Verbose = logical(inputs.Verbose);
            obj.UseParallel = logical(validateUseParallel(inputs.UseParallel));
            
            if ~obj.IsInputConfusionMatrix
                % UseParallel only supported when inputs are pixelLabelDatastore
                % or pixelLabelImageDatastore.
                canSupportParallel = false(1,obj.NumSources);
                for k = 1:obj.NumSources
                    isExpPXDS = isa(obj.ExpDatastore{k},'matlab.io.datastore.PixelLabelDatastore');
                    isExpPXIMDS = isa(obj.ExpDatastore{k},'pixelLabelImageDatastore');
                    isActPXDS = isa(obj.ActDatastore{k},'matlab.io.datastore.PixelLabelDatastore');
                    isActPXIMDS = isa(obj.ActDatastore{k},'pixelLabelImageDatastore');
                    
                    canSupportParallel(k) = (isExpPXDS || isExpPXIMDS) && (isActPXDS || isActPXIMDS);
                    
                    if canSupportParallel(k)
                        if isExpPXDS
                            obj.NumFiles(k) = numel(obj.ExpDatastore{k}.Files);
                        elseif isExpPXIMDS
                            obj.NumFiles(k) = numel(obj.ExpDatastore{k}.Images);
                        end
                    else
                        obj.NumFiles(k) = inf;
                    end
                end
                if ~all(canSupportParallel)
                    wasUserSpecified = ~ismember('UseParallel',parser.UsingDefaults);
                    if wasUserSpecified
                        error(message('vision:semanticseg:parallelNotSupported'));
                    end
                    
                    % Force UseParallel to false.
                    obj.UseParallel = false;
                end
                
            end
            metrics = matlab.images.internal.stringToChar(inputs.Metrics);
            
            % Validate the names of the metrics
            if ischar(metrics)
                metrics = {validatestring(metrics,validMetrics)};
            else
                metrics = cellfun( ...
                    @(x) validatestring(x,validMetrics), ...
                    unique(metrics), 'UniformOutput',false);
            end
            
            obj.WantGlobalAccuracy = false;
            obj.WantAccuracy       = false;
            obj.WantJaccard        = false;
            obj.WantFreqJaccard    = false;
            obj.WantBFScore        = false;
            for m = metrics
                switch m{:}
                    case validMetrics{1}
                        % 'all'
                        obj.WantGlobalAccuracy = true;
                        obj.WantAccuracy       = true;
                        obj.WantJaccard        = true;
                        obj.WantFreqJaccard    = true;
                        obj.BlockMetricsVariableNames = ["GlobalAccuracy","MeanAccuracy","MeanIoU","WeightedIoU"];
                        if ~obj.IsInputConfusionMatrix
                            obj.WantBFScore        = true;
                        end
                    case validMetrics{2}
                        % 'global-accuracy'
                        obj.WantGlobalAccuracy = true;
                        obj.BlockMetricsVariableNames = "GlobalAccuracy";
                    case validMetrics{3}
                        % 'accuracy'
                        obj.WantAccuracy       = true;
                        obj.BlockMetricsVariableNames = "MeanAccuracy";
                    case validMetrics{4}
                        % 'jaccard'
                        obj.WantJaccard        = true;
                        obj.BlockMetricsVariableNames = "MeanIoU";
                    case validMetrics{5}
                        % 'freq'jaccard'
                        obj.WantFreqJaccard    = true;
                        obj.BlockMetricsVariableNames = "WeightedIoU";
                    case validMetrics{6}
                        % 'bfscore'
                        obj.WantBFScore        = true;
                        % BFScore is not supported by blockMetrics
                end
            end
        end
        
        %------------------------------------------------------------------
        function obj = initializeAndAllocate(obj)
            % Initialize parameters
            obj.NumClasses = numel(obj.ClassNames);
            
            % Allocate accumulators and other arrays
            obj.ConfusionMat = zeros(obj.NumClasses);
            
            if obj.WantJaccard || obj.WantFreqJaccard
                obj.InterAccumulator = zeros(obj.NumClasses,1);
                obj.UnionAccumulator = zeros(obj.NumClasses,1);
            end
            
            % Reset the datastores
            if ~obj.IsInputConfusionMatrix
                for k = 1:obj.NumSources
                    reset(obj.ActDatastore{k});
                    reset(obj.ExpDatastore{k});
                end
            end
            % Create a MessagePrinter.
            obj.Printer = vision.internal.MessagePrinter.configure(obj.Verbose);
            if obj.IsInputConfusionMatrix                
                % Create a Console Window wait bar.
                obj.WaitBar = vision.internal.ConsoleWaitBar( ...
                    obj.NumImages, ...     % total number of iterations
                    "Verbose",obj.Verbose, ... % whether to print or not
                    "DisplayPeriod",2, ...     % refresh every 2 seconds
                    "PrintElapsedTime",1, ...  % print elapsed time
                    "PrintRemainingTime",1);   % print estimated time remaining                
            else
                if all(isfinite(obj.NumFiles))
                    % Create a Console Window wait bar.
                    obj.WaitBar = vision.internal.ConsoleWaitBar( ...
                        sum(obj.NumFiles), ...     % total number of iterations
                        "Verbose",obj.Verbose, ... % whether to print or not
                        "DisplayPeriod",2, ...     % refresh every 2 seconds
                        "PrintElapsedTime",1, ...  % print elapsed time
                        "PrintRemainingTime",1);   % print estimated time remaining
                end
            end
        end
        
        %------------------------------------------------------------------
        function [obj, blockMetrics] = computeMetrics(obj)
            % Read each image and compute intermediate variables
            obj = printHeader(obj);
            
            blockMetrics = {};
            if obj.IsInputConfusionMatrix
                blockCounts = computeBlockCountsForAllImages(obj);
                if obj.UseParallel
                    metrics = processImageMetricsFromBlockCountsParallel(obj,blockCounts);
                else
                    metrics = processImageMetricsFromBlockCountsSerial(obj,blockCounts);
                end
                
                if obj.CanComputeBlockMetrics
                    blockMetrics = finalizeBlockMetrics(obj, blockCounts);
                end
            else
                if obj.UseParallel
                    metrics = processImagesParallel(obj);
                else
                    metrics = processImagesSerial(obj);
                end
            end
            
            % Finalize metrics based on the intermediate variables
            obj = printFooter(obj);
            obj = finalizeMetrics(obj,metrics);
            obj = printDone(obj);
        end
        
        %------------------------------------------------------------------
        function blockMetricsForImageNumber = computeBlockMetricsForImageNumber(obj, blockCounts, imageNumber)
            % Algorithm to compute block metrics from block counts
            % corresponding to image specified by imageNumber. The output,
            % blockMetricsForImageNumber is output as a table with each row
            % containing metrics for each block in the image referred by
            % imageNumber.
            
            variableNames = obj.BlockMetricsVariableNames;
            numVariables = numel(variableNames);
            variableTypes = repmat("double",1, numVariables);
            variableNames = [variableNames ,"BlockInfo"];
            numVariables = numel(variableNames);
            variableTypes = [variableTypes,"struct"];
            
            blockCountsPerImage = vertcat(blockCounts{obj.ImageNumber == imageNumber});
            numBlocks = numel(blockCountsPerImage);
            blockMetricsForImageNumber = table('Size', [numBlocks numVariables], ...
                'VariableTypes', variableTypes, ...
                'VariableNames', variableNames);
            
            if obj.WantAccuracy
                blockMetricsForImageNumber.GlobalAccuracy = vertcat(blockCountsPerImage.GlobalAccuracies);
            end
            
            if obj.WantGlobalAccuracy
                blockMetricsForImageNumber.MeanAccuracy = mean(vertcat(blockCountsPerImage.ClassAccuracies),2,'omitnan');
            end
            
            if obj.WantJaccard
                blockMetricsForImageNumber.MeanIoU = mean(vertcat(blockCountsPerImage.Jaccards),2,'omitnan');
            end
            
            if obj.WantFreqJaccard
                blockMetricsForImageNumber.WeightedIoU = iFrequencyWeightedJaccards(vertcat(blockCountsPerImage.Jaccards),vertcat(blockCountsPerImage.ClassCardinals));
            end
            
            blockMetricsForImageNumber.BlockInfo = obj.BlockInfo(obj.ImageNumber == imageNumber);
            
        end
        
        
        %------------------------------------------------------------------
        function blockMetrics = finalizeBlockMetrics(obj, blockCounts)
            % Output block metrics for each image given block counts. The
            % block metrics per image is output as a table with each row
            % containing metrics for each block in that image.
            
            numImages = obj.NumImages;
            blockMetrics = cell(numImages,1);
            imageNumbers = unique(obj.ImageNumber);
            
            numWorkers = getNumWorkers(obj);
            parfor (k = 1:numImages, numWorkers)
                blockMetrics{k} = computeBlockMetricsForImageNumber(obj, blockCounts, imageNumbers(k));
            end
        end
        
        %------------------------------------------------------------------
        function blockCountsKthBlock = computeBlockCountsKthBlock(obj, k)
            % Compute counts for the kth block from input confusion
            % matrices.
            
            % Compute the class accuracy and global
            % accuracy metrics on individual images
            if obj.WantAccuracy || obj.WantGlobalAccuracy
                [blockCountsKthBlock.GlobalAccuracies,blockCountsKthBlock.ClassAccuracies(1,:)] = ...
                    images.internal.segmentation.accuracy(obj.InputConfMatrices{k});
            end
            
            % Compute the IoU and weighted IoU on individual
            % images and incrementally build the data set IoU.
            if obj.WantJaccard || obj.WantFreqJaccard
                [blockCountsKthBlock.Jaccards, blockCountsKthBlock.ImageInter, blockCountsKthBlock.imageUnion] = iConfusionMatrixToJaccard(obj.InputConfMatrices{k});
                if obj.WantFreqJaccard
                    blockCountsKthBlock.ClassCardinals(1,:) = sum(obj.InputConfMatrices{k},2,'omitnan');
                end
            end
            blockCountsKthBlock.confMat = obj.InputConfMatrices{k};
        end
        
        %------------------------------------------------------------------
        function blockCounts = computeBlockCountsForAllImages(obj)
            % Compute counts for all blocks in all images from input
            % confusion matrices for each block.
            
            numBlocks = numel(obj.InputConfMatrices);
            blockCounts = cell(numBlocks,1);
            numWorkers = getNumWorkers(obj);
            parfor (k = 1:numBlocks, numWorkers)                
                blockCounts{k} = computeBlockCountsKthBlock(obj, k);
            end
        end
        
        %------------------------------------------------------------------
        
        function metricsForImageNumber = processImageMetricsFromBlockCountsForImageNumber(obj, blockCounts, imageNumber)
            % Aggregate block counts for all blocks in the image specified
            % by imageNumber to get image metrics.
            
            blockCountsPerImage = vertcat(blockCounts{obj.ImageNumber== imageNumber});
            metricsForImageNumber.confMat = sum(cat(3,blockCountsPerImage(:).confMat),3);
            
            % Compute the class accuracy and global
            % accuracy metrics on individual images
            if obj.WantAccuracy || obj.WantGlobalAccuracy
                
                [metricsForImageNumber.GlobalAccuracies,metricsForImageNumber.ClassAccuracies(1,:)] = ...
                    images.internal.segmentation.accuracy(metricsForImageNumber.confMat);
            end
            
            % Compute the IoU and weighted IoU on individual
            % images and incrementally build the data set IoU.
            if obj.WantJaccard || obj.WantFreqJaccard
                [metricsForImageNumber.Jaccards, metricsForImageNumber.imageInter, metricsForImageNumber.imageUnion] = iConfusionMatrixToJaccard(metricsForImageNumber.confMat);
                if obj.WantFreqJaccard
                    metricsForImageNumber.ClassCardinals(1,:) = sum(metricsForImageNumber.confMat,2,'omitnan');
                end
            end
        end
        
        
        %------------------------------------------------------------------
        function metrics = processImageMetricsFromBlockCountsParallel(obj, blockCounts)
            % Aggregate block counts for all blocks in each image to get
            % image metrics. metrics must be in the same format as that
            % returned by processImagesSerial/processImagesParallel
            % functions
            
            assert(obj.UseParallel)
            
            % Create a DataQueue to send progress
            % from the workers back to the client.
            queue = parallel.pool.DataQueue;
            afterEach(queue, @(~) update(obj.WaitBar));
            
            % Force finish displaying wait bar when we exit this function
            % or if the user sends an interrupt signal.
            cleanUpObj = onCleanup(@() stop(obj.WaitBar));
            
            obj.Printer.printMessage('vision:semanticseg:processingNImages', ...
                num2str(obj.NumImages));
            
            % Start displaying wait bar
            start(obj.WaitBar);
            
            imageNumbers = unique(obj.ImageNumber);
            metrics = cell(obj.NumImages,1);
            % Accumulate metrics across all sources and observations.
            parfor k = 1:obj.NumImages
                % Send progress back to the client
                f = k + 1;
                send(queue,f);
                
                metrics{k,1} = processImageMetricsFromBlockCountsForImageNumber(obj, blockCounts, imageNumbers(k));
            end
            metrics = {vertcat(metrics{:})};
        end
        %------------------------------------------------------------------
        function metrics = processImageMetricsFromBlockCountsSerial(obj, blockCounts)
            % Aggregate block counts for all blocks in each image to get
            % image metrics. metrics must be in the same format as that
            % returned by processImagesSerial/processImagesParallel
            % functions
            
            msg = iPrintProgress(obj.Printer,'',0);
            
            imageNumbers = unique(obj.ImageNumber);
            metrics = cell(obj.NumImages,1);
            % Accumulate metrics across all sources and observations.
            for k = 1:obj.NumImages
                metrics{k,1} = processImageMetricsFromBlockCountsForImageNumber(obj, blockCounts, imageNumbers(k));
                
                msg = iPrintProgress(obj.Printer,msg,k);
            end
            obj.Printer.linebreak();
            metrics = {vertcat(metrics{:})};
        end
        
        %------------------------------------------------------------------
        function s = processImages(obj,data)
            
            for k = 1:size(data,1)
                
                act = data{k,1};
                exp = data{k,2};
                
                iAssertDataIsCategorical(act);
                iAssertDataIsCategorical(exp);
                iAssertCategoricalsHaveSameSize(act,exp);
                iAssertCategoricalsHaveSameCategories(act,exp,obj.ClassNames);
                
                act_bw = images.internal.segmentation.convertToCellOfLogicals(act,obj.ClassNames);
                exp_bw = images.internal.segmentation.convertToCellOfLogicals(exp,obj.ClassNames);
                
                act_bw = obj.maskUnlabeledPixels(act_bw, exp);
                
                % Incrementally build the data set confusion matrix
                confMat = images.internal.segmentation.bwconfmat(act_bw,exp_bw);
                
                % Compute the class accuracy and global
                % accuracy metrics on individual images
                if obj.WantAccuracy || obj.WantGlobalAccuracy
                    [s.GlobalAccuracies,s.ClassAccuracies(1,:)] = ...
                        images.internal.segmentation.accuracy(confMat);
                end
                
                % Compute the IoU and weighted IoU on individual
                % images and incrementally build the data set IoU.
                if obj.WantJaccard || obj.WantFreqJaccard
                    for c = 1:obj.NumClasses
                        [s.Jaccards(1,c),s.imageInter(c,1),s.imageUnion(c,1)] = ...
                            images.internal.segmentation.bwjaccard( ...
                            act_bw{c},exp_bw{c});
                    end
                    
                    if obj.WantFreqJaccard
                        s.ClassCardinals(1,:) = sum(confMat,2,'omitnan');
                    end
                end
                s.confMat = confMat;
                
                % Compute the BFScore on individual images
                if obj.WantBFScore
                    % Use 0.75% of the image diagonal as threshold.
                    % This is the value used by the paper.
                    if(ismatrix(exp))
                        theta = 0.75 / 100 * sqrt(size(exp,1)^2 + size(exp,2)^2);
                        BFScoreFcn = @images.internal.segmentation.bwbfscore2;
                    elseif(ndims(exp) == 3)
                        theta = 0.75 / 100 * sqrt(size(exp,1)^2 + size(exp,2)^2 + size(exp,3)^2);
                        BFScoreFcn = @images.internal.segmentation.bwbfscore3;
                    else
                        error(message('vision:semanticseg:invalidImageNumDims'));
                    end
                    
                    for c = 1:obj.NumClasses
                        s.BFScores(1,c) = BFScoreFcn( ...
                            act_bw{c},exp_bw{c},theta);
                    end
                end
            end
        end
        
        %------------------------------------------------------------------
        function allSourceMetrics = processImagesSerial(obj)
            
            msg = iPrintProgress(obj.Printer,'',0);
            
            % For each data set
            allSourceMetrics = cell(1,obj.NumSources);
            itemsProcessed = 0;
            for s = 1:obj.NumSources
                pxdsAct = obj.ActDatastore{s};
                pxdsExp = obj.ExpDatastore{s};
                
                act = transform(pxdsAct,@iSelectionTransform);
                exp = transform(pxdsExp,@iSelectionTransform);
                
                ds = combine(act,exp);
                
                metrics = {};
                while hasdata(ds)
                    data = read(ds);
                    m = processImages(obj,data);
                    itemsProcessed = itemsProcessed + numel(m);
                    msg = iPrintProgress(obj.Printer,msg,itemsProcessed);
                    metrics{end+1,1} = m; %#ok<AGROW>
                end
                allSourceMetrics{s} = vertcat(metrics{:});
            end
            obj.Printer.linebreak();
            
        end
        
        %------------------------------------------------------------------
        function metrics = processImagesParallel(obj)
            % Create the parallel pool *before* starting to print progress.
            % Open a parallel pool if one does not already exist.
            assert(obj.UseParallel);
            p = gcp;
            if isempty(p)
                % Automatic pool starts are disabled in the parallel pref.
                % Run the algo in serial instead.
                metrics = processImagesSerial(obj);
                return
            end
            
            % Create a DataQueue to send progress
            % from the workers back to the client.
            queue = parallel.pool.DataQueue;
            afterEach(queue, @(~) update(obj.WaitBar));
            
            numPrecedingFiles = cumsum(obj.NumFiles);
            numPrecedingFiles = [0 numPrecedingFiles(1:end-1)];
            
            % Force finish displaying wait bar when we exit this function
            % or if the user sends an interrupt signal.
            cleanUpObj = onCleanup(@() stop(obj.WaitBar));
            
            obj.Printer.printMessage('vision:semanticseg:processingNImages', ...
                num2str(sum(obj.NumFiles)));
            
            % Start displaying wait bar
            start(obj.WaitBar);
            
            % For each data set
            metrics = cell(1,obj.NumSources);
            for s = 1:obj.NumSources
                pxdsAct = obj.ActDatastore{s};
                pxdsExp = obj.ExpDatastore{s};
                
                % Offset used by the counter for the wait bar.
                offset = numPrecedingFiles(s);
                
                % For each file in the data set
                parfor f = 1:obj.NumFiles(s)
                    % Send progress back to the client
                    k = f + offset;
                    send(queue,k);
                    
                    doActRead = iManageReadForDS(pxdsAct);
                    doExpRead = iManageReadForDS(pxdsExp);
                    
                    % Read predicted and actual images
                    act = iSelectionTransform(doActRead(pxdsAct,f));
                    exp = iSelectionTransform(doExpRead(pxdsExp,f));
                    
                    m(f) = processImages(obj,[act, exp])
                    
                end
                metrics{s} = m;
            end
            
        end
        
        %------------------------------------------------------------------
        function obj = finalizeMetrics(obj, metrics)
            
            % Accumulate metrics across all sources and observations.
            for s = 1:numel(obj.NumSources)
                for k = 1:numel(metrics{s})
                    obj.ConfusionMat = obj.ConfusionMat + metrics{s}(k).confMat;
                end
            end
            
            % 1. Confusion matrix
            % -------------------
            
            % 1.1 Confusion matrix as raw counts
            obj.ConfusionMatrix = array2table( ...
                obj.ConfusionMat, ...
                'VariableNames', obj.ClassNames, ...
                'RowNames', obj.ClassNames);
            
            % 1.2 Normalized confusion matrix
            ratios = obj.ConfusionMat;
            for i = 1:size(ratios,1)
                ratios(i,:) = ratios(i,:) / max(eps, sum(ratios(i,:),'omitnan'));
            end
            obj.NormalizedConfusionMatrix = array2table( ...
                ratios, ...
                'VariableNames', obj.ClassNames, ...
                'RowNames', obj.ClassNames);
            
            % 2. Other Metrics
            % ----------------
            obj.ClassMetrics = table('RowNames',obj.ClassNames);
            obj.DataSetMetrics = table;
            
            % ImageMetrics is a table or, if there are
            % more than 1 source, a cell array of tables.
            if (obj.NumSources == 1)
                obj.ImageMetrics = table;
            else
                obj.ImageMetrics = cell(obj.NumSources,1);
                for s = 1:obj.NumSources
                    obj.ImageMetrics{s} = table;
                end
            end
            
            % 2.1 Global Accuracy
            if obj.WantGlobalAccuracy
                % Global accuracy for the whole set
                obj.DataSetMetrics.GlobalAccuracy = sum(diag(obj.ConfusionMat),'omitnan') / sum(sum(obj.ConfusionMat,2,'omitnan'));
                
                % Global accuracy for each image
                if (obj.NumSources == 1)
                    obj.ImageMetrics.GlobalAccuracy = vertcat(metrics{1}(:).GlobalAccuracies);
                else
                    for s = 1:obj.NumSources
                        obj.ImageMetrics{s}.GlobalAccuracy = vertcat(metrics{s}(:).GlobalAccuracies);
                    end
                end
            end
            
            % 2.2 Accuracy
            if obj.WantAccuracy
                % Accuracy for each class
                classAccuracy = zeros(obj.NumClasses,1);
                for i = 1:obj.NumClasses
                    classAccuracy(i) = obj.ConfusionMat(i,i) / sum(obj.ConfusionMat(i,:),'omitnan');
                end
                obj.ClassMetrics.Accuracy = classAccuracy;
                
                % Mean class accuracy over the whole set. Include NaN when
                % computing the dataset level metrics. NaNs occur in class
                % accuracy when one of the classes is missing from the
                % entire dataset. In this scenario, returning NaN at the
                % dataset level propagates this information up to the
                % dataset level. If we omit NaNs, we would mask this
                % scenario from users. 
                meanAccuracy = mean(classAccuracy,'includenan');
                obj.DataSetMetrics.MeanAccuracy = meanAccuracy;
                
                % Mean class accuracy for each image
                if (obj.NumSources == 1)
                    ca = vertcat(metrics{1}(:).ClassAccuracies);
                    obj.ImageMetrics.MeanAccuracy = mean(ca,2,'includenan');
                else
                    
                    for s = 1:obj.NumSources
                        ca = vertcat(metrics{s}(:).ClassAccuracies);
                        obj.ImageMetrics{s}.MeanAccuracy = mean(ca,2,'includenan');
                    end
                end
            end
            
            if obj.WantJaccard || obj.WantFreqJaccard
                for s = 1:numel(obj.NumSources)
                    for k = 1:numel(metrics{s})
                        obj.InterAccumulator = obj.InterAccumulator + metrics{s}(k).imageInter;
                        obj.UnionAccumulator = obj.UnionAccumulator + metrics{s}(k).imageUnion;
                    end
                end
            end
            
            % 2.3 Jaccard
            if obj.WantJaccard
                % Jaccard coeff for each class over the set
                classJaccard = obj.InterAccumulator ./ obj.UnionAccumulator;
                obj.ClassMetrics.IoU = classJaccard;
                
                % Mean class Jaccard over the whole set. See note in
                % Accuracy section about use of 'includenan'. 
                meanJaccard = mean(classJaccard,'includenan');
                obj.DataSetMetrics.MeanIoU = meanJaccard;
                
                % Mean class Jaccard for each image
                if (obj.NumSources == 1)
                    jac = vertcat(metrics{1}(:).Jaccards);
                    obj.ImageMetrics.MeanIoU = mean(jac,2,'includenan');
                else
                    
                    for s = 1:obj.NumSources
                        jac = vertcat(metrics{s}(:).Jaccards);
                        obj.ImageMetrics{s}.MeanIoU = mean(jac,2,'includenan');
                    end
                end
            end
            
            % 2.4 Frequency-weighted Jaccard
            if obj.WantFreqJaccard
                % Frequency-weighted mean class Jaccard
                classJaccard = obj.InterAccumulator ./ obj.UnionAccumulator;
                freqWeightedJaccard = sum(obj.ConfusionMat, 2, 'omitnan') .* classJaccard;
                obj.DataSetMetrics.WeightedIoU = sum(freqWeightedJaccard,'omitnan') / sum(sum(obj.ConfusionMat,2,'omitnan'));
                
                % Freq-weighted Jaccard for each image
                if (obj.NumSources == 1)
                    jaccards = vertcat(metrics{1}(:).Jaccards);
                    classCardinals = vertcat(metrics{1}(:).ClassCardinals);
                    fw = iFrequencyWeightedJaccards(jaccards,classCardinals);
                    obj.ImageMetrics.WeightedIoU = fw;
                else
                    
                    for s = 1:obj.NumSources
                        jaccards = vertcat(metrics{s}(:).Jaccards);
                        classCardinals = vertcat(metrics{s}(:).ClassCardinals);
                        fw = iFrequencyWeightedJaccards(jaccards,classCardinals);
                        obj.ImageMetrics{s}.WeightedIoU = fw;
                    end
                end
            end
            
            % 2.5 BF score
            if obj.WantBFScore
                % Mean BF score for each class over the whole set. 
                m = vertcat(metrics{:});
                bfScores = vertcat(m(:).BFScores);
                classBFScore = mean(bfScores,1,'omitnan')';
                obj.ClassMetrics.MeanBFScore = classBFScore;
                
                % Mean BF score over all the classes for the whole set
                meanBFScore = mean(mean(bfScores,2,'omitnan'));
                obj.DataSetMetrics.MeanBFScore = meanBFScore;
                
                % Mean BF score over all the classes for each image
                if (obj.NumSources == 1)
                    bfScores = vertcat(metrics{1}(:).BFScores);
                    obj.ImageMetrics.MeanBFScore = mean(bfScores,2,'omitnan');
                else
                    
                    for s = 1:obj.NumSources
                        bfScores = vertcat(metrics{s}(:).BFScores);
                        obj.ImageMetrics{s}.MeanBFScore = mean(bfScores,2,'omitnan');
                        
                    end
                end
            end
        end
        
        %------------------------------------------------------------------
        function obj = printHeader(obj)
            obj.Printer.printMessage('vision:semanticseg:evaluationHeader');
            N = length(getString(message('vision:semanticseg:evaluationHeader')));
            obj.Printer.print(repmat('-',1,N));
            obj.Printer.linebreak();
            obj.Printer.printMessageNoReturn('vision:semanticseg:selectedMetrics');
            obj.Printer.print(' ');
            printComma = false;
            if obj.WantGlobalAccuracy
                obj.Printer.print('global accuracy');
                printComma = true;
            end
            if obj.WantAccuracy
                if printComma
                    obj.Printer.print(', ');
                end
                obj.Printer.print('class accuracy');
                printComma = true;
            end
            if obj.WantJaccard
                if printComma
                    obj.Printer.print(', ');
                end
                obj.Printer.print('IoU');
                printComma = true;
            end
            if obj.WantFreqJaccard
                if printComma
                    obj.Printer.print(', ');
                end
                obj.Printer.print('weighted IoU');
                printComma = true;
            end
            if obj.WantBFScore
                if printComma
                    obj.Printer.print(', ');
                end
                obj.Printer.print('BF score');
            end
            obj.Printer.print('.');
            obj.Printer.linebreak();
        end
        
        %------------------------------------------------------------------
        function obj = printFooter(obj)
            obj.Printer.printMessage('vision:semanticseg:finalizingResults');
        end
        
        %------------------------------------------------------------------
        function obj = printDone(obj)
            obj.Printer.print('\b ');
            obj.Printer.printMessage('vision:semanticseg:done');
            obj.Printer.printMessage('vision:semanticseg:dataSetMetrics');
            obj.Printer.linebreak();
            if obj.Verbose
                disp(obj.DataSetMetrics)
            end
        end
        
        %------------------------------------------------------------------
        function numWorkers = getNumWorkers(obj)
            % Returns the number of workers to be used in a parfor loop
            % based on UseParallel flag.
            if obj.UseParallel
                numWorkers = Inf;
            else
                numWorkers = 0;
            end
        end
        
        %------------------------------------------------------------------
        function obj = parseInputsForDatastores(obj, varargin)
            
            obj.IsInputConfusionMatrix = false;
            
            datastoreTypes = {'matlab.io.datastore.Datastore','matlab.io.Datastore'};
            validTypes = [datastoreTypes,{'cell'}];
            validAttribs = {'nonempty', 'vector'};
            validateInput = @(x,validType,validAttribs,name,pos) validateattributes(x, ...
                validType, ...
                validAttribs, ...
                mfilename,name,pos);
            
            dsResults = varargin{1};
            validateInput(dsResults,validTypes,validAttribs,'dsResults',1);
            
            dsTruth   = varargin{2};
            validateInput(dsTruth,validTypes,validAttribs,'dsTruth',2);
            
            if ~isequal(numel(dsResults),numel(dsTruth))
                error(message('images:validate:unequalNumberOfElements', ...
                    'dsResults','dsTruth'))
            end
            
            if (iscell(dsResults) || iscell(dsTruth)) && ...
                    ~(iscell(dsResults) && iscell(dsTruth))
                % Both must be cell.
                error(message('vision:semanticseg:mustBePXDSOrCell', ...
                    'dsResults','dsTruth'))
            end
            
            validateInputIsDatastore = @(x,name,pos) validateattributes(x, ...
                datastoreTypes, ...
                {'nonempty','scalar'}, ...
                mfilename,name,pos);
            
            if iscell(dsResults)
                % now we know both are cell arrays
                % with the same number of elements
                obj.ActDatastore = cell(numel(dsResults),1);
                obj.ExpDatastore = cell(numel(dsResults),1);
                
                for k = 1:numel(dsResults)
                    try
                        validateInputIsDatastore(dsResults{k},'dsResults',1);
                        validateInputIsDatastore(dsTruth{k},  'dsTruth',  2);
                    catch
                        error(message('vision:semanticseg:mustBeCellArrayOfDatastores'));
                    end
                    obj.validatePXDSPair(dsResults{k},dsTruth{k});
                    obj.ActDatastore{k} = copy(dsResults{k});
                    obj.ExpDatastore{k} = copy(dsTruth{k});
                end
                
            else
                % both are scalar pxds objects
                obj.validatePXDSPair(dsResults,dsTruth);
                obj.ActDatastore = {copy(dsResults)};
                obj.ExpDatastore = {copy(dsTruth)};
            end
            
            obj.NumSources = numel(obj.ExpDatastore);
            
            % Determine ClassNames by reading from one of the datastores.
            reset(obj.ActDatastore{1});
            C = read(obj.ActDatastore{1});
            reset(obj.ActDatastore{1});
            C = iSelectionTransform(C);
            if iscell(C)
                C = C{1};
            end
            iAssertDataIsCategorical(C);
            obj.ClassNames = cellstr(categories(C));
            
        end
        
        %------------------------------------------------------------------
        function obj = parseInputsForTableOrCellArrays(obj, varargin)
            % istable || iscell with confusion matrices
            obj.IsInputConfusionMatrix = true;
            
            validTypes1stArg = {'cell','table'};
            validAttribs1stArg = {'nonempty'};
            validateInput = @(x,validType,validAttribs,name,pos) validateattributes(x, ...
                validType, ...
                validAttribs, ...
                mfilename,name,pos);
            
            dsResults = varargin{1};
            validateInput(dsResults,validTypes1stArg,validAttribs1stArg,'dsResults',1);
            
            classnames = varargin{2};
            iValidateClasses(classnames);
            
            % Verify the table has ConfusionMatrix
            obj.CanComputeBlockMetrics = false;
            if istable(dsResults)
                if ~any(cellfun(@(x)strcmpi(x,'ConfusionMatrix'),dsResults.Properties.VariableNames))
                    error(message('vision:semanticseg:tableMustHaveConfusionMatrix'));
                end
                
                validateattributes(dsResults.ConfusionMatrix,{'cell'},{'column','nonempty'},mfilename,'datasetConfMat.ConfusionMatrix',1);
                
                obj.InputConfMatrices = dsResults.ConfusionMatrix;
                
                if any(cellfun(@(x)strcmpi(x,'BlockInfo'),dsResults.Properties.VariableNames)) && ...
                        any(cellfun(@(x)strcmpi(x,'ImageNumber'),dsResults.Properties.VariableNames))
                    
                    validateattributes(dsResults.BlockInfo,{'struct'},{'column','nonempty'},...
                        mfilename,'datasetConfMat.BlockInfo',1);
                    validateattributes(dsResults.ImageNumber,{'numeric'},{'column','nonempty',...
                        'nonzero','nonsparse','finite','integer','positive','nonnan'},...
                        mfilename,'datasetConfMat.ImageNumber',1);
                    
                    obj.CanComputeBlockMetrics = true;
                    obj.NumImages = numel(unique(dsResults.ImageNumber));
                    obj.BlockInfo = dsResults.BlockInfo;
                    obj.ImageNumber = dsResults.ImageNumber;
                else
                    obj.NumImages = height(dsResults);
                    obj.ImageNumber = 1:obj.NumImages;
                end
            else
                % cell array
                obj.InputConfMatrices = dsResults;
                validateattributes(dsResults,{'cell'},{'column','nonempty'},mfilename,'datasetConfMat',1);
                obj.CanComputeBlockMetrics = false;
                obj.NumImages = numel(dsResults);
                obj.InputConfMatrices = dsResults;
                obj.ImageNumber = 1:obj.NumImages;
            end
            
            iValidateConfusionMatricesSameSize(obj.InputConfMatrices);
            iValidateConfusionMatricesType(obj.InputConfMatrices);
            iValidateConfusionMatricesAndNumClassesAreConsistent(obj.InputConfMatrices,classnames);
            
            obj.NumSources = 1;
            obj.ClassNames = cellstr(classnames);
        end
        
    end
    
    methods (Hidden = true, Access = protected, Static = true)
        
        %------------------------------------------------------------------
        function validatePXDSPair(pxds1,pxds2)
            % When both datastores are pixelLabelDatastores, check that
            % they are consistent.
            type = 'matlab.io.datastore.PixelLabelDatastore';
            if isa(pxds1,type) && isa(pxds2,type)
                
                % both must hold the same number of files
                if ~isequal(numel(pxds1.Files),numel(pxds2.Files))
                    error(message('vision:semanticseg:mustHaveSameNumOfFiles', ...
                        'dsResults','dsTruth'))
                end
            end
        end
        
        %------------------------------------------------------------------
        function TF = isInputDatastore(dsResults)
            % Returns true if the input is a datastore or cell array of
            % datastores
            datastoreTypes = {'matlab.io.datastore.Datastore','matlab.io.Datastore'};
            
            TF = any(cellfun(@(x)isa(dsResults,x),datastoreTypes)) ||...
                (iscell(dsResults) && any(cellfun(@(x)isa(dsResults{1},x),datastoreTypes)));
        end
        
    end
    
    methods (Hidden = true, Access = public, Static = true)
        %--------------------------------------------------------------------------
        function act_bw = maskUnlabeledPixels(act_bw, exp)
            % Undefined pixels in ground truth must be masked out in predicted results
            % before computing metrics.
            valid = ~isundefined(exp);
            act_bw = cellfun(@(A) A .* valid, act_bw,'UniformOutput',false);
        end
    end
end

%--------------------------------------------------------------------------
function [Jaccards, imageInter, imageUnion] = iConfusionMatrixToJaccard(confusionMatrix)

numClasses = size(confusionMatrix,1);
numElements = numel(confusionMatrix);
% The image intersection is the True Positive (TP) for each class i.e.
% diagonal elements of the confusion matrix
imageInter = confusionMatrix(1:numClasses+1:numElements)';
% The image union value is the sum of 3 values - True Positive (TP) + False
% Positive (FP) + False Negative (FN) for each class.
% For a given confusion matrix, FP is the sum of all row elements for each
% class and FN is the sum of all column elements for each class (excluding
% the TP value for both).
imageUnion = arrayfun(@(x)sum(confusionMatrix(:,x)+confusionMatrix(x,:)'),(1:numClasses)')-imageInter;
Jaccards = (imageInter ./ imageUnion)';

end
%--------------------------------------------------------------------------
function freqWeightedJaccards = iFrequencyWeightedJaccards(jaccards,classCardinals)
freqWeightedJaccards = jaccards .* classCardinals;
freqWeightedJaccards = sum(freqWeightedJaccards,2,'omitnan');
freqWeightedJaccards = freqWeightedJaccards ./ sum(classCardinals,2,'omitnan');
end

%--------------------------------------------------------------------------
function data = iSelectionTransform(data)
% When data is a cell or table with more than two columns, choose the
% second column and assert data is categorical.

isCellOrTable = iscell(data) || istable(data);

if isCellOrTable && size(data,2) > 1
    if iscell(data)
        data = data(:,2);
    else
        data = data{:,2};
    end
elseif ~isCellOrTable
    % Wrap in cell to let combine horzcat correctly.
    data = {data};
end
end

%--------------------------------------------------------------------------
function iAssertDataIsCategorical(data)
if ~isa(data,'categorical')
    error(message('vision:semanticseg:readMustReturnCategorical',...
        'dsResults','dsTruth'));
end
end

%--------------------------------------------------------------------------
function iAssertCategoricalsHaveSameCategories(A,B,classes)
if ~isequal(categories(A),classes)
    error(message('vision:semanticseg:mustHaveSameClasses', ...
        'dsResults','dsTruth'))
end
if ~isequal(categories(B),classes)
    error(message('vision:semanticseg:mustHaveSameClasses', ...
        'dsResults','dsTruth'))
end
end

%--------------------------------------------------------------------------
function iAssertCategoricalsHaveSameSize(A,B)
if ~isequal(size(A),size(B))
    error(message('vision:semanticseg:mustHaveSameSize',...
        'dsResults','dsTruth'));
end
end

%--------------------------------------------------------------------------
function updateMessage(printer, prevMessage, nextMessage)
backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
printer.print([backspace nextMessage]);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintProgress(printer, prevMessage, k)
nextMessage = getString(message('vision:semanticseg:verboseProgressTxt',k));
updateMessage(printer, prevMessage, nextMessage);
end

%--------------------------------------------------------------------------
function reader = iManageReadForDS(in)
if isa(in,'matlab.io.datastore.PixelLabelDatastore')
    reader = @(ds,idx)readimage(ds,idx);
else
    reader = @(ds,idx)readByIndex(ds,idx);
end
end

%------------------------------------------------------------------
function tf = iValidateClasses(classes)

validateattributes(classes, {'string', 'char', 'cell'}, {'nonempty'}, 'evaluateSemanticSegmentation', 'classNames',2)
if iscell(classes)
    for ind = 1:numel(classes)
        validateattributes(classes{ind},...
            {'char'}, {'nonempty'}, 'evaluateSemanticSegmentation', 'classNames')
    end
else
    tf = true;
end

if numel(classes) ~= numel(unique(classes))
    error(message('vision:semanticseg:tableNonUniqueClassNames'));
end
end

%------------------------------------------------------------------
function iValidateConfusionMatricesType(inputConfMatrices)

cellfun(@(x)validateattributes(x,{'numeric'},{'nonnegative'},mfilename,'inputConfMatrices',1),inputConfMatrices)

end

%------------------------------------------------------------------
function iValidateConfusionMatricesSameSize(inputConfMatrices)
confMatFirstImage =  inputConfMatrices{1};
if any(cellfun(@(x)~isequal(size(x),size(confMatFirstImage)),inputConfMatrices))
    error(message('vision:semanticseg:confusionMatrixMustBeSameSize'));
end
end

%------------------------------------------------------------------
function iValidateConfusionMatricesAndNumClassesAreConsistent(inputConfMatrices,classNames)
if ~isequal(numel(classNames),size(inputConfMatrices{1},1)) ||...
        ~isequal(numel(classNames),size(inputConfMatrices{1},2))
    error(message('vision:semanticseg:confusionMatrixAndClassMustBeConsistent'));
end

end
