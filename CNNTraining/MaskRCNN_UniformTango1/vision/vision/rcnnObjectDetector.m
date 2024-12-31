classdef rcnnObjectDetector < vision.internal.EnforceScalarValue & matlab.mixin.CustomDisplay

    % Copyright 2016-2023 The MathWorks, Inc.
    
    properties(Access = public)
        Network
    end
    
    properties(Dependent)
        ClassNames
    end
    
    properties(Access = public)
        RegionProposalFcn
    end
    
    properties(Hidden, Access = protected)
        % UsingDefaultRegionProposalFcn A logical flag to determine whether
        % or not the user has trained the detector with a custom region
        % proposal function.
        UsingDefaultRegionProposalFcn
        
        % BackgroundLabel Label to use for background class. Default is
        % 'Background'.
        BackgroundLabel
        
        % ClassificationLayerIndex Index to classification layer.
        ClassificationLayerIndex
        
        % ImageLayerIndex Index to image input layer.
        ImageLayerIndex
    end
    
    properties(GetAccess = public, SetAccess = protected)
        BoxRegressionLayer
    end
    
    properties(Hidden)
        UseBBoxRegression
        BBoxRegressionModel
    end
    
    methods(Static, Access = public, Hidden)                
        %------------------------------------------------------------------
        % Returns bounding boxes in an M-by-4 array. Each bounding box is a
        % row vector containing [x y width height]. The scores is an M-by-1
        % vector. Higher score values indicate the bounding box is more
        % likely to contain an object.
        %------------------------------------------------------------------
        function [bboxes, scores] = proposeRegions(I)
            alpha = 0.65;
            beta  = 0.75;
            minScore = 0.1;
            
            [bboxes, scores] = vision.internal.rcnn.edgeBoxes(I, alpha, beta, minScore);
            
        end
        
        %------------------------------------------------------------------
        % Returns a utility object to crop and resize training samples.
        %------------------------------------------------------------------
        function resizer = createRegionResizer(networkInputSize)
            resizer = vision.internal.rcnn.RegionCropAndResizer();
            % Expands region proposals by 8 pixels in every direction to
            % capture more background pixels.
            resizer.ExpansionAmount = 8;
            
            % Resize region proposals to size of network input. Do not
            % preserve aspect ratio. Set this to true to preserve the
            % aspect ratio.
            resizer.PreserveAspectRatio = false;
            
            resizer.ImageSize = networkInputSize;
            
            % approximate average image using half value of uint8 range.
            resizer.PadValue = 128;
        end
        
        %------------------------------------------------------------------
        function [detector, regionProposals, info] = train(trainingData, layers, opts, params)
            
            detector = rcnnObjectDetector();
            
            detector.RegionProposalFcn = params.RegionProposalFcn;
            detector.UsingDefaultRegionProposalFcn = params.UsingDefaultRegionProposalFcn;
            detector.BackgroundLabel = params.BackgroundLabel;
            
            imds = imageDatastore(trainingData{:,1});
            
            % Use network training options to control verbosity.
            params.Verbose = opts.Verbose;
            printer = vision.internal.MessagePrinter.configure(params.Verbose);
            
            rcnnObjectDetector.printHeader(printer, trainingData);
             
            if strcmp('auto',params.BoxRegressionLayer)                               
                bboxFeatureLayerNameOrIndex = rcnnObjectDetector.autoSelectBBoxFeatureLayer(layers); 
                              
                % Error if unable to automatically find a layer. 
                if isempty(bboxFeatureLayerNameOrIndex)
                    error(message('vision:rcnn:noConvLayerSkipBoxReg'));          
                else
                    detector.UseBBoxRegression = true;                                       
                end               
                
            else
                % user specified box regression layer.                
                detector.UseBBoxRegression  = true;   
                bboxFeatureLayerNameOrIndex = params.BoxRegressionLayer;
            end

            % Setup region proposal function. For parallel processing using
            % multiple MATLAB workers, copy the function handle before
            % assigning passing it to the extraction routine. This prevents
            % the rcnnObjectDetector object from being copied to all the
            % workers.            
            fcnCopy = detector.RegionProposalFcn;
            fcn = @(x,filename)rcnnObjectDetector.invokeRegionProposalFcn(fcnCopy, x, filename);
            
            params.MinObjectSize = [1 1];
            [regionProposals, trainingData, validTrainingData] = rcnnObjectDetector.extractRegionProposals(fcn, imds, params, trainingData);
                                  
            rcnnObjectDetector.issueWarningIfRequired(validTrainingData);
                       
            dispatcher = createTrainingDispatcher(detector, trainingData, regionProposals, opts, params.InputSize, params);                        
            
            printer.printMessage('vision:rcnn:trainNetworkBegin');
            printer.linebreak;
            
            if iIsParallelExecutionEnvironment(opts.ExecutionEnvironment)
                % disable warning message about DispatchInBackground being
                % disabled when there are no workers for background
                % dispatching. A user has no control over this because the
                % datastore is not user-visible so the warning makes no
                % sense.
                s(1) = warning('off','nnet_cnn:internal:cnn:util:setupAndValidateParallel:BackgroundPrefetchDisabledLocal');
                s(2) = warning('off','nnet_cnn:internal:cnn:util:setupAndValidateParallel:BackgroundPrefetchDisabledCluster');
                reset = onCleanup(@()warning(s)); 
            end
            
            [net, info] = trainNetwork(dispatcher, layers, opts);
            
            iWarnIfTrainingLossHasNaNs(info);
            
            % Set BoxRegressionLayer with the name of the regression layer
            % after training because when the input network is an array of
            % layers some of them may not have any names. 
            detector.BoxRegressionLayer = iLayerName(bboxFeatureLayerNameOrIndex,net);
            detector.Network = net;
            
            printer.linebreak;
            printer.printMessage('vision:rcnn:trainNetworkEnd');
            printer.linebreak;
            
            % Do bounding box regression model training                      
            if detector.UseBBoxRegression         
                bboxopts.FeatureLayerName = detector.BoxRegressionLayer;
                bboxopts.MaxEpochs       = 1;
                bboxopts.Lambda          = 1000;
                
                params.miniBatchSize  = opts.MiniBatchSize;
                params.endOfEpoch     = 'truncateLast';
                params.precision      = nnet.internal.cnn.util.Precision('single');
                params.resizer        = rcnnObjectDetector.createRegionResizer( imageInputSizeFromImageLayer(detector) );
                params.RandomSelector = vision.internal.rcnn.RandomSelector();
                params.PositiveOverlapRange = [0.7 1];
                
                detector.BBoxRegressionModel = rcnnObjectDetector.trainBBoxRegressor(...
                    trainingData, detector.Network, params, bboxopts);
                
                detector.UseBBoxRegression = true;                
            end
            
            rcnnObjectDetector.printFooter(printer);
            rcnnObjectDetector.printRowOfStars(printer);        
        end                
        
        %------------------------------------------------------------------
        function models = trainBBoxRegressor(groundTruth, ...
                network, params, bboxopts)
            
            printer = vision.internal.MessagePrinter.configure(params.Verbose);
            
            printer.printMessageNoReturn('vision:rcnn:bboxRegressionBegin');
            
            dispatcher = vision.internal.rcnn.BBoxTrainingDataDispatcher(...
                groundTruth, params);
            
            classNames = groundTruth.Properties.VariableNames(2:end);
            
            maxEpochs = bboxopts.MaxEpochs;
            
            sdata = cell(1, numel(classNames) * 2);
            sdata(1:2:end) = classNames;
            allFeatures = struct(sdata{:});
            allTargets  = struct(sdata{:});
            
            regressionParams.Lambda = bboxopts.Lambda;
            sdata(2:2:end) = {vision.internal.rcnn.BoundingBoxRegressionModel(regressionParams)};
            
            models = struct(sdata{:});
            
            % For printing training progression.
            numIterations = ceil(dispatcher.NumObservations / dispatcher.MiniBatchSize) * maxEpochs;
            currentIteration = 1;
            msg = '';
            
            for i = 1:maxEpochs
                dispatcher.start();
                while ~dispatcher.IsDone
                    
                    msg = rcnnObjectDetector.printProgress(...
                        printer, msg, currentIteration, numIterations);
                    
                    [batch, targets, labels] = dispatcher.next();
                    
                    features = activations(network, batch, bboxopts.FeatureLayerName, ...
                        'OutputAs', 'columns', ...
                        'MiniBatchSize', params.miniBatchSize);
                    
                    % group features by category label.
                    for j = 1:numel(labels)
                        
                        label = labels{j};
                        allFeatures.(label) = ...
                            [allFeatures.(label) features(:, j)];
                        
                        allTargets.(label)  = ...
                            [allTargets.(label); targets(j,:)];
                    end
                    
                    for k = 1:numel(classNames)
                        
                        name = classNames{k};
                        
                        if ~isempty(allFeatures.(name))
                            
                            try
                                % update class specific models
                                update(models.(name), allFeatures.(name), allTargets.(name));
                            catch ME
                                
                                if iAllElementsAreFinite(allFeatures.(name))
                                    % Rethrow original error.
                                    rethrow(ME);
                                else
                                    % Throw specialized error for
                                    % non-finite features.
                                    printer.linebreak;
                                    error(message('vision:rcnn:nonfiniteRCNNBoxRegressionData'));
                                end
                                
                            end
                            % clear features
                            allFeatures.(name) = [];
                            allTargets.(name)  = [];
                        end
                        
                    end
                    
                    currentIteration = currentIteration + 1;
                end
                
            end
            
            if numIterations > 0
                % print last iteration
                rcnnObjectDetector.printProgress(...
                    printer, msg, numIterations, numIterations);
                
                printer.print('...');
                printer.printMessage('vision:rcnn:bboxRegressionEnd');
                printer.linebreak;
            end
        end
        
        %------------------------------------------------------------------
        % Returns region proposals. Proposals are sorted by score. Only
        % strongest N are kept. The ordering of Files and output region
        % proposals must match.
        %------------------------------------------------------------------
        function [targets, info, regionProposals, posIdx, negIdx, labels, images] = extractTargetsUsingDataFromDatastore(dataFromRead, infoFromRead, fcn, params)

            if istable(dataFromRead)
                images = dataFromRead{:, 1};
                boxes = dataFromRead{:, 2};
                labelNames = dataFromRead{:, 3};
            else
                images = dataFromRead(:, 1);
                boxes = dataFromRead(:, 2);
                labelNames = dataFromRead(:, 3);
            end

            numFiles            = size(dataFromRead, 1);

            targets             = cell(numFiles, 1);
            if nargout > 2
                regionProposals = cell(numFiles, 1);
                posIdx          = cell(numFiles, 1);
                negIdx          = cell(numFiles, 1);
                labels          = cell(numFiles, 1);
            end

            minBoxSize          = params.MinObjectSize;
            needsZeroCenter     = params.NeedsZeroCenterNormalization;
            numStrongestRegions = params.NumStrongestRegions; 
            posOverlap          = params.PositiveOverlapRange;
            negOverlap          = params.NegativeOverlapRange;
            accum               = zeros(1,1,params.InputSize(3));

            for ii = 1:numFiles

                I = images{ii};

                imageSize = size(I);

                if needsZeroCenter
                    % Average needs to be calculated for zero center
                    % normalization, if needed.
                    Imean = vision.internal.cnn.utils.perChannelMean(I, params.InputSize);
                    accum = accum + Imean;
                end

                validBoxes = boxes{ii};

                filename         = iGetFilenameFromReadInfo(infoFromRead, ii, imageSize);
                [bboxes, scores] = fcn(I, filename);
                [bboxes, scores] = fastRCNNObjectDetector.filterSmallBBoxes(bboxes, scores, minBoxSize);
                proposals        = rcnnObjectDetector.selectStrongestRegions(bboxes, scores, numStrongestRegions);
                if nargout > 2
                    % We need label assignment and positive/negative indices of the targets,
                    % from the read method of the Transformed datastore.
                    labelInfo.Labels          = cellstr(labelNames{ii});
                    labelInfo.BackgroundLabel = params.BackgroundLabel;
                    labelInfo.ClassNames      = params.ClassNames;
                    regionProposals{ii}       = proposals;
                    [targets(ii), posIdx{ii}, negIdx{ii}, labels{ii}] = ...
                        iCreatePositiveRegressionTargets(posOverlap, negOverlap, validBoxes, proposals, labelInfo);
                else
                    % We just need positive regression targets for calculating box regression mean and std.
                    targets(ii)               = iCreatePositiveRegressionTargets(posOverlap, negOverlap, validBoxes, proposals);
                end
            end

            info.AccumulatedMean = accum;
        end

        %------------------------------------------------------------------
        % Returns region proposals. Proposals are sorted by score. Only
        % strongest N are kept. The ordering of Files and output region
        % proposals must match.
        %------------------------------------------------------------------
        function [regionProposals, trainingData, validTrainingData] = extractRegionProposals(fcn, imds, params, trainingData)
            
            numfiles = numel(imds.Files);
            
            printer = vision.internal.MessagePrinter.configure(params.Verbose);
            
            printer.printMessageNoReturn('vision:rcnn:regionProposalBegin', numfiles);
            s(numfiles) = struct('RegionProposalBoxes',[]);
            isvalid(numfiles) = struct('HasValidData',[],'HasNoBoxes',[]);
            
            minBoxSize = params.MinObjectSize;
            
            if params.UseParallel
                parfor i = 1:numfiles
                    I = readimage(imds, i);
                    
                    imageSize = size(I);
                    
                    [trainingData(i,:), isvalid(i).HasValidData, isvalid(i).HasNoBoxes] = vision.internal.cnn.utils.hasValidTrainingData(imageSize,trainingData(i,:)); %#ok<PFOUS>
                    
                    [bboxes, scores] = fcn(I, imds.Files{i}); %#ok<PFBNS>
                    
                    [bboxes, scores] = fastRCNNObjectDetector.filterSmallBBoxes(bboxes, scores, minBoxSize);
                    
                    bboxes = rcnnObjectDetector.selectStrongestRegions(bboxes, scores, params.NumStrongestRegions); %#ok<PFBNS>
                    
                    s(i).RegionProposalBoxes = bboxes;                                       
                end                
            else
                
                for i = 1:numfiles
                    I = readimage(imds, i);
                    
                    imageSize = size(I);
                    
                    [trainingData(i,:), isvalid(i).HasValidData, isvalid(i).HasNoBoxes] = vision.internal.cnn.utils.hasValidTrainingData(imageSize,trainingData(i,:));
                                       
                    [bboxes, scores] = fcn(I, imds.Files{i});
                    
                    [bboxes, scores] = fastRCNNObjectDetector.filterSmallBBoxes(bboxes, scores, minBoxSize);
                    
                    bboxes = rcnnObjectDetector.selectStrongestRegions(bboxes, scores, params.NumStrongestRegions);
                    
                    s(i).RegionProposalBoxes = bboxes;
                end
            end                        
            
            regionProposals   = struct2table(s, 'AsArray', true);               
            hasNoBoxes        = vertcat(isvalid(:).HasNoBoxes);
            validTrainingData = ~hasNoBoxes & vertcat(isvalid(:).HasValidData);
            
            % Remove rows with no data
            trainingData(hasNoBoxes,:) = [];
            regionProposals(hasNoBoxes,:) = [];
            
            if isempty(trainingData)
                error(message('vision:rcnn:noValidTrainingData'));
            end
            
            printer.printMessage('vision:rcnn:regionProposalEnd');
            printer.linebreak;
        end

        %------------------------------------------------------------------
        function checkRegionProposalOutputs(bboxes, scores)
            if ~ismatrix(bboxes) || size(bboxes, 2) ~= 4
                error(message('vision:rcnn:invalidBBoxDim'));
            end
            
            if ~iscolumn(scores)
                error(message('vision:rcnn:invalidScoreDim'));
            end
            
            if ~isreal(bboxes) || issparse(bboxes) || ~all(isfinite(bboxes(:)))
                error(message('vision:rcnn:invalidBBox'));
            end
            
            if ~isreal(scores) || issparse(scores) || ~all(isfinite(scores))
                error(message('vision:rcnn:invalidScores'));
            end
            
            if size(bboxes, 1) ~= size(scores, 1)
                error(message('vision:rcnn:proposalInconsistentNumel'));
            end
            
            if any(bboxes(:,3) <= 0) || any(bboxes(:,4) <= 0)
                error(message('vision:rcnn:proposalInvalidWidthHeight'));
            end                                    
            
        end
        
        %------------------------------------------------------------------
        function issueWarningIfRequired(validRows)
            % Issue warning about training images being removed.
            invalidRows = find(~validRows);
            
            numRemoved = nnz(invalidRows);
            
            if numRemoved > 0
                total = numel(validRows);
                
                header = sprintf('Invalid Rows\n____________\n');
                rows   = sprintf('%d\n',reshape(invalidRows,[],1));
                msg    = sprintf('%s\n%s',header,rows);
                
                msg = message('vision:rcnn:removingInvalidTrainingData',...
                    numRemoved,total,msg);
                vision.internal.cnn.WarningLogger.warning(msg);
            end
        end
       
        %------------------------------------------------------------------
        function printRowOfStars(printer)
            printer.print('*******************************************************************\n');
            printer.linebreak;
        end
        
        %------------------------------------------------------------------
        function printFooter(printer)
            if vision.internal.cnn.WarningLogger.hasWarnings()
                printer.printMessage('vision:rcnn:trainingFooterWithWarnings');
                if isa(printer,'vision.internal.VerbosePrinter')
                    vision.internal.cnn.WarningLogger.reissueAllWarnings();
                end
            else
                printer.printMessage('vision:rcnn:trainingFooter');
            end
        end
        
        %------------------------------------------------------------------
        function [bboxes, scores] = invokeRegionProposalFcn(fcn, I, filename)
            
            % Call the custom function. Catch errors and issue as warning.
            % This is only used during training. For detect and
            % classifyRegions, the proposal function is called directly and
            % allowed to error.
            try
                [bboxes, scores] = fcn(I);
                
                rcnnObjectDetector.checkRegionProposalOutputs(bboxes, scores);
                
                imageSize = size(I);
                
                % Remove boxes that are completely outside the image.
                x1 = bboxes(:,1);
                y1 = bboxes(:,2);
                x2 = bboxes(:,3) + x1 - 1;
                y2 = bboxes(:,4) + y1 - 1;
                
                boxOverlapsImage = ...
                    (x1 < imageSize(2) & x2 > 1) & ...
                    (y1 < imageSize(1) & y2 > 1);
                
                bboxes = bboxes(boxOverlapsImage,:);
                scores = scores(boxOverlapsImage);
                
                % Clip boxes that extend beyond image boundaries.
                bboxes = vision.internal.detector.clipBBox(bboxes, imageSize);
                            
                % cast output to required type
                bboxes = double(bboxes);
                scores = single(scores);
                
            catch exception
                str = func2str(fcn);
                
                vision.internal.cnn.WarningLogger.warning(...
                    message('vision:rcnn:proposalFcnErrorOccurred', ...
                    str, filename, exception.message));
                
                bboxes = zeros(0, 4);
                scores = zeros(0, 1, 'single');
            end
            
        end
        
        %------------------------------------------------------------------
        % Returns the strongest N bboxes based on the scores.
        %------------------------------------------------------------------
        function [bboxes, scores] = selectStrongestRegions(bboxes, scores, N)
            if ~isinf(N)
                [~, idx] = sort(scores, 'descend');
                topN = min(N, numel(scores));
                idx = idx(1:topN);
                bboxes = bboxes(idx,:);
                scores = scores(idx,:);
            end
        end
    end
    
    %----------------------------------------------------------------------
    methods
        function [bboxes, scores, labels] = detect(this, I, varargin)
            params = rcnnObjectDetector.parseDetectInputs(I, varargin{:});
            
            roi    = params.ROI;
            useROI = params.UseROI;
            
            Iroi = vision.internal.detector.cropImageIfRequested(I, roi, useROI);
            
            [bboxes, boxScores] = this.RegionProposalFcn(Iroi);
            
            rcnnObjectDetector.checkRegionProposalOutputs(bboxes, boxScores);
            
            bboxes = rcnnObjectDetector.selectStrongestRegions(bboxes, boxScores, params.NumStrongestRegions);
            
            imageSize = imageInputSizeFromImageLayer(this);
            
            dispatcher = configureRegionDispatcher(this, Iroi, bboxes, params.MiniBatchSize, imageSize);
            
            [labels, allScores] = classify(this.Network, dispatcher, ...
                'MiniBatchSize', params.MiniBatchSize, ...
                'ExecutionEnvironment', params.ExecutionEnvironment);                   
                     
            % Remove undefined labels (generated by NaN scores)
            undefined = isundefined(labels);
            labels(undefined) = [];
            allScores(undefined,:) = [];
            bboxes(undefined,:) = [];
                        
            scores = getScoreAssociatedWithLabel(this, labels, allScores);
            
            % remove background class
            remove = labels == this.BackgroundLabel;
            bboxes(remove, :) = [];
            scores(remove, :) = [];
            labels(remove, :) = [];
            labels = removecats(labels, this.BackgroundLabel);
            
            % Apply class specific regression models
            if this.UseBBoxRegression
                dispatcher = configureRegionDispatcher(this, Iroi, bboxes, params.MiniBatchSize, imageSize);
                
                features   = activations(this.Network, dispatcher, this.BoxRegressionLayer,'OutputAs', 'columns');
                
                classNames = this.ClassNames;
                classNames(strcmp(this.BackgroundLabel, classNames)) = [];
                
                for j = 1:numel(classNames)
                    
                    name = classNames{j};
                    
                    if this.BBoxRegressionModel.(name).IsTrained
                        
                        idx = labels == name;
                        
                        bboxes(idx,:) = this.BBoxRegressionModel.(name).apply(features(:, idx), bboxes(idx,:));
                    end
                end
            end
            
            if params.SelectStrongest
                [bboxes, scores, labels] = selectStrongestBboxMulticlass(bboxes, scores, labels,...
                    'RatioType', 'Min', 'OverlapThreshold', 0.5);
            end
            
            % return bboxes in original image space.
            bboxes(:,1:2) = vision.internal.detector.addOffsetForROI(bboxes(:,1:2), roi, useROI);
            
        end
        
        %------------------------------------------------------------------
        function [labels, scores, allScores] = classifyRegions(this, I, roi, varargin)
            [roi, params] = rcnnObjectDetector.parseClassifyInputs(I, roi, varargin{:});
            
            imageSize = imageInputSizeFromImageLayer(this);
            
            dispatcher = configureRegionDispatcher(this, I, roi, params.MiniBatchSize, imageSize);
            
            [labels, allScores] = classify(this.Network, dispatcher, ...
                'MiniBatchSize', params.MiniBatchSize, ...
                'ExecutionEnvironment', params.ExecutionEnvironment);
                       
            % Remove undefined labels (generated by NaN scores)
            undefined = isundefined(labels);
            labels(undefined,:) = [];
            allScores(undefined,:) = [];
                        
            scores = getScoreAssociatedWithLabel(this, labels, allScores);
            
        end
    end
    
    methods
        
        %------------------------------------------------------------------
        function this = rcnnObjectDetector()
            this.UsingDefaultRegionProposalFcn = false;
            this.UseBBoxRegression = false;
            this.BackgroundLabel = 'Background';
        end
        
        %------------------------------------------------------------------
        function this = set.Network(this, network)
            
            validateattributes(network,{'SeriesNetwork','DAGNetwork'},{'scalar'});
            
            analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(network);
            analysis.applyConstraints();
            analysis.throwIssuesIfAny();
            vision.internal.cnn.validation.checkNetworkLayers(analysis);
            
            checkLayerForBackgroundLabel(this, network)
            
            this = updateLayerIndices(this, network);                   
            
            checkIfNetworkHasBBoxFeatureLayer(this,network);
           
            this.Network = network;
        end
        
        %------------------------------------------------------------------
        function this = set.RegionProposalFcn(this, fcn)
            rcnnObjectDetector.checkRegionProposalFcn(fcn);
            this.RegionProposalFcn = fcn;
        end
        
        %------------------------------------------------------------------
        function cls = get.ClassNames(this)
            if isempty(this.Network)
                cls = {};
            else
                cls = this.Network.Layers(this.ClassificationLayerIndex).ClassNames;
            end
        end
    end
    
    methods (Access = protected)
        
        function sz = imageInputSizeFromImageLayer(this)
            sz = this.Network.Layers(this.ImageLayerIndex).InputSize;
        end
        
        function dispatcher = createTrainingDispatcher(this, groundTruth, regionProposals, opts, imageSize, params)                        
            params.MiniBatchSize   = opts.MiniBatchSize;
            params.RegionResizer   = rcnnObjectDetector.createRegionResizer(imageSize);
            params.RandomSelector  = vision.internal.rcnn.RandomSelector();
            params.BackgroundLabel = this.BackgroundLabel;
            params.DispatchInBackground = params.UseParallel;
            
            ds = vision.internal.rcnn.TrainingImageRegionDatastore(...
                groundTruth, regionProposals, params);
            
            precision = nnet.internal.cnn.util.Precision('single');
            
            % Return a MBDS dispatcher to let R-CNN DS do its own
            % mini-batch sampling. Otherwise, the generic dispatcher used
            % in trainNetwork will not produce balanced batches.
            dispatcher = nnet.internal.cnn.dispatcher.MiniBatchDatastoreDispatcher (...
                ds, params.MiniBatchSize, 'truncateLast', opts.Shuffle, precision );
        end
        
        function dispatcher = configureRegionDispatcher(~, I, bboxes, miniBatchSize, imageSize)
            endOfEpoch    = 'truncateLast';
            precision     = nnet.internal.cnn.util.Precision('single');
            regionResizer = rcnnObjectDetector.createRegionResizer(imageSize);
            dispatcher    = vision.internal.rcnn.ImageRegionDispatcher(...
                I, bboxes, miniBatchSize, endOfEpoch, precision, imageSize, regionResizer);
        end
        
        %------------------------------------------------------------------
        % Returns classification score associated with a label.
        %------------------------------------------------------------------
        function scores = getScoreAssociatedWithLabel(this, labels, allScores)
            N = numel(this.ClassNames);
            M = numel(labels);
            
            ind = sub2ind([M N], 1:M, double(labels)');
            
            scores = allScores(ind)';
        end
        
        %------------------------------------------------------------------
        % The classification layer must have a "Background" class to
        % support the detect method. Networks trained using
        % trainRCNNObjectDetector will have this type of network.
        %------------------------------------------------------------------
        function checkLayerForBackgroundLabel(this, network)
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.ClassificationOutputLayer'),network.Layers);
            if ~ismember(this.BackgroundLabel, network.Layers(idx).ClassNames)
                error(message('vision:rcnn:missingBackgroundClass'));
            end
        end
        
        %------------------------------------------------------------------
        function this = updateLayerIndices(this, network)
            if isa(network,'SeriesNetwork')
                this.ClassificationLayerIndex = numel(network.Layers);
                this.ImageLayerIndex = 1;
                
            else % DAG Network
                isClassificationLayer = arrayfun(@(x)...
                    isa(x,'nnet.cnn.layer.ClassificationOutputLayer'),network.Layers);
                this.ClassificationLayerIndex = find(isClassificationLayer,1,'last');
                
                isImageLayer = arrayfun(@(x)...
                    isa(x,'nnet.cnn.layer.ImageInputLayer'),network.Layers);
                this.ImageLayerIndex = find(isImageLayer,1);
            end            
        end
        
        %------------------------------------------------------------------
        function checkIfNetworkHasBBoxFeatureLayer(this,network)
            if this.UseBBoxRegression
                % Setting network on a trained detector returned by
                % trainRCNNObjectDetector. BoxRegressionLayer must exist
                % exist in network. Otherwise, error out.
                names = {network.Layers.Name};
                
                if ~any(strcmp(this.BoxRegressionLayer, names))
                    error(message('vision:rcnn:networkMissingBBoxFeatureLayer'));
                end
            end
        end
    end
    
    %======================================================================
    % Save/Load
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.Network           = this.Network;
            s.RegionProposalFcn = this.RegionProposalFcn;
            s.UsingDefaultRegionProposalFcn = this.UsingDefaultRegionProposalFcn;
            s.UseBBoxRegression   = this.UseBBoxRegression;
            s.BBoxRegressionModel = this.BBoxRegressionModel;
            s.BoxRegressionLayer  = this.BoxRegressionLayer;
            s.BackgroundLabel     = this.BackgroundLabel;
            s.Version             = 2;
        end
    end
    
    methods(Static, Hidden)
        function this = loadobj(s)
            
            % Check if object can be loaded. Errors here will present
            % themselves as warnings during load.
            try
                vision.internal.requiresNeuralToolbox(mfilename);
                vision.internal.requiresStatisticsToolbox(mfilename);
                
                s = rcnnObjectDetector.updateToLatestVersion(s);
                
                this = rcnnObjectDetector();
                
                this.BackgroundLabel   = s.BackgroundLabel;
                
                if ~isempty(s.Network)
                    this.Network = s.Network;
                end
                
                if ~isempty(s.RegionProposalFcn)
                    this.RegionProposalFcn = s.RegionProposalFcn;
                end
                
                this.UsingDefaultRegionProposalFcn = s.UsingDefaultRegionProposalFcn;
                
                this.UseBBoxRegression   = s.UseBBoxRegression;
                this.BBoxRegressionModel = s.BBoxRegressionModel;
                this.BoxRegressionLayer    = s.BoxRegressionLayer;
                
            catch ME
                rethrow(ME)
            end
            
            
        end
    end
    
    %----------------------------------------------------------------------
    % Shared parameter validation routines.
    %----------------------------------------------------------------------
    methods(Hidden, Static)
        
        function checkRegionProposalFcn(func)
            
            validateattributes(func, {'function_handle'}, {'scalar'}, ...
                '', 'RegionProposalFcn');
            
            % get num args in/out. This errors out if func does not exist.
            numIn  = nargin(func);
            numOut = nargout(func);
            
            % functions may have varargin/out (i.e. anonymous functions)
            isVarargin  = (numIn  < 0);
            isVarargout = (numOut < 0);
            
            numIn  = abs(numIn);
            numOut = abs(numOut);
            
            % validate this API: [bboxes, scores] = func(I)
            if ~isVarargin && numIn ~= 1
                error(message('vision:rcnn:proposalFcnInvalidNargin'));
            end
            
            if ~isVarargout && numOut ~= 2
                error(message('vision:rcnn:proposalFcnInvalidNargout'));
            end
        end

        function imageInfo = imageRegressionInfoFromGroundTruthDatastore(trainingDatastore, params)
            printer = vision.internal.MessagePrinter.configure(params.Verbose);

            printer.printMessageNoReturn('vision:rcnn:regionProposalBeginDatastore');

            fcnCopy = params.RegionProposalFcn;
            fcn = @(x,filename)rcnnObjectDetector.invokeRegionProposalFcn(fcnCopy, x, filename);

            accum = zeros(1,1,params.InputSize(3));

            targets = {};
            numImages = 0;
            % TODO: Make this UseParallel aware using parfor.
            while hasdata(trainingDatastore)
                [data,info] = read(trainingDatastore);
                numImages = numImages + size(data, 1);

                [t, info] = ...
                    rcnnObjectDetector.extractTargetsUsingDataFromDatastore(data, info, fcn, params);

                targets = vertcat(targets, t);
                if params.NeedsZeroCenterNormalization
                    accum = accum + info.AccumulatedMean;
                end
            end

            missing = cellfun(@isempty, targets);

            targets(missing) = [];

            if isempty(targets)
                % Maybe increasing positive overlap range would help.
                error(message('vision:rcnn:noTrainingSamples'));
            end

            if params.NeedsZeroCenterNormalization
                avg = single(accum./numImages);
            else
                avg = single.empty(0,1);
            end

            imageInfo.AverageImage = avg;

            [imageInfo.BoxRegressionMean, imageInfo.BoxRegressionStd] = vision.internal.cnn.utils.meanStdForTargets(targets);

            printer.printMessage('vision:rcnn:regionProposalEnd');
            printer.linebreak;
        end
    end
    
    %----------------------------------------------------------------------
    methods(Hidden, Static, Access = private)
        function [roi, params] = parseClassifyInputs(I, roi, varargin)
            p = inputParser;
            p.addParameter('MiniBatchSize', 128);
            p.addParameter('ExecutionEnvironment', 'auto');
            parse(p, varargin{:});
            
            userInput = p.Results;
            
            % grayscale or RGB images allowed
            vision.internal.inputValidation.validateImage(I, 'I');
            
            roi = rcnnObjectDetector.checkROIs(roi, size(I));
            
            vision.internal.cnn.validation.checkMiniBatchSize(...
                userInput.MiniBatchSize, mfilename);
            
            exeenv = vision.internal.cnn.validation.checkExecutionEnvironment(...
                userInput.ExecutionEnvironment, mfilename);
            
            params.MiniBatchSize        = double(userInput.MiniBatchSize);
            params.ExecutionEnvironment = exeenv;
        end
        
        %------------------------------------------------------------------
        function params = parseDetectInputs(I, varargin)
            
            p = inputParser;
            p.addOptional('roi', zeros(0,4));
            p.addParameter('NumStrongestRegions', 2000);
            p.addParameter('SelectStrongest', true);
            p.addParameter('MiniBatchSize', 128);
            p.addParameter('ExecutionEnvironment', 'auto');
            parse(p, varargin{:});
            
            userInput = p.Results;
            
            % grayscale or RGB images allowed
            vision.internal.inputValidation.validateImage(I, 'I');
            
            useROI = ~ismember('roi', p.UsingDefaults);
            
            if useROI
                vision.internal.detector.checkROI(userInput.roi, size(I));
            end
            
            vision.internal.inputValidation.validateLogical(...
                userInput.SelectStrongest, 'SelectStrongest');
            
            rcnnObjectDetector.checkStrongestRegions(userInput.NumStrongestRegions);
            vision.internal.cnn.validation.checkMiniBatchSize(...
                userInput.MiniBatchSize, mfilename);
            
            exeenv = vision.internal.cnn.validation.checkExecutionEnvironment(...
                userInput.ExecutionEnvironment, mfilename);
            
            params.ROI                  = double(userInput.roi);
            params.UseROI               = useROI;
            params.NumStrongestRegions  = double(userInput.NumStrongestRegions);
            params.SelectStrongest      = logical(userInput.SelectStrongest);
            params.MiniBatchSize        = double(userInput.MiniBatchSize);
            params.ExecutionEnvironment = exeenv;
        end
        
        %------------------------------------------------------------------
        function checkStrongestRegions(N)
            if isinf(N)
                % OK, use all regions.
            else
                validateattributes(N, ...
                    {'numeric'},...
                    {'scalar', 'real', 'positive', 'integer', 'nonempty', 'finite', 'nonsparse'}, ...
                    mfilename, 'NumStrongestRegions');
            end
        end
        
        %------------------------------------------------------------------
        function roi = checkROIs(roi, imageSize)
            validateattributes(roi, {'numeric'}, ...
                {'size', [NaN 4], 'real', 'nonsparse', 'finite', 'nonsparse'},...
                mfilename, 'roi');
            
            % rounds floats and casts to int32 to avoid saturation of smaller integer types.
            roi = vision.internal.detector.roundAndCastToInt32(roi);
            
            % width and height must be >= 0
            if any(roi(:,3) < 0) || any(roi(:,4) < 0)
                error(message('vision:validation:invalidROIWidthHeight'));
            end
            
            % roi must be fully contained within I
            if any(roi(:,1) < 1) || any(roi(:,2) < 1)...
                    || any(roi(:,1) + roi(:,3) > imageSize(2)+1) ...
                    || any(roi(:,2) + roi(:,4) > imageSize(1)+1)
                
                error(message('vision:validation:invalidROIValue'));
            end
            
        end                
        
        %------------------------------------------------------------------
        function printHeader(printer, groundTruth)
            printer.print('*******************************************************************\n');
            printer.printMessage('vision:rcnn:trainingHeader');
            printer.linebreak;
            
            classNames = groundTruth.Properties.VariableNames(2:end);
            
            for i = 1:numel(classNames)
                printer.print('* %s\n', classNames{i});
            end
            
            printer.linebreak;
            
        end               
        
        %------------------------------------------------------------------
        function updateMessage(printer, prevMessage, nextMessage)
            backspace = sprintf(repmat('\b',1,numel(prevMessage))); % figure how much to delete
            printer.printDoNotEscapePercent([backspace nextMessage]);
        end
        
        %------------------------------------------------------------------
        function nextMessage = printProgress(printer, prevMessage, k, K)
            nextMessage = sprintf('%.2f%%%%',100*k/K);
            rcnnObjectDetector.updateMessage(printer, prevMessage(1:end-1), nextMessage);
        end
        
        %------------------------------------------------------------------
        function layerNameOrIndex = autoSelectBBoxFeatureLayer(layerGraphOrLayers)
            if isa(layerGraphOrLayers,'nnet.cnn.layer.Layer')
                % For series networks, choose the last convolution layer.
                lastConvolutionalLayer = find( ...
                    arrayfun(@(x)...
                    isa(x,'nnet.cnn.layer.Convolution2DLayer'), ...
                    layerGraphOrLayers), ...
                    1, 'last');
                layerNameOrIndex = char(layerGraphOrLayers(lastConvolutionalLayer).Name);
                
                if isempty(layerNameOrIndex)
                    % layers in array can have empty names, return as index
                    % instead. It will be converted to a name after
                    % network training.
                    layerNameOrIndex = lastConvolutionalLayer;
                end
            else
                % For dag networks, choose the layer preceding the last
                % fully connected layer. This is the least ambiguous layer
                % to select for DAG networks and provides a sensible
                % feature layer for bounding box regression.
                lastFCLayerIndex = vision.internal.cnn.utils.findLastFullyConnectedLayer(layerGraphOrLayers);
                if isempty(lastFCLayerIndex)
                    error(message('vision:rcnn:noConvLayerSkipBoxReg'));  
                end
                lastFCLayerName = layerGraphOrLayers.Layers(lastFCLayerIndex).Name;
                
                % Find entry in Connections table.
                index = find(strcmp(lastFCLayerName,layerGraphOrLayers.Connections.Destination));
                
                % Find source of last FC layer.
                layerNameOrIndex = layerGraphOrLayers.Connections.Source{index}; %#ok<FNDSB>                                
            end
        end              
               
        %------------------------------------------------------------------
        function s = updateToLatestVersion(s)
            if ~isfield(s,'Version') 
                % version 1          
                
                % In v1, BoxRegressionLayer is an index. Update it to the
                % corresponding layer name that it references. Because in
                % v1 the network is a SeriesNetwork we can index into the
                % Network.Layers to find the name.
                if ~isempty(s.Network)
                    s.BoxRegressionLayer = s.Network.Layers(s.BBoxFeatureLayer).Name;                
                end
            end
        end                
    end
    
    methods(Hidden, Access = protected)
        function groups = getPropertyGroups( this )
            propertyList = struct;
            propertyList.Network = this.Network;
            propertyList.RegionProposalFcn = this.RegionProposalFcn;
            % Transpose class names, which is a column vector
            propertyList.ClassNames = this.ClassNames';
            propertyList.BoxRegressionLayer = char(this.BoxRegressionLayer);
            
            groups = matlab.mixin.util.PropertyGroup(propertyList);
        end
    end
end

%--------------------------------------------------------------------------
function name = iLayerName(index,network)
name = index;
if isnumeric(index)
    name = network.Layers(index).Name;
end
end

%--------------------------------------------------------------------------
function iWarnIfTrainingLossHasNaNs(info)
if ~iAllElementsAreFinite(info.TrainingLoss)
    % network is not trained properly.
    msg = message('vision:rcnn:networkTrainingLossHasNaNs');
    vision.internal.cnn.WarningLogger.warning(msg); 
end
end

%--------------------------------------------------------------------------
function tf = iAllElementsAreFinite(x)
tf = all(isfinite(x(:)));
end

%--------------------------------------------------------------------------
function filename = iGetFilenameFromReadInfo(info, index, imgSize, retry)
if nargin == 3
    % Let's not recurse into a cell array more than once just to find the
    % filename.
    retry = 1;
end

if iscell(info) && numel(info) == 2 && retry == 1
    info = info{1};
    filename = iGetFilenameFromReadInfo(info, index, imgSize, retry + 1);
elseif isstruct(info) && isfield(info, 'Filename')
    filename = info.Filename;
    if iscell(filename)
        filename = filename{index};
    end
else
    % We are not able to find a filename from the info of the datastore's
    % read method.
    % This is just used in the error message when invoking RegionProposalFcn.
    % Just use the image size to provid some information about the image.
    sizeStr = join(string(imgSize),",")
    filename = "Image of size: [" + sizeStr + "]";
end
end

%--------------------------------------------------------------------------
function [targets, posIdx, negIdx, labels] = iCreatePositiveRegressionTargets(posOverlap, negOverlap, groundTruth, regionProposals, info)
    narginchk(4,5);
    % Generate targets for ground truth boxes.
    matchAllGroundTruth = false;
    [targets, posIdx, negIdx, assignments] = vision.internal.rcnn.BoundingBoxRegressionModel.generateRegressionTargetsFromProposals(...
                                    regionProposals, groundTruth, posOverlap, negOverlap, matchAllGroundTruth);

    if nargout > 3
        % Assign label to each region proposal.
        labels = vision.internal.cnn.boxAssignmentUtils.boxLabelsFromAssignment(...
            assignments, info.Labels, posIdx, negIdx, ...
            info.ClassNames, info.BackgroundLabel);
    end
end

%--------------------------------------------------------------------------
function tf = iIsParallelExecutionEnvironment(executionEnvironment)
    tf = ismember( executionEnvironment, {'multi-gpu', 'parallel'} );
end
