classdef SOLOv2 < coder.internal.NetworkWrapper  %#codegen
    % Codegen class for solov2 network

    % Copyright 2024 The MathWorks, Inc.
    properties
        FeatureNet

        NeckNet

        MaskFeatNet

        KernHeadNet

        KernHeadNet2

        KernHeadNet3

        KernHeadNet4

        KernHeadNet5

        ClsHeadNet

        ClsHeadNet2

        ClsHeadNet3

        ClsHeadNet4

        ClsHeadNet5

        ModelName

        ClassNames

        InputSize

        GridSizes

        NormalizationStatistics

        Strides = [32, 32, 16, 8, 8];

        InitialThresh = 0.1;

        ScoreThreshold

        MaskThreshold = 0.5;

        MinSize

        MaxSize

        UseSelectStrongest

    end

    methods (Hidden, Static, Access = public)

        %------------------------------------------------------------------
        % Function to make the properties constant at compile time.
        %------------------------------------------------------------------

        function n = matlabCodegenNontunableProperties(~)
            n = {'ModelName', 'ClassNames', 'InputSize', 'GridSizes', 'NormalizationStatistics'};
        end

        function name = matlabCodegenUserReadableName(~)
            name = 'solov2';
        end

        function n = matlabCodegenNetworkProperties(~)
            n = {'FeatureNet', 'NeckNet', 'MaskFeatNet', 'KernHeadNet','KernHeadNet2', ...
                'KernHeadNet3', 'KernHeadNet4', 'KernHeadNet5', ...
                'ClsHeadNet', 'ClsHeadNet2', 'ClsHeadNet3', 'ClsHeadNet4', 'ClsHeadNet5'};
        end
    end

    methods
        function obj = SOLOv2(matfile, networkName)
            % Initialize the detector
            obj = obj@coder.internal.NetworkWrapper(matfile, networkName);

            % Get all the network properties
            coder.extrinsic('vision.internal.codegen.SOLOv2.getNetworkProperties');
            [obj.ModelName, obj.ClassNames, obj.InputSize, obj.GridSizes,...
                obj.NormalizationStatistics] = ...
                coder.const(@vision.internal.codegen.SOLOv2.getNetworkProperties, matfile);
        end

        function [masks, labels, scores] = segmentObjects(this, I, varargin)

            [params] = this.iParseDetectInputs(I, varargin{:});

            this.ScoreThreshold = params.Threshold;
            this.MaskThreshold = params.MaskThreshold;
            this.UseSelectStrongest = params.SelectStrongest;
            this.MinSize = params.MinSize;
            this.MaxSize = params.MaxSize;

            [masks, labels, scores] =  this.segmentObjectsInImgStack(I, params.MiniBatchSize, params.MaxNumKernels);
        end

        function tf = isa(~, class)
            coder.inline("always");
            coder.internal.prefer_const(class);
            coder.internal.errorIf(~coder.internal.isConst(class),"dlcoder_spkg:CoderNetwork:ExpectedConstClassName");
            tf = coder.const(strcmp(class,'SOLOv2')) || ...
                coder.const(strcmp(class, 'vision.internal.codegen.SOLOv2'));
        end

        function className = class(~)
            coder.inline("always");
            className = 'SOLOv2';
        end

    end

    methods(Hidden = true, Static)
        %------------------------------------------------------------------
        % Get function to fetch the solov2 Network properties.
        %------------------------------------------------------------------
        function [modelName, classNames, inputSize, gridSize, normStats] = getNetworkProperties(matfile)
            detectorObj = coder.internal.loadDeepLearningNetwork(matfile);

            classNames = detectorObj.ClassNames;
            inputSize = detectorObj.InputSize;
            gridSize = detectorObj.GridSizes;
            modelName = detectorObj.ModelName;
            normStats = detectorObj.NormalizationStatistics;
        end
    end

    methods(Hidden,Access = private)
        function [params] = iParseDetectInputs(this, I, varargin)
            coder.inline('always');
            coder.internal.prefer_const(I, varargin)

            possibleNameValues = struct(...
                'Threshold',uint32(0),...
                'MaskThreshold',uint32(0),...
                'SelectStrongest',uint32(0),...
                'MinSize',uint32(0), ...
                'MaxSize',uint32(0), ...
                'ExecutionEnvironment',uint32(0), ...
                'Acceleration',uint32(0),...
                'MiniBatchSize', uint32(0),...
                'MaxNumKernels',uint32(0));

            poptions = struct( ...
                'CaseSensitivity',false, ...
                'PartialMatching','unique', ...
                'StructExpand',false, ...
                'IgnoreNulls',true);

            % Validate input image
            tf = isnumeric(I) && ndims(I)<=4 && (size(I,3)==3||size(I,3)==1) && isreal(I);
            coder.internal.errorIf((~tf || isempty(I)),...
                'vision:solov2:invalidImageInput');

            % Get input size
            imageSize = coder.nullcopy(zeros(1,4));
            [imageSize(1), imageSize(2) ,imageSize(3), imageSize(4)] = size(I);

            % Assert for variable sized channel or batch dimensions
            coder.internal.assert(coder.internal.isConst([imageSize(3) imageSize(4)]), ...
                'vision:solov2:VariableSizeChannelBatch',mfilename);

            defaults = struct('Threshold', 0.5,...
                'MaskThreshold', 0.5, ...
                'SelectStrongest', true, ...
                'MinSize', [1 1],...
                'MaxSize', this.InputSize(1:2), ...
                'ExecutionEnvironment', 'auto', ...
                'Acceleration', 'auto', ...
                'MiniBatchSize', 1, ...
                'MaxNumKernels', 3097); % random default value for codegen

            pstruct = coder.internal.parseParameterInputs(possibleNameValues, poptions, varargin{:});

            % Check if MaxNumKernels is provided by user
            coder.internal.errorIf((coder.const(pstruct.MaxNumKernels == zeros('uint32'))),...
                'vision:solov2:MaxNumKernelsRequired');

            params.Threshold = coder.internal.getParameterValue(pstruct.Threshold, defaults.Threshold, varargin{:});
            params.MaskThreshold = coder.internal.getParameterValue(pstruct.MaskThreshold, defaults.MaskThreshold, varargin{:});
            params.SelectStrongest = coder.internal.getParameterValue(pstruct.SelectStrongest, defaults.SelectStrongest, varargin{:});
            params.MinSize = coder.internal.getParameterValue(pstruct.MinSize, defaults.MinSize, varargin{:});
            params.MaxSize = coder.internal.getParameterValue(pstruct.MaxSize, defaults.MaxSize, varargin{:});
            params.MiniBatchSize = coder.internal.getParameterValue(pstruct.MiniBatchSize, defaults.MiniBatchSize, varargin{:});
            params.ExecutionEnvironment = coder.internal.getParameterValue(pstruct.ExecutionEnvironment, defaults.ExecutionEnvironment, varargin{:});
            params.Acceleration = coder.internal.getParameterValue(pstruct.Acceleration, defaults.Acceleration, varargin{:});
            params.MaxNumKernels = coder.internal.getParameterValue(pstruct.MaxNumKernels, defaults.MaxNumKernels, varargin{:});

            % Validate Threshold
            validateattributes(params.Threshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>', 0, '<=', 1}, ...
                mfilename, 'Threshold');

            % Validate MaskThreshold
            validateattributes(params.MaskThreshold, {'single', 'double'}, {'nonempty', 'nonnan', ...
                'finite', 'nonsparse', 'real', 'scalar', '>', 0, '<=', 1}, ...
                mfilename, 'MaskThreshold');

            % Validate SelectStrongest
            vision.internal.inputValidation.validateLogical(...
                params.SelectStrongest, 'SelectStrongest');

            % Validate MiniBatchSize.
            validateattributes(params.MiniBatchSize, {'numeric'}, ...
                {'scalar', 'finite', 'real', 'integer', 'positive'}, mfilename, ...
                'MiniBatchSize');

            % Validate MaxNumKernels
            validateattributes(params.MaxNumKernels, {'numeric'}, ...
                {'scalar', 'finite', 'real', 'integer', 'positive'}, mfilename, ...
                'MaxNumKernels');

            coder.internal.errorIf((params.MaxNumKernels > 3096 || params.MaxNumKernels < 1),...
                'vision:solov2:invalidMaxNumKernels');

            % Validate MinSize and MaxSize only if they are set to non-default values
            validateMinSize = coder.const(pstruct.MinSize ~= zeros('uint32')) && ~isequal(params.MinSize,[1 1]);
            validateMaxSize = coder.const(pstruct.MaxSize ~= zeros('uint32')) && ~isequal(params.MaxSize,this.InputSize(1:2));

            if isequal(params.MaxSize, this.InputSize(1:2))
                params.MaxSize = imageSize(1:2);
            end

            if validateMinSize
                vision.internal.detector.ValidationUtils.checkMinSize(params.MinSize, [1,1], mfilename);
            end

            if validateMaxSize
                vision.internal.detector.ValidationUtils.checkSize(params.MaxSize, 'MaxSize', mfilename);
            end

            if(ndims(I)<=3)
                params.MiniBatchSize = 1;
            end

            % Ignore ExecutionEnvironment
            if coder.const(pstruct.ExecutionEnvironment ~= zeros('uint32'))
                coder.internal.compileWarning(...
                    'vision:solov2:IgnoreInputArg', 'segmentObjects', 'ExecutionEnvironment');
            end

            % Ignore Acceleration
            if coder.const(pstruct.Acceleration ~= zeros('uint32'))
                coder.internal.compileWarning(...
                    'vision:solov2:IgnoreInputArg', 'segmentObjects', 'Acceleration');
            end
        end

        %-------------------------------------------------------------------------------------
        function [masks, labels, scores] =  segmentObjectsInImgStack(this, I,...
                miniBatchSize, maxNumKern)
            coder.inline('always');
            coder.internal.prefer_const(I, miniBatchSize);

            isBatchOfImages = coder.const(size(I,4)>1);
            stackSize = coder.internal.indexInt(size(I,4));

            if isBatchOfImages
                coder.varsize('masks');
                coder.varsize('labels');
                coder.varsize('scores');

                masks = coder.nullcopy(cell(stackSize, 1));
                labels = coder.nullcopy(cell(stackSize, 1));
                scores = coder.nullcopy(cell(stackSize, 1));
                currPos = 1;
                imBatch = zeros(size(I,1), size(I,2), size(I,3), miniBatchSize, 'like', I);

                for startIdx = 1 : miniBatchSize : stackSize
                    endIdx = min(startIdx+miniBatchSize-1, stackSize);
                    actualBatchSize = endIdx - startIdx + 1;
                    imBatch(:,:,:,1:actualBatchSize) = I(:,:,:,startIdx:endIdx);

                    % Check if actualBatchSize < miniBatchSize
                    if actualBatchSize < miniBatchSize
                        % Pad imBatch by repeating last image to fill the batch
                        for padIdx = actualBatchSize+1 : miniBatchSize
                            imBatch(:,:,:,padIdx) = I(:,:,:,endIdx); % Repeat last image
                        end
                    end

                    [masksCell, labelsCell, scoresCell] = this.segmentObjectsInBatch(imBatch, maxNumKern, miniBatchSize);

                    numMasks = numel(masksCell);
                    for i = 1 : numMasks
                        if currPos+i-1 <= stackSize
                            masks{currPos+i-1} = masksCell{i};
                            labels{currPos+i-1} = labelsCell{i};
                            scores{currPos+i-1} = scoresCell{i};
                        end
                    end
                    currPos = currPos+numMasks;
                end
            else
                imBatch = I(:,:,:,1);
                miniBatchSize = 1;
                [masksCell, labelsCell, scoresCell] = this.segmentObjectsInBatch(imBatch, maxNumKern, miniBatchSize);
                masks = masksCell{1};
                labels = labelsCell{1};
                scores = scoresCell{1};
            end
        end

        %-------------------------------------------------------------------------------------
        function [masks, labels, scores] = segmentObjectsInBatch(this, I, maxNumKern, miniBatchSize)
            coder.internal.prefer_const(I);

            % Preprocess input
            [I , info] = preProcessInput(I, this.InputSize(1:2), miniBatchSize);
            Ipreprocessed = dlarray(I, 'SSCB');

            % Predict on the solov2 network
            [kernPreds, clsPreds, maskFeats] = this.predict(Ipreprocessed);

            % Post process predictions
            [masks, labels, scores] = this.postProcessOutputs(kernPreds, clsPreds, maskFeats, info, maxNumKern);
        end

        %-------------------------------------------------------------------------------------
        function [kernPreds, clsPreds, maskFeats] = predict(this, X)
            coder.internal.prefer_const(X);

            % Forward pass on the backbone
            [P2, P3, P4, P5] = predict(this.FeatureNet, X);

            % Forward pass on the FPN Neck
            F = coder.nullcopy(cell(1,5));
            [F{5}, F{4}, F{3}, F{2}, F{1}] = predict(this.NeckNet, P2, P3, P4, P5);

            kernPreds = cell(1,5);
            clsPreds = cell(1,5);

            % Input to predict function should be same on given network
            % object. Loop is unrolled and duplicate networks are created
            % to operate on different inputs - g3132084

            % level 1
            lvlFeatOne = dlresize(F{1}, OutputSize=[25,25], Method="linear",...
                GeometricTransformMode="half-pixel",...
                NearestRoundingMode="floor");

            lvlCoordsOne = this.generateCoordinates(size(lvlFeatOne));
            lvlFeatOne = cat(3, lvlFeatOne, lvlCoordsOne);

            featLvlOne = dlresize(lvlFeatOne, OutputSize=[this.GridSizes(1) this.GridSizes(1)], ...
                Method="linear", GeometricTransformMode="half-pixel",...
                NearestRoundingMode="floor");

            % Forward pass on the Kernel head (with coordconv features)
            kernPreds{1} = predict(this.KernHeadNet, featLvlOne);
            % Forward pass on the Classification head (without coordconv features)
            clsPreds{1} = predict(this.ClsHeadNet, featLvlOne(:,:,1:end-2,:));

            % level 2
            lvlFeatTwo = F{2};

            lvlCoordsTwo = this.generateCoordinates(size(lvlFeatTwo));
            lvlFeatTwo = cat(3, lvlFeatTwo, lvlCoordsTwo);

            featLvlTwo = dlresize(lvlFeatTwo, OutputSize=[this.GridSizes(2) this.GridSizes(2)], ...
                Method="linear", GeometricTransformMode="half-pixel",...
                NearestRoundingMode="floor");

            kernPreds{2} = predict(this.KernHeadNet2, featLvlTwo);
            clsPreds{2} = predict(this.ClsHeadNet2,featLvlTwo(:,:,1:end-2,:));

            % level 3
            lvlFeatThree = F{3};

            lvlCoordsThree = this.generateCoordinates(size(lvlFeatThree));
            lvlFeatThree = cat(3, lvlFeatThree, lvlCoordsThree);

            featLvlThree = dlresize(lvlFeatThree, OutputSize=[this.GridSizes(3) this.GridSizes(3)], ...
                Method="linear", GeometricTransformMode="half-pixel",...
                NearestRoundingMode="floor");

            kernPreds{3} = predict(this.KernHeadNet3, featLvlThree);
            clsPreds{3} = predict(this.ClsHeadNet3, featLvlThree(:,:,1:end-2,:));

            % level 4
            lvlFeatFour = F{4};

            lvlCoordsFour = this.generateCoordinates(size(lvlFeatFour));
            lvlFeatFour = cat(3, lvlFeatFour, lvlCoordsFour);

            featLvlFour = dlresize(lvlFeatFour, OutputSize=[this.GridSizes(4) this.GridSizes(4)], ...
                Method="linear", GeometricTransformMode="half-pixel",...
                NearestRoundingMode="floor");

            kernPreds{4} = predict(this.KernHeadNet4, featLvlFour);
            clsPreds{4} = predict(this.ClsHeadNet4, featLvlFour(:,:,1:end-2,:));

            % level 5
            lvlFeatFive = dlresize(F{5},Scale=[0.5,0.5], Method="linear",...
                GeometricTransformMode="half-pixel",...
                NearestRoundingMode="floor");

            lvlCoordsFive = this.generateCoordinates(size(lvlFeatFive));
            lvlFeatFive = cat(3, lvlFeatFive, lvlCoordsFive);

            featLvlFive = dlresize(lvlFeatFive, OutputSize=[this.GridSizes(5) this.GridSizes(5)], ...
                Method="linear", GeometricTransformMode="half-pixel",...
                NearestRoundingMode="floor");

            kernPreds{5} = predict(this.KernHeadNet5, featLvlFive);
            clsPreds{5} = predict(this.ClsHeadNet5, featLvlFive(:,:,1:end-2,:));

            F2 = cat(3, F{2}, this.generateCoordinates(size(F{2})));
            maskFeats = predict(this.MaskFeatNet,F{5}, F{4}, F{3}, F2);
        end

        %-------------------------------------------------------------------------------
        function coordFeats = generateCoordinates(~, featSize)
            % This function generates a normalized map of feature
            % coordinates for the coordconv maps.
            coder.internal.prefer_const(featSize);

            xR = linspace(-1, 1, featSize(2));
            yR = linspace(-1, 1, featSize(1));

            [xCoord, yCoord] = meshgrid(xR,yR);

            xCoord = repmat(xCoord, 1,1,1,featSize(4));
            yCoord = repmat(yCoord, 1,1,1,featSize(4));

            coordFeats = dlarray(cat(3,xCoord, yCoord), 'SSCB');
        end

        %-------------------------------------------------------------------------------
        function [masks, labels, scores] = postProcessOutputs(this, kernPredsBatch, clsPredsBatch, maskFeatsBatch, scaleInfo, maxNumKern)
            coder.internal.prefer_const(kernPredsBatch, clsPredsBatch, maskFeatsBatch, scaleInfo, maxNumKern);

            batchSize = size(maskFeatsBatch, 4);
            masks = coder.nullcopy(cell(batchSize,1));
            labels = coder.nullcopy(cell(batchSize,1));
            scores = repmat({zeros(coder.ignoreConst(0),coder.ignoreConst(0),'single')},batchSize,1);

            for idx = 1 : coder.internal.indexInt(batchSize)
                clsPredsBatchSize = size(clsPredsBatch);
                mlvlClassScore = coder.nullcopy(cell(1,5));

                numCls = clsPredsBatchSize(2);
                for j = 1 : coder.internal.indexInt(numCls)
                    clsPreds = clsPredsBatch{j};
                    mlvlClassScore{j} = clsPreds(:,:,:,idx);
                end

                kernPredsBatchSize = size(kernPredsBatch);
                mlvlKern = coder.nullcopy(cell(1,5));

                kern = kernPredsBatchSize(2);
                for k = 1 : coder.internal.indexInt(kern)
                    kernPredictions = kernPredsBatch{k};
                    mlvlKern{k} = kernPredictions(:,:,:,idx);
                end

                maskFeat = maskFeatsBatch(:,:,:,idx);

                %-------------------------------------------------------------------------------------------------------------
                % Step 1: Perform non max suppression on the class scores in spatial dimension
                %-------------------------------------------------------------------------------------------------------------

                % maxpool is not supported for variable size inputs. Loop
                % unrolling will make inputs fixed size - g3173354

                classScore1 = mlvlClassScore{1};
                localMax1 = maxpool(classScore1, [2,2], 'Stride', 1, 'Padding','same');
                keepValues1 = (classScore1==localMax1);
                classScore1 = classScore1.*keepValues1;
                mlvlClassScore1 = classScore1;

                classScore2 = mlvlClassScore{2};
                localMax2 = maxpool(classScore2, [2,2], 'Stride', 1, 'Padding','same');
                keepValues2 = (classScore2==localMax2);
                classScore2 = classScore2.*keepValues2;
                mlvlClassScore2 = classScore2;

                classScore3 = mlvlClassScore{3};
                localMax3 = maxpool(classScore3, [2,2], 'Stride', 1, 'Padding','same');
                keepValues3 = (classScore3==localMax3);
                classScore3 = classScore3.*keepValues3;
                mlvlClassScore3 = classScore3;

                classScore4 = mlvlClassScore{4};
                localMax4 = maxpool(classScore4, [2,2], 'Stride', 1, 'Padding','same');
                keepValues4 = (classScore4==localMax4);
                classScore4 = classScore4.*keepValues4;
                mlvlClassScore4 = classScore4;

                classScore5 = mlvlClassScore{5};
                localMax5 = maxpool(classScore5, [2,2], 'Stride', 1, 'Padding','same');
                keepValues5 = (classScore5==localMax5);
                classScore5 = classScore5.*keepValues5;
                mlvlClassScore5 = classScore5;

                mlvlClassScores = {mlvlClassScore1, mlvlClassScore2, mlvlClassScore3, mlvlClassScore4, mlvlClassScore5};

                %-------------------------------------------------------------------------------------------------------------
                % Step 2: Reshape Class scores and kernel to SxC & platten all levels to a single list
                %-------------------------------------------------------------------------------------------------------------
                maskClsPreds1 = coder.nullcopy(cell(1,5));
                for k = 1:5
                    maskClsPreds1{k} = reshape(mlvlClassScores{k} , [size(mlvlClassScores{k},1)*size(mlvlClassScores{k},2), size(mlvlClassScores{k},3)]);
                end
                maskClsPreds = cat(1, maskClsPreds1{:});

                kernPreds = calculateKernPreds(mlvlKern);

                % Threshold scores with the initialThresh value
                keepScores = maskClsPreds>this.InitialThresh;

                [keepIdxsRow, keepIdxsCol] = find(extractdata(keepScores));
                keepIdxs = [keepIdxsRow, keepIdxsCol];
                maskClsPredsFilt = maskClsPreds(keepScores);
                clsLabels = keepIdxs(:,2);
                kernPredsFilt = kernPreds(keepIdxs(:,1), :);

                % Sort the scores
                maskClsPredsArr = extractdata(maskClsPredsFilt);

                if coder.gpu.internal.isGpuEnabled
                    [maskClsPredsSort1,index] = gpucoder.sort(maskClsPredsArr, 1,'descend');
                else
                    [maskClsPredsSort1,index] = sort(maskClsPredsArr, 1,'descend');
                end
                maskClsPredsSort = dlarray(maskClsPredsSort1);
                clsLabelsSort = clsLabels(index);
                kernPredsSort = kernPredsFilt(index,:);

                scale = scaleInfo.Scale;
                originalSize = scaleInfo.OriginalSize;

                numClasses = length(this.ClassNames);

                % If no predictions, return empty masks, scores and labels
                if(isempty(kernPredsSort))
                    masks{idx} = false(originalSize(1), originalSize(2), 0);
                    scores{idx} = zeros(0, 0, 'single');
                    labels{idx} = categorical([], 1:numClasses, this.ClassNames);
                    continue;
                end

                %----------------------------------------------------------------------------------------------------------
                % Step 3: Compute strides for each of the kept indices
                %----------------------------------------------------------------------------------------------------------

                intervalsSize = this.GridSizes.^2;
                fpnStrides = zeros(0,1);
                for i = 1:numel(intervalsSize)
                    lvlStrides = repmat(this.Strides(i), intervalsSize(i),1);
                    fpnStrides = [fpnStrides;lvlStrides];
                end
                fpnStridesFilt = fpnStrides(keepIdxs(:,1));

                %---------------------------------------------------------------------------------------------------------
                % Step4: Generate conv using dynamic conv reshape kernels to 1x1xnumFeaturesxnumFilters
                %---------------------------------------------------------------------------------------------------------
                kernPredsArr = extractdata(kernPredsSort);
                kernPredsPerm = permute(kernPredsArr, [3 4 2 1]);

                % maxNumKernels is greater than actual kernels
                numFilt = size(kernPredsPerm, 4);
                if maxNumKern > numFilt
                    kernpredsPadSize = [size(kernPredsPerm,1:3), maxNumKern];
                    kernPredsPad = zeros(kernpredsPadSize, class(kernPredsPerm));
                    kernPredsPad(:,:,:,1:numFilt) = kernPredsPerm(:,:,:,:);
                    fpnStridesFiltNew = zeros(maxNumKern,1);
                    fpnStridesFiltNew(1:numFilt) = fpnStridesFilt(1:numFilt);
                    fpnStridesFiltNew(numFilt+1:maxNumKern) = 0;
                else
                    kernPredsPad = kernPredsPerm;
                    fpnStridesFiltNew = fpnStridesFilt;
                end

                outKernPreds = kernPredsPad(:,:,:,1:maxNumKern);
                maskPreds = dlconv(maskFeat, outKernPreds, 0);
                maskPreds = sigmoid(maskPreds);
                masksPreNMS = maskPreds>this.MaskThreshold;
                %---------------------------------------------------------------------------------------------------------
                % Step 5: Filter out masks less than stride
                %---------------------------------------------------------------------------------------------------------
                masksFilt = sum(masksPreNMS,[1 2]);
                maskArea = masksFilt(:);
                keepMasksIdx = maskArea>fpnStridesFiltNew(1:maxNumKern);

                masksPreNMSFilt = masksPreNMS(:,:,keepMasksIdx);
                maskPredsFilt = maskPreds(:,:,keepMasksIdx);
                maskAreaFilt = maskArea(keepMasksIdx);

                position = extractdata(keepMasksIdx);

                if maxNumKern > numFilt
                    clsScores = maskClsPredsSort(keepMasksIdx(1:numFilt));
                    clsLabelsFilt = clsLabelsSort(position(1:numFilt));
                else
                    clsScores = maskClsPredsSort(keepMasksIdx);
                    clsLabelsFilt = clsLabelsSort(position);
                end
                %---------------------------------------------------------------------------------------------------------
                % Step 6: get maskScores & combine it with class-scores
                %---------------------------------------------------------------------------------------------------------
                if ~isempty(masksPreNMSFilt)
                    scoresVal = sum(masksPreNMSFilt .* maskPredsFilt, [1 2]);
                else
                    scoresVal = dlarray(zeros(1,1,0,'single'));
                end

                if(isempty(masksPreNMSFilt))
                    masks{idx} = false(originalSize(1), originalSize(2), 0);
                    scores{idx} = zeros(0, 0, 'single');
                    labels{idx} = categorical([], 1:numClasses, this.ClassNames);
                    continue;
                end

                scoresCol = scoresVal(:);
                maskScores = scoresCol./maskAreaFilt;

                if maxNumKern > numFilt
                    scoresSize = size(clsScores,1);
                    classScores = clsScores.*maskScores(1:scoresSize);
                else
                    classScores = clsScores.*maskScores;
                end
                %---------------------------------------------------------------------------------------------------------
                % Step7: matrix NMS
                %---------------------------------------------------------------------------------------------------------
                numPreNMS = coder.const(500);
                maxPerImg = coder.const(100);
                kernel = 'gaussian';
                sigma = coder.const(2);
                filterThresh = this.ScoreThreshold;

                if(this.UseSelectStrongest)
                    [currScores, currLabels, ~, keepIdx] = vision.internal.solov2.matrixNMS(extractdata(masksPreNMSFilt), clsLabelsFilt,...
                        extractdata(classScores),...
                        extractdata(maskAreaFilt),...
                        numPreNMS, maxPerImg,...
                        kernel, sigma, filterThresh);
                else
                    currScores = extractdata(classScores);
                    currLabels = clsLabelsFilt;
                    keepIdx   = 1:numel(classScores);
                end

                maskPredsKept = extractdata(maskPredsFilt(:,:,keepIdx));

                % If no predictions, return empty masks, scores and labels
                if(isempty(maskPredsKept))
                    masks{idx} = false(originalSize(1), originalSize(2), 0);
                    scores{idx} = zeros(0, 0, 'single');
                    labels{idx} = categorical([], 1:numClasses, this.ClassNames);
                    continue;
                end

                maskPredsOut = imresize(maskPredsKept, this.InputSize(1:2), 'bilinear');
                maskPredsScaled = imresize(maskPredsOut, scale,'bilinear');

                currMasks = maskPredsScaled(1:originalSize(1), 1:originalSize(2), :)>this.MaskThreshold;

                masks{idx} = gather(currMasks);
                labels{idx} = categorical(gather(currLabels), 1:numClasses, this.ClassNames);
                scores{idx} = gather(currScores);
            end
        end
    end

end
%------------------------------------------------------------------------
% Helper functions
%------------------------------------------------------------------------

function [outImg, info] = preProcessInput(data, targetSize, miniBatchSize)
coder.inline('always');
coder.internal.prefer_const(data, targetSize);

% Preprocess input data for inference and training
info.OriginalSize = size(data);

% Handle grayscale inputs
if(size(data,3)==1)
    img = repmat(single(data),[1 1 3 1]);
else
    img = single(data);
end

[outImg, scale] = resizePreservingAspectRatio(img, targetSize(1:2), 0, miniBatchSize);
info.Scale = scale;
end

%------------------------------------------------------------------------
function [resizedImageOut, scale] = resizePreservingAspectRatio(img, targetSize, pad, miniBatchSize)
coder.inline('always');
coder.internal.prefer_const(img, targetSize, pad);

% Compute the scale to resize the groundtruth bounding boxes.
imgSize = size(img);

% Compute Aspect Ratio.
imgAspectRatio = imgSize(2)/imgSize(1);

resizeRowsToTargetOutputSize = ceil(imgAspectRatio*targetSize(1));
resizeColsToTargetOutputSize = ceil(targetSize(2)/imgAspectRatio);
padSizeIfResizeRowsToTarget = resizeRowsToTargetOutputSize-targetSize(2);
padSizeIfResizeColsToTarget = resizeColsToTargetOutputSize-targetSize(1);

resizedImageOut = coder.nullcopy(zeros(targetSize(1), targetSize(2), 3, miniBatchSize, 'single'));

% Resize and pad image to final size
if padSizeIfResizeRowsToTarget < padSizeIfResizeColsToTarget
    scale = imgSize(1)/targetSize(1);
    resizedImage = imresize(img, [targetSize(1), nan]);
    col = size(resizedImage, 2);
    resizedImageOut(:,1:col,:,:) = resizedImage;
    resizedImageOut(:,end+1:targetSize(2),:,:) = pad;
elseif padSizeIfResizeColsToTarget < padSizeIfResizeRowsToTarget
    scale = imgSize(2)/targetSize(2);
    resizedImage = imresize(img, [nan, targetSize(2)]);
    row = size(resizedImage, 1);
    resizedImageOut(1:row,:,:,:) = resizedImage;
    resizedImageOut(end+1:targetSize(1),:,:,:) = pad;
else
    scale = imgSize(1)/targetSize(1);
    resizedImage = imresize(img, targetSize(1:2));
    resizedImageOut = resizedImage;
end
end

%------------------------------------------------------------------------
function kern_preds = calculateKernPreds(mlvl_kern)
coder.inline('never');

kern_preds = cell(1,5);
for k = 1:5
    kern_preds{k} = reshape(mlvl_kern{k} , [size(mlvl_kern{k},1)*size(mlvl_kern{k},2), size(mlvl_kern{k},3)]);
end
kern_preds = cat(1, kern_preds{:});
end