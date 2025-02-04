% posemaskrcnn Create a Pose Mask R-CNN network for 6-DoF pose estimation.

% Copyright 2023 The MathWorks, Inc.

classdef posemaskrcnn < deep.internal.sdk.LearnableParameterContainer

    properties(Hidden=true)
        % Pose Mask R-CNN sub networks
        FeatureExtractionNet
        RegionProposalNet
        PostPoolFeatureExtractionNet
        DetectionHeads
        MaskSegmentationHead

        % InputSize  - [H W C] network Input size, C=6 for Pose Mask R-CNN.
        InputSize
    end

    properties(Dependent, Hidden=true)
        AllLearnables
    end
    
    % Network properties
    properties(SetAccess = private)
        % ModelName       - string name for the detector object
        ModelName

        % ClassNames - Cell array of strings
        ClassNames

        % [H W C] RGB or grayscale image input size.
        ImageInputSize

        % [H W 1] depth image input size
        DepthImageInputSize

        % AnchorBoxes- Mx2 anchor boxes for proposal generation.
        AnchorBoxes
    end

    properties (SetAccess=private, GetAccess = {?matlab.unittest.TestCase})
        %ImageMean for normalizing images. Default is the COCO dataset mean
        ImageMean = single([123.675, 116.28, 103.53]);

        % ScaleFactor - Ratio of feature size to image Size
        ScaleFactor = [0.0625 0.0625];
        
        % Number of classes on the detection  head
        NumClasses

    end

    % ROI Pooling properties
    properties(SetAccess=private, GetAccess = {?matlab.unittest.TestCase})
        PoolSize = [14 14];
        MaskPoolSize = [14 14];
    end
    
    % Region proposal properties
    properties(Access=private)      
        ProposalsOutsideImage = 'clip'
        BoxFilterFcn =  @(a,b,c,d)fasterRCNNObjectDetector.filterBBoxesBySize(a,b,c,d)

        MinSize = [1 1]
        MaxSize = [inf inf]

        MinScore = 0;
        
        OverlapThreshold = 0.7

        NumStrongestRegionsBeforeProposalNMS = 6000
        NumStrongestRegions = 1000

        RPNBoxStd = [1 1 1 1]
        RPNBoxMean= [0 0 0 0]
    end
    
    % Prediction params
    properties (SetAccess = private, GetAccess = ?matlab.unittest.TestCase)
        BoxMean = [0 0 0 0];
        BoxStd = 1./[10 10 5 5]
        
        % Classification confidence score threshold
        ScoreThreshold

        % NMS (selectStrongest) settings
        NumStrongestRegionsPrediction = 100
        SelectStrongestThreshold = 0.3
        SelectStrongestMulticlassThreshold = 0.5
        
        % Mask confidence score threshold
        MaskThreshold = 0.5
    end

    % properties needed by dltrain
    properties(Dependent, Hidden=true)
        Learnables
        Layers
        State
        OutputNames
    end
    properties(Hidden=true)
        InputNames
    end


    %Training params
    properties (Access=private)
        FreezeBackbone = false
        FreezeRPN = false
    end

    % ---------------------------------------------------------------------

    methods (Access=private)
        function [imageOut, ratio] = preprocessImage(~, imageIn, imageSize, imageMean, gray2rgb)
            % preprocessImage- resize, normalize and pad the input image.
            arguments
                ~
                imageIn
                imageSize
                imageMean
                gray2rgb = true
            end
            
            % Step 1: Resize the image while maintaining the aspect ratio
            imSize = size(imageIn);
            maxSize = max(imSize(1), imSize(2));
            minSize = min(imSize(1), imSize(2));
            
            if maxSize/minSize <= imageSize(2)/imageSize(1)
                % dim 1 is dominant
                ratio = imageSize(1)/minSize;
            else
                % dim 2 is dominant
                ratio = imageSize(2)/maxSize;
            end
            imageIn = imresize(imageIn,[int32(ratio * imSize(1)) int32(ratio*imSize(2))],'nearest');
            
            % Convert grayscale images to 3 channel
            if(size(imageIn,3)==1) && gray2rgb
                imageIn = repmat(imageIn, [1 1 3]);
            end

            if gray2rgb
                % Step 2: Normalize (zero-center)
                imageIn(:,:,1) = imageIn(:,:,1) - imageMean(1);
                imageIn(:,:,2) = imageIn(:,:,2) - imageMean(2);
                imageIn(:,:,3) = imageIn(:,:,3) - imageMean(3);
            end
    
            % Step 3: Make the input divisible by 32
            sz = size(imageIn);
            paddedH = int32(ceil(sz(1)/32)*32);
            paddedW = int32(ceil(sz(2)/32)*32);
            if gray2rgb
                imageOut = zeros(paddedH,paddedW,3, 'like', imageIn);
                imageOut(1:sz(1),1:sz(2),:) = imageIn;
            else 
                imageOut = zeros(paddedH,paddedW,'like', imageIn);
                imageOut(1:sz(1),1:sz(2)) = imageIn;
            end
            
        end

        function translation = compute3DTranslationFromOffsets(~,boxes,tCoeffs,xyzIm,Kscaled)
                % predicted box centers
                boxCenters = cat(2, boxes(:,1) + 0.5*boxes(:,3), boxes(:,2) + 0.5*boxes(:,4));

                % centroids 2D XY
                centroids2DXY = zeros(size(boxes,1), 2, 'like', tCoeffs);
                centroids2DXY(:,1:2) = ( boxes(:,1:2) .* (1-tCoeffs(:,1:2)) )...
                    + ( (boxes(:,1:2)+boxes(:,3:4)).*(tCoeffs(:,1:2)) );

                % Handle invalid or overflowing predicted 2D centroids - replace by box centers
                overflowSel = or( or( or( centroids2DXY(:,1) < 1, centroids2DXY(:,2) < 1),...
                    centroids2DXY(:,1) >  size(xyzIm,2)), centroids2DXY(:,2) > size(xyzIm,1));
                nanSel = or( isnan(centroids2DXY(:,1)), isnan(centroids2DXY(:,2)) );
                infSel = or( isinf(centroids2DXY(:,1)), isinf(centroids2DXY(:,2)) );
                invalidSel = or(overflowSel, or(nanSel, infSel));
                centroids2DXY(invalidSel,:) = boxCenters(invalidSel,:); % 
                
                % Handle invalid predicted Z offset - replace with mid-point (0.5) of depth bounds
                invalidZ = or(isinf(tCoeffs(:,3)), isnan(tCoeffs(:,3)));
                tCoeffs(invalidZ,3) = 0.5;
                
                % Calculate bounds on Z (depth) and centroid 3D Z
                [lowerBoundZ,upperBoundZ] = vision.internal.cnn.pose.poseTranslationBoundsZ(boxes,xyzIm,centroids2DXY,Kscaled);
                centroids3DZ = ( lowerBoundZ .* (1-tCoeffs(:,3)) )...
                    + ( upperBoundZ.*(tCoeffs(:,3)) );

                % "un"-project 2D centroid XY to 3D XY, using centroid Z and
                % scaled intrinsics matrix
                centroids3DXY = vision.internal.cnn.pose.poseImageXY2World(centroids2DXY,centroids3DZ,Kscaled);

                translation = cat(2, centroids3DXY, centroids3DZ);
        end


        function [varargout] = computeXYZImage(~, imD, K, scaleRatio)
            % Create X,Y,Z "image" in camera-centered world coordinate system
            % given depth image, intrinsics and image rescaling ratio
            Kscaled = [scaleRatio 0 0; 0 scaleRatio 0; 0 0 1] * K;
            [U,V] = meshgrid(1:size(imD,2), 1:size(imD,1)); % image coordinates of each pixel
            U = U .* imD;
            V = V .* imD;
            uvz = cat(3,U,V,imD); % imD is Z in world coordinates
            uvz = reshape(uvz, size(U,1)*size(U,2), 3);
            xyz = pinv(Kscaled)* uvz'; % XYZ in world coordinates
            xyz = xyz';
            xyz = reshape(xyz, size(U,1), size(U,2), 3);
            varargout{1} = xyz;
            if nargout == 2
                varargout{2} = Kscaled;
            end
        end

        function K = extractMatrixFromCameraIntrinsics(~, intrinsics)
            % Return intrinsics matrix K from input cameraIntrinsics
            % objects as a 3x3xnumIntrinsics array
            if isscalar(intrinsics)
                K = intrinsics.K;
            else
                batchSize = length(intrinsics);
                K = zeros(3, 3, batchSize);
                for i = 1:batchSize
                    K(:,:,i) = intrinsics(i).K;
                end
            end
        end
    end
    
    % Constructors
    methods
        function obj = posemaskrcnn(pretrained, classNames, anchorBoxes, options)
            arguments
                pretrained {mustBeMember(pretrained, {'resnet50-coco','none','resnet50-pvc-parts'})}
                classNames {validateClassNames, mustBeUniqueNames} = ''
                anchorBoxes (:,2){mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite} = []
                options.ImageInputSize {mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite, mustBeRGBSize} = []
                options.PoolSize (1,2) {mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite} = [14 14]
                options.MaskPoolSize (1,2) {mustBeNumeric, mustBePositive, mustBeReal, mustBeFinite} = [14 14]
                options.ModelName {mustBeTextScalar} = 'posemaskrcnn'
            end
            
            vision.internal.requiresNeuralToolbox(mfilename);
            
            % This is added to support load-network workflows where all the
            % weights and properties will be populated by the loadobj
            % method.
            if(strcmp(pretrained, 'none'))
                return;
            end

            % Pose Mask R-CNN support package tripwire
            [spkgroot, spkgrootMaskrcnn] = iTripwirePoseMaskRCNN();

            % Assign input sizes in the constructor
            options = processInputSizes(options);

            if strcmp(pretrained, 'resnet50-coco')
                % Load dlnetworks from a MC-COCO pre-trained maskrcnn.
                % In this setting pose-specific features are learned from
                % scratch, starting from the Mask R-CNN weights trained
                % for instance segmentation on MS-COCO.
                data = iLoadNetwork(spkgrootMaskrcnn,"data","dlnetFeature.mat");
                obj.FeatureExtractionNet = data.dlnetFeature;
                data = iLoadNetwork(spkgrootMaskrcnn,"LargeData","dlnetPostFeature.mat");
                obj.PostPoolFeatureExtractionNet = data.dlnetPostFeature;
                data = iLoadNetwork(spkgrootMaskrcnn,"data","dlnetRPN.mat" );
                obj.RegionProposalNet = data.dlnetRPN;
                data = iLoadNetwork(spkgrootMaskrcnn,"data","dlnetDetectHead.mat" );
                obj.DetectionHeads = data.dlnetDetectHead;
                data = iLoadNetwork(spkgrootMaskrcnn,"data","dlnetMaskHead.mat" );
                obj.MaskSegmentationHead = data.dlnetMaskHead;
            else
                % Loads dlnetworks from a pre-trained bin picking
                % posemaskrcnn model. This is the default behavior.
                dat = iLoadNetwork(spkgroot,"data","poseMaskRCNN-PVCParts-Random2k.mat");
                obj = posemaskrcnn.loadobj(dat.net);
                clear dat;
            end
            
            % Customize network for new input size
            if(isempty(options.InputSize))
                % Default input size = inputLayer size, if no input size is
                % provided
                layer = obj.FeatureExtractionNet.Layers(1);
                obj.InputSize = layer.InputSize;
                obj.ImageInputSize = [obj.InputSize(1:2) 3];
                obj.DepthImageInputSize = [obj.InputSize(1:2) 1];
            else
                % Update the input layer with new size
                inputName = 'Input_data';
                obj.InputSize = options.InputSize;
                obj.ImageInputSize = options.ImageInputSize;
                obj.DepthImageInputSize = options.DepthImageInputSize;
                inputLayer = imageInputLayer(options.InputSize,'Normalization',"none",'Name','Input_data');

                featureGraph = layerGraph(obj.FeatureExtractionNet);

                featureGraph = replaceLayer(featureGraph, inputName, inputLayer);

                % Modify the first conv layer of the feature extraction
                % network, if needed, to match the number of channels in
                % the input image (e.g. if depth-maps are appended to RGB).
                if featureGraph.Layers(2).NumChannels ~= inputLayer.InputSize(3)
                    conv1Layer = convolution2dLayer( ...
                        featureGraph.Layers(2).FilterSize,...
                        featureGraph.Layers(2).NumFilters,...
                        NumChannels = inputLayer.InputSize(3),...
                        Stride = featureGraph.Layers(2).Stride,...
                        DilationFactor = featureGraph.Layers(2).DilationFactor,...
                        Padding = 'same',...
                        Name = 'conv1',...
                        WeightLearnRateFactor=5, BiasLearnRateFactor=5); % higher learning rates for the new layers
                    featureGraph = replaceLayer(featureGraph, 'conv1', conv1Layer);
                end
                obj.FeatureExtractionNet = dlnetwork(featureGraph);
            end

            % Set default input names for the task network
            obj.InputNames = {'Input_data', 'Input_XYZ'} ;
            
            % Customize network for new classes
            if isempty(classNames)
                if isempty(obj.ClassNames)
                    % occurs when MS-COCO pretrained model is chosen
                    obj.ClassNames = iGetCOCOClasses();
                    numClasses = length(classNames);
                    obj.NumClasses = numClasses;
                end
                % when PVC-Parts pretrained model is chosen, ClassNames is
                % also loaded from the pretrained parameters
            else
                if(iscategorical(classNames))
                    classNames = string(classNames);
                end

                if(isstring(classNames))
                    classNames = cellstr(classNames);
                end

                obj.ClassNames = classNames;
                numClasses = length(classNames);
                obj.NumClasses = numClasses;

                % ---------------------------------------------------------
                % Update the detect branch architecture - regression head &
                % classification head & Mask head.
               
                % Update regression conv layer              
                detGraph = layerGraph(obj.DetectionHeads);
                regConvNode = 'detectorRegOut';
                convReg = convolution2dLayer([1 1], numClasses*4,...
                    'Name', 'detectorRegOut','WeightsInitializer','narrow-normal');
                
                detGraph = replaceLayer(detGraph, regConvNode, convReg);
                    
                % Update classification conv layer
                classConvNode = 'node_133';
                convClass = convolution2dLayer([1 1], numClasses+1,...
                    'Name', 'node_133','WeightsInitializer','narrow-normal');
                detGraph = replaceLayer(detGraph, classConvNode, convClass);
                
                % ---------------------------------------------------------
                layerNames = {detGraph.Layers(:).Name};
                % if length(obj.DetectionHeads.OutputNames) < 4

                % Pose estimation branches are added to the detection 
                % head to output 3D rotation and translation.
                
                % Rotation branch - predicts a 4-element quaternion
                convPoseR_FC1 = convolution2dLayer([1 1], 1024, ...
                    'Name', 'poseR_FC1',...
                    'WeightsInitializer','narrow-normal');
                convPoseR_FC2 = convolution2dLayer([1 1], 512, ...
                    'Name', 'poseR_FC2',...
                    'WeightsInitializer','narrow-normal'); 
                convPoseR_FC3 = convolution2dLayer([1 1], numClasses*4, ...
                    'Name', 'detectorPoseRotationOut',...
                    'WeightsInitializer','narrow-normal');
                
                % remove existing layers of the same name if any
                if any(strcmp(layerNames, 'poseR_Relu1'))
                    detGraph = removeLayers(detGraph,'poseR_Relu1');
                end
                if any(strcmp(layerNames, 'poseR_Relu2'))
                    detGraph = removeLayers(detGraph,'poseR_Relu2');
                end
                if any(strcmp(layerNames, 'poseR_FC1'))
                    detGraph = removeLayers(detGraph,'poseR_FC1');
                end
                if any(strcmp(layerNames, 'poseR_FC2'))
                    detGraph = removeLayers(detGraph,'poseR_FC2');
                end
                if any(strcmp(layerNames, 'poseR_FC3'))
                    detGraph = removeLayers(detGraph,'poseR_FC3');
                end
                if any(strcmp(layerNames, 'detectorPoseRotationOut'))
                    detGraph = removeLayers(detGraph,'detectorPoseRotationOut');
                end
                
                detGraph = addLayers(detGraph,...
                    [convPoseR_FC1,...
                    reluLayer('Name','poseR_Relu1'),...
                    convPoseR_FC2,...
                    reluLayer('Name','poseR_Relu2'),...
                    convPoseR_FC3]);
                detGraph = connectLayers(detGraph, 'gapool', 'poseR_FC1');

                % Translation branch - predicts relative offsets inside
                % a detected ROI projected to 3D.
                convPoseT_FC1 = convolution2dLayer([1 1], 512, ...
                    'Name', 'poseT_FC1',...
                    'WeightsInitializer','narrow-normal'); 
                convPoseT_FC2 = convolution2dLayer([1 1], numClasses*3, ...
                    'Name', 'poseT_FC2',...
                    'WeightsInitializer','narrow-normal');

                % remove existing layers of the same name if any
                if any(strcmp(layerNames, 'poseT_FC1'))
                    detGraph = removeLayers(detGraph,'poseT_FC1');
                end
                if any(strcmp(layerNames, 'poseT_FC2'))
                    detGraph = removeLayers(detGraph,'poseT_FC2');
                end
                if any(strcmp(layerNames, 'poseT_Relu1'))
                    detGraph = removeLayers(detGraph,'poseT_Relu1');
                end
                if any(strcmp(layerNames, 'detectorPoseTranslationOut'))
                    detGraph = removeLayers(detGraph,'detectorPoseTranslationOut');
                end

                detGraph = addLayers(detGraph,...
                    [convPoseT_FC1,...
                    reluLayer('Name','poseT_Relu1'),...
                    convPoseT_FC2,...
                    sigmoidLayer('Name', 'detectorPoseTranslationOut')]);
                detGraph = connectLayers(detGraph, 'gapool', 'poseT_FC1');

                obj.DetectionHeads = dlnetwork(detGraph, 'Initialize', false);

                % Update mask head
                maskGraph = layerGraph(obj.MaskSegmentationHead);
                convMaskNode = 'node_167';
                convMask = convolution2dLayer([1 1], numClasses, 'Name','node_167','WeightsInitializer','narrow-normal');
                maskGraph = replaceLayer(maskGraph, convMaskNode, convMask);
                obj.MaskSegmentationHead = dlnetwork(maskGraph, 'Initialize', false);
            end

            % Customize network for anchor boxes
            if isempty(anchorBoxes)
                if isempty(obj.AnchorBoxes)
                    % when MS-COCO pretrained model is chosen
                    obj.AnchorBoxes = iGetDefaultCOCOAnchorBoxes();
                end
                % when PVC-Parts pretrained model is chosen, AnchorBoxes is
                % also loaded from the pretrained parameters
            else
                obj.AnchorBoxes = double(anchorBoxes);
                numAnchors = size(anchorBoxes,1);

                % update the RPN branch architecture
                rpnGraph = layerGraph(obj.RegionProposalNet);
                rpnRegConvNode = 'RPNRegOut';
                convReg = convolution2dLayer([1 1], numAnchors*4, 'Name', 'RPNRegOut','WeightsInitializer','narrow-normal');
                
                rpnGraph = replaceLayer(rpnGraph, rpnRegConvNode, convReg);

                rpnClassConvNode = 'RPNClassOut';
                convClass = convolution2dLayer([1 1], numAnchors, 'Name', 'RPNClassOut','WeightsInitializer','narrow-normal');
                
                rpnGraph = replaceLayer(rpnGraph, rpnClassConvNode, convClass);

                obj.RegionProposalNet = dlnetwork(rpnGraph, 'Initialize', false);
            end

            obj.PoolSize = options.PoolSize;
            obj.MaskPoolSize = options.MaskPoolSize;
            obj.ModelName = options.ModelName;
            
            % Initialize the dlnetworks
            obj = initialize(obj);
            
        end
    end
    
    methods(Access={?matlab.unittest.TestCase})

        % Declare regionproposal method (definition in separate file)
        proposals = regionProposal(obj, regressionBatch, scoresBatch)

        % Declare roiAlignPooling method (definition in separate file)
        outFeatures = roiAlignPooling (obj, X, boxes, poolSize)

        % Declare regionProposalBounds3D method (definition in separate file)
        [lowerBounds3D,upperBounds3D] = regionProposalBounds3D(obj, boxes, xyzImg)
    end

    methods(Access=private)

        % initialize all sub - dlnetworks
        function obj = initialize(obj)
            
            dlX = dlarray(rand(obj.InputSize, 'single'), 'SSCB');
            if(canUseGPU)
                % If a GPU is available then run this forward pass on GPU
                % to avoid extremely slow CPU computation, during the task 
                % network creation step.
                dlX = gpuArray(dlX);        
            end

            dlFeatures = predict(obj.FeatureExtractionNet, dlX, 'Acceleration','none');
    
            % predict on RPN
            if(~obj.RegionProposalNet.Initialized)
                obj.RegionProposalNet = initialize(obj.RegionProposalNet, dlFeatures);
            end

            [dlRPNScores, dlRPNReg] = predict(obj.RegionProposalNet, dlFeatures, 'Acceleration','none');
            
            % Call region proposal
            dlProposals = regionProposal(obj, dlRPNReg, dlRPNScores);
            
            dlPooled = roiAlignPooling(obj, dlFeatures, dlProposals, obj.PoolSize);
    
            if(~obj.PostPoolFeatureExtractionNet.Initialized)
                obj.PostPoolFeatureExtractionNet = initialize(...
                                            obj.PostPoolFeatureExtractionNet,...
                                            dlPooled, dlPooled);
            end
            
            dlFinalFeatures = predict(obj.PostPoolFeatureExtractionNet, dlPooled, dlPooled, 'Acceleration','none');
    
            % Box regression
            if(~obj.DetectionHeads.Initialized)
                obj.DetectionHeads = initialize(obj.DetectionHeads, dlFinalFeatures);
            end
    
            % Predict on mask branch to get h x w x numClasses x numProposals
            % cropped masks.
            if(~obj.MaskSegmentationHead.Initialized)
                obj.MaskSegmentationHead = initialize(obj.MaskSegmentationHead,...
                                                          dlFinalFeatures);
            end

        end

    end

    methods(Access=public)

        function outputFeatures = predict(obj, dlX, dlXYZ)
        % outputFeatures = predict(detector,dlX) predicts features of
        % the preprocessed image dlX. The outputFeatures is a 6-by-1
        % cell array. Each cell of outputFeature contains
        % predictions from an output layer. detector is a
        % posemaskrcnn object and dlX is a SSCB formatted dlarray. dlXYZ is
        % a SSCB formatted 3-channel dlarray of same spatial dimensions as 
        % dlX, containing the X,Y and Z (depth) in world-coordinates for
        % each pixel of the input image dlX.
        
            % predict on feature extractor
            if obj.InputSize(3) == size(dlX, 3)
                % 3-channel images as inputs
                dlFeatures = predict(obj.FeatureExtractionNet, dlX, 'Acceleration','auto');
            else
                % 6-channel: RGB and (depthmap + world-coordinate XY) as inputs
                dlFeatures = predict(obj.FeatureExtractionNet, cat(3, dlX, dlXYZ), 'Acceleration','auto');
            end
    
            [dlRPNScores, dlRPNReg] = predict(obj.RegionProposalNet, dlFeatures, 'Acceleration','auto');
            
            % Call region proposal
            dlProposals = regionProposal(obj, dlRPNReg, dlRPNScores);
            
            dlPooled = roiAlignPooling(obj, dlFeatures, dlProposals, obj.PoolSize);
    
            dlFinalFeatures = predict(obj.PostPoolFeatureExtractionNet, dlPooled, dlPooled, 'Acceleration','auto');

            % Pose (rotation, translation) along with boxes, scores from common Detection head
            [dlBoxReg, dlBoxScores, dlRotation, dlTranslationOffsets] = predict(obj.DetectionHeads, dlFinalFeatures, 'Acceleration','auto'); 

            % Normalize per-class rotation quaternions to unit L2-norm
            %   i.e. L2Norm(dlRotation(1,1, (c-1)*numClasses+1 : (c-1)*numClasses+1 + quaternionSize , i) = 1
            %       For c-th class, i-th proposal and quaternionSize=4
            %       (since a quaternion has 4 elements).
            dlRotation = normalizePerClassQuaternions(obj, dlRotation);

            outputFeatures{1} = dlProposals;
            outputFeatures{2} = dlBoxReg;
            outputFeatures{3} = dlBoxScores;
            outputFeatures{4} = dlFeatures;
            outputFeatures{5} = dlRotation;
            outputFeatures{6} = dlTranslationOffsets;
        end

        function [dlRPNScores,dlRPNReg,dlProposals,dlBoxScores,dlBoxReg,dlRotation, dlTranslation,dlMasks,state]...
                = forward(obj, dlX , dlXYZ, varargin)
        % computes outputs of the network after a foward pass on the input dlX.
        % detector is a posemaskrcnn object and dlX is a formatted dlarray of type 'SSCB'.
        % dlXYZ is a SSCB formatted 3-channel dlarray of same spatial dimensions as 
        % dlX, containing the X,Y and Z (depth) in world-coordinates for each
        % pixel of the input image dlX.

            % forward on feature extractor
            if obj.InputSize(3) == size(dlX, 3)
                % 3-channel images as inputs
                [dlX, localState] = forward(obj.FeatureExtractionNet, dlX, 'Acceleration','none');
            else
                % 6-channel: RGB and (depthmap + world-coordinate XY) as inputs
                dlX = cat(3, dlX, dlXYZ);
                [dlX, localState] = forward(obj.FeatureExtractionNet, dlX, 'Acceleration','none');
            end
            state = localState;
        
            % forward on RPN
            [dlRPNScores, dlRPNReg, localState] = forward(obj.RegionProposalNet, dlX, 'Acceleration','none');
            state = vertcat(state, localState);
            
            % Call region proposal
            dlProposals = regionProposal(obj, dlRPNReg, dlRPNScores); % 1x1xNx5 (xyxyb) where b = image id in batch
            
            % Perform region pooling
            dlPooled = roiAlignPooling(obj, dlX, dlProposals, obj.PoolSize);
            
            % forward on post region pooling feature extractor
            [dlFinalFeatures, localState] = forward(obj.PostPoolFeatureExtractionNet, dlPooled, dlPooled, 'Acceleration','none');
            state = vertcat(state, localState);

            % Forward on detection and segmentation heads - include pose
            % rotation and 3D translation offsets
            [dlBoxReg, dlBoxScores, dlRotation, dlTranslationOffsets, localState] = forward(obj.DetectionHeads, dlFinalFeatures, 'Acceleration','none');
            state = vertcat(state, localState);

            dlTranslation = dlTranslationOffsets; % return 3D translation offsets from the roi bounds

            % Normalize per-class rotation quaternions to unit L2-norm
            %   i.e. L2Norm(dlRotation(1,1, (c-1)*numClasses+1 : (c-1)*numClasses+1 + quaternionSize , i) = 1
            %       For c-th class, i-th proposal and quaternionSize=4
            %       (since a quaternion has 4 elements).
            dlRotation = normalizePerClassQuaternions(obj, dlRotation);
            
            % Semantic segmentation masks for roi proposals
            [dlMasks, localState] = forward(obj.MaskSegmentationHead, dlFinalFeatures, 'Acceleration','none');
            state = vertcat(state, localState);
            
        end

        function varargout = predictPose(obj, im, imDepth, intrinsics, options)
            arguments
                obj 
                im {validateImageInput}
                imDepth {validateDepthImageInput}
                intrinsics cameraIntrinsics {mustBeVector}
                options.Threshold (1,1){mustBeNumeric, mustBeNonnegative, mustBeLessThanOrEqual(options.Threshold, 1), mustBeReal} = 0.7
                options.NumStrongestRegions (1,1) {mustBeNumeric, mustBePositive, mustBeReal} = 1000
                options.SelectStrongestThreshold (1,1){mustBeNumeric,mustBeNonnegative,mustBeReal,validateInfThreshold} = 0.3
                options.SelectStrongestMulticlassThreshold (1,1){mustBeNumeric,mustBeNonnegative,mustBeReal,validateInfThreshold} = 0.5
                options.MinSize (1,2) {mustBeNumeric, mustBePositive, mustBeReal, mustBeInteger} = [1,1]
                options.MaxSize (1,2) {mustBeNumeric, mustBePositive, mustBeReal, mustBeInteger} = obj.InputSize(1:2)
                options.ExecutionEnvironment {mustBeTextScalar, mustBeMember(options.ExecutionEnvironment,{'gpu','cpu','auto'})} = 'auto'
                options.MiniBatchSize (1,1) {mustBeNumeric, mustBePositive, mustBeReal, mustBeInteger} = 1
                options.NamePrefix {mustBeTextScalar} = "segmentObj"
                options.Verbose (1,1) {validateLogicalFlag} = true
            end

            % Replicate scalar intrinsics to batchsize
            batchSize = size(im,4);
            if isscalar(intrinsics) &&  batchSize > 1
                intrinsics = repmat(intrinsics, [batchSize 1]);
            end

            % Validate input arguments jointly for matching dimensions
            validateInferenceInputs(im, imDepth, intrinsics);
            
            % Send the data to device or to host based on ExecutionEnvironment
            % option
            if(isequal(options.ExecutionEnvironment, 'auto'))
                if(canUseGPU)
                    options.ExecutionEnvironment = 'gpu';
                end
            end
    
            % Update the prediction parameters
            obj.ScoreThreshold = options.Threshold;
            obj.NumStrongestRegions = options.NumStrongestRegions;
            obj.SelectStrongestThreshold = options.SelectStrongestThreshold;
            obj.SelectStrongestMulticlassThreshold = options.SelectStrongestMulticlassThreshold;
            obj.MinSize = options.MinSize;
            obj.MaxSize = options.MaxSize;
    
            % Extract the matrix from the cameraIntrinsics object (also
            % handles batches)
            K = extractMatrixFromCameraIntrinsics(obj, intrinsics);
    
            % Check if the input image is a single image or a batch
            if(matlab.io.datastore.internal.shim.isDatastore(im))
                % TODO - datastore support not implemented yet
            elseif(ndims(im)<=3)
                nargoutchk(0,5);
                [varargout{1:nargout}] =...
                                             predictPoseInSingleImg(obj, im, imDepth, K,...
                                                       options.ExecutionEnvironment);
            elseif(ndims(im)==4)
                nargoutchk(0,5);
                [varargout{1:nargout}] = ...
                                            predictPoseInImgStack(obj, im, imDepth, K,...
                                                        options.MiniBatchSize,...
                                                        options.ExecutionEnvironment);  
            else
                % Code flow shouldn't reach here (ensured by validation code).
                assert(false, 'Invalid image input.');
            end
            
        end
        
    end
    
    
    %======================================================================
    methods(Hidden)
        function s = saveobj(this)
            s.Version                      = 1.0;
            s.ModelName                    = this.ModelName;
            s.FeatureExtractionNet         = this.FeatureExtractionNet;
            s.RegionProposalNet            = this.RegionProposalNet;
            s.PostPoolFeatureExtractionNet = this.PostPoolFeatureExtractionNet;
            s.DetectionHeads               = this.DetectionHeads;
            s.MaskSegmentationHead         = this.MaskSegmentationHead;
            s.ClassNames                   = this.ClassNames;
            s.AnchorBoxes                  = this.AnchorBoxes;
            s.PoolSize                     = this.PoolSize;
            s.MaskPoolSize                 = this.MaskPoolSize;
            s.InputNames                   = this.InputNames;
            if isempty(this.InputSize)
                layer = this.FeatureExtractionNet.Layers(1);
                s.InputSize = layer.InputSize;
            else
                s.InputSize = this.InputSize;
            end
            if isempty(this.ImageInputSize)
                layer = this.FeatureExtractionNet.Layers(1);
                s.ImageInputSize = [layer.InputSize(1:2) 3];
            else
                s.ImageInputSize = this.ImageInputSize;
            end
            if isempty(this.DepthImageInputSize)
                layer = this.FeatureExtractionNet.Layers(1);
                s.DepthImageInputSize = [layer.InputSize(1:2) 1];
            else
                s.DepthImageInputSize = this.DepthImageInputSize;
            end
            s.FreezeBackbone               = this.FreezeBackbone;
            s.FreezeRPN                    = this.FreezeRPN;
        end

        function obj = configureForTraining(obj, tNumStrongestRegions, freezeSubnetStr)
            
            % Set params for training
            obj.NumStrongestRegions = tNumStrongestRegions;
            
            if(lower(freezeSubnetStr)=="rpn")
                obj.FreezeRPN = true;
                obj.FreezeBackbone = false;
            elseif(lower(freezeSubnetStr)=="backbone")
                obj.FreezeBackbone = true;
                obj.FreezeRPN = false;
            elseif(lower(freezeSubnetStr)=="none")
                obj.FreezeBackbone = false;
                obj.FreezeRPN = false;
            elseif(all(sort(lower(freezeSubnetStr))==["backbone", "rpn"]))
                obj.FreezeBackbone = true;
                obj.FreezeRPN = true;
            else
                assert(false,"Invalid freezeSubnetStr");
            end

        end
        
    end
    
    methods(Static, Hidden)
        function this = loadobj(s)
            try
                vision.internal.requiresNeuralToolbox(mfilename);

                this = posemaskrcnn("none");
                   
                this.FeatureExtractionNet         = s.FeatureExtractionNet;
                this.RegionProposalNet            = s.RegionProposalNet;
                this.PostPoolFeatureExtractionNet = s.PostPoolFeatureExtractionNet;
                this.DetectionHeads               = s.DetectionHeads;
                this.MaskSegmentationHead         = s.MaskSegmentationHead;
                this.ClassNames                   = s.ClassNames;
                this.NumClasses                   = length(s.ClassNames);
                this.AnchorBoxes                  = s.AnchorBoxes;
                this.InputSize                    = s.InputSize;
                this.PoolSize                     = s.PoolSize;
                this.MaskPoolSize                 = s.MaskPoolSize;
                this.ModelName                    = s.ModelName;
                if isfield(s, 'InputNames')
                    this.InputNames               = s.InputNames;
                end
                if isempty(this.InputSize)
                    layer = this.FeatureExtractionNet.Layers(1);
                    this.InputSize = layer.InputSize;
                end
                if isempty(this.ImageInputSize)
                    layer = this.FeatureExtractionNet.Layers(1);
                    this.ImageInputSize = [layer.InputSize(1:2) 3];
                end
                if isempty(this.DepthImageInputSize)
                    layer = this.FeatureExtractionNet.Layers(1);
                    this.DepthImageInputSize = [layer.InputSize(1:2) 1];
                end
                if isfield(s, 'FreezeBackbone')
                    this.FreezeBackbone               = s.FreezeBackbone;
                end
                if isfield(s, 'FreezeRPN')
                    this.FreezeRPN                    = s.FreezeRPN;
                end

            catch ME
                rethrow(ME)
            end
        end
    end

    methods(Access=private)
        
        function [poses, boxLabel, boxScore, boxes, masks] = predictPoseInSingleImg(obj, im, imDepth, K, executionEnvironment)
        
            orgSize = size(im);
    
            % Resize and normalize image and depth-map
            [im, scaleRatio] = preprocessImage(obj, single(im), obj.InputSize, obj.ImageMean);
            [imDepth, ~] = preprocessImage(obj, single(imDepth), obj.InputSize, [0 0 0], false);

            % Compute XYZ map
            [xyzIm, Kscaled] = computeXYZImage(obj, imDepth, K, scaleRatio);
            
            if(isequal(executionEnvironment, 'gpu'))
                %GPU
                im = gpuArray(im);
                xyzIm = gpuArray(xyzIm);
            else
                %Host
                if(isgpuarray(im))
                    im = extractData(im);
                    xyzIm = extractData(xyzIm);
                end
            end
            
            % Inference
            dlX = dlarray(im, 'SSCB');
            dlXYZ = dlarray(xyzIm, 'SSCB');
            netOut = predict(obj, dlX, dlXYZ);
            imageSize = size(im);

            % Apply regressed deltas from the DetectionHead to RPN roi boxes,
            % apply confidence threshold and NMS on the predictions to get
            % a final set of bboxes (xywh), labels, scores, rotations and
            % translation coefficients (not the final translations).
            [boxes, boxLabel, boxScore, rotation, translationCoeffs] = postProcessOutputs(obj,...
                                          extractdata(netOut{2}),...
                                          extractdata(netOut{3}),...
                                          extractdata(netOut{1}), ...
                                          extractdata(netOut{5}), ... 
                                          extractdata(netOut{6}), ...
                                          imageSize, 1);
            
            boxes = boxes{1};
            % If predicted boxes are empty, return empty mask, empty pose & empty cat for
            % labels
            if(isempty(boxes))
                boxLabel = categorical(boxLabel, 1:numel(obj.ClassNames), obj.ClassNames);
                masks = zeros(0,0,0, 'like', boxes);
                poses = rigidtform3d.empty(0);
                return;
            end

            boxLabel = boxLabel{1};
            boxScore = boxScore{1};
            rotation = rotation{1};
            translation = compute3DTranslationFromOffsets(obj, boxes, translationCoeffs{1}, xyzIm, Kscaled);

            % Computation of masks on the final predicted bboxes
            maskBranchBoxes = [boxes  ones(size(boxes,1), 1)];
            
            maskBranchBoxes = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(...
                                                                         maskBranchBoxes);
    
            dlBoxes = dlarray(maskBranchBoxes', 'SSCB');
            
            % Feature pooling for mask branch
            dlMaskPooled = roiAlignPooling(obj, netOut{4}, dlBoxes, obj.MaskPoolSize);
            
            dlPostFeatureMask = predict(obj.PostPoolFeatureExtractionNet, dlMaskPooled, dlMaskPooled, 'Acceleration','auto');
            
            % Predict on mask branch to get h x w x numClasses x numProposals
            % cropped masks.
            dlMasks = predict(obj.MaskSegmentationHead, dlPostFeatureMask);
            dlMasks = extractdata(dlMasks);
    
            % Extract the cropped mask corresponding to the boxLabel for
            % each proposal.
            finalCroppedMasks = zeros(size(dlMasks,1), size(dlMasks, 2),...
                                      size(boxes,1));
            
            for i = 1:size(boxes,1)
                finalCroppedMasks(:,:,i) = dlMasks(:,:, boxLabel(i), i);
            end
            
            % Final box detections in original image coordinates
            if(isgpuarray(boxes))
                boxes = gather(boxes);
            end
            boxes = bboxResizePixel(obj, floor(boxes), 1/scaleRatio);
            boxes = clipBoxes(obj, boxes, orgSize);
            
            % Generate full sized masks from cropped masks
            masks = generateFullSizedMasks(obj, finalCroppedMasks, boxes, orgSize);
            
            % Convert boxLabels to categorical
            boxLabel = categorical(boxLabel, 1:numel(obj.ClassNames), obj.ClassNames);

            % Final rotation and translation
            if(isgpuarray(rotation))
                rotation = gather(rotation);
            end
            if(isgpuarray(translation))
                translation = gather(translation);
            end
            
            % Package translations and rotations into ridigtform3d objects
            rotationMatrices = vision.internal.quaternion.quaternionToRotation(rotation.');
            for ii = 1:length(boxScore)
                poses(ii) = rigidtform3d(...
                    rotationMatrices(:,:,ii),...
                    translation(ii,:)); %#ok<AGROW>
            end
            poses = poses';
        end


        function [poses, boxLabel, boxScore, boxes, masks] = predictPoseInBatch(obj, im, imDepth, K, executionEnvironment)
        
            if(iscell(im))
                batchSize = numel(im);
            else
                batchSize = size(im,4);
            end
            
            for batchIdx = 1:batchSize
                % Resize and normalize image
                if(iscell(im))
                    orgSize{batchIdx} = size(im{batchIdx}); %#ok<AGROW> 
                    [imResized(:,:,:,batchIdx), scaleRatio(batchIdx)] = preprocessImage(obj,...
                            single(im{batchIdx}), obj.InputSize, obj.ImageMean); %#ok<AGROW> 
                    imDepthResized = preprocessImage(obj,...
                            single(imDepth{batchIdx}), obj.InputSize, [0 0 0], false);  
                    [xyzImResized(:,:,:,batchIdx), Kscaled(:,:,batchIdx)] = computeXYZImage(obj,...
                        imDepthResized, K{batchIdx}, scaleRatio(batchIdx)); %#ok<AGROW>
                else
                    orgSize{batchIdx} = size(im); %#ok<AGROW> 
                    [imResized(:,:,:,batchIdx), scaleRatio(batchIdx)] = preprocessImage(obj,...
                            single(im(:,:,:,batchIdx)), obj.InputSize, obj.ImageMean); %#ok<AGROW> 
                    imDepthResized = preprocessImage(obj,...
                        single(imDepth(:,:,:,batchIdx)), obj.InputSize, [0 0 0], false); 
                    [xyzImResized(:,:,:,batchIdx), Kscaled(:,:,batchIdx)] = computeXYZImage(obj,...
                        imDepthResized, K(:,:,batchIdx), scaleRatio(batchIdx)); %#ok<AGROW>
                end
            end

            assert( isequal(size(imResized), size(xyzImResized)) );
            
            if(isequal(executionEnvironment, 'gpu'))
                %GPU
                imResized = gpuArray(imResized);
                xyzImResized = gpuArray(xyzImResized);
            else
                %Host
                if(isgpuarray(imResized))
                    imResized = extractData(imResized);
                    xyzImResized = extractData(xyzImResized);
                end
            end
            dlX = dlarray(imResized, 'SSCB');
            dlXYZ = dlarray(xyzImResized, 'SSCB');
            netOut = predict(obj, dlX, dlXYZ);

            imageSize = size(imResized);

            proposals = extractdata(netOut{1});

            [boxes, boxLabel, boxScore, rotation, translationCoeffs] = postProcessOutputs(obj,...
                                          extractdata(netOut{2}),...
                                          extractdata(netOut{3}),...
                                                       proposals,...
                                          extractdata(netOut{5}),...
                                          extractdata(netOut{6}),...
                                            imageSize, batchSize);

            % If predicted boxes are empty, return empty mask & empty cat for
            % labels & empty pose
            if(isempty(boxes))
                boxLabel = categorical(boxLabel, 1:numel(obj.ClassNames), obj.ClassNames);
                masks = zeros(0,0,0, 'like', boxes);
                poses = rigidtform3d.empty(0);
                return;
            end

            % Computation of 3D translations on the final predicted bboxes
            % in a batch
            translation = cell(1, length(translationCoeffs));
            for batchIdx = 1:batchSize
                if ~(isempty(boxes{batchIdx}))
                    translation{batchIdx} = compute3DTranslationFromOffsets(obj,...
                        boxes{batchIdx}, translationCoeffs{batchIdx},...
                        xyzImResized(:,:,:,batchIdx), Kscaled(:,:,batchIdx));
                end
            end
            
            % create a flattened list of boxes
            maskBranchBoxes = [];
            maskBranchLabels = [];
            for idx = 1:length(boxes)
                if(~isempty(boxes{idx}))
                    maskBranchBoxes = [maskBranchBoxes ; [boxes{idx}  idx*ones(size(boxes{idx},1), 1)]]; %#ok<AGROW> 
                    maskBranchLabels = [maskBranchLabels; boxLabel{idx}];                                %#ok<AGROW> 
                end
            end

            % If predicted boxes are empty, return empty mask and empty
            % pose
            if(isempty(maskBranchBoxes))
                masks = zeros(0,0,0, 'like', boxes{1});
                masks = repmat({masks}, 1, length(boxes));
                poses = rigidtform3d.empty(0);
                poses = repmat({poses}, 1, length(poses));
                return;
            end
            
            maskBranchBoxes = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(...
                maskBranchBoxes);
    
            dlBoxes = dlarray(maskBranchBoxes', 'SSCB');
            
            % Feature pooling for mask branch
            dlMaskPooled = roiAlignPooling(obj, netOut{4}, dlBoxes, obj.MaskPoolSize);
            
            dlPostFeatureMask = predict(obj.PostPoolFeatureExtractionNet, dlMaskPooled, dlMaskPooled, 'Acceleration','auto');
            
            % Predict on mask branch to get h x w x numClasses x numProposals
            % cropped masks.
            dlMasks = predict(obj.MaskSegmentationHead, dlPostFeatureMask);
            dlMasks = extractdata(dlMasks);
    
            % Extract the cropped mask corresponding to the boxLabel for
            % each proposal.
            finalCroppedMasks = zeros(size(dlMasks,1), size(dlMasks, 2),...
                                      size(maskBranchBoxes,1));
            
            for i = 1:size(maskBranchBoxes,1)
                finalCroppedMasks(:,:,i) = dlMasks(:,:, maskBranchLabels(i), i);
            end
            
            % group masks in a cell array by the image idx
            imIdx = maskBranchBoxes(:,5);

            % package rotation and translation into rigidtform3d array
            poses = cell(1, batchSize);

            for idx = 1:batchSize
                % Final box detections in original image coordinates
                if(isgpuarray(boxes{idx}))
                    boxes{idx} = gather(boxes{idx});
                end
                boxes{idx} = bboxResizePixel(obj, floor(boxes{idx}), 1/scaleRatio(idx));
    
                boxes{idx} = clipBoxes(obj, boxes{idx}, orgSize{idx});
                % Generate full sized masks from cropped masks
                masks{idx} = generateFullSizedMasks(obj, finalCroppedMasks(:,:,imIdx==idx), boxes{idx}, orgSize{idx}); %#ok<AGROW> 
                
                % Convert boxLabels to categorical
                boxLabel{idx} = categorical(boxLabel{idx}, 1:numel(obj.ClassNames), obj.ClassNames);

                % Final rotation and translation
                if(isgpuarray(rotation{idx}))
                    rotation{idx} = gather(rotation{idx});
                end
                if(isgpuarray(translation{idx}))
                    translation{idx} = gather(translation{idx});
                end

                % Package translations and rotations into ridigtform3d objects
                rotationMatrices = vision.internal.quaternion.quaternionToRotation(rotation{idx}.');
                for ii = 1:length(boxScore{idx})
                    posesTemp(ii) = rigidtform3d(...
                        rotationMatrices(:,:,ii),...
                        translation{idx}(ii,:)); %#ok<AGROW> 
                end
                poses{idx} = posesTemp';
            end
        end

        function [poses, boxLabel, boxScores, boxes, masks] = predictPoseInImgStack(obj, im, imDepth, K, miniBatchSize, executionEnvironment)
            
            stackSize = size(im,4);

            masks = [];
            boxes = [];
            boxLabel = [];
            boxScores = [];
            poses = [];

            % Process images from the imageStack
            for startIdx = 1 : miniBatchSize : stackSize
                
                endIdx = min(startIdx+miniBatchSize-1, stackSize);

                imBatch = im(:,:,:,startIdx:endIdx);
                imDepthBatch = imDepth(:,:,:,startIdx:endIdx);
                KBatch = K(:,:,startIdx:endIdx);
               
                [posesCell, boxLabelCell, boxScoreCell, boxesCell, masksCell] = ...
                                            predictPoseInBatch(obj, imBatch, imDepthBatch, KBatch, executionEnvironment); 

                masks = [masks; masksCell']; %#ok<AGROW> 
                boxes = [boxes; boxesCell']; %#ok<AGROW> 
                boxLabel = [boxLabel; boxLabelCell']; %#ok<AGROW> 
                boxScores = [boxScores; boxScoreCell']; %#ok<AGROW> 
                poses = [poses; posesCell']; %#ok<AGROW> 
            end

        end
    
        
        function [finalBoxes, finalLabels, finalScores, finalRotations, finalTranslations] = postProcessOutputs(obj, deltas, scores, proposals, rotations, translations, imageSize, batchSize)
            % postProcessOutputs converts network outputs to box detections
            % deltas - numClasses*4 x numProposals
            % scores - numClasses x numProposals
            % proposals - 5 x numProposals
            % rotations - numClasses*4 x numProposals (quaternions)
            % translationCoeffs - numClasses*3 x numProposals
    
            % deltas hold the box regressions in the format [dx1 dy1 dw1 dh1 ...
            % ... dx_numProposals dy_numProposals dw_numProposals dh_numProposals]
            
            deltas = squeeze(deltas);
            scores = squeeze(scores);
            proposalImageIdx = proposals(5,:);
            proposals = proposals';
            rotations = squeeze(rotations);
            translations = squeeze(translations);
            
            % Network outputs proposals in [x1 y1 x2 y2] format in the image
            % space. Convert to [x y w h] for further processing.
            proposals = vision.internal.cnn.boxUtils.x1y1x2y2ToXYWH(proposals);
            
            numClasses = length(obj.ClassNames);
    
            % Step 1: apply regression weights
            deltas(1:4:numClasses*4,:) = deltas(1:4:numClasses*4,:) * obj.BoxStd(1) + obj.BoxMean(1);
            deltas(2:4:numClasses*4,:) = deltas(2:4:numClasses*4,:) * obj.BoxStd(2) + obj.BoxMean(2);
            deltas(3:4:numClasses*4,:) = deltas(3:4:numClasses*4,:) * obj.BoxStd(3) + obj.BoxMean(3);
            deltas(4:4:numClasses*4,:) = deltas(4:4:numClasses*4,:) * obj.BoxStd(4) + obj.BoxMean(4);
            
            % Step 2: Apply regression to proposals
            boxes = applyRegression(obj, proposals, deltas);%, params.MinSize, params.MaxSize);
    
            % Step 3: convert boxes from numClasses*4 x numProposals to
            % 4xnumClassesxnumProposals
            boxes = reshape(boxes, 4, [], size(proposals,1));
            rotations =  reshape(rotations, 4, [], size(proposals,1)); % also apply to per-class poses
            translations =  reshape(translations, 3, [], size(proposals,1));

            % Step 4: Remove scores corresponding to background & get the label
            % with max score
            backgroundIdx = numClasses+1;
            scores(backgroundIdx, :) = [];
            [boxScore, boxLabel] = max(scores, [], 1);
    
            % Step 5: Keep boxes corresponding to the highest score
            boxesTrim = zeros(4, size(boxes,3), 'like', boxes);
            for i = 1:size(boxes,3)
                boxesTrim(:,i) = squeeze(boxes(:,boxLabel(i), i));
            end
            boxes = boxesTrim;

            % Keep highest-scoring class predictions from per-class pose
            % predictions
            % - rotation
            rotationsTrim = zeros(4, size(rotations,3), 'like', rotations);
            for i = 1:size(rotations,3)
                rotationsTrim(:,i) = squeeze(rotations(:,boxLabel(i), i));
            end
            rotations = rotationsTrim;
            % - translation
            translationsTrim = zeros(3, size(translations,3), 'like', translations);
            for i = 1:size(translations,3)
                translationsTrim(:,i) = squeeze(translations(:,boxLabel(i), i));
            end
            translations = translationsTrim;
            
            % Step 6: Filter out boxes with lowscores & outside the image
            lowScores = boxScore <= obj.ScoreThreshold;
            invalidBoxes = getInvalidBoxesAndScores(obj, boxes, boxScore, imageSize);
            
            removeIdx = lowScores|invalidBoxes;
            boxes(:, removeIdx) = [];
            boxScore(removeIdx) = [];
            boxLabel(removeIdx) = [];
            proposalImageIdx(removeIdx) = [];
            rotations(:, removeIdx) = [];
            translations(:, removeIdx) = [];
            assert(size(boxes,2) == size(rotations,2))
            assert(size(boxes,2) == size(translations,2))
    
            % Step 7: Clip the boxes to image bounds
            % the boxes here get flipped from 4xnumPropoals to numProposalsx4
            boxes = clipBoxes(obj, boxes', imageSize);

            % Step 8: Group predictions by image and apply NMS

            for imageIdx = 1:batchSize
                
                boxIdxs = (proposalImageIdx==imageIdx);
                
                finalBoxes{imageIdx} = boxes(boxIdxs,:);                 %#ok<AGROW> 
                finalScores{imageIdx} = boxScore(boxIdxs)';              %#ok<AGROW> 
                finalLabels{imageIdx} = boxLabel(boxIdxs)';              %#ok<AGROW> 
                finalRotations{imageIdx} = rotations(:,boxIdxs)';        %#ok<AGROW>   % flipped to N x 4
                finalTranslations{imageIdx} = translations(:,boxIdxs)';  %#ok<AGROW>   % flipped to N x 3
    
                % Step 9a: Multi-class NMS per image
                if ~isinf(obj.SelectStrongestMulticlassThreshold)
                    [finalBoxes{imageIdx},finalScores{imageIdx},finalLabels{imageIdx}, nmsIdxs] = ...
                    selectStrongestBboxMulticlass(gather(finalBoxes{imageIdx}),...
                                                    gather(finalScores{imageIdx}),...
                                                    gather(finalLabels{imageIdx}),...
                                                    'RatioType', 'Union', ...
                                                    'OverlapThreshold', obj.SelectStrongestMulticlassThreshold,...
                                                    'NumStrongest', obj.NumStrongestRegionsPrediction); %#ok<AGROW> 
                    finalRotations{imageIdx} = finalRotations{imageIdx}(nmsIdxs,:);                     %#ok<AGROW> 
                    finalTranslations{imageIdx} = finalTranslations{imageIdx}(nmsIdxs,:);               %#ok<AGROW> 
                end

                % Step 9b: Class-agnostic NMS per image
                if ~isinf(obj.SelectStrongestThreshold)
                    [finalBoxes{imageIdx},finalScores{imageIdx},finalLabels{imageIdx}, nmsIdxs] = ...
                    selectStrongestBboxMulticlass(gather(finalBoxes{imageIdx}),...
                                                    gather(finalScores{imageIdx}),...
                                                    gather(finalLabels{imageIdx}),...
                                                    'RatioType', 'Union', ...
                                                    'OverlapThreshold', obj.SelectStrongestThreshold,...
                                                    'NumStrongest', obj.NumStrongestRegionsPrediction); %#ok<AGROW> 
                    finalRotations{imageIdx} = finalRotations{imageIdx}(nmsIdxs,:);                     %#ok<AGROW> 
                    finalTranslations{imageIdx} = finalTranslations{imageIdx}(nmsIdxs,:);               %#ok<AGROW> 
                end
                
            end

        end

        function [lowerBounds3D,upperBounds3D] = bboxBounds3D(obj, boxes, xyzImg)
        % bboxBounds3D get 3D corners in world coordinates from 2D-boxes
        %   boxes is a numPrediction-by-4 array in format xywh.
        %   xyzImg is H-by-W-by-3 image containing the X, Y and Z (depth) values in
        %   world coordinates corresponding to each pixel position in the input 
        %   RGB image.
            numPreds = size(boxes,1);
            lowerBounds3D = zeros(numPreds, 3);
            upperBounds3D = zeros(numPreds, 3);

            function z = iPercentiles(data, p)
                data = data(:);
                z = interp1(linspace(1/numel(data),1,numel(data)), sort(data), p);
            end

            % Ensure 2D bounding box coordinates are integers and lie
            % inside the image dimensions
            boxes = round(boxes);
            boxes = clipBoxes(obj, boxes, size(xyzImg));
        
            % The "XYZ image" contains the 3-D world-coordinates for every pixel
            % position in the input image. For each 2D bbox, taking the min/max  
            % of the corresponding XYZ Image patch will give the 3-D bounds
            % corresponding to that box in world-coordinates.
            for i = 1:numPreds
                xyzPatch = xyzImg(...
                    boxes(i,2):boxes(i,2)+boxes(i,4)-1,...
                    boxes(i,1):boxes(i,1)+boxes(i,3)-1,...
                    :);
                lowerBounds3D(i,1:2) = min(xyzPatch(:,:,1:2), [], [1 2]);
                upperBounds3D(i,1:2) = max(xyzPatch(:,:,1:2), [], [1 2]);
        
                % Use robust statistics for estimating bounds of the depth 
                % channel (Z) to provide some robustness to noisy depth maps.
                z = iPercentiles(xyzPatch(:,:,3), [0.05 0.95]);
                lowerBounds3D(i,3) = z(1);
                upperBounds3D(i,3) = z(2);
            end
        end

        function bboxB = bboxResizePixel(~,bboxA,scale)
            % bboxResizePixel resizes the bbox in pixel coordinates space.
            % This is a stand-in implementation for resize until we move to
            % spatial coordinates (when we will use the bboxresize function)

            % Calculate foward mapping such that 0.5 in input space maps to 0.5 in
            % output space, and 1.5 in input space maps to 0.5+scale in output space.
            % This is the mapping used by IMRESIZE.
            sy = scale;
            sx = scale;
            
            tx = (1-sx)*0.5;
            ty = (1-sy)*0.5;
            
            % Convert xywh to box corner format. Place box corners at pixel centers in
            % spatial coordinates.
            boxes = bboxA(:,[1 2]);
            boxes = [boxes boxes + bboxA(:,[3 4]) - 1];
            
            % Scale boxes
            boxes = boxes .* [sx sy sx sy] + [tx ty tx ty];
            bboxB = max(1,round(boxes));
            
            % Covert from box corner to xywh.
            bboxB(:,[3 4]) = bboxB(:,[3 4]) - bboxB(:,[1 2]) + 1;
            
        end
 

        function boxes = applyRegression(~, proposals, deltas)
        % applyRegression applies regression deltas to the input proposals- boxes
        % deltas - numClasses*4 x numProposals (regression for each box, for each class)
        % proposals - numProposals x 5
        % boxes - numClasses*4 x numProposals

        boxes = zeros(size(deltas), 'like', deltas);

        dx = deltas(1:4:end,:);
        dy = deltas(2:4:end,:);
        dw = deltas(3:4:end,:);
        dh = deltas(4:4:end,:);

        % Center of proposals
        cx = proposals(:,1) + proposals(:,3)*0.5;
        cy = proposals(:,2) + proposals(:,4)*0.5; 
        
        % Apply regression
        
        gx = dx.*proposals(:,3)' + cx';
        gy = dy.*proposals(:,4)' + cy';
        gw = proposals(:,3)' .* exp(dw);
        gh = proposals(:,4)' .* exp(dh);
        
        boxes(1:4:end, :) = gx - 0.5*gw;
        boxes(2:4:end, :) = gy - 0.5*gh;
        boxes(3:4:end, :) = gw;
        boxes(4:4:end, :) = gh;

        end

        function invalidIdx = getInvalidBoxesAndScores(~, boxes, scores, imageSize)
        % getInvalidBoxesAndScores, returns indices of boxes completely outside the
        % image or with invalid scores (infs and NaNs)
        x1 = boxes(1,:);
        y1 = boxes(2,:);
        x2 = boxes(3,:) + x1 - 1;
        y2 = boxes(4,:) + y1 - 1;
    
        boxOverlapsImage = ...
            (x1 < imageSize(2) & x2 > 1) & ...
            (y1 < imageSize(1) & y2 > 1);

        invalidIdx = ~boxOverlapsImage;

        invalidIdx = invalidIdx | ~all(isfinite(boxes),1);
        invalidIdx = invalidIdx | ~(isfinite(scores));

        end

        function bbox = clipBoxes(~, bbox, imgSize)
        % clipBoxes, clips the boxes to lower and upper image bounds.
        % bbox - Nx4 box coordinates ordered as  [x y w h].
        % Assumption: The original bounding boxes all overlap the image. 
    
        % Get coordinates of upper-left (x1,y1) and bottom-right (x2,y2) corners. 
        x1 = bbox(:,1);
        y1 = bbox(:,2);
    
        offset = 1;
    
        x2 = bbox(:,1) + bbox(:,3) - offset;
        y2 = bbox(:,2) + bbox(:,4) - offset;
    
        x1(x1 < 1) = 1;
        y1(y1 < 1) = 1;

        x1(x1>imgSize(2)) = imgSize(2);
        y1(y1>imgSize(1)) = imgSize(1);


        x2(x2 < 1) = 1;
        y2(y2 < 1) = 1;

        x2(x2 > imgSize(2)) = imgSize(2);
        y2(y2 > imgSize(1)) = imgSize(1);
    
            
        bbox = [x1 y1 x2-x1+offset y2-y1+offset];
        end

        function masks = generateFullSizedMasks(obj, croppedMasks, boxes, imageSize)
        % generateFullSizedMasks inserts croppedMaks back into a full sized
        % mask using the object locations specified in boxes
        % croppedMasks - h x w x numObjects
        % boxes        - numObjects x 4
        % masks        - imageSize(1) x imageSize(2) x numObjects
            numObjects = size(boxes,1);
            masks = zeros([imageSize(1:2) numObjects], 'logical');
            for i=1:numObjects
        
                m = imresize(croppedMasks(:,:,i), [boxes(i,4) boxes(i,3)],'cubic') > obj.MaskThreshold ;
                masks(boxes(i,2):boxes(i,2)+boxes(i,4)-1, ...
                    boxes(i,1):boxes(i,1)+boxes(i,3)-1, i) = m;
        
            end
        end

        function T = rescale3DLocations(~,translations,scale)
            T = [scale 0 0; 0 scale 0; 0 0 1] * translations';
            T = T';
        end

        function dlRotation = normalizePerClassQuaternions(obj, dlRotation)
            rotationSize = floor(size(dlRotation,3) / obj.NumClasses);
            dlRotation = reshape(dlRotation, 1, 1, rotationSize, obj.NumClasses,[]);
            quaternionNorms = sqrt(sum(dlRotation.^2, 3));
            dlRotation = dlRotation ./ quaternionNorms;
            dlRotation = reshape(dlRotation, 1, 1, rotationSize * obj.NumClasses, []);
        end

    end
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Learnables update methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
        function s = get.AllLearnables(obj)
            % Package all subnetwork learnables into a structure
            if(~obj.FreezeBackbone)
                s.FeatureExtractionNet = obj.FeatureExtractionNet.Learnables;
                s.PostPoolFeatureExtractionNet = obj.PostPoolFeatureExtractionNet.Learnables;
            end

            if(~obj.FreezeRPN)
                s.RegionProposalNet = obj.RegionProposalNet.Learnables;
            end
            
            s.DetectionHeads = obj.DetectionHeads.Learnables;
            s.MaskSegmentationHead = obj.MaskSegmentationHead.Learnables;
        end

        function obj = set.AllLearnables(obj, in)
            % Update Learnables on each subnetwork
            if(isfield(in, 'FeatureExtractionNet'))
                obj.FeatureExtractionNet.Learnables = in.FeatureExtractionNet;
            end

            if(isfield(in, 'RegionProposalNet'))
                obj.RegionProposalNet.Learnables = in.RegionProposalNet;
            end
            
            if(isfield(in, 'PostPoolFeatureExtractionNet'))
                obj.PostPoolFeatureExtractionNet.Learnables = in.PostPoolFeatureExtractionNet;
            end

            if(isfield(in, 'DetectionHeads'))
                obj.DetectionHeads.Learnables = in.DetectionHeads;
            end

            if(isfield(in, 'MaskSegmentationHead'))
                obj.MaskSegmentationHead.Learnables = in.MaskSegmentationHead;
            end

        end

        
    end

    methods(Access=protected)
        function obj = updateLearnableParameters(obj, updater)
            % Get the learnables, update them and then set them back on
            % the subnetworks.
            learnables = obj.AllLearnables;
            learnables = updater.apply(learnables);
            obj.AllLearnables = learnables;
        end
    end

    % getters for hidden properties needed by dltrain
    methods

        function layers = get.Layers(obj)

            layers = vertcat(obj.FeatureExtractionNet.Layers,...
                             obj.RegionProposalNet.Layers,...
                             obj.PostPoolFeatureExtractionNet.Layers,...
                             obj.DetectionHeads.Layers,...
                             obj.MaskSegmentationHead.Layers);

        end
        
        function obj =  set.State(obj, ~)

        end
        
        function state = get.State(obj)
            state = vertcat(obj.FeatureExtractionNet.State,...
                             obj.RegionProposalNet.State,...
                             obj.PostPoolFeatureExtractionNet.State,...
                             obj.DetectionHeads.State,...
                             obj.MaskSegmentationHead.State);
        end

        function learnables = get.Learnables(obj)
            learnables = obj.AllLearnables;
        end

        function inputs = get.InputNames(obj)
            inputs = obj.InputNames;
        end

        function obj = set.InputNames(obj, inputNames)
            obj.InputNames = inputNames;
        end

        function outnames = get.OutputNames(obj)
            outnames = horzcat(...
                             obj.RegionProposalNet.OutputNames,...
                             {'proposals'},...
                             obj.DetectionHeads.OutputNames,...
                             obj.MaskSegmentationHead.OutputNames);
        end

    end

end

% Helpers
%--------------------------------------------------------------------------
function classes = iGetCOCOClasses()
classes = {'person','bicycle','car','motorbike','aeroplane','bus',...
    'train','truck','boat','traffic light','fire hydrant',...
    'stop sign','parking meter','bench','bird','cat','dog',...
    'horse','sheep','cow','elephant','bear','zebra','giraffe',...
    'backpack','umbrella','handbag','tie','suitcase','frisbee',...
    'skis','snowboard','sports ball','kite','baseball bat',...
    'baseball glove','skateboard','surfboard','tennis racket',...
    'bottle','wine glass','cup','fork','knife','spoon','bowl',...
    'banana','apple','sandwich','orange','broccoli','carrot',...
    'hot dog','pizza','donut','cake','chair','sofa',...
    'pottedplant','bed','diningtable','toilet','tvmonitor',...
    'laptop','mouse','remote','keyboard','cell phone',...
    'microwave','oven','toaster','sink','refrigerator',...
    'book','clock','vase','scissors','teddy bear',...
    'hair drier','toothbrush'};
end

%--------------------------------------------------------------------------
function [spkgroot,spkgrootMaskrcnn] = iTripwirePoseMaskRCNN()
    % Check if posemaskrcnn and maskrcnn support packages are both
    % installed, and return their respective spkg root locations

    % Check if posemaskrcnn support package is installed
    breadcrumbFile = 'vision.internal.cnn.supportpackages.IsPoseMaskRCNNInstalled';
    fullPath = which(breadcrumbFile);
    if isempty(fullPath)
        name     = 'Computer Vision Toolbox Model for Pose Mask R-CNN 6-DoF Object Pose Estimation';
        basecode = 'CVT_Pose_Mask';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired',...
            mfilename, name, basecode)));
    end
    relPath =  fullfile(filesep,'+vision','+internal','+cnn',...
        '+supportpackages','IsPoseMaskRCNNInstalled.m');
    idx = strfind(fullPath, relPath);
    spkgroot = fullPath(1:idx);
    
    % Check if maskrcnn support package is installed
    breadcrumbFileMaskrcnn = 'vision.internal.cnn.supportpackages.IsMaskRCNNInstalled';
    fullPathMaskRCNN = which(breadcrumbFileMaskrcnn);
    if isempty(fullPathMaskRCNN)
        nameMaskrcnn     = 'Computer Vision Toolbox Model for Mask R-CNN Instance Segmentation';
        basecodeMaskrcnn = 'RCNN';
        throwAsCaller(MException(message('nnet_cnn:supportpackages:InstallRequired',...
            mfilename, nameMaskrcnn, basecodeMaskrcnn)));
    end
    relPathMaskrcnn =  fullfile(filesep,'+vision','+internal','+cnn',...
        '+supportpackages','IsMaskRCNNInstalled.m');
    idx = strfind(fullPathMaskRCNN, relPathMaskrcnn);
    spkgrootMaskrcnn = fullPathMaskRCNN(1:idx);
end

%--------------------------------------------------------------------------
function anchorBoxes = iGetDefaultCOCOAnchorBoxes()
baseAnchor = [11.313708305358887 22.627416610717773; 16 16; 22.627416610717773 11.313708305358887]*2;
anchorBoxes = round([baseAnchor; baseAnchor*2; baseAnchor*4; baseAnchor*8; baseAnchor*16]);
end

% Input validators
%--------------------------------------------------------------------------
function mustBeUniqueNames(input)    
     %Check for unique class names
     % input is a single char string, return
     if(ischar(input))
         return;
     end

     % For cell array of char strings, array of strings and categorical
     % arrays, check of uniqueness
     if(length(input)~=length(unique(input)))
        eid = 'vision:posemaskrcnn:duplicateClassNames';
        msg = vision.getMessage(eid);
        error(eid, msg);
     end
end

%--------------------------------------------------------------------------
function validateInferenceInputs(im, imDepth, intrinsics)

    % validate batch dimensions are matching
    if ~( (size(intrinsics,2) == 1) )
        eid = 'vision:posemaskrcnn:incorrectIntrinsicsSize';
        msg = vision.getMessage(eid);
        error(eid, msg)
    end

    % validate batch dimensions are matching
    if ~( (size(im,4) == size(imDepth,4)) && (size(imDepth,4) == size(intrinsics,1)) )
        eid = 'vision:posemaskrcnn:incorrectBatchDimension';
        msg = vision.getMessage(eid);
        error(eid, msg)
    end
    
    % image and depth-map spatial dimensions are matching
    if ~( (size(im,1) == size(imDepth,1)) && (size(im,2) == size(imDepth,2)) )
        eid = 'vision:posemaskrcnn:incorrectSpatialDimension';
        msg = vision.getMessage(eid);
        error(eid, msg)
    end

end

%--------------------------------------------------------------------------
function validateInfThreshold(threshold)
    % SelectStrongestBbox (NMS) thresholds must be between [0,1] unless inf
    % is used to disable NMS altogether
    if ~isinf(threshold)
        mustBeLessThanOrEqual(threshold, 1);
    end
end

%--------------------------------------------------------------------------
function validateClassNames(input)
    
    % Check of single char string
    if(ischar(input))
        return;
    end
    
    % Check for cell array of char strings
    if(iscell(input))
        if(all(cellfun(@ischar, input)) && isvector(input))
            return;
        end
    end

    % Check for string & categorical vectors
    if( (isstring(input)||iscategorical(input)) && isvector(input))
        return;
    end

    eid = 'vision:posemaskrcnn:incorrectClassNameType';
    msg = vision.getMessage(eid);
    error(eid, msg)

end

%--------------------------------------------------------------------------
function options = processInputSizes(options)
    % Given ImageInputSize NVP in the class constructor, assign correct
    % values of InputSize (network input layer) and DepthImageInputSize
    options.InputSize = [];
    options.DepthImageInputSize = [];
    if ~isempty(options.ImageInputSize)
        options.DepthImageInputSize = [options.ImageInputSize(1:2) 1];
        options.InputSize = [options.ImageInputSize(1:2) 6];
    end
end

%--------------------------------------------------------------------------
function mustBeRGBSize(input)    
     % the size must either be [] or (1,3) with the channel dim =3 
     if~( isempty(input) || (all(size(input)==[1 3]) && (input(3)==3)) )
        eid = 'vision:posemaskrcnn:incorrectImageInputSize';
        msg = vision.getMessage(eid);
        error(eid, msg)
     end
end

%--------------------------------------------------------------------------
function validateImageInput(in)
    
    im = [];
    if(isnumeric(in))
        im = in;
    elseif(matlab.io.datastore.internal.shim.isDatastore(in))
        out = preview(in);
        if(iscell(out))
            if(isempty(out))
                im = [];
            else
                im = out{1};
            end
        else
            im = out;
        end
    end

    if(~validateImage(im)||isempty(im))
        throwAsCaller(MException('vision:posemaskrcnn:invalidImageInput',...
               vision.getMessage('vision:posemaskrcnn:invalidImageInput')));
    end

end

%--------------------------------------------------------------------------
function validateDepthImageInput(in)
    
    im = [];
    if(isnumeric(in))
        im = in;
    elseif(matlab.io.datastore.internal.shim.isDatastore(in))
        out = preview(in);
        if(iscell(out))
            if(isempty(out))
                im = [];
            else
                im = out{1};
            end
        else
            im = out;
        end
    end

    if(~validateDepthImage(im)||isempty(im))
        throwAsCaller(MException('vision:posemaskrcnn:invalidDepthImageInput',...
               vision.getMessage('vision:posemaskrcnn:invalidDepthImageInput')));
    end

end

%--------------------------------------------------------------------------
function tf = validateImage(in)
    tf = isnumeric(in)&&...
         ndims(in)<=4 && ...             % numdims should be less than 3 
         (size(in,3)==3||size(in,3)==1); % gray scale or RGB image
end

%--------------------------------------------------------------------------
function tf = validateDepthImage(in)
    tf = isnumeric(in)&&...
         ndims(in)<=4 && ... % numdims should be less than 3 
         (size(in,3)==1);    % single channel depth image
end

%--------------------------------------------------------------------------
function validateLogicalFlag(in)
    validateattributes(in,{'logical'}, {'scalar','finite', 'real'});
end

%--------------------------------------------------------------------------
function data = iLoadNetwork(spkgroot, datafolder, matfile)
matfile = fullfile(spkgroot, datafolder, matfile);
data = load(matfile);
end

