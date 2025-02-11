classdef RegionProposalLayer < nnet.internal.cnn.layer.Layer & nnet.internal.cnn.layer.SizePropagationValueSink
    % Region proposal layer.
    
    % Copyright 2018-2022 The MathWorks, Inc.
    
    properties
        LearnableParameters = [];
        
        % Name
        Name
        
        % MinSize The minimum proposal box size. Boxes smaller than MinSize
        % are discarded.
        MinSize
        
        % MaxSize The maximum proposal box size. Boxes larger than MaxSize
        % are discarded.
        MaxSize
        
        % ScaleFactor ([sx sy]) Used to scale boxes from feature map to
        %             image.
        ScaleFactor
        
        % ImageSize The size of the image for which boxes are proposed.
        ImageSize
        
        % NumStrongestRegions The number of strongest proposal boxes to
        %                     return.
        NumStrongestRegions
    end
    
    properties (Constant)
        % DefaultName Default layer's name
        DefaultName = 'regionproposal'
    end
    
    properties(SetAccess = private)
        % InputNames
        InputNames = {'scores','boxDeltas'}
        
        % OutputNames
        OutputNames = {'out'}
        
        % HasSizeDetermined True for layers with size determined.
        HasSizeDetermined = true;
        
        % AnchorBoxes The anchor boxes used as reference for final
        %             detections.
        AnchorBoxes
        
        % ProposalsOutsideImage Whether proposals outside image should be
        %                       clipped or discarded. Set to 'clip' to
        %                       clip. Otherwise boxes are discarded.
        ProposalsOutsideImage
        
        % OverlapThreshold The overlap threshold used for NMS.
        OverlapThreshold                
        
        % NumStrongestRegionsBeforeProposalNMS
        % The number of strongest proposals to keep prior to running NMS.
        % This help reduce the number of boxes that need to be processed.
        NumStrongestRegionsBeforeProposalNMS
        
        % MinScore The minimum proposal score required for a box to be
        %          output by this layer.
        MinScore               
        
        % The standard dev and mean of the boxes. Used when applying the
        % box regression offsets.
        RPNBoxStd
        RPNBoxMean                
        
        % BoxFilterFcn A function handle. The specified function is used to
        % filter out boxes based on MinSize and MaxSize. A function handle
        % allows customization of box filtering. For example, a monoCamera
        % based filtering is used in ADST.
        BoxFilterFcn
        
    end
    
    methods
        function val = get.NumStrongestRegions(this)
            val = vision.internal.cnn.layer.util.PropertyCache.fetchValue(this.NumStrongestRegions);
        end
        
        function val = get.ImageSize(this)
            val = vision.internal.cnn.layer.util.PropertyCache.fetchValue(this.ImageSize);
        end
        
        function val = get.MinSize(this)
            val = vision.internal.cnn.layer.util.PropertyCache.fetchValue(this.MinSize);
        end
        
        function val = get.MaxSize(this)
            val = vision.internal.cnn.layer.util.PropertyCache.fetchValue(this.MaxSize);
        end
        
        function val = get.ScaleFactor(this)
            val = vision.internal.cnn.layer.util.PropertyCache.fetchValue(this.ScaleFactor);
        end
    end
    
    methods
        function this = RegionProposalLayer(name, anchorBoxes, params)
            
            % anchor boxes may be fractional for backward compatibility
            % with previous releases.
            this.AnchorBoxes = anchorBoxes;
            this.Name = name;
            
            if nargin > 2
                this.ImageSize = params.ImageSize;
                this.RPNBoxStd = params.RPNBoxStd;
                this.RPNBoxMean = params.RPNBoxMean;
                this.MinSize = params.MinSize;
                this.MaxSize = params.MaxSize;
                this.NumStrongestRegions = params.NumStrongestRegions;
                this.NumStrongestRegionsBeforeProposalNMS = params.NumStrongestRegionsBeforeProposalNMS;
                this.ProposalsOutsideImage = params.ProposalsOutsideImage;
                this.MinScore = params.MinScore;
                this.OverlapThreshold = params.OverlapThreshold;
                this.ScaleFactor = params.ScaleFactor;
                this.BoxFilterFcn = params.BoxFilterFcn;
            end
        end
                     
        %------------------------------------------------------------------
        function allBoxes = predict(this, inputs)
            % Return bboxes in [x1 y1 x2 y2 idx] format. The boxes are in
            % the image space.
            clsBatch = inputs{1};
            regBatch = inputs{2};
            
            featureSize = size(clsBatch);
            
            if isempty(this.ImageSize)
                imageSize = ceil(featureSize(1:2) .* fliplr(this.ScaleFactor));
            else
                imageSize = this.ImageSize;
            end
            
            % Get number of observations;
            N = size(clsBatch,4);
            allBoxes = cell(N,1);
            for i = 1:N
                cls = clsBatch(:,:,:,i);
                reg = regBatch(:,:,:,i);
                
                [bboxes, scores] = this.proposeImpl(cls, reg);
                
                bboxes = this.postProcessing(bboxes, scores, imageSize);
                
                bboxes = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(bboxes);
                
                % Associate batch indices with each set of boxes.
                numBoxes = size(bboxes,1);
                allBoxes{i} = [bboxes repelem(i,numBoxes,1)];
            end
            
            allBoxes = vertcat(allBoxes{:});
            
        end
        
        %------------------------------------------------------------------
        function  [dX, dW] = backward(~, inputs, ~, ~, ~)
            cls = inputs{1};
            reg = inputs{2};
            dCls = zeros(size(cls),'like',cls);
            dReg = zeros(size(reg),'like',reg);
            dX = {dCls,dReg};
            dW = [];
        end
        
        %------------------------------------------------------------------
        function Zs = forwardExampleInputs(this, Zs)
            % The actual output size depends on image content. Report the
            % minimum output size. Outputs boxes in [x1 y1 x2 y2 idx].

            % First validate the input data
            this.validateInputs(Zs);

            % Now set size for the placeholder arrays.
            Zs{1} = setSizeForDim(Zs{1},'S',1);
            Zs{1} = setSizeForDim(Zs{1},'C',5);
            Zs(2) = [];
        end

        function this = configureForInputs(this,Zs)
            this.validateInputs(Zs);
        end  
    end
    
    %----------------------------------------------------------------------
    % Following methods do not require an implementation, but must be
    % defined.
    %----------------------------------------------------------------------
    methods                
        
        function this = initializeLearnableParameters(this, ~)        
        end
        
        function this = prepareForTraining(this)            
        end
        
        function this = prepareForPrediction(this)            
        end
        
        function this = setupForHostPrediction(this)            
        end
        
        function this = setupForGPUPrediction(this)            
        end
        
        function this = setupForHostTraining(this)            
        end
        
        function this = setupForGPUTraining(this)            
        end
    end
    
    methods(Hidden)
        %------------------------------------------------------------------
        function [bboxes, scores] = proposeImpl(this, cls, reg)
            % Input cls are scores from the rpn cls conv layer 
            %   size(cls): [H W 2*NumAnchors numObs]
            %
            % Input reg are scores from the rpn reg conv layer
            %   size(reg): [H W 4*NumAnchors numObs]
            %
            % Output bboxes in [x y w h] format. In image space.
            
            % anchor boxes in [width height]
            aboxes = fliplr(this.AnchorBoxes);
            
            [fmapSizeY,fmapSizeX,numAnchorsTimesTwo,~] = size(cls);
            [y,x,k] = ndgrid(1:single(fmapSizeY), 1:single(fmapSizeX),1:size(aboxes,1));
            x = x(:);
            y = y(:);
            k = k(:);
            widthHeight = aboxes(k,:);
            
            % Box centers in image space (spatial coordinates).
            sx = this.ScaleFactor(1);
            sy = this.ScaleFactor(2);
            xCenter = x * sx + (1-sx)/2;
            yCenter = y * sy + (1-sy)/2;
            
            halfWidthHeight = widthHeight / 2 ;
            
            % top left in spatial coordinates
            x1 = xCenter - halfWidthHeight(:,1);
            y1 = yCenter - halfWidthHeight(:,2);
            
            % Convert to pixel coordinates.
            x1 = floor(x1 + 0.5);
            y1 = floor(y1 + 0.5);
            
            % anchor boxes as [x y w h]
            bboxes = [x1 y1 widthHeight];
            
            % Regression input is [H W 4*NumAnchors], where the 4
            % box coordinates are consecutive elements. Reshape data to
            % [H W 4 NumAnchors] so that we can gather the regression
            % values based on the max score indices.
            reg = reshape(reg, size(reg,1), size(reg,2), 4, []);
            
            % permute reg so that it is [H W NumAnchors 4]
            reg = permute(reg, [1 2 4 3]);
            
            % reshape reg so that it is [H*W*NumAnchors 4]
            reg = reshape(reg,[],4);
            
            % Apply box target normalization
            reg = (reg .* this.RPNBoxStd) + this.RPNBoxMean;
            
            bboxes = fastRCNNObjectDetector.applyReg(bboxes, reg);
            
            numAnchors = numAnchorsTimesTwo / 2;
            scores = reshape(cls(:,:,1:numAnchors,:),[],1);
            
            [bboxes, scores] = fastRCNNObjectDetector.removeInvalidBoxesAndScores(bboxes, scores);
            
        end
    end
    
    methods(Access = private)
        %------------------------------------------------------------------
        function [bboxes, scores] = postProcessing(this,bboxes,scores, imageSize)
            % Post processing is only supported on the CPU.
            bboxes = gather(bboxes);
            scores = gather(scores);
            
            if strcmp(this.ProposalsOutsideImage,'clip')
                bboxes = vision.internal.detector.clipBBox(bboxes,imageSize);
                
            else
                H = imageSize(1);
                W = imageSize(2);
                outside =  bboxes(:,1) < 1 | bboxes(:,2) < 1 | (bboxes(:,1)+bboxes(:,3)-1) > W | (bboxes(:,2)+bboxes(:,4)-1) > H;
                bboxes(outside,:) = [];
                scores(outside) = [];
            end
            
            [bboxes, scores] = this.BoxFilterFcn(bboxes, scores, this.MinSize, this.MaxSize);
            
            % remove low scoring proposals
            if this.MinScore > 0
                lowScores = scores < this.MinScore;
                bboxes(lowScores,:) = [];
                scores(lowScores,:) = [];
            end
            
            [bboxes, scores] = rcnnObjectDetector.selectStrongestRegions(bboxes, scores, this.NumStrongestRegionsBeforeProposalNMS);
            
            [bboxes, scores] = selectStrongestBbox(bboxes, scores, ...
                'RatioType', 'Union', ...
                'OverlapThreshold', this.OverlapThreshold,...
                'NumStrongest', this.NumStrongestRegions); 
        end

        function validateInputs(this, Zs)
            % Validate the input placeholder arrays are correct input sizes. 
                        
            % Validate classification scores input. 
            % Expected size is [H W 2*numAnchors].
            sz1 = size(Zs{1});
            numAnchors = size(this.AnchorBoxes,1);
            if ~(numel(sz1) >= 3 && sz1(3) == 2*numAnchors)
                error(message("vision:rcnn:incorrectInputRPNSoftmax",iSizeToString(sz1),numAnchors));
            end
          
            % Validate regression value input. 
            % Expected size is [H W 4*numAnchors N].
            sz2 = size(Zs{2});
            if ~(numel(sz2) >= 3 && sz2(3) == 4 * numAnchors)
                error(message("vision:rcnn:incorrectInputRPNboxReg",iSizeToString(sz2),numAnchors));
            end

            % Validate first 2 dimensions.
            if ~isequal(sz1(1:2),sz2(1:2))
                error(message("vision:rcnn:incorrectInputRegionProposal",iSizeToString(sz1),iSizeToString(sz2)));
            end
        end
    end
    
    methods(Static,Hidden)
        function s = emptyProposalParamsStruct()
            s.ImageSize = [];
            s.RPNBoxStd = [];
            s.RPNBoxMean = [];
            s.MinSize = [];
            s.MaxSize = [];
            s.NumStrongestRegions = [];
            s.NumStrongestRegionsBeforeProposalNMS = [];
            s.ProposalsOutsideImage = [];
            s.MinScore = [];
            s.OverlapThreshold = [];
            s.ScaleFactor = [];
            s.BoxFilterFcn = [];
        end
        
        function lgraph = updateParameters(lgraph, proposalParams)
            
            idx = arrayfun(@(x)isa(x,'nnet.cnn.layer.RegionProposalLayer'),...
                lgraph.Layers);
            idx = find(idx,1,'first');
            name = lgraph.Layers(idx).Name;
            
            % Use getAnchorBoxes to retrieve anchor boxes as they may
            % be fractional if detector was trained before 18b. In 18b
            % and beyond, the anchor boxes will be integer valued.
            anchorBoxes = lgraph.Layers(idx).getAnchorBoxes();
            
            proposalLayer = nnet.cnn.layer.RegionProposalLayer(...
                nnet.internal.cnn.layer.RegionProposalLayer(...
                name, anchorBoxes, proposalParams));

            lgraph = lgraph.replaceLayer(name, proposalLayer);
            
        end
        
    end
    
end

function str = iSizeToString(sz)
str = join(string(sz), matlab.internal.display.getDimensionSpecifier);
end