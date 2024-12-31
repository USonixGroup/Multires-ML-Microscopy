classdef (HandleCompatible) FasterRCNNEndToEndLossUtilities
    % Faster R-CNN loss computation utilities. Used in loss layer and in
    % summary layer to display training progress metrics.
    
    % Copyright 2018-2020 The MathWorks, Inc.
    properties
        ClassNames
        
        % BackgroundClass (char array) The name of the background class.
        BackgroundClass = 'Background';
        
        PositiveOverlapRange
        
        NegativeOverlapRange
        
        ProposalParameters
        
        % The number of regions to sample for each training image.
        NumRegionsToSample
        
        % RPN softmax layer
        RPNSoftmaxLayer
        
        % The input data is from a general datastore with Mx3 read output,
        % with first column as images, second column as boxes, and
        % the third as labels.
        IsGeneralDatastore
        
        % Categorical array containing all the classes as categories.
        %   Typically, categorical(this.ClassNames, this.ClassNames);
        ClassCategoricals
    end
    properties(Dependent)
        NumClasses
    end
    
    methods
        function v = get.NumClasses(this)
            v = numel(this.ClassNames);
        end
    end
    
    methods(Access = protected)
        function [Treg, Wreg, assignedLabels] = regressionResponse(this, proposals, gTruthBoxesInBatch, gTruthLabelsInBatch)
            [assignedGTruthBoxes, assignedLabels] = this.assign(proposals, gTruthBoxesInBatch, gTruthLabelsInBatch);
            
            % A proposal may not be assigned to a ground truth, in which
            % case the label will be <undefined>. We use a one-hot encoding
            % routine that supports <undefined> only for 4-D arrays.
            % Reshape the labels to allow usage of that routine.
            assignedLabels = reshape(assignedLabels, 1, 1, [], numel(assignedLabels));
            
            [Treg, Wreg] = vision.internal.cnn.FasterRCNNEndToEndLossUtilities.regressionResponsesAndInstanceWeights(proposals, assignedGTruthBoxes, assignedLabels, this.BackgroundClass);
        end
        
        function [Ycls, Tcls, Yreg, Treg, Wreg, hasResponsesInBatch, numRegionsToSample] = fastRCNNClassificationRegressionPredictionResponses(this, Y, T)
            
            proposals = this.proposalOutput(Y);  % [N 5]
            
            if isempty(proposals)
                Ycls = [];
                Tcls = [];
                Yreg = [];
                Treg = [];
                Wreg = [];
                
                hasResponsesInBatch = false;
                numRegionsToSample  = [];
            else
                hasResponsesInBatch = true;
                [gTruthBoxesInBatch, gTruthLabelsInBatch] = this.unpackResponses(T);
                
                if ~isempty(this.NumRegionsToSample)
                    numObs             = numel(gTruthBoxesInBatch);
                    numRegionsToSample = this.NumRegionsToSample * numObs;
                else
                    numRegionsToSample = [];
                end
                
                % Compute regression loss.
                Yreg = this.regressionOutput(Y); % [1 1 4 N]
                [Treg, Wreg, assignedLabels] = this.regressionResponse(proposals, gTruthBoxesInBatch, gTruthLabelsInBatch);
                
                % Compute classification
                Ycls = this.classificationOutput(Y);
                Tcls = this.classificationResponses(assignedLabels);
            end
        end
        
        function [Ycls, Tcls, Yreg, Treg, Wreg, hasResponsesInBatch] = rpnClassificationRegressionPredictionResponses(this, Y, T)
            % We need to update some of our functions (like bboxOverlapRatio, boxAssignmentUtils,...
            % vision.internal.cnn.rpn.selectTrainingSamples, etc.,) to accept gpuArrays. Until then, gather.
            Y = gather(Y);
            T = cellfun(@gather, T, 'UniformOutput', false);
            % First part of depthConcatenation:
            %    Y(:,:,1:regression,:)
            Yreg = this.rpnRegressionOutput(Y);
            % Second part of depthConcatenation:
            %    Y(:,:,regression+1:end,:)
            Ycls = this.rpnClassificationOutput(Y);
            
            hasResponsesInBatch = true;
            [gTruthBoxesInBatch, imageSizesInBatch] = this.unpackRPNResponses(T);
            
            % Compute regression loss.
            featureSize = size(Ycls);
            [Treg, Wreg, assignedLabels] = this.rpnRegressionResponse(featureSize, gTruthBoxesInBatch, imageSizesInBatch);
            
            % Targets for RPN classification.
            % Get the classification response from labels.
            Tcls = this.rpnClassificationResponses(assignedLabels);
        end
        
        function [Treg, Wreg, assignedLabels] = rpnRegressionResponse(this, featureSize, gTruthBoxesInBatch, imageSizesInBatch)
            proposalParams = iSetupProposalParams(this.ProposalParameters);
            
            numObservations = numel(imageSizesInBatch);
            data            = cell(numObservations, 3);
            for ii = 1:numObservations
                imageSize   = imageSizesInBatch{ii};
                samples     = vision.internal.cnn.rpn.selectTrainingSamples(imageSize, featureSize, proposalParams, gTruthBoxesInBatch{ii});
                [data{ii, 1}, data{ii, 2}, data{ii, 3}] = vision.internal.cnn.rpn.calculateClsRegressionResponses(samples, this.ProposalParameters);
            end
            
            % Categorical response labels will be a
            % numObservations size of each H*W-by-3*numAnchors.
            % Categorical responses need to be a 4-D array.
            assignedLabels = cat(4, data{:,1});
            
            % Regression responses are numObservations size of each
            % H-by-W-by-4*numAnchors array.
            Treg = cat(4, data{:,2});
            Wreg = cat(4, data{:,3});
        end
        
        function Yreg = regressionOutput(this, Y)
            R    = (4 * this.NumClasses) + 5 - 1;
            Yreg = Y(:,:,end-R:end-5,:);
        end
        
        function Ycls = classificationOutput(this, Y)
            Ycls = Y(:,:,1:(this.NumClasses+1),:);
        end
        
        function Yreg = rpnRegressionOutput(this, Y)
            % Regression size is 4*NumAnchors: the first part
            % of depthConcatenation layer
            regSize = 4 * this.ProposalParameters.NumAnchors;
            Yreg    = Y(:,:,1:regSize,:);
        end
        
        function Ycls = rpnClassificationOutput(this, Y)
            % Classification size is 2*NumAnchors: the second part
            % of depthConcatenation layer,
            clsSize = 2 * this.ProposalParameters.NumAnchors;
            Ycls    = Y(:,:,end-clsSize+1:end,:);
        end
        
        function [assignedGTruthBoxes, assignedLabels] = assign(this, proposals, gTruthBoxesInBatch, gTruthLabelsInBatch)
            
            numImagesInBatch = size(gTruthBoxesInBatch,1);
            proposalsInBatch = iGroupProposalsByImageIndex(proposals, numImagesInBatch);
            
            assignedGTruth   = cell(numImagesInBatch,1);
            assignedLabels   = cell(numImagesInBatch,1);
            for i = 1:numImagesInBatch
                proposalBoxes = proposalsInBatch{i};
                gTruthBoxes   = gTruthBoxesInBatch{i};
                gTruthLabels  = gTruthLabelsInBatch{i};
                
                % Assign a ground truth box to each proposal based on overlap
                % criteria.
                [assignedGTruth{i}, assignedLabels{i}] = iAssignProposalsToGroundTruthBoxes(...
                    proposalBoxes, gTruthBoxes, gTruthLabels, ...
                    this.PositiveOverlapRange, this.NegativeOverlapRange, ...
                    this.ClassNames, ...
                    this.BackgroundClass);
                
            end
            
            assignedGTruthBoxes = vertcat(assignedGTruth{:});
            assignedLabels      = vertcat(assignedLabels{:});
        end
        
        function [gTruthBoxesInBatch, gTruthLabelsInBatch] = unpackResponses(this, T)
            if this.IsGeneralDatastore
                % Input training data as datastores.
                % Ground truth data has 2 columns, 1 for boxes and the next for categorical labels.
                gTruthBoxesInBatch  = T(:,1);
                gTruthLabelsInBatch = cellfun(@(x)iFillLabelNames(this.ClassCategoricals,x),T(:,2),'UniformOutput',false);
            else
                % RPN network unchanged. RPN Proposal parameters are empty for training data as tables.
                % Ground truth table has columns of boxes for each class.
                [gTruthBoxesInBatch, gTruthLabelsInBatch] = cellfun(@(x)iMergeBoxLabels(this.ClassNames,x{:}),T,'UniformOutput',false);
            end
        end
        
        function T = classificationResponses(~, labels)
            % N.B. labels is [1 1 1 numProposals].
            T = iDummify4dArray(labels);
        end
        
        function T = rpnClassificationResponses(~, labels)
            % For RPN, labels can be a H*W-by-3*numAnchors matrix
            % when numObservations is 1, else it will be a
            % H*W-by-3*numAnchors-by-1-by-numObservations
            % Dummify this to an array of size:
            %     H*W-by-3*numAnchors-by-numCategories
            %     or
            %     H*W-by-3*numAnchors-by-numCategories-by-numObservations
            % Here numCategories is 2 for object is present or not.
            T = nnet.internal.cnn.util.dummify(labels);
        end
        
        function [gTruthBoxesInBatch, imageSizesInBatch] = unpackRPNResponses(~, T)
            imageSizesInBatch  = T(:,1);
            gTruthBoxesInBatch = T(:,2);
        end
        
        function proposals = proposalOutput(this, Y)
            % Get proposals from network output Y.
            
            numClasses = this.NumClasses;
            
            % Number of channels from classification branch.
            C = numClasses + 1; % include background
            
            % Number of channels from regression branch.
            R = numClasses * 4;
            
            if size(Y,3) == (C + R + 5)
                % Proposals are present in merged data.
                
                proposals = Y(:,:,end-4:end,:);      % [1 1 5 N]
                proposals = reshape(proposals,5,[]); % [5 N]
                proposals = proposals';              % [N 5]
                
                % convert [x1 y1 x2 y2 idx] boxes to [x y w h idx] format.
                proposals(:,3:4) = proposals(:,3:4) - proposals(:,1:2) + 1;
            else
                % No valid proposals generated by region proposal layer.
                proposals = zeros(0,5,'like',Y);
            end
            assert(size(proposals,2)==5, 'Expected size(proposals,2) to equal 5.');
            assert(all(isfinite(proposals(:))),'Expected proposals to be finite.');
        end
        
    end
    
    methods (Static)
        function [T,W] = regressionResponsesAndInstanceWeights(proposals, gTruth, labels, bgClass)
            % encode boxes and return as [N 4] array. 
            % 
            % Inputs:
            %    proposals [N 4] array of boxes.
            %    gTruth [N 4] array of boxes.
            %    labels [1 1 1 N] categorical array.
            %    bgClass is the name of the background class.
            %
            % Outputs:
            %    T is a [1 1 4*numClasses N] array of regression targets.
            %    W is a [1 1 4*numClasses N] array of weights. 
            
            % Instance weights for responses associated with positive assigned
            % proposals.
            W = cast(iDummify4dArray(labels),'like',proposals); % -> [1 1 numClasses+1 N] indicator matrix
            W = squeeze(W); % [numClasses+1 N]
            bgIndex = strcmp(bgClass,categories(labels));
            W(bgIndex,:) = []; % [numClasses N]
            
            % Only compute targets for proposal assigned to boxes.
            targets = zeros(size(proposals,1),4,'like',proposals);
            posIdx = any(W,1); % select which proposals are positive (i.e. have been assigned to a ground truth box).
            targets(posIdx,:) = vision.internal.rcnn.BoundingBoxRegressionModel.generateRegressionTargets(gTruth(posIdx,:), proposals(posIdx,:));
            
            numClasses = numel(categories(labels)) - 1; % exclude background
            
            targets = targets'; % [N 4] -> [4 N], N == number of proposals
            
            % [tx ty tw th] are grouped together. We replicate the group
            % for each class as R-CNN uses class-specific regression. 
            T = repmat(targets,numClasses,1); % -> [4*numClasses N], per class regression response.
            
            T = reshape(T,1, 1, 4 * numClasses, []); % -> [1 1 4*numClasses N]
            
            % Make instance weights match dimension of T. The weights
            % select the group of targets for one class.  
            %
            % For example if there are 3 proposals with assignments 
            % [c2 bg c1] then the weights are:
            %
            % W = [
            %     0 0 1
            %     1 0 0
            %    ];
            %
            % These weights are replicated to select the corresponding 4
            % regression targets for each proposal.
            %
            % W = [
            %    0 0 1
            %    0 0 1
            %    0 0 1
            %    0 0 1
            %    1 0 0
            %    1 0 0
            %    1 0 0
            %    1 0 0
            %    ];
            W = repelem(W,4,1); % [4*numClasses N]
            W = reshape(W,1,1,4*numClasses,[]); % [1 1 4*numClasses N]
            
            % Incorporate normalization term into instance weights. Treat
            % each positive as an observations. Times 4 because there are 4
            % independent targets per positive observation.
            numObservations = max(1,4*nnz(posIdx)); % prevent division by zero.
            W = (1./numObservations) * W;
        end
    end
end


function [AG] = iGroupProposalsByImageIndex(A,N)
% Group by indices.
ia = gather(A(:,5)); % gather: gpuArray does not support accumarray w/ @(x){x}.
iag = accumarray(ia, 1:numel(ia), [N 1], @(x){x});

AG = cell(N,1);
for i = 1:numel(iag)
    if isempty(iag{i})
        AG{i} = zeros(0,5,'like',A);
    else
        AG{i} = A(iag{i},:);
    end
end
end

function [gTruthBoxesAssignedToProposals, assignedLabels] = iAssignProposalsToGroundTruthBoxes(...
    proposals, gTruthBoxes, gTruthLabels, posOverlap, negOverlap, classNames, backgroundClassName)

proposals = gather(proposals);
gTruthBoxes = gather(gTruthBoxes);

assignAllGroundTruthBoxes = false;

[assignments, positiveIndex, negativeIndex] = vision.internal.cnn.boxAssignmentUtils.assignBoxesToGroundTruthBoxes(...
    proposals(:,1:4), gTruthBoxes, posOverlap, negOverlap, assignAllGroundTruthBoxes);

% labels assigned to proposals after matching proposal boxes to ground
% truth boxes.
assignedLabels = vision.internal.cnn.boxAssignmentUtils.boxLabelsFromAssignment(...
    assignments, gTruthLabels, ...
    positiveIndex, negativeIndex, ...
    classNames, backgroundClassName);

% gTruth boxes not assigned to proposals are left as [0 0 0 0];
gTruthBoxesAssignedToProposals = zeros([size(proposals,1) 4],'like',proposals);
gTruthBoxesAssignedToProposals(positiveIndex,:) = gTruthBoxes(assignments(positiveIndex),:);

end

%--------------------------------------------------------------------------
function [bboxes, labels] = iMergeBoxLabels(classNames, varargin)
% cat all multi-class bounding boxes into one M-by-4 matrix.
bboxes = vertcat(varargin{1:numel(varargin)});

% create list of class names corresponding to each ground truth
% box.
cls = cell(1, numel(classNames));
for i = 1:numel(varargin)
    cls{i} = repelem(classNames(i), size(varargin{i},1),1);
end
labels = vertcat(cls{:});

% convert labels to categorical labels
labels = categorical(labels, classNames);

end

%--------------------------------------------------------------------------
function dummy = iDummify4dArray(C)

numCategories = numel(categories(C));
[H, W, ~, numObservations] = size(C);
dummifiedSize = [H, W, numCategories, numObservations];
dummy = zeros(dummifiedSize, 'single');
C = iMakeVertical( C );

[X,Y,Z] = meshgrid(1:W, 1:H, 1:numObservations);

X = iMakeVertical(X);
Y = iMakeVertical(Y);
Z = iMakeVertical(Z);

% Remove missing labels. These are pixels we should ignore during
% training. The dummified output is all zeros along the 3rd dims and are
% ignored during the loss computation.
[C, removed] = rmmissing(C);
X(removed) = [];
Y(removed) = [];
Z(removed) = [];

idx = sub2ind(dummifiedSize, Y(:), X(:), int32(C), Z(:));
dummy(idx) = 1;
end

function vec = iMakeHorizontal( vec )
vec = reshape( vec, 1, numel( vec ) );
end

function vec = iMakeVertical( vec )
vec = reshape( vec, numel( vec ), 1 );
end
function labels = iFillLabelNames(classCategoricals, labelsInPrecision)
% Label names are cast to precision, we need to convert them to categorical
% that represent the class names in class categorical.
labelsInPrecision  = gather(labelsInPrecision);
classesInPrecision = cast(classCategoricals, 'like', labelsInPrecision);
labelsInCat        = categorical(labelsInPrecision, classesInPrecision);
labels             = renamecats(labelsInCat, categories(classCategoricals));
end

%--------------------------------------------------------------------------
function out = iSetupProposalParams(in)
% Setup parameters for generating RPN regression and classification responses
out.AnchorBoxes          = in.AnchorBoxes;
out.NumAnchors           = in.NumAnchors;
out.ScaleFactor          = in.ScaleFactor;
out.RandomSelector       = in.RandomSelector;
out.PositiveOverlapRange = in.PositiveOverlapRange;
out.NegativeOverlapRange = in.NegativeOverlapRange;
% selectTrainingSamples expects a varargin of ground truth boxes by default,
% this tells the function to treat that ground truth boxes are already vertcat
% to M-by-4 matrix.
out.GroundTruthMby4      = true;
end

% LocalWords:  RPN bbox rpn datastores Dummify ty tw th dummified
