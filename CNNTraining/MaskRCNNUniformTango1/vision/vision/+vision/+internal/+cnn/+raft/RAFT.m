classdef RAFT < handle
    % Internal class for the RAFT dlnetworks.
    % This is used to instantiate the Raft property of the public class
    % opticalFlowRAFT.

    % Copyright 2021-2024 The MathWorks, Inc.

    properties (SetAccess = private)
        FeatureNet % feature network
        ContextNet % context network
        FlowUpdateNet % decoder network
    end

    % Network properties
    % The following properties are tied to the loaded update network, the
    % feature dimension to be exact. We will need to train a new network if
    % they are changed.
    properties (Access = private, Constant)
        % Correlation radius and levels for update steps
        CorrelationRadius = 4
        CorrelationLevels = 4
        % Dimension split of the decoder output for hidden unit and context
        HiddenDim = 128
        ContextDim = 128
    end

    % Memory properties
    properties (Access = {?opticalFlowRAFT, ?matlab.unittest.TestCase})
        InitialFlow = []
    end

    methods
        % Constructor
        function obj = RAFT()
            % Load imported network
            data = load('dlnetRAFTSubNets.mat','dlnetFeature','dlnetContext','dlnetFlowUpdate');
            obj.FeatureNet = data.dlnetFeature;
            obj.ContextNet = data.dlnetContext;
            obj.FlowUpdateNet = data.dlnetFlowUpdate;
        end

        % Predict method
        function opticalFlows = estimateFlow(obj, image1, image2, maxIterations, iterTolerance, executionEnvironment, usePreviousFlow, acceleration)
            % This estimates the optical flow from image1 to image2
            % - Images should be RGB format
            % - numIterations is the number of refinement iterations to the
            %   estimated optical flow from an initial prediction. Smaller 
            %   values are faster but less accurate. Typical value is 12.
            % - UsePreviousFlow is a logical flag to control whether using 
            %   the previous flow as the initial guess for the current 
            %   flow. It does not affect the first frame. Default is false.
            % - Acceleration sets the dlnetwork Acceleration NV. Default is
            %   "auto". Can be one of "auto", "mex" or "none".

            if strcmpi(executionEnvironment, "auto")
                if(canUseGPU)
                    executionEnvironment = "gpu";
                else
                    executionEnvironment = "cpu";
                end
            end

            % Normalize and format image to single, [0, 1], dlarray
            image1 = preprocessImg(image1,executionEnvironment);
            image2 = preprocessImg(image2,executionEnvironment);

            % Extract feature maps from both image1 and image2
            accelerationF = acceleration;
            if strcmp(accelerationF, "mex")
                % g3237044: workaround since instancenorm in feature net 
                % does not support mex
                accelerationF = "auto"; 
            end
            fmap1 = predict(obj.FeatureNet,image1,Acceleration=accelerationF);
            fmap2 = predict(obj.FeatureNet,image2,Acceleration=accelerationF);

            % Build correlation maps
            correlationSampler = vision.internal.cnn.raft.MutliScalePairwise4DCorr(fmap1, fmap2, ...
                obj.CorrelationLevels, obj.CorrelationRadius);

            % Extract context map from image1 only
            cmap = predict(obj.ContextNet,image1,Acceleration=acceleration);

            % Split context map into two parts for hidden unit and context
            [hiddenUnit, context] = splitContextMap(cmap,[obj.HiddenDim, obj.ContextDim]);

            % Get coordinates for all pixels in image1 and the initial
            % guess of their corresponding pixel positions in image2
            [coords1, coords2] = initializeFlowCoordinates(image1);

            % Add initial flow if there is one
            if ~isempty(obj.InitialFlow) && usePreviousFlow
                coords2 = coords2 + obj.InitialFlow;
            end

            % Iteratively update the flow
            for i = 1:maxIterations
                % Get correlation on the corresponding pixels in frame 2
                corr = correlationSampler.sample(coords2);

                % Compute flow
                flow = coords2 - coords1;

                % Get delta flow from the update net
                % - hiddenUnit: the hidden unit output of this recurrent net [H/8,W/8,HiddenDim,B]
                % - context: the context net output [H/8,W/8,ContextDim,B]
                % - upMask: upsampling mask [H/8,W/8,8*8*9 = 576,B] to
                %   restore the original resolution [x8,x8] using the neighboring
                %   pixels [3x3=9].
                % - deltaFlow: delta flow to update the current guess [H/8,W/8,2,B]
                [hiddenUnit, upMask, deltaFlow] = predict(obj.FlowUpdateNet,...
                    hiddenUnit,context,corr,flow, ...
                    Acceleration=acceleration);

                % Update corresponding pixels in frame 2
                coords2 = coords2 + deltaFlow;
                
                % If iterative updates differ by less than the tolerance
                % threshold, then stop further refinement steps
                if (mean(abs(deltaFlow(:))) < iterTolerance)
                    break;
                end
            end

            coarseFlow = coords2 - coords1;
            % Save coarseFlow for next initial guess
            if usePreviousFlow
                obj.InitialFlow = coarseFlow;
            end

            % Upsample the flow to full resolution
            flow = vision.internal.cnn.raft.upsampleFlow(coarseFlow, upMask);% [HxWx2xB]
            
            % If flow is gpuArray, transfer to local workspace
            if(isgpuarray(flow))
                flow = gather(flow);
            end
            
            % Convert to a column of opticalFlow object
            opticalFlows = flow2opticalFlow(flow);
        end
    end
end

% -------------------------------------------------------------------------
% Helper functions
% -------------------------------------------------------------------------
function image = preprocessImg(image,env)
    % This normalizes the image from uint8 [0,255] to single [0,1], puts it 
    % into a gpuArray, and formats it as dlarray.
    if (isa(gather(image),'uint8') || isa(gather(image),'int16'))
        image = single(image)/255;
    end
    if isa(gather(image),'double')
        image = single(image);
    end
    if strcmpi(env,"gpu")
        %GPU
        image = gpuArray(image);
    else
        %Host
        if(isgpuarray(image))
            image = gather(image);
        end
    end
    image = dlarray(image,"SSCB");
end

function [hiddenUnit, context] = splitContextMap(cmap,splitLength)
    % This splits the context map into two outputs along the 3rd dim using
    % split length
    hiddenUnit = cmap(:,:,1:splitLength(1),:);
    context = cmap(:,:,splitLength(1)+(1:splitLength(2)),:);
    % Perform different activation for the split outputs
    hiddenUnit = tanh(hiddenUnit);
    context = relu(context);
end

function [coords1, coords2] = initializeFlowCoordinates(image)
    % This creates two coordinates for flow starting and ending pixels.
    % - coords1 is the pixel coordinates in image1.
    % - coords2 is the corresponding pixel coordinates in image2.
    % - optical flow = coords2 - coords1
    [H, W, ~, B] = size(image);

    % Coordinates are computed based on the feature maps [H/8,W/8]
    H8 = ceil(H/8);
    W8 = ceil(W/8);

    coords1 = coordsGrid(B, H8, W8);
    coords2 = coords1; % initial guess of flow is zero.
end

function coords = coordsGrid(batchSize, h, w)
    % This creates a grid of coordinates based on the feature map size.
    % The coordinates are 0-based, the size is [b, h, w, 2]
    % The last dimension is for [width, height]

    [X, Y] = meshgrid(0:h-1, 0:w-1);
    coords = cat(3, Y, X);
    coords = single(coords);

    coords = repmat(coords, [1, 1, 1, batchSize]); % can be removed for batch size = 1
    coords = permute(coords, [2, 1, 3 ,4]);%[h,w,2(vx,vy),b]
end

function opticalFlows = flow2opticalFlow(flow)
    % This converts flow [HW2B] to opticalFlow object.
    B = size(flow,4);
    flowCell = cell(B,1);
    for b = 1:B
        flowCell{b} = opticalFlow(flow(:,:,1,b),flow(:,:,2,b));
    end
    opticalFlows = [flowCell{:}]';% Use column
end

