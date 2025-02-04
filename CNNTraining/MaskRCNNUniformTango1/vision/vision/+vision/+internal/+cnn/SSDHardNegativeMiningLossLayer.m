classdef SSDHardNegativeMiningLossLayer < nnet.layer.ClassificationLayer
    % SSDHardNegativeMiningLossLayer
    % A classification loss layer that incorporate data imbalance handling
    % using hard negative mining based under sampling technique. It incorporate
    % Binary cross entropy for sampled data.
    % Reference:
    % Liu, W., Anguelov, D., Erhan, D., Szegedy, C., Reed, S., Fu, C. Y., & Berg, 
    % A. C. (2016, October). Ssd: Single shot multi-box detector. In European 
    % conference on computer vision (pp. 21-37). Springer, Cham.

    %   Copyright 2021-2022 The MathWorks, Inc.

    methods
        function layer = SSDHardNegativeMiningLossLayer(name)
            % Set layer name.
            layer.Name = name;

            % Set layer description.
            layer.Description = 'SSD Binary Cross Entropy Hard Negative Mining Layer';
        end

        function loss = forwardLoss(~,Y,T)
            % forwardLoss    Return the binary cross entropy loss between
            % estimate and true responses averaged by the number of positive 
            % observations
            % Syntax:
            %   loss = layer.forwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network H-by-W-by-numClasses-by-numObs
            %       or H-by-W-by-D-by-numClasses-by-numObs
            %   T   Targets (actual values), H-by-W-by-numClasses+1-by-numObs
            %       or H-by-W-by-D-by-numClasses+1-by-numObs

            [~,~,nclasses,~] = size(Y);
            positiveFlag = logical(T(:,:,nclasses+1,:));
            T = T(:,:,1:nclasses,:);
            negativeFlag = ~positiveFlag;


            % Observations are encoded in T as non-zero values.
            positiveObservations = sum(squeeze(positiveFlag));
            numHardNegatives = positiveObservations.*3;
            numHardNegatives(numHardNegatives > size(positiveFlag(:,1,1,1),1)) = size(positiveFlag(:,1,1,1),1);



            if sum(positiveObservations) > 0

                P = nnet.internal.cnn.util.boundAwayFromZero(Y);
                oneMinusP = nnet.internal.cnn.util.boundAwayFromZero(1-Y);

                % Binary cross-entropy.
                loss = sum(-1.*((T.*log(P))+(1-T).*log(oneMinusP)),3);
                % In hard negative mining we are not using all the negative examples, we sort
                % them using the highest confidence loss for each default box and pick the
                % top ones so that the ratio between the negatives and positives is at most 3:1.
                loss = loss.*negativeFlag;
                [~, negative_order ]= sort(loss, 'descend');
                for i = 1:size(numHardNegatives,2)
                    x = negative_order(1:numHardNegatives(i),:,:,i);
                    positiveFlag(x,:,:,i) = true;
                end
                loss = loss.*positiveFlag;
                loss = (1/sum(positiveObservations)) * sum(loss(:));
            else
                loss = zeros(1, 'like', Y);
            end
        end

        function dLdY = backwardLoss(~, Y,T)
            % backwardLoss    Back propagate the derivative of the loss
            % function
            %
            % Syntax:
            %   dX = layer.backwardLoss( Y, T );
            %
            % Inputs:
            %   Y   Predictions made by network, H-by-W-by-numClasses-by-numObs
            %       or H-by-W-by-D-by-numClasses-by-numObs
            %   T   Targets (actual values), H-by-W-by-numClasses-by-numObs
            %       or H-by-W-by-D-by-numClasses-by-numObs

 
            [~,~,nclasses,~] = size(Y);
            positiveFlag = logical(T(:,:,nclasses+1,:));
            T = T(:,:,1:nclasses,:);
            negativeFlag = ~positiveFlag;
            % Observations are encoded in T as non-zero values.
            positiveObservations = sum(squeeze(positiveFlag));
            numHardNegatives = positiveObservations.*3;
            numHardNegatives(numHardNegatives > size(positiveFlag(:,1,1,1),1)) = size(positiveFlag(:,1,1,1),1);

            if sum(positiveObservations) > 0

                P = nnet.internal.cnn.util.boundAwayFromZero(Y);
                oneMinusP = nnet.internal.cnn.util.boundAwayFromZero(1-Y);

                Z = sum(-1.*((T.*log(P))+(1-T).*log(oneMinusP)),3);
                loss = Z.*negativeFlag;
                [~, negative_order ]= sort(loss, 'descend');
                for i = 1:size(numHardNegatives,2)
                    if numHardNegatives(i)~=0
                    x = negative_order(1:numHardNegatives(i),:,:,i);
                    positiveFlag(x,:,:,i) = true;
                    end
                end

                positiveFlagRepeat = repmat(positiveFlag,1,1,nclasses,1);

                dLdY = -1.*T./(P) + (1-T)./(oneMinusP);

                dLdY = dLdY.*positiveFlagRepeat;
                dLdY = dLdY./ sum(positiveObservations);

            else
                dLdY = zeros(size(Y), 'like', Y);
            end

        end

    end
end


