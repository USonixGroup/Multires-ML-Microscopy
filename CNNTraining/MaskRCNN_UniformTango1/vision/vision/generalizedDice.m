%GENERALIZEDDICE Generalized dice similarity metric for semantic segmentation.
%
%   The generalized dice similarity is based on the Sorensen-Dice
%   similarity coefficient for measuring overlap between two segmented
%   images or voxels. Generalized dice similarity controls the contribution
%   that each class makes to the similarity by weighting classes by the
%   inverse size of the expected region. When working with imbalanced
%   datasets, this helps prevent the more prevalent classes from dominating
%   the similarity metric.
%
%   S = GENERALIZEDDICE(X,TARGETS) computes the generalized dice similarity
%   between numeric inputs X and TARGETS. The inputs X and TARGETS can be a
%   multi-dimensional array, where the last dimension represents the
%   classes. Output S is a scalar numeric output with generalized dice
%   metric in the range [0,1].
%
%   S = GENERALIZEDDICE(dlX,dlTARGETS) computes the generalized dice
%   similarity between the dlarray inputs dlX and dlTARGETS. The inputs dlX
%   and dlTARGETS can be formatted or unformatted dlarray objects. The size
%   of dlX and dlTARGETS must be the same. For unformatted dlarray inputs
%   the dimension format must be specified using the DataFormat Name-Value
%   pair (see below). If dlX is a formatted dlarray, its dimension format
%   must match the format of dlTARGETS. Valid format options are 'S', 'C',
%   and 'B', which correspond to spatial, channel, and batch dimensions
%   respectively. Formatted dlarray inputs must contain a channel dimension
%   e.g., "SSSC". Formatted dlarray inputs can contain a batch dimension
%   e.g., "SSCB" or "SSSCB", where C represents the channel dimension and B
%   represents the batch dimension, then the output S is a dlarray with
%   generalized dice metric as each element in the range [0,1] along the
%   batch dimension.
%
%   S = GENERALIZEDDICE(...,'DataFormat',dataFormat) specifies dimension
%   labels for the input data dlX and dlTARGETS, specified as a char array
%   or string containing characters 'S', 'C', or 'B', which correspond to
%   spatial, channel, or batch dimensions respectively. You must specify
%   this argument for unformatted dlarray inputs. The specified dimension
%   labels must contain a channel dimension e.g., 'SSC'.
%
%   Example 1: Generalized dice similarity for semantic image segmentation
%   ----------------------------------------------------------------------
%
%   % Load pretrained network.
%   data = load('triangleSegmentationNetwork');
%   net = data.net;
%
%   % Load images using imageDatastore.
%   dataDir = fullfile(toolboxdir('vision'),'visiondata','triangleImages');
%   testImageDir = fullfile(dataDir,'testImages');
%   imds = imageDatastore(testImageDir)
%
%   % Load ground truth labels.
%   labelDir = fullfile(dataDir,'testLabels');
%   classNames = ["triangle" "background"];
%   pixelLabelID = [255 0];
%   pxdsTruth = pixelLabelDatastore(labelDir,classNames,pixelLabelID);
%
%   % Read a sample image and the corresponding ground truth labels.
%   I = readimage(imds, 1);
%   gTruthLabels = readimage(pxdsTruth,1);
%
%   % Run semantic segmentation on the image.
%   [predictions,scores] = semanticseg(I,net);
%
%   % Use onehotencode to encode the categorical predictions and targets.
%   featureDim = ndims(predictions) + 1;
%   encodedPredictions = onehotencode(predictions,featureDim);
%   encodedGroundTruthLabels = onehotencode(gTruthLabels,featureDim);
%
%   % Ignore any undefined classes in the onehotencoded data.
%   encodedPredictions(isnan(encodedPredictions)) = 0;
%   encodedGroundTruthLabels(isnan(encodedGroundTruthLabels)) = 0;
%
%   % Compute generalized dice metric between the segmented image and the
%   % ground truth.
%   gDice = generalizedDice(encodedPredictions,encodedGroundTruthLabels)
%
%   Example 2: Generalized dice loss for semantic segmentation
%   ----------------------------------------------------------
%   % Create input data as a formatted dlarray containing 32
%   % observations with unnormalized scores for ten output categories.
%   spatial = 10;
%   numCategories = 10;
%   batchSize = 32;
%   X = dlarray(rand(spatial,numCategories,batchSize),'SCB');
%
%   % Convert unnormalized scores to probabilities of membership of
%   % each of the ten categories.
%   X = sigmoid(X);
%
%   % Create target values as numeric array for membership in the
%   % second and sixth category.
%   targets = zeros(spatial, numCategories,batchSize);
%   targets(:,2,:) = 1; targets(:,6,:) = 1;
%   targets = dlarray(targets, 'SCB');
%
%   % Compute generlized dice similarity between probability vectors X and
%   % targets for multi-label classification.
%   Z = generalizedDice(X,targets);
%
%   % Compute generalized dice loss.
%   loss = 1 - mean(Z,'all')
%
%   See also dice, onehotencode, dicePixelClassificationLayer, semanticseg,
%            dlarray.

%   Copyright 2020-2021 Mathworks, Inc.

function dice = generalizedDice(X,T,NameValue)
    arguments
        X {mustBeA(X,["numeric","dlarray","gpuArray"]),...
            mustBeNonsparse,mustBeReal,mustBeNonempty}
        T {mustBeA(T,["numeric","dlarray","gpuArray"]),...
            mustBeNonsparse,mustBeReal,mustBeNonempty,iValidateXAndTargets(X,T)}
        NameValue.DataFormat {mustBeNonempty,mustBeTextScalar, iValidateDataFormat(NameValue.DataFormat)}
    end

    dataFormatSpecified = isfield(NameValue, 'DataFormat');
    if dataFormatSpecified
        dataFormat = NameValue.DataFormat;
    else
        dataFormat = [];
    end
    [X,perm] = iValidateDataFormatWithInput(X, dataFormat, 1);
    T = iValidateDataFormatWithInput(T, dataFormat, 2);

    if isdlarray(X)
        if ~isequal(dims(X), dims(T))
            error(message('vision:generalizedDice:InputsMustHaveSameDimension'));
        end

        % dlarray input
        channelDim = finddim(X,'C');
    else
        % numeric input
        channelDim = ndims(X);
    end
    dice = iCalcGeneralizedDice(X,T,channelDim);

    if dataFormatSpecified
        dice = ipermute(stripdims(dice),perm);
    end
end

%--------------------------------------------------------------------------
function dice = iCalcGeneralizedDice(X,T,channelDim)
    spatialDimVec = 1:channelDim-1;

    % Ensure that automatic differentiation returns real-only derivatives
    X = real(X);
    T = real(T);
    
    % Weights by inverse of region size.
    regionSize = sum(T, spatialDimVec);
    epsilon = eps(underlyingType(regionSize));
    W = 1 ./ max(epsilon,regionSize.^2);

    intersection = sum(X.*T, spatialDimVec);
    union = sum(X.^2 + T.^2, spatialDimVec);

    numer = 2*sum(W.*intersection,channelDim) + epsilon;
    denom = sum(W.*union,channelDim) + epsilon;

    % Compute Dice score
    dice = numer./denom;
    
    % Ensure that automatic differentiation returns real-only derivatives
    dice = real(dice);
end

%--------------------------------------------------------------------------
function iValidateXAndTargets(X,Targets)
    if ~isequal(size(X), size(Targets))
        error(message('vision:generalizedDice:InputsMustBeEqualSize'));
    end
    if islogical(X) || islogical(Targets)
        error(message('deep:dlarray:LogicalsNotSupported'));
    end
    if ~isequal(class(X), class(Targets))
        error(message('vision:generalizedDice:InputsMustBeSameClass'));
    end
    if ~isequal(underlyingType(X), underlyingType(Targets))
        error(message('vision:generalizedDice:InputsMustBeSameClass'));
    end
end

%--------------------------------------------------------------------------
function iValidateDataFormat(dataFormat)

    if ~isempty(dataFormat)
        dataFormat = convertStringsToChars(dataFormat);
        if ~ischar(dataFormat) || ~isrow(dataFormat) || ~all(ismember(dataFormat,'SCB'))
            error(message('vision:generalizedDice:InvalidDataFormatCharacters'));
        end
        if ~any(ismember(dataFormat, 'C'))
            error(message('vision:generalizedDice:DataFormatNeedsChannel'));
        end
        if ~any(ismember(dataFormat, 'S'))
            error(message('vision:generalizedDice:DataFormatNeedsSpatial'));
        end
    end

end

%--------------------------------------------------------------------------
function [input, perm] = iValidateDataFormatWithInput(input,dataFormat,inputArgNumber)
%IVALIDATEDATAFORMATWITHINPUT Check validity of DataFormat with the input argument.
%
% [Y, PERM] = iValidateDataFormatWithInput(X,FMT,INPUTARGNUM) checks the validity
% of FMT against the input data X. If X is an unformatted dlarray or a numeric
% array, the number of labels in FMT must match the number of dimensions of
% X. In this case, output Y is a dlarray with format FMT. The function can
% also return the permutation vector PERM applied to input X when FMT is
% applied to construct the formatted dlarray Y. If X is a formatted
% dlarray, FMT must be empty and X must not contain logicals. In this case,
% output Y equals X and PERM is empty.

    if ~isdlarray(input)
        if ~isempty(dataFormat) % Cannot have a DataFormat if input is not dlarray
            error(message('vision:generalizedDice:NonDlarrayInputCannotHaveDataFormat', inputArgNumber))
        end
        perm = [];
        return;
    end

    if isempty(dims(input)) % Unformatted dlarray input
        if isempty(dataFormat) % Unformatted input need a non-empty DataFormat
            error(message('vision:generalizedDice:UnlabeledInputMustHaveDataFormat', inputArgNumber))
        else
            try
                inputLabeled = dlarray(input, dataFormat);
            catch e
                % Reformat the error message if the DataFormat does not match
                % the number of dimensions of the input
                if e.identifier == "deep:dlarray:MoreDimsThanLabels" || ...
                        e.identifier == "deep:dlarray:OneLabelVectorOnly"
                    error(message('deep:dlarray:InvalidDataFormat', ndims(input)))
                else
                    throw(e)
                end
            end
            if nargout > 1
                perm = iDimsPermutation(input, dataFormat);
            end
            input = inputLabeled;
        end
    else % formatted input
        if ~isempty(dataFormat) % Cannot have a DataFormat if formatted
            error(message('vision:generalizedDice:LabeledInputCannotHaveDataFormat', inputArgNumber))
        end
        perm = [];
    end

    if ~all(ismember(dims(input),'SCB'))
        error(message('vision:generalizedDice:InvalidDlarrayFormat'));
    end
    if ~any(dims(input) == 'C')
        error(message('vision:generalizedDice:DlarrayNeedsChannel'));
    end

    if ~any(dims(input) == 'S')
        error(message('vision:generalizedDice:DlarrayNeedsSpatial'));
    end

end

%--------------------------------------------------------------------------
function perm = iDimsPermutation(input, dims)
% The permutation applied to input based on the dims vector passed into it.
% input is an unformatted dlarray or a numeric array, and dims represents its
% data format. No input checking, this subfunction assumes that
% dlarray(input, dims) has already been called successfully.

    dims = char(dims);

    if isscalar(dims)
        % This is only possible if input is a vector, otherwise dlarray
        % constructor would have errored
        if iscolumn(input)
            perm = [1 2];
        else
            % input has no labels, so ndims is the only non-singleton dimension
            % of input
            dim = ndims(input);
            % Permutation that makes input a column vector
            perm = [dim 1:dim-1];
        end
    else
        % Index w.r.t the order of dimensions array for each dimension label
        [~, dimInd] = ismember(dims, dlarray.OrderOfDims);

        % Sort by index
        [~, perm] = sort(dimInd);
    end
end
