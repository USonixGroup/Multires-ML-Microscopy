classdef FastRCNNCatAndSliceStrategy < handle
    % FastRCNNCatAndSliceStrategy A strategy for concatenating and slicing
    %    batches of inputs and responses, along the observation dimensions.
    %

    %   Copyright 2020 The MathWorks, Inc.

    properties
        Ordering
    end

    methods
        % catBatch Cat function to be used for concatenating two batches of
        % inputs and responses along the observation dimension.
        % For example,
        %   data.start();
        %   [X1,response1] = data.next();
        %   [X2,response2] = data.next();
        %   [X,response] = data.CatAndSliceStrategy.catBatch(XDim,responseDim,X1,X2,response1,response2);
        function [X, response] = catBatch(this, xDim, responseDim, X, leftOverX, response, leftOverResponse)
            X = iCatInputsFcn(xDim, X, leftOverX, this.Ordering.InputOrdering);
            response = iCatResponseFcn(responseDim, response, leftOverResponse, this.Ordering.OutputOrdering);
        end


        % sliceBatch Slice function to be used for slicing a batch of input
        % and responses along the observation dimension.
        % For example,
        %   data.start();
        %   [X1,response1] = data.next();
        %   [X2,response2] = data.next();
        %   [X,response] = data.CatAndSliceStrategy.catBatch(XDim,responseDim,X1,X2,response1,response2);
        %   sliceIndices = [4,1,3];
        %   [slicedX,slicedResponse] = data.CatAndSliceStrategy.sliceBatch(...
        %                        XDim,responseDim,sliceIndices,X,response);
        function [slicedX, slicedResponse] = sliceBatch(this,~, ~, trimdices, X, response)
            [slicedX,indices] = iSliceInputsFcn(X, trimdices, this.Ordering.InputOrdering);
            slicedResponse    = iSliceResponseFcn(response, indices, this.Ordering.OutputOrdering);
        end

    end

    methods (Static, Hidden)
        function batch = regressionResponseBatch(TColumn)
            % function to batch a column of regression response data.

            t1 = cellfun(@(x)x{1},TColumn,'UniformOutput',false);
            t2 = cellfun(@(x)x{2},TColumn,'UniformOutput',false);

            batch = { cat(4,t1{:}) cat(4,t2{:}) };
        end

        function batchROI = roiBatch(roiCells)
            % Function to batch ROIs
            for ii = 1:numel(roiCells)
                % Associate idx with each ROI.
                roiCells{ii}(:,end+1) = ii;
            end
            batchROI = vertcat(roiCells{:});
        end

    end
end

%--------------------------------------------------------------------------
function [slicedX,roiSelection] = iSliceInputsFcn(X, trimdices, ordering)
    if isempty(X)
        slicedX = X;
        roiSelection = [];
        return;
    end
    imageBatch = X{ordering(1)};
    roiBatch = X{ordering(2)};
    roiSelection = ismember(roiBatch(:,5),trimdices);
    roiBatch = roiBatch(roiSelection,:);
    [~,~,roiBatchIdx] = unique(roiBatch(:,5),'stable');
    roiBatch(:,5) = roiBatchIdx;

    slicedX{ordering(1)} = imageBatch(:,:,:,trimdices);
    slicedX{ordering(2)} = roiBatch;
end

%--------------------------------------------------------------------------
function response = iSliceResponseFcn(response, indices, ordering)
    if isempty(response)
        return;
    end
    response{ordering(1)} = response{ordering(1)}(:,:,:,indices);
    response{ordering(2)}{1} = response{ordering(2)}{1}(:,:,:,indices);
    response{ordering(2)}{2} = response{ordering(2)}{2}(:,:,:,indices);
end

%--------------------------------------------------------------------------
function X = iCatInputsFcn(~, X, leftoverObservations, ordering)
    if isempty(X) && isempty(leftoverObservations)
        X = {};
        return;
    elseif isempty(X)
        X = leftoverObservations;
        return;
    elseif isempty(leftoverObservations)
        %% No -op
        return;
    end
    imageBatch = X{ordering(1)};
    roiBatch = X{ordering(2)};
    imageLeftOver = leftoverObservations{ordering(1)};
    roiLeftOver = leftoverObservations{ordering(2)};
    X{ordering(1)} = cat(4,imageBatch,imageLeftOver);
    X{ordering(2)} = iROICatFcn(roiBatch,roiLeftOver);
end

%--------------------------------------------------------------------------
function batchROI = iROICatFcn(roiBatch, roiLeftOver)
    [~,~,roiBatchIdx]=unique(roiBatch(:,5),'stable');
    [~,~,roiLeftOverIdx]=unique(roiLeftOver(:,5),'stable');
    roiBatch(:,5) = roiBatchIdx;
    roiLeftOver(:,5) = roiLeftOverIdx + roiBatchIdx(end);
    batchROI = vertcat(roiBatch,roiLeftOver);
end

%--------------------------------------------------------------------------
function response = iCatResponseFcn(~, response, extras, ordering)
    if isempty(response) && isempty(extras)
        response = {};
        return;
    elseif isempty(response)
        response = extras;
        return;
    elseif isempty(extras)
        %% No -op
        return;
    end
    responseOut = cell(1,2);

    % Cat classification responses batches.
    responseOut{ordering(1)} = cat(4,response{:,ordering(1)}, extras{:,ordering(1)});


    import vision.internal.cnn.catAndSliceStrategy.FastRCNNCatAndSliceStrategy;
    % Cat regression response batches.
    regResponse = response(:,ordering(2));
    regResponseExtras = extras(:,ordering(2));
    responseOut{ordering(2)} = FastRCNNCatAndSliceStrategy.regressionResponseBatch(vertcat(regResponse, regResponseExtras));

    response = responseOut;
end
