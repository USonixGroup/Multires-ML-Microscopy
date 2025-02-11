classdef YOLOv2CatAndSliceStrategy < handle
    % YOLOv2CatAndSliceStrategy A strategy for concatenating and slicing
    %    batches of inputs and responses, along the observation dimensions.
    %

    %   Copyright 2020 The MathWorks, Inc.

    methods
        % catBatch Cat function to be used for concatenating two batches of
        % inputs and responses along the observation dimension.
        % For example,
        %   data.start();
        %   [X1,response1] = data.next();
        %   [X2,response2] = data.next();
        %   [X,response] = data.CatAndSliceStrategy.catBatch(XDim,responseDim,X1,X2,response1,response2);
        function [X, response] = catBatch(~, xDim, responseDim, X, leftOverX, response, leftOverResponse)
            X = iCatInputsFcn(xDim, X, leftOverX);
            response = iCatResponseFcn(responseDim, response, leftOverResponse);
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
        function [slicedX, slicedResponse] = sliceBatch(~, ~, ~, trimdices, X, response)
            slicedX = iSliceInputsFcn(X, trimdices);
            slicedResponse = iSliceResponseFcn(response, trimdices);
        end

    end
end

%--------------------------------------------------------------------------
function X = iCatInputsFcn(~, X, leftoverObservations)
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
    imageBatch = X{1};
    imageLeftOver = leftoverObservations{1};
    X{1} = cat(4,imageBatch,imageLeftOver);
end

%--------------------------------------------------------------------------
function response = iCatResponseFcn(~, response, extras)
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

    response{1}{1} = vertcat(response{1}{1},extras{1}{1});
end

%--------------------------------------------------------------------------
function X = iSliceInputsFcn(X, trimdices)
    if isempty(X)
        return;
    end
    imageBatch = X{1};
    X{1} = imageBatch(:,:,:,trimdices);
end

%--------------------------------------------------------------------------
function response = iSliceResponseFcn(response, indices)
    if isempty(response)
        return;
    end
    batch = response{1}{1};
    response{1}{1} = batch(indices);
end
