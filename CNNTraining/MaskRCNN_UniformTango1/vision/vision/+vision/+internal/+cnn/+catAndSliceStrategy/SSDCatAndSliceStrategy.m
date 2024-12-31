classdef SSDCatAndSliceStrategy < handle
    % SSDCatAndSliceStrategy A strategy for concatenating and slicing
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
            slicedX = iSliceInputsFcn(X, trimdices, this.Ordering.InputOrdering);
            slicedResponse = iSliceResponseFcn(response, trimdices, this.Ordering.OutputOrdering);
        end

    end
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
    imageLeftOver = leftoverObservations{ordering(1)};
    X{ordering(1)} = cat(4,imageBatch,imageLeftOver);
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
    clsResponse = response(:,ordering(1));
    clsResponseExtras = extras(:,ordering(1));
    responseOut{ordering(1)} = cat(4,clsResponse{:},clsResponseExtras{:});


    % Cat regression response batches.
    regResponse = response(:,ordering(2));
    regResponseExtras = extras(:,ordering(2));
    regResponseIn = vertcat(regResponse{:},regResponseExtras{:});
    regResponse{1} = cat(4,regResponseIn{:,1});
    regResponse{2} = cat(4,regResponseIn{:,2});

    responseOut{ordering(2)} = regResponse;

    response = responseOut;
end

%--------------------------------------------------------------------------
function X = iSliceInputsFcn(X, trimdices, ordering)
    if isempty(X)
        return;
    end
    imageBatch = X{ordering(1)};
    X{ordering(1)} = imageBatch(:,:,:,trimdices);
end

%--------------------------------------------------------------------------
function response = iSliceResponseFcn(response, indices, ordering)
    if isempty(response)
        return;
    end
    % Slice classification responses
    response{ordering(1)} = response{ordering(1)}(:,:,:,indices);
    % Slice regression responses
    response{ordering(2)}{1} = response{ordering(2)}{1}(:,:,:,indices);
    response{ordering(2)}{2} = response{ordering(2)}{2}(:,:,:,indices); 
end
