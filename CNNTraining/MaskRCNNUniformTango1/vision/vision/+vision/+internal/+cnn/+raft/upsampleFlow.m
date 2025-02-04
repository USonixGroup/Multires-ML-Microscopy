function upFlow = upsampleFlow(flow, mask)
    % Upsample the predicted flow back to the original image resolution.
    % - flow size is [h, w, 2, b]
    % - mask size is [h, w, 8*8*9 = 576, b]
    % - For each pixel, the mask uses the neighboring 3*3 pixels to
    % upsample it into 8*8 pixels for full resolution.

%   Copyright 2024 The MathWorks, Inc.

    flow = extractdata(flow); % padarray doesn't support dlarray g3184771
    [H, W, ~, B] = size(flow);
    % reshape mask to separate the 576 into 8*8*9
    % mask = reshape(mask, [B, 1, 8, 8, 9, H, W]);
    mask = reshape(mask, [H, W, 8, 8, 9, B]);
    
    % permute the dimensions to match python reshaping preference
    % order. [8(3),8(4),9(5)] -> [9(5),8(4),8(3)]
    % mask = permute(mask,[1 2 5 4 3 6 7]);
    mask = permute(mask,[6 7 5 4 3 1 2]);
    
    % perform softmax along the dimension with length 9 for
    % neighboring pixels.
    mask = softmaxWithDim(mask, 3);

    % unfold flow using a [3,3] kernel to prepare it for the mask
    unFoldedFlow = unfoldFlow3x3(8 * flow); % x8 for original resolution

    % apply the mask to flow
    % - mask is   [B 1        9(3x3 neighbor) 8(upsample) 8(upsample) H W]
    % - unfold is [B 2(vx,vy) 9(3x3 neighbor) 1           1           H W]
    upFlow = sum(mask .* unFoldedFlow, 3);

    % reshape it back to the full resolution
    upFlow = permute(upFlow, [4, 6, 5, 7, 3, 2, 1]);
    upFlow = reshape(upFlow, [8 * H, 8 * W, 2, B]); % HW2B
    
    upFlow = extractdata(upFlow); % from dlarray
end

function X = softmaxWithDim(X,dim)
    % This performs softmax along dimension = dim
    X = exp(X);
    X = X./sum(X,dim);
end

function unFoldedFlow = unfoldFlow3x3(flow)
    % This unfolds the flow into columns using [3,3] kernel size.
    [H,W,~,B] = size(flow);
    
    U = zeros(B,2,9,W*H);
    for b = 1:B
        % transpose the flow from h,w to w,h to match mask order
        U(b,1,:,:) = unfold2D3x3(flow(:,:,1,b)');
        U(b,2,:,:) = unfold2D3x3(flow(:,:,2,b)');
    end

    % reshape and permute to match python reshape preference order
    unFoldedFlow = reshape(U, [B 2 9 1 1 W H]); % W H is transposed during unfold to match mask order
    unFoldedFlow = permute(unFoldedFlow, [1 2 3 4 5 7 6]);
end

function U = unfold2D3x3(X)
    % This unfolds X into columns using [3,3] kernel size.
    % pad array so the output has the same size
    Xpadded = padarray(X,[1 1]);
    U = im2col(Xpadded,[3 3]);
end
