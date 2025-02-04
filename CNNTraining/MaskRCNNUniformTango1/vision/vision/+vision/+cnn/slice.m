function Y = slice(X)
%slice Apply slice operation.
%
% Y = slice(X) divides the number of channels present in the given input X 
% into two groups and forwards the first group to the next layer in the 
% layer graph. Use this function to create a slice layer using functionLayer.
%
% Example: Create a slice layer.
% ------------------------------
% fun = @vision.cnn.slice;
% name = "slice layer";
% sliceLayer = functionLayer(fun, "Name", name);
%
% See also vision.cnn.mish

% Copyright 2021 The MathWorks, Inc.

%#codegen

groups = 2;
group_id = 2;
X = reshape(X,[size(X),1]);
channels = size(X,3);
deltaChannels = channels/groups;
selectStart = (group_id-1)*deltaChannels+1;
selectEnd = group_id*deltaChannels;
Y = X(:,:,selectStart:selectEnd,:);
end