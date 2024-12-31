function Y = mish(X)
%mish Apply mish activation.
%
% Y = mish(X) computes the mish activation on the given input X. Use this 
% function to create a mish layer using functionLayer.
% 
% The mish operation is defined by
%     Y = X * tanh(log(1+exp(X))), 
%
% Example: Create a mish layer.
% -----------------------------
% fun = @vision.cnn.mish;
% name = "mish layer";
% mishLayer = functionLayer(fun, "Name", name);
%
% See also vision.cnn.slice

% Copyright 2021 The MathWorks, Inc.

%#codegen

Z1 = max(X,0) + log(1 + exp(-abs(X)));
Y = X.*tanh(Z1);
end