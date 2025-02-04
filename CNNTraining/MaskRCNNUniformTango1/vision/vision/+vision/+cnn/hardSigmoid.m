function Y = hardSigmoid(X, slope, offset)
%hardSigmoid Apply hard sigmoid activation.
%
% Y = hardSigmoid(X) computes the hard sigmoid activation on the given input X. 
% Use this function to create a hardSigmoid layer using functionLayer.
% 
% The hard sigmoid operation is defined by
%    Y = max(0, min(1, slope * X + offset)) 
%
% Example: Create a hard sigmoid layer.
% -----------------------------
% fun = @vision.cnn.hardSigmoid;
% name = "hard sigmoid layer";
% hardSigmoidLayer = functionLayer(fun, "Name", name);
%
% See also vision.cnn.mish
% Copyright 2024 The MathWorks, Inc.
%#codegen

if nargin < 2 || isempty(slope)
    slope = 0.166667; % Default value for slope
end

if nargin < 3 || isempty(offset)
    offset = 1/2; % Default value for offset
end

Y = max(0, min(1, slope * X + offset));

end