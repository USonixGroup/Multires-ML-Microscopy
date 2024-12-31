function data = computeChannels( I, params )%#codegen
% Compute channel features at a single scale given an input image.
%
% Compute the channel features as described in [1].
%
% Currently, three channel types are available by default
%  (1) color channels (computed using rgbConvert.m)
%  (2) gradient magnitude (computed using gradient.m)
%  (3) quantized gradient channels (computed using gradientHist.m)
%
% References
% ----------
% [1] P. Dollar, Z. Tu, P. Perona and S. Belongie
%  "Integral Channel Features", BMVC 2009.

% This code is a modified version of that found in:
%
% Piotr's Computer Vision Matlab Toolbox      Version 3.23
% Copyright 2014 Piotr Dollar & Ron Appel.  [pdollar-at-gmail.com]
% Licensed under the Simplified BSD License [see pdollar_toolbox.rights]

% Crop I so divisible by shrink and get target dimensions
shrink = params.Shrink;

[h, w, ~] = size(I);
cr = mod([h w], shrink);

if(any(cr))
    h = h - cr(1);
    w = w - cr(2);
    I = I(1:h, 1:w, :);
end

h = h / shrink;
w = w / shrink;

if isempty(coder.target)
    I = vision.internal.acf.convTri(I, params.PreSmoothColor);
else
    I = vision.internal.buildable.acfObjectDetectorBuildable.convTri(I, params.PreSmoothColor );
end

% Compute gradient channel
if isempty(coder.target)
    [M, O] = vision.internal.acf.gradient(I, params.gradient);
else
    [M,O] = vision.internal.buildable.acfObjectDetectorBuildable.gradient(I, params.gradient);
end


% resize I and gradient magnitude at same time for efficiency.
if isempty(coder.target)
    data = visionACFResize(cat(3, I, M), h, w, 1);
else
    data = vision.internal.buildable.acfObjectDetectorBuildable.resize(cat(3, I, M), int32(h), int32(w), single(1));
end

% Compute HOG channels
if isempty(coder.target)
    H = vision.internal.acf.gradientHist(M, O, params.hog);
else
    H = vision.internal.buildable.acfObjectDetectorBuildable.gradientHist(M, O, params.hog);
end

% Shrink data
[h1, w1, ~] = size(H);

if(h1 ~= h || w1 ~= w)
    if isempty(coder.target)
        H = visionACFResize(H, h, w, 1);
    else
        H = vision.internal.buildable.acfObjectDetectorBuildable.resize(H, int32(h), int32(w), single(1));
    end
end

data = cat(3, data, H);