function pyramid = computePyramid(I, params)%#codegen
% Compute channel feature pyramid given an input image.
%
% computePyramid repeatedly calls computeChannels on different scale
% images to create a scale-space pyramid of channel features
%
% References
% ----------
% [1] P. Dollar, R. Appel, S. Belongie and P. Perona
%   "Fast Feature Pyramids for Object Detection", PAMI 2014.
% [2] P. Dollar, Z. Tu, P. Perona and S. Belongie
%  "Integral Channel Features", BMVC 2009.

% This code is a modified version of that found in:
%
% Piotr's Computer Vision Matlab Toolbox      Version 3.23
% Copyright 2014-2020 Piotr Dollar & Ron Appel.  [pdollar-at-gmail.com]
% Licensed under the Simplified BSD License [see pdollar_toolbox.rights]

nPerOct = params.NumScaleLevels;
nApprox = params.NumApprox;
pad     = params.ChannelPadding;
minDs   = params.ModelSize;
smooth  = params.SmoothChannels;
shrink  = params.Shrink;
nOctUp  = params.NumUpscaledOctaves;
lambdas = params.Lambdas;

imageSize = [size(I,1) size(I,2)];
% I_single = coder.nullcopy(zeros(size(I),'single'));

if isfloat(I)
    I_single = single(mat2gray(I)); % scales floating point values between [0 1].
else
    I_single = im2single(I);
end

if ismatrix(I)
    I_single = im2single(cat(3, I, I, I));
end

% Generate the LUV color channels at the original scale and sample it
% during pyramid construction, faster than generating the color channels at
% each scale separately. Output is single.
if isempty(coder.target)
    I_single = vision.internal.acf.rgb2luv(I_single, true);
else
    I_single = vision.internal.buildable.acfObjectDetectorBuildable.rgb2luv(I_single);
end

% Get scales at which to compute features and list of real/approx scales
[imageScales,scaledImageSize] = getScales(nPerOct, nOctUp, minDs, shrink, imageSize);

% Apply min/max object size constraints
if (~isempty(imageScales) && all(isfield(params, 'MaxSize')) && isfield(params, 'MinSize'))
    
    detectionSize = floor(bsxfun(@times, 1./imageScales, round(minDs)'));
    detectionSize(:,1) = round(minDs); % should be value of round(minDs);
    
    % Find the range of scales between min and max size
    lessThanOrEqToMaxSize    = all(bsxfun(@le, detectionSize, params.MaxSize'));
    greaterThanOrEqToMinSize = all(bsxfun(@ge, detectionSize, params.MinSize'));
    
    minmaxRange = lessThanOrEqToMaxSize & greaterThanOrEqToMinSize;  % check if empty!
    
    if ~any(minmaxRange)
        % min max range falls in between two scales. In this edge case, select
        % the upper and lower scale for processing.
        idx = [find(lessThanOrEqToMaxSize, 1, 'last') ...
            find(greaterThanOrEqToMinSize, 1)];
        if numel(idx) > 1
            minmaxRange(idx) = true;
        end
    end
    
    % Only keep the scales between min and max size
    imageScales   = imageScales(minmaxRange);
    scaledImageSize = scaledImageSize(minmaxRange, :);
end

nScales = length(imageScales);

% The scales which channels are computed exactly
realScaleInd = 1;
realScaleInd = realScaleInd : nApprox + 1 : nScales;

% The scales which channels are approximated
approxScaleInd = 1:nScales;
approxScaleInd(realScaleInd) = [];

% Index denoting which scales are used to approximate
j = [0 floor((realScaleInd(1:end-1) + realScaleInd(2:end)) / 2) nScales];
referenceScales = 1:nScales;

for i = 1:length(realScaleInd)
    referenceScales(j(i) + 1:j(i + 1)) = realScaleInd(i);
end
% Initialising the data to hold features
data = coder.nullcopy(cell(nScales, 1));

% Compute image pyramid [real scales]
for count = 1:nScales
    for realscaleCounter = 1: length(realScaleInd)
        
        if(count == realScaleInd(realscaleCounter))
            i = realScaleInd(realscaleCounter);
            s = imageScales(i);
            imageScaledSize = round(imageSize * s / shrink) * shrink;
            if (all(imageSize == imageScaledSize))
                Is = I_single;
            else
                [X, Y] = generateRemapXY(I, imageScaledSize(2), imageScaledSize(1));
                fillValue = zeros(1, size(I,3),'like',I_single);
                if isempty(coder.target)
                    
                    
                    if images.internal.useIPPLibrary
                        Is = images.internal.builtins.remap(I_single, X, Y, 'bilinear', fillValue);
                    else                       
                        Is = remapUsingInterp2d(I_single, X, Y, 'bilinear', fillValue);                       
                    end
                else
                    arch = coder.const(feval('computer', 'arch'));
                    if strcmpi(arch,'MACA64')                                        
                        Is = remapUsingInterp2d(I_single, X, Y, 'bilinear', fillValue);  
                    else
                        if coder.isColumnMajor
                            Is = images.internal.coder.remapmex(I_single, X, Y, 'bilinear', fillValue);
                        else
                            Is = zeros([size(X,1),size(X,2),3],'like',X);
                            Is(:,:,1) = images.internal.coder.remapmex(I_single(:,:,1), X, Y, 'bilinear', fillValue);
                            Is(:,:,2) = images.internal.coder.remapmex(I_single(:,:,2), X, Y, 'bilinear', fillValue);
                            Is(:,:,3) = images.internal.coder.remapmex(I_single(:,:,3), X, Y, 'bilinear', fillValue);
                        end
                    end
                end
                
            end
            data{count} = coder.nullcopy(zeros(imageScaledSize(1), imageScaledSize(2), 'single'));
            % Generate channels for scaled image Is
            data{count} = vision.internal.acf.computeChannels(Is, params);
            
        end
        
    end
end
%
% Compute image pyramid [approximated scales]
for count = 1:nScales
    for approxScaleCounter = 1:length(approxScaleInd)
        
        if(count == approxScaleInd(approxScaleCounter))
            i = approxScaleInd(approxScaleCounter);
            iR = referenceScales(i);
            imageScaledSize=round(imageSize * imageScales(i) / shrink);
            data{count} = coder.nullcopy(zeros(imageScaledSize(1), imageScaledSize(2), 'single'));
            
            if isempty(coder.target)
                data{count} = visionACFResize(data{iR}, imageScaledSize(1), imageScaledSize(2), 1);
            else
                data{count} = vision.internal.buildable.acfObjectDetectorBuildable.resize(data{iR}, int32(imageScaledSize(1)), int32(imageScaledSize(2)), 1);
            end
            
            ratio = (imageScales(i)/imageScales(iR)).^-lambdas;
            
            scaling = reshape(repelem(ratio, [3 1 params.hog.NumBins]), 1, 1, []);
            sz = size(data{count});
            scalingFactor = repmat(scaling,[sz(1) sz(2)]);
            data{count} = data{count}.*scalingFactor;
            
        end
    end
end

% Smooth channels and add padding for detections near border.
for i=1:nScales
    
    if isempty(coder.target)
        data{i} = vision.internal.acf.convTri(data{i}, smooth);
    else
        data{i} = vision.internal.buildable.acfObjectDetectorBuildable.convTri(data{i}, smooth);
    end
    
    data{i} = ConstantPadBothWithZero(data{i}, [pad/shrink, 0]);
end

pyramid.NumScales = nScales;
pyramid.Channels = data;
pyramid.Scales = imageScales;
pyramid.ScaledImageSize = scaledImageSize;


%--------------------------------------------------------------------------
function [scales,scaleshw] = getScales(nPerOct, nOctUp, minDs, shrink, sz)
% set each scale s such that max(abs(round(sz*s/shrink)*shrink-sz*s)) is
% minimized without changing the smaller dim of sz (tricky algebra)
if(any(sz == 0))
    scales = [];
    scaleshw = [];
    return;
end

nScales = floor(nPerOct * (nOctUp + log2(min(sz ./ minDs))) + 1);
scales = 2.^(-(0:nScales - 1) / nPerOct + nOctUp);

if(sz(1) < sz(2))
    d0 = sz(1);
    d1 = sz(2);
else
    d0 = sz(2);
    d1 = sz(1);
end

for i = 1:nScales
    s = scales(i);
    s0 = (round(d0 * s / shrink) * shrink - .25 * shrink) ./ d0;
    s1 = (round(d0 * s / shrink) * shrink + .25 * shrink) ./ d0;
    ss = (0:.01:1 - eps) * (s1 - s0) + s0;
    es0 = d0 * ss;
    es0 = abs(es0 - round(es0 / shrink) * shrink);
    es1 = d1 * ss;
    es1 = abs(es1 - round(es1 / shrink) * shrink);
    [~, x] = min(max(es0,es1));
    scales(i) = ss(x);
end

kp = [scales(1:end-1) ~= scales(2:end) true];

if isempty(scales)
    scaleshw = zeros(2,0);
else
    scales = scales(kp);
    scaleshw = [round(sz(1) * scales / shrink) * shrink / sz(1);
        round(sz(2) * scales / shrink) * shrink / sz(2)]';
end

%--------------------------------------------------------------------------
function b = ConstantPadBothWithZero(a, padSize)

numDims = numel(padSize);

% Form index vectors to subsasgn input array into output array.
% Also compute the size of the output array.
idx   = cell(1,numDims);
sizeB = zeros(1,numDims);

for k = 1:numDims
    M = size(a,k);
    idx{k}   = (1:M) + padSize(k);
    sizeB(k) = M + 2*padSize(k);
end

% Initialize output array with the padding value.  Make sure the
% output array is the same type as the input.
b = zeros(sizeB, class(a));
b((1+padSize(1):size(a,1)+padSize(1)), (1+padSize(2):size(a,2)+padSize(2)), :) = a;

%--------------------------------------------------------------------------
function [X, Y] = generateRemapXY(data, w, h)
[X, Y] = meshgrid(0:single(w-1), 0:single(h-1));
X = X * size(data, 2) / w;
Y = Y * size(data, 1) / h;

%--------------------------------------------------------------------------
function v = remapUsingInterp2d(I,xq,yq,interp,fillVal)
v = images.internal.interp2d(I, xq+1, yq+1, interp, fillVal, false);