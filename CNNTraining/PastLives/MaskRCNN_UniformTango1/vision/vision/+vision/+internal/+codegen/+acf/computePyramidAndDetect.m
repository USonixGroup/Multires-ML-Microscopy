function [bboxes, scores] = computePyramidAndDetect(I, params, paramsNonTunable, detector, threshold) %#codegen
% Constructs a feature pyramid of different scales.
% Apply AdaBoost to detect objects in each scaled feature input.

% Copyright 2020-2022 The MathWorks, Inc.
%
% References
% ----------
%   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
%   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
%   (2014): 1532-1545.

    nPerOct = params.NumScaleLevels;
    nApprox = params.NumApprox;
    nOctUp  = params.NumUpscaledOctaves;
    windowStride = params.WindowStride;

    pad     = paramsNonTunable.ChannelPadding;
    minDs   = paramsNonTunable.ModelSize;
    minDsPad= paramsNonTunable.ModelSizePadded;
    lambdas = paramsNonTunable.Lambdas;
    preSmoothColor = paramsNonTunable.PreSmoothColor;
    gradient = paramsNonTunable.gradient;
    hog = paramsNonTunable.hog;
    shrink  = paramsNonTunable.Shrink;

    imageSize = [size(I,1) size(I,2)];
    coder.internal.prefer_const(imageSize);

    if isfloat(I)
        singleImage = single(mat2gray(I)); % scales floating point values between [0 1].
    else
        singleImage = im2single(I);
    end

    if ismatrix(I)
        singleImage = im2single(cat(3, I, I, I));
    end

    singleImage = vision.internal.codegen.acf.rgb2luv(singleImage);

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

    % The scales for which channels are computed exactly
    realScaleInd = 1;
    realScaleInd = realScaleInd : nApprox + 1 : nScales;

    % Indices of real scales used to approximate each estimated scale
    j = [0 floor((realScaleInd(1:end-1) + realScaleInd(2:end)) / 2) nScales];
    upperLimit = zeros(1,length(j)-1);
    for i = 1:length(j)-1
        upperLimit(i) = j(i+1)-j(i);
    end

    % Initialize data to hold features
    numAccurateScale = size(realScaleInd, 2);
    boundingBoxesCell = coder.nullcopy(cell(numAccurateScale, 1));
    boundingBoxesCellEstimation = coder.nullcopy(cell(numAccurateScale, 1));
    for i = 1 : numAccurateScale
        boundingBoxesCellEstimation{i} = coder.nullcopy(cell(upperLimit(i), 1));
    end

    bboxCount = 0;

    %%%% detect while constructing pyramid %%%%%%%%%%%%%%%%%%%
    parfor i = 1:numAccurateScale
        s = imageScales(realScaleInd(i));
        imageScaledSize = round(imageSize * s / shrink) * shrink;

        if (scaledImageSize(realScaleInd(i)) == 1)  % no scaling needed for original scale
            Is = singleImage;
        else
            [X, Y] = generateRemapXY(I, imageScaledSize(2), imageScaledSize(1));
            fillValue = zeros(1, size(I,3),'like',singleImage);
            Is = images.internal.coder.interp2d(singleImage, X+1, Y+1, 'bilinear', fillValue, false);            
        end

        % Add assertion to prevent emx array for un-bounded matrix size
        assert(size(Is, 1) <= imageSize(1));
        assert(size(Is, 2) <= imageSize(2));
        data = computeChannels(Is, shrink, preSmoothColor, gradient, hog);

        for k = 1: upperLimit(i)
            index = j(i) + k;
            if index == realScaleInd(i) % don't estimate real scale
                continue;
            end

            imageScaledSize = round(imageSize * imageScales(index) / shrink);
            est = vision.internal.codegen.acf.imResample(data, imageScaledSize(1), imageScaledSize(2), single(1));
            ratio = (imageScales(index)/imageScales(realScaleInd(i))).^-lambdas;
            scaling = reshape(repelem(ratio, [3 1 hog.NumBins]), 1, 1, []);
            est = bsxfun(@times, est, scaling);
            est = vision.internal.codegen.acf.convTriR1(est);
            isPaddingRequired = (pad(1) || pad(2));
            if (isPaddingRequired) % if padding is required for any direction
                paddedEst = constantPadBothWithZero(est, [pad/shrink, 0]);
            else % padding not required
                paddedEst = est;
            end
            [boundingBoxesCellEstimation{i}{k}, inc] = detectOnce(paddedEst, detector, shrink, pad, minDsPad, minDs,...
                                                                  scaledImageSize(index, :), imageScales(index), windowStride, threshold);
            bboxCount = bboxCount + inc;
        end
        convData = vision.internal.codegen.acf.convTriR1(data);
        isPaddingRequired = (pad(1) || pad(2));
        if (isPaddingRequired) % if padding is required for any direction
            paddedData = constantPadBothWithZero(convData, [pad/shrink, 0]);
        else % padding not required
            paddedData = convData;
        end
        [boundingBoxesCell{i}, inc] = detectOnce(paddedData, detector, shrink, pad, minDsPad, minDs,...
                                                 scaledImageSize(realScaleInd(i), :), imageScales(realScaleInd(i)), windowStride, threshold);
        bboxCount = bboxCount + inc;
    end

    bboxes = zeros(bboxCount,4,'double');
    scores = zeros(bboxCount,1,'double');
    c = 1;
    for i = 1:numAccurateScale
        for j = 1: size(boundingBoxesCell{i},1)
            bboxes(c,:) = boundingBoxesCell{i}(j,1:4);
            scores(c,1) = boundingBoxesCell{i}(j,5);
            c = c+1;
        end
        for j = 1: upperLimit(i)
            for k = 1:size(boundingBoxesCellEstimation{i}{j},1)
                bboxes(c,:) = boundingBoxesCellEstimation{i}{j}(k,1:4);
                scores(c,1) = boundingBoxesCellEstimation{i}{j}(k,5);
                c = c+1;
            end
        end
    end
end

function [boundingBox, count] = detectOnce(channel, detector, shrink, pad, modelDsPad, modelDs, ...
                                           scaledImageSize, imageScale, windowStride, threshold)
    % Detect object in a single scale

    count = 0;
    boundingBox = [];

    % Apply sliding window classifiers
    bb = visionACFDetectorCG(channel, detector, shrink, ...
                             modelDsPad(1), ...
                             modelDsPad(2), ...
                             windowStride, ...
                             threshold);

    % Shift and scale the detections due to actual object size
    % (shift-= modelDsPad-modelDs)/2), channel padding (shift -= pad)
    % and scale difference (bb=(bb+shift)/P.scaleshw)
    if (bb(:,5) ~= 0 )
        shift   = (modelDsPad - modelDs)/2 - pad;
        bb(:,1) = (bb(:,1) + shift(2))/scaledImageSize(2);
        bb(:,2) = (bb(:,2) + shift(1))/scaledImageSize(1);
        bb(:,3) = modelDs(2)/imageScale;
        bb(:,4) = modelDs(1)/imageScale;
        bbSize = size(bb);
        if (bbSize(1) ~= 0)
            boundingBox = bb(:,1:5);
            count = bbSize(1);
        end
    end
end

function [scales,scaleshw] = getScales(nPerOct, nOctUp, minDs, shrink, sz)
% Set each scale s such that max(abs(round(sz*s/shrink)*shrink-sz*s)) is
% minimized without changing the smaller dim of sz (tricky algebra)
    if(any(sz == 0))
        scales = [];
        scaleshw = [];
        return;
    end

    nScales = floor(nPerOct * (nOctUp + log2(min(sz ./ minDs))) + 1);
    scales = 2.^(-(0:nScales - 1) / nPerOct + nOctUp);

    if (sz(1) < sz(2))
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
end

function b = constantPadBothWithZero(a, padSize)
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
end

function [X, Y] = generateRemapXY(data, w, h)
    [X, Y] = meshgrid(0:single(w-1), 0:single(h-1));
    X = X * size(data, 2) / w;
    Y = Y * size(data, 1) / h;
end

function data = computeChannels( I, shrink, PreSmoothColor, gradient, hog )%#codegen
% Compute channel features at a single scale given an input image.
% Currently, three channel types are available by default
%  (1) color channels (computed using rgbConvert.m)
%  (2) gradient magnitude (computed using gradient.m)
%  (3) quantized gradient channels (computed using gradientHist.m)
%
% This code is a modified version of that found in:
%
% Piotr's Computer Vision Matlab Toolbox      Version 3.23
% Copyright 2014 Piotr Dollar & Ron Appel.  [pdollar-at-gmail.com]
% Licensed under the Simplified BSD License [see pdollar_toolbox.rights]

    % Crop I so it becomes divisible by shrink and get target dimensions
    [h, w, ~] = size(I);
    cr = mod([h w], shrink);

    if (any(cr))
        h = h - cr(1);
        w = w - cr(2);
    end

    J = vision.internal.codegen.acf.convTri(I(1:h, 1:w, :), PreSmoothColor);
    [M,O] = vision.internal.codegen.acf.gradientMO(J, gradient);

    h = h / shrink;
    w = w / shrink;

    % Compute HOG channels
    H = vision.internal.codegen.acf.gradientHist(M, O, hog);
    % Shrink data
    [h1, w1, ~] = size(H);
    if (h1 ~= h || w1 ~= w)
        H = vision.internal.codegen.acf.imResample(H, h, w, single(1));
    end
    J = cat(3, J, M);
    data = vision.internal.codegen.acf.imResample(J, h, w, single(1));

    data = cat(3, data, H);
end

function bbs = visionACFDetectorCG(chns,trees,shrink,modelHt,modelWd,stride,cascThr,flag)
% Apply classifier on scaled feature channels. Then transform detection
% back to the original scale.

    if ~isa(chns,'single')
        ME = MException('vision:visionlib','unsupportedDataType');
        throw(ME)
    end

    if (nargin == 8)
        if ~isa(flag,'logical')
            ME = MException('vision:visionlib','unsupportedDataType');
            throw(ME)
        end
    else
        flag = -1;
    end

    thrs = trees.thrs;
    hs = trees.hs;
    [nTreeNodes, nTrees] = size(trees.fids,[1 2]);
    fids = trees.fids(:);
    child = trees.child;
    if isfield(trees, 'treeDepth')
        treeDepth = trees.treeDepth;
    end

    % get dimensions and constants
    height = size(chns,1);
    width = size(chns,2);
    height1 = ceil((height * shrink - modelHt + 1) / stride);
    width1 = ceil((width * shrink - modelWd + 1) / stride);

    [m,rs,cs,hs1] = vision.internal.codegen.acf.getBoundingBoxes(chns, thrs, hs, fids, child, shrink, modelHt, ...
                                                                       modelWd, stride, cascThr, nTreeNodes, nTrees, height1, width1, treeDepth, flag);

    % convert to bounding boxes
    bbs = zeros([m,5]);
    for i = 1:m
        bbs(i,1) = cs(i) * stride;
        bbs(i,3) = modelWd;
        bbs(i,2) = rs(i) * stride;
        bbs(i,4) = modelHt;
        bbs(i,5) = hs1(i);
    end
end
