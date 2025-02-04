classdef HRNETObjectKeypointDetector
%#codegen
%
  
% Copyright 2023-2024 The MathWorks, Inc.

    properties
        Network
        Threshold
        InputSize
        KeyPointClasses
    end
    methods (Hidden, Static)
        function n = matlabCodegenNontunableProperties(~)
            n = {'InputSize','Threshold','KeyPointClasses'};
        end
    end

    methods
        function obj = HRNETObjectKeypointDetector(matfile)
            coder.extrinsic('vision.internal.codegen.HRNETObjectKeypointDetector.getOtherProps');
            obj.Network = coder.internal.loadDeepLearningNetwork(matfile, 'ReturnNetwork', true);
           
            [obj.Threshold,obj.InputSize,obj.KeyPointClasses] = coder.const(@vision.internal.codegen.HRNETObjectKeypointDetector.getOtherProps,matfile);
        end

        function varargout = detect(this,I,bboxes)
            
            arguments
                this   
                I      {mustBeNonempty,mustBeNumeric}
                bboxes {mustBeNumeric,mustBeReal,mustBeFinite,mustBePositive,mustBeNonsparse}
            end

            coder.internal.assert(coder.internal.isConst(size(I)), 'vision:hrnetObjectKeypoint:imageSizeConstant');
 
            % Error out if input is batch of images or gray image.
            coder.internal.errorIf(size(I,3) ~= 3 || ndims(I)>3, ...
                'vision:hrnetObjectKeypoint:invalidInputImage');

            coder.internal.prefer_const(bboxes);
            % Error out if size of bboxes is not constant.
            coder.internal.assert(coder.internal.isConst(size(bboxes)), ...
                'vision:hrnetObjectKeypoint:bboxesConstant');

            coder.internal.errorIf(size(bboxes,2)~=4, 'vision:hrnetObjectKeypoint:invalidBBoxDimension');
          

            [preprocessedCropped, center, scale,bboxOffset] = preprocessHRNet(this,I,bboxes);

            networkOutput = predict(this.Network,preprocessedCropped);
            networkOutput = extractdata(networkOutput);
           [varargout{1},varargout{2},varargout{3}] = postprocessHRNet(this,networkOutput,center,scale,bboxOffset);
        end

        function [Ipreprocessed, center, scale,bboxOffset] = preprocessHRNet(this,I,bboxes)
            coder.inline('always')
            coder.internal.prefer_const(bboxes);
            % Make the compile time function calls as extrinsic
            coder.extrinsic('vision.internal.detector.checkROI');

            trainingImageSize = this.InputSize;
            I = rescale(I);
            croppedImages = coder.nullcopy(zeros(trainingImageSize(1), trainingImageSize(2),trainingImageSize(3), ...
                size(bboxes,1),1,'like',I));
            center = coder.nullcopy(zeros(size(bboxes,1),2));
            scale = coder.nullcopy(zeros(size(bboxes,1),2));
            bboxOffset = coder.nullcopy(zeros(size(bboxes,1),4));
            for k = 1:size(bboxes,1)
                box = bboxes(k,:);
                % Check whether bboxes is fully contained in image.
                feval('vision.internal.detector.checkROI', box, size(I));
                if box(1,4)==this.InputSize(1) && box(1,3)==this.InputSize(2)
                    % Crop the image based on bounding boxes if bounding
                    % box is same as network input size or image size.
                    croppedImages(:,:,:,k) = vision.internal.detector.cropImage(I, box, false);
                    bboxOffset(k,:) = box;
                else
                    [center(k,:),scale(k,:)] = iBoxToCenterScale(box,trainingImageSize(1),trainingImageSize(2));
                    trans = iGetAffineTransform(center(k,:), scale(k,:), [trainingImageSize(1),trainingImageSize(2)],false);
                    croppedImage = imwarp(I,trans,...
                        'OutputView',imref2d([trainingImageSize(1) trainingImageSize(2)]));
                    croppedImages(:,:,:,k) = croppedImage;
                    bboxOffset(k,:) = 0;
                end
            end

            Ipreprocessed = dlarray(single(croppedImages),'SSCB');
        end

        function [keypoints,score,visibilityFlag] = postprocessHRNet(this,networkOutput,center,scale,bboxOffset)
            coder.inline('always')
            threshold = this.Threshold;
            nObjects = size(center,1);
            nKeypoints = size(this.KeyPointClasses,1);
            keypoints = zeros(nKeypoints,2,nObjects,'like',networkOutput);
            score = coder.nullcopy(zeros(nKeypoints,nObjects,'like',networkOutput));
            visibilityFlag = true(nKeypoints,nObjects);
            [heatmapheight,heatmapWidth, numKeypoints,~] = size(networkOutput);
            for j = 1:nObjects
                [keypoints(:,:,j), score(:,j)] = iPostPros(networkOutput(:,:,:,j),center(j,:), scale(j,:),heatmapheight,heatmapWidth, numKeypoints,bboxOffset(j,:));
                visibilityFlag(:,j) = score(:,j)>threshold;
            end
        end
    end

    methods(Static)

        function [propThreshold,propInputSize,propKeyPointClasses] = getOtherProps(matfile)
            keypointDetectorobj = loadHRNETObjectKeypointDetector(matfile);
            propThreshold = keypointDetectorobj.Threshold;
            propInputSize = keypointDetectorobj.InputSize;
            propKeyPointClasses = keypointDetectorobj.KeyPointClasses;

        end
    end
end

%--------------------------------------------------------------------------
function [center,scale] = iBoxToCenterScale(box,modelImageHeight,modelImageWidth)
% Convert bbox from [x, y, w, h] to center and scale.
% The center is the coordinates of the bbox center, and the scale is the
% bbox width and height normalized by a scale factor.

center = [0 0];
boxWidth = box(:,3);
boxHeight = box(:,4);
center(1) = box(:,1) + floor(boxWidth/2);
center(2) = box(:,2) + floor(boxHeight/2);


aspectRatio = modelImageWidth * 1.0 / modelImageHeight;
% Pixel std is 200.0, which serves as the normalization factor to
% to calculate bbox scales.
% https://github.com/leoxiaobin/deep-high-resolution-net.pytorch/blob/master/demo/demo.py#L180
pixelStd = 200;

if boxWidth > aspectRatio * boxHeight
    boxHeight = boxWidth * 1.0 / aspectRatio;
elseif boxWidth < aspectRatio * boxHeight
    boxWidth = boxHeight * aspectRatio;
end
scale = double([boxWidth * 1.0 / pixelStd, boxHeight * 1.0 / pixelStd]);

% 1.25 value is taken from author's github repo.
% https://github.com/leoxiaobin/deep-high-resolution-net.pytorch/blob/master/demo/demo.py#L189
scale = scale * 1.25;
end
%----------------------------------------------------------------------------------
function [targetCords,detectionConfScore] = iPostPros(networkOutput,center,scale,heatmapheight,heatmapWidth, ...
    numKeypoints,bboxOffset)
% Post processing of networkOutput to get axis points.
[confs,idx] = max(networkOutput,[],[1,2],'linear');
[y,x,~,~] = ind2sub(size(networkOutput),idx);
joints = permute(cat(1,x,y-1),[3,1,4,2]);
detectionConfScore = squeeze(confs);
detectionConfScore(~lt(detectionConfScore,1)) = 1;

heatMapOutputSize = [heatmapheight, heatmapWidth];
% Preallocate the targetCords.
targetCords = coder.nullcopy(zeros(numKeypoints,2,'like',networkOutput));

if ~all(bboxOffset==0)
    % Skip the post processing steps when bounding box dimensions match the
    % network input or image dimensions. Calculate the target coordinate using
    % bounding box (offset).
    targetCords(:,1) = joints(:,1).*4 + bboxOffset(1)-1;
    targetCords(:,2) = joints(:,2).*4 + bboxOffset(2)-1;
else
    trans = iGetAffineTransform(center, scale, heatMapOutputSize,true);
    trans = trans.A(1:2,:);
    appendOne = ones(length(joints),1);
    joints1 = [joints appendOne];
    targetCords(:,1) = sum(joints1.*trans(1,:),2);
    targetCords(:,2) = sum(joints1.*trans(2,:),2);
    targetCords(~gt(targetCords,0)) = 1;
end
end


%--------------------------------------------------------------------------
function transformMatrix = iGetAffineTransform(center, scale, outputHeatMapSize,invAffineTransform)
% center: Center of the bounding box (x, y).
% scale: Scale of the bounding box wrt [width, height].
% outputHeatMapSize: Size of the destination heatmaps.
% invAffineTransform (boolean): Option to inverse the affine transform direction.
% (inv=False: src->dst or inv=True: dst->src).

% shift (0-100%): Shift translation ratio wrt the width/height.
shift = [0 0];

% pixel_std is 200 as per author and mmpose github
% https://github.com/open-mmlab/mmpose/blob/master/mmpose/core/post_processing/post_transforms.py.
scaleTmp = scale*200.0;
srcWidth = scaleTmp(1);
dstHeight = outputHeatMapSize(1);
dstWidth = outputHeatMapSize(2);

srcPoint = [1 , srcWidth * -0.5];
dstDir = double([1, dstWidth * -0.5]);

src = coder.nullcopy(zeros(3, 2));
dst = coder.nullcopy(zeros(3, 2));
src(1, :) = center + scaleTmp .* shift;
src(2, :) = center + srcPoint + scaleTmp .* shift;
dst(1, :) = [dstWidth * 0.5, dstHeight * 0.5];
dst(2, :) = [dstWidth * 0.5, dstHeight * 0.5] + dstDir;

src(3, :) = iGetThirdPoint(src(1,:), src(2,:));
dst(3, :) = iGetThirdPoint(dst(1,:), dst(2, :));

if invAffineTransform
    transformMatrix = fitgeotform2d(dst,src,"affine");
else
    transformMatrix = fitgeotform2d(src,dst,"affine");
end
end
%--------------------------------------------------------------------------
function thirdPoint =  iGetThirdPoint(a, b)
% To calculate the affine matrix, three pairs of points are required. This
% function is used to get the 3rd point, given 2D points a & b.
% The 3rd point is defined by rotating vector `a - b` by 90 degrees
% anticlockwise, using b as the rotation center.
% Args:
%     a : point(x,y)s
%     b : point(x,y)
% Returns:
%     The 3rd point.
direction = a - b;
thirdPoint = b + [-direction(2)-1, direction(1)+1];
end