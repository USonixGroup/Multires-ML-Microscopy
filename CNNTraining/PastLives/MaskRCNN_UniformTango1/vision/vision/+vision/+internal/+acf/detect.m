function [bboxes, scores] = detect(P, detector, params, flag)%#codegen
% Run aggregate channel features object detector on given image.

% This code is a modified version of that found in:
%
% Piotr's Computer Vision Matlab Toolbox      Version 3.23
% Copyright 2014 Piotr Dollar & Ron Appel.  [pdollar-at-gmail.com]
% Licensed under the Simplified BSD License [see pdollar_toolbox.rights]

shrink        = params.Shrink;
pad           = params.ChannelPadding;
modelDsPad    = params.ModelSizePadded;
modelDs       = params.ModelSize;
flagPresent = (nargin == 4);
count = 1;
boundingBoxes_cell = coder.nullcopy(cell(P.NumScales, 1));
% Apply sliding window classifiers
for i=1:P.NumScales
    
    if flagPresent
        if isempty(coder.target)
            bb = visionACFDetector(P.Channels{i}, detector, shrink, ...
                modelDsPad(1), ...
                modelDsPad(2), ...
                params.WindowStride, ...
                params.Threshold, ...
                flag{i});
        else
            [bb] = vision.internal.buildable.acfObjectDetectorBuildable.detector_flag(P.Channels{i},...
                detector, params, flag{i});
        end
        
    else
        if isempty(coder.target)
            bb = visionACFDetector(P.Channels{i}, detector, shrink, ...
                params.ModelSizePadded(1), ...
                params.ModelSizePadded(2), ...
                params.WindowStride, ...
                params.Threshold);
        else
            [bb] = vision.internal.buildable.acfObjectDetectorBuildable.detector(P.Channels{i}, detector, params);
        end
        
    end
    % Shift and scale the detections due to actual object size
    % (shift-= modelDsPad-modelDs)/2), channel padding (shift -= pad)
    % and scale difference (bb=(bb+shift)/P.scaleshw)
    if(bb(:,5)~=0 )
        shift   = (modelDsPad - modelDs)/2 - pad;
        bb(:,1) = (bb(:,1) + shift(2))/P.ScaledImageSize(i,2);
        bb(:,2) = (bb(:,2) + shift(1))/P.ScaledImageSize(i,1);
        bb(:,3) = modelDs(2)/P.Scales(i);
        bb(:,4) = modelDs(1)/P.Scales(i);
        bbSize = size(bb);
        boundingBoxes_cell{i} = coder.nullcopy(zeros(bbSize(1), 5));
        
        if(bbSize(1) ~= 0)
            boundingBoxes_cell{i,1} = bb(:,1:5);
            count = count + bbSize(1);
        end
        
    end
    
end

if (count == 1 )
    bboxes = zeros(0,4);
    scores = zeros(0,1);
else
bboxes = zeros(count-1,4);
scores = zeros(count-1,1);
end
c = 1;
while( c <= count -1)
    for i=1:P.NumScales
        if(size(boundingBoxes_cell{i,1},1) ~=0)
            for j = 1: size(boundingBoxes_cell{i,1},1)
                bboxes(c,:) =  boundingBoxes_cell{i,1}(j,1:4);
                scores(c,1) = boundingBoxes_cell{i,1}(j,5);
                c = c+1;
            end
        end
    end
end

