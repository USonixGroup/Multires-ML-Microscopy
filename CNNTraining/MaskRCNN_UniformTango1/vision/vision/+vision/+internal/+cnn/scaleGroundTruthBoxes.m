function [trainingData, scales, boxesRemoved] = scaleGroundTruthBoxes(trainingData, imageSizes, imageLength, useParallel)
% Returns scaled groundTruth, scales used, and logical vector of
% groundTruth images where a box was removed because of scaling.

%   Copyright 2016-2020 The MathWorks, Inc.


numImages = size(imageSizes,1);
scales = zeros(numImages,1);

isvalid(numImages) = struct('HasValidData',[],'HasNoBoxes',[]);

if useParallel 
    
    parfor i = 1:numImages
        
        sz = imageSizes(i,:);
        
        [trainingData(i,:), isvalid(i).HasValidData, isvalid(i).HasNoBoxes] = vision.internal.cnn.utils.hasValidTrainingData(sz,trainingData(i,:));        
        
        [~, smallestSide] = min(sz(1:2));
        
        % scale the smallest side to value specified
        scales(i) = imageLength / sz(smallestSide);               
    end
    
    hasNoBoxes = vertcat(isvalid(:).HasNoBoxes);    
    
    % Remove rows with no data
    trainingData(hasNoBoxes,:) = [];
      
    s = table2struct(trainingData(:,2:end));        
    
    boxesRemoved = false(1,height(trainingData));
    
    % scale boxes in ground truth table
    fields = fieldnames(s);
    parfor i = 1:numel(s)
        
        a = s(i);
        bool = boxesRemoved(i);
        for j = 1:numel(fields)
            b = a.(fields{j});
           
            [bboxes, removed] = scaleBoxes(b, scales(i,:)); 
            
            % Pack boxes as cell arrays to ensure single 1x4 boxes get
            % converted to cell array entries in table when struct2table is
            % used at the end of this processing.
            a.(fields{j}) = {bboxes};
            
            % keep track if any boxes in i_th image were removed.
            bool = bool | removed;
            
        end  
        boxesRemoved(i) = bool;
        
        s(i) = a;
    end
    trainingData = [trainingData(:,1) struct2table(s, 'AsArray', true)];
  
else        

    for i = 1:numImages
        
        sz = imageSizes(i,:);
        
        [trainingData(i,:), isvalid(i).HasValidData, isvalid(i).HasNoBoxes] = vision.internal.cnn.utils.hasValidTrainingData(sz,trainingData(i,:)); 
        
        [~, smallestSide] = min(sz(1:2));
        
        % scale the smallest side to value specified
        scales(i) = imageLength / sz(smallestSide);
    end
       
    hasNoBoxes = vertcat(isvalid(:).HasNoBoxes);    
    
    % Remove rows with no data
    trainingData(hasNoBoxes,:) = [];
        
    % scale boxes in ground truth table
    boxesRemoved = false(1,height(trainingData));
    for i = 1:height(trainingData)
        
        for j = 2:width(trainingData)
            
            [bboxes, removed] = scaleBoxes(trainingData{i,j}{1}, scales(i,:));
            trainingData{i,j} = {bboxes};
            
            boxesRemoved(i) = boxesRemoved(i) | removed;            
            
        end
        
    end
  
end

if isempty(trainingData)
    error(message('vision:rcnn:noValidTrainingData'));
end

validTrainingData = ~hasNoBoxes & vertcat(isvalid(:).HasValidData);
rcnnObjectDetector.issueWarningIfRequired(validTrainingData);

function [scaledBoxes, boxesRemoved] = scaleBoxes(boxes, scale)
% scale is [sy sx]
if isempty(boxes)
    scaledBoxes = zeros(0,4);
    boxesRemoved = false;
else
    
    % returned boxes are in [x1 y1 x2 y2] format
    boxes = vision.internal.cnn.boxUtils.xywhToX1Y1X2Y2(boxes);
    scaledBoxes = vision.internal.cnn.boxUtils.scaleX1X2Y1Y2(boxes, scale, scale);
    
    % convert to [x y w h]
    scaledBoxes(:, 3) =  scaledBoxes(:, 3) - scaledBoxes(:, 1) + 1;
    scaledBoxes(:, 4) =  scaledBoxes(:, 4) - scaledBoxes(:, 2) + 1;

    % remove boxes that have been scaled to 0 width height.
    zeroWidthHeight = any(scaledBoxes(:, 3:4) < 1, 2);
    scaledBoxes(zeroWidthHeight,:) = [];    
    
    boxesRemoved = any(zeroWidthHeight);
end
