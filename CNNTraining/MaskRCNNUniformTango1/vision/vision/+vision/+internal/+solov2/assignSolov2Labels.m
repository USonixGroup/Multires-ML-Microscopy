function [mlvlPosIdx, mlvlPosMaskTargets, mlvlLabels, mlvlPosMasks] = assignSolov2Labels(gtMasks, gtLabels, gtBoxes, featSize, objSizeRanges, gridSizes, numClasses)
% Assign groundtruth for target generation for SoloV2
%
% Input:
% gtMasks: HxWxnumGT logical
% gtLabels: numGTx1 categorical
% gtBoxes: numGTx4 numeric: [x y h w] format
% featSize: Feature size of the mask_feat branch
% objSizeRanges: numLvls x2
% gridSizes: numLvlsx2
%
% Output:
% mlvlPosIdx : numLvlsx1 cell-array. Each cell contains numPosx1 indices of
% positive grid locations
% mlvlPosMaskTargets: numLvlsx1 cell-array. Each cell contains a hxwxnumPos
% list of positive masks at that level.
% mlvlLabels: numLvlsx1 cell-array. Each cell contains a numGridxnumGrid
% list of positive masks at that level.
% mlvlPosMasks: numLvlsx1 cell-array. Each cell contains a numGrid^2 x 1
% boolean list indicating postive samples along linear indices.


% Loop through each FPN Level
numLvls = size(gridSizes,2);
% Get ground truth areas
gt_areas = sqrt(gtBoxes(:,3).*gtBoxes(:,4));

mlvlPosMaskTargets = [];
mlvlPosIdx = [];
mlvlLabels = [];
mlvlPosMasks = [];

for i=1:numLvls

    % Step 1: Filter the groundtruth based on object size range assigned to
    % each level
    lvlGridSize = gridSizes(i);
    lowerObjSize = objSizeRanges(i,1);
    upperObjSize = objSizeRanges(i,2);
    
    lvlPosIdx = [];
    lvlMaskTarget = [];

    % Intialize all labels at current location to background (bgidx = numClasses+1)
    lvlLabels = zeros(lvlGridSize, lvlGridSize) + (numClasses+1);
    lvlPosMask = false(lvlGridSize^2,1); % linear idx

    gtIdx = find((gt_areas>=lowerObjSize)&(gt_areas<=upperObjSize));

    % TODO: handle if gtIdx is empty
    if(isempty(gtIdx))
        mlvlPosMaskTargets{end+1,1} = false(featSize(1), featSize(2), 0);
        mlvlLabels{end+1,1} = lvlLabels;
        mlvlPosMasks{end+1,1} = lvlPosMask;
        mlvlPosIdx{end+1,1} = [];
        continue;
    end

    validGtBoxes = gtBoxes(gtIdx, :);
    validGtLabels = gtLabels(gtIdx);
    validGtMasks = gtMasks(:,:,gtIdx);


   % Step 2: Compute the bounds of the window around the center of  mass of
   % the object
   posScale = 0.2; % epsilon for the window around center of mass
   
   % Window range from the center of the object
   posWrange = 0.5 * validGtBoxes(:,3) * posScale;
   posHrange = 0.5 * validGtBoxes(:,4) * posScale;

   % Ensure the validmask have some value in it
   validMaskFlag = squeeze(sum(validGtMasks, [1 2])) > 0;

   numValid = size(validGtBoxes,1);


   for idx = 1:numValid

       if(~validMaskFlag(idx))
           continue;
       end

       currMask = validGtMasks(:,:,idx);

       %TODO: Move to class properties
       maskStride = 4;
       upsampledSize = featSize([1 2]).*maskStride;

       % Get center of mass of object in original mask coordinates.
       [centerH, centerW] = vision.internal.solov2.centerOfMass(currMask);

       % Get the top, bottom, left & right grid boxes idx
       % TODO: 1 index
       topBox = max(1,...
                     floor(...
                     ((centerH - posHrange(idx))/upsampledSize(1))/...
                     (1/lvlGridSize)));
       bottomBox = min(lvlGridSize,...
                     floor(...
                     ((centerH + posHrange(idx))/upsampledSize(1))/...  
                     (1/lvlGridSize)));
       leftBox = max(1,...
                     floor(...
                     ((centerW - posWrange(idx))/upsampledSize(2))/...
                     (1/lvlGridSize)));
       rightBox = min(lvlGridSize,...
                     floor(...
                     ((centerW + posWrange(idx))/upsampledSize(2))/...
                     (1/lvlGridSize)));

        % Get center grid box location
        centerHBox = floor(...
                            (centerH/upsampledSize(1)) / (1/lvlGridSize));
        centerWBox = floor(...
                            (centerW/upsampledSize(2)) / (1/lvlGridSize));

        % Ensure that the center region box location is atleast one box
        % away from the center
        top = max(topBox,centerHBox-1);
        bottom = min(bottomBox, centerHBox+1);
        left = max(leftBox, centerWBox-1);
        right = min(rightBox, centerWBox+1);

        % Step 3:Assign the positive boxes the corresponding label
        lvlLabels(top:bottom, left:right) = gtLabels(idx);

        % Step 4: Generate mask target. For each positive grid
        currMask = imresize(currMask,1/maskStride);

        for c = left:right
            for r = top:bottom
            
                linIdx = (c-1)*lvlGridSize+r;
                
                if(isempty(lvlMaskTarget))
                    lvlMaskTarget = currMask;
                else
                    lvlMaskTarget(:,:,end+1)=currMask;
                end

                lvlPosMask(linIdx) = true;
                lvlPosIdx = [lvlPosIdx;linIdx];
            end
        end
   end

   if(isempty(mlvlPosMaskTargets))
       mlvlPosMaskTargets = {lvlMaskTarget};
   else
       mlvlPosMaskTargets = [mlvlPosMaskTargets; {lvlMaskTarget}];
   end
   if(isempty(mlvlLabels))
       mlvlLabels = {lvlLabels};
   else
       mlvlLabels = [mlvlLabels; {lvlLabels}];
   end
   if(isempty(mlvlPosMasks))
       mlvlPosMasks = {lvlPosMask};
   else
       mlvlPosMasks = [mlvlPosMasks; {lvlPosMask}];
   end
   if(isempty(mlvlPosIdx))
       mlvlPosIdx = {lvlPosIdx};
   else
       mlvlPosIdx = [mlvlPosIdx; {lvlPosIdx}];
   end
    
end

    












