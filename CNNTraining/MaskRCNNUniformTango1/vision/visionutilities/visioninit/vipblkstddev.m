function vipblkstddev
% VIPBLKSTDDEV Mask dynamic dialog function for 2-D Standard Deviation Block

%   Copyright 2017-2020 The MathWorks, Inc.

blk = gcbh;   % Cache handle to block

oldVisibilities = get_param(blk, 'MaskVisibilities');
newVisibilities = oldVisibilities;
maskNames    = get_param(blk,'MaskNames');
blkMask = Simulink.Mask.get(gcb);

% =========================================================================
% Parameter values
% =========================================================================
runValue = get_param(blk, 'run');
directionModeValue = get_param(blk, 'directionMode');
roiEnableValue = get_param(blk, 'roiEnable');
roiTypeValue = get_param(blk, 'roiType');

% =========================================================================
% Dialog Controls
% =========================================================================
ROIProcessingGroup = blkMask.getDialogControl('ROIProcessing');

% =========================================================================
% Parameter Ids for indexing visibilities
% =========================================================================
runIdx = strcmp(maskNames(:),'run');
resetIdx = find(strcmp(maskNames(:),'reset_popup'));
directionModeIdx = find(strcmp(maskNames(:),'directionMode'));
dimensionIdx = find(strcmp(maskNames(:),'dimension'));
roiEnableIdx = strcmp(maskNames(:),'roiEnable');
roiTypeIdx = find(strcmp(maskNames(:),'roiType'));
roiPortionIdx = find(strcmp(maskNames(:),'roiPortion'));
roiOutputIdx = find(strcmp(maskNames(:),'roiOutput'));
roiFlagIdx = find(strcmp(maskNames(:),'roiFlag'));

% =========================================================================
% Parameters Group Visibilities
% =========================================================================

if strcmp(newVisibilities{runIdx},'on') && strcmp(runValue, 'on')
    newVisibilities{resetIdx} = 'on';
    newVisibilities{directionModeIdx} = 'off';
    newVisibilities{dimensionIdx} = 'off';
else
    newVisibilities{resetIdx} = 'off';
    newVisibilities{directionModeIdx} = 'on';
    if (strcmp(directionModeValue,getString(message('vision:masks:SpecifiedDimension'))))
        newVisibilities{dimensionIdx} = 'on';
    else
        newVisibilities{dimensionIdx} = 'off';
    end
end


% =========================================================================
% ROI Group Visibilities
% =========================================================================

if strcmp(newVisibilities{roiEnableIdx},'on') && strcmp(roiEnableValue, 'on')
    newVisibilities{roiTypeIdx} = 'on';
    % Make ROI portion selection visible for ROIs that make sense
    if (strcmp(roiTypeValue,getString(message('vision:masks:Rectangles'))))
        newVisibilities{roiPortionIdx} = 'on';
    else
        newVisibilities{roiPortionIdx} = 'off';
    end
    % Make Output & Flag selections invisible for Binary Mask
    if (strcmp(roiTypeValue,getString(message('vision:masks:BinaryMask'))))
        newVisibilities{roiOutputIdx} = 'off';
        newVisibilities{roiFlagIdx} = 'off';
    else
        newVisibilities{roiOutputIdx} = 'on';
        newVisibilities{roiFlagIdx} = 'on';
    end
else
    newVisibilities{roiTypeIdx} = 'off';
    newVisibilities{roiPortionIdx} = 'off';
    newVisibilities{roiOutputIdx} = 'off';
    newVisibilities{roiFlagIdx} = 'off';
end


if strcmp(runValue, 'on') %% ROIs are not available in the running mode
    ROIProcessingGroup.Visible = 'off';
else % non-running
    if (strcmp(directionModeValue, getString(message('vision:masks:EntireInput'))))
        ROIProcessingGroup.Visible = 'on';
    else
        ROIProcessingGroup.Visible = 'off';
    end
end

% =========================================================================
% Setting the mask visibilities
% =========================================================================
if ~isequal(newVisibilities,oldVisibilities)
    set_param(blk, 'MaskVisibilities', newVisibilities);
end

end
