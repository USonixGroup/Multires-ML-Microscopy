function vipblk2dminmax

% VIPBLK2DMINMAX is a Mask dynamic function for 2-D Minimum and 2-D Maximum
% Blocks of visionstatistics library

% Copyright 2018 The MathWorks, Inc.

blk = gcbh;
blkMask = Simulink.Mask.get(blk);

mode          = get_param(blk, 'fcn');
operateOver   = get_param(blk, 'operateOver');
roiEnable     = get_param(blk, 'roiEnable');
roiType       = get_param(blk, 'roiType');
dimension            = blkMask.getParameter('Dimension');
indexBase            = blkMask.getParameter('indexBase');
resetPort            = blkMask.getParameter('reset');
operateOverMaskParam = blkMask.getParameter('operateOver');
roiEnableMaskParam   = blkMask.getParameter('roiEnable');
roiTypeMaskParam     = blkMask.getParameter('roiType');
roiPortion           = blkMask.getParameter('roiPortion');
roiOutput            = blkMask.getParameter('roiOutput');
roiFlag              = blkMask.getParameter('roiFlag');
roiProcessing        = blkMask.getDialogControl('roiProcessing');
if strcmp(mode,'Value and Index') || strcmp(mode, 'Index')
    indexBase.Visible            = 'on';
    operateOverMaskParam.Visible = 'on';
    resetPort.Visible            = 'off';
elseif strcmp(mode,'Value')
    indexBase.Visible            = 'off';
    operateOverMaskParam.Visible = 'on';
    resetPort.Visible            = 'off';
elseif strcmp(mode,'Running')
    indexBase.Visible            = 'off';
    operateOverMaskParam.Visible = 'off';
    resetPort.Visible            = 'on';
end
if ~strcmp(mode,'Running')
    roiProcessing.Visible = 'on';
    if strcmp(operateOver,'Entire input')
        dimension.Visible          = 'off';
        roiEnableMaskParam.Visible = 'on';
        if strcmp(roiEnable,'on')
            roiTypeMaskParam.Visible = 'on';
            if strcmp(roiType,'Rectangles')
                roiPortion.Visible       = 'on';
                roiOutput.Visible        = 'on';
                roiFlag.Visible          = 'on';
                roiFlag.Prompt           = 'vision:masks:OutputFlagROIImageBound';
            elseif strcmp(roiType,'Lines')
                roiPortion.Visible       = 'off';
                roiOutput.Visible        = 'on';
                roiFlag.Visible          = 'on';
                roiFlag.Prompt           = 'vision:masks:OutputFlagROIImageBound';
            elseif strcmp(roiType,'Label matrix')
                roiPortion.Visible       = 'off';
                roiOutput.Visible        = 'on';
                roiFlag.Visible          = 'on';
                roiFlag.Prompt           = 'vision:masks:OutputFlagIndicatingIfInputLabelNumAreValid';
            else
                roiPortion.Visible       = 'off';
                roiOutput.Visible        = 'off';
                roiFlag.Visible          = 'off';
            end
        else
            roiTypeMaskParam.Visible    = 'off';
            roiPortion.Visible          = 'off';
            roiOutput.Visible           = 'off';
            roiFlag.Visible             = 'off';
        end
    elseif strcmp(operateOver, 'Specified dimension')
        dimension.Visible        = 'on';
        roiProcessing.Visible    = 'off';
    else
        dimension.Visible        = 'off';
        roiProcessing.Visible    = 'off';
    end
else
    roiProcessing.Visible = 'off';
end