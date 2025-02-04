function vipblk2dmean

% VIPBLK2DMINMAX is a Mask dynamic function for 2-D Mean
% Block of visionstatistics library

% Copyright 2018-2020 The MathWorks, Inc.

blk = gcbh;
blkMask = Simulink.Mask.get(blk);

run                    =   get_param(blk, 'run');
directionMode          =   get_param(blk, 'directionMode');
roiEnable              =   get_param(blk, 'roiEnable');
roiType                =   get_param(blk, 'roiType');
dimension              =   blkMask.getParameter('dimension');
resetPort              =   blkMask.getParameter('reset_popup');
directionModeMaskParam =   blkMask.getParameter('directionMode');
roiEnableMaskParam     =   blkMask.getParameter('roiEnable');
roiTypeMaskParam       =   blkMask.getParameter('roiType');
roiPortion             =   blkMask.getParameter('roiPortion');
roiOutput              =   blkMask.getParameter('roiOutput');
roiFlag                =   blkMask.getParameter('roiFlag');
roiProcessing          =   blkMask.getDialogControl('roiProcessing');

if strcmp(run,'off')
    directionModeMaskParam.Visible  = 'on';
    resetPort.Visible               = 'off';
    if strcmp(directionMode, 'Entire input')
        roiProcessing.Visible      = 'on';
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
    elseif strcmp(directionMode, 'Specified dimension')
        dimension.Visible        = 'on';
        roiProcessing.Visible    = 'off';
    else
        dimension.Visible        = 'off';
        roiProcessing.Visible    = 'off';
    end
else
    directionModeMaskParam.Visible  = 'off';
    resetPort.Visible               = 'on';
    roiProcessing.Visible           = 'off';
    dimension.Visible               = 'off';
end
