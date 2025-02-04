function vipblk2dfirfilterdynamic
% VIPBLK2DFIRFILTER Mask dynamic function for 2D FIR Filter block

% Copyright 2018 The MathWorks, Inc.

blk = gcbh;
blkMask = Simulink.Mask.get(gcb);

filtCoeffSrc      = get_param(blk,'filtSrc');
isSeparable       = get_param(blk,'separable');
outSize           = get_param(blk,'outSize');
padType           = get_param(blk,'padType');
padSrc            = get_param(blk,'padSrc');

filterMtrx            = blkMask.getParameter('filterMtrx');
verticalFilter        = blkMask.getParameter('verticalFilter');
horizontalFilter      = blkMask.getParameter('horizontalFilter');
padTypeMaskParam      = blkMask.getParameter('padType');
padSrcMaskParam       = blkMask.getParameter('padSrc');
padVal                = blkMask.getParameter('padVal');
firstCoeffDataTypeStr = blkMask.getParameter('firstCoeffDataTypeStr');
firstCoeffMin         = blkMask.getParameter('firstCoeffMin');
firstCoeffMax         = blkMask.getParameter('firstCoeffMax');
firstCoeffLabel       = blkMask.getDialogControl('firstCoeffLabel');
if strcmp(filtCoeffSrc,'Specify via dialog') && strcmp(isSeparable,'off')
    filterMtrx.Visible            = 'on';
    verticalFilter.Visible        = 'off';
    horizontalFilter.Visible      = 'off';
    firstCoeffDataTypeStr.Visible = 'on';
    firstCoeffMin.Visible         = 'on';
    firstCoeffMax.Visible         = 'on';
    firstCoeffLabel.Visible       = 'on';
elseif strcmp(filtCoeffSrc,'Specify via dialog') && strcmp(isSeparable,'on')
    filterMtrx.Visible            = 'off';
    verticalFilter.Visible        = 'on';
    horizontalFilter.Visible      = 'on';
    firstCoeffDataTypeStr.Visible = 'on';
    firstCoeffMin.Visible         = 'on';
    firstCoeffMax.Visible         = 'on';
    firstCoeffLabel.Visible       = 'on';
else
    firstCoeffDataTypeStr.Visible = 'off';
    firstCoeffMin.Visible         = 'off';
    firstCoeffMax.Visible         = 'off';
    filterMtrx.Visible            = 'off';
    verticalFilter.Visible        = 'off';
    horizontalFilter.Visible      = 'off';
    firstCoeffLabel.Visible       = 'off';
end
if strcmp(outSize,'Valid')
    padTypeMaskParam.Visible     = 'off';
    padSrcMaskParam.Visible      = 'off';
    padVal.Visible               = 'off';
else
    padTypeMaskParam.Visible = 'on';
    if strcmp(padType,'Constant')
        padSrcMaskParam.Visible  = 'on';
        if strcmp(padSrc,'Specify via dialog')
            padVal.Visible       = 'on';
        else
            padVal.Visible       = 'off';
        end
    else
        padSrcMaskParam.Visible  = 'off';
        padVal.Visible           = 'off';
    end
end

