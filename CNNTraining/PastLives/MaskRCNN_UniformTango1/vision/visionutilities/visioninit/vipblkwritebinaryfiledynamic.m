function vipblkwritebinaryfiledynamic(blk,action)
% VIPBLKWRITEBINARYFILEDYNAMIC callback function for Editor Based Mask of Write Binary File Block

%   Copyright 2018-2020 The MathWorks, Inc.

% This function modifies settings of blocks in the subsystem. This
% should not dirty the model:

model = bdroot(blk);
dirtyFlag = get_param(model,'Dirty');
c = onCleanup(@()set_param(model,'Dirty',dirtyFlag));


switch action
    
    case 'saveas'
        [filename, pathname] = uiputfile('*');
        if filename
            fname = fullfile(pathname,filename);
            set_param(blk,'Filename',fname);
        end
    case 'dynamic'
        dynamicBlockUpdate(blk);
end
function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);

videoFormat          = get_param(blk, 'VideoFormat');
numInputs            = get_param(blk, 'NumInputs');
inheritBits          = get_param(blk, 'InheritBits');

numInMaskParam       = blkMask.getParameter('NumInputs');
bsFormatMaskParam    = blkMask.getParameter('BitStreamFormat');
inheritBitsMaskParam = blkMask.getParameter('InheritBits');
component1           = blkMask.getParameter('Component1');
bits1                = blkMask.getParameter('Bits1');
component2           = blkMask.getParameter('Component2');
bits2                = blkMask.getParameter('Bits2');
component3           = blkMask.getParameter('Component3');
bits3                = blkMask.getParameter('Bits3');
component4           = blkMask.getParameter('Component4');
bits4                = blkMask.getParameter('Bits4');
componentOrder       = blkMask.getParameter('ComponentOrder');
interlaced           = blkMask.getParameter('Interlaced');
signedInput          = blkMask.getParameter('SignedInput');
fileEndian           = blkMask.getParameter('FileEndian');
fourcc               = blkMask.getParameter('FOURCC');
fourCharacterLink    = blkMask.getDialogControl('fourCharacterLink');
component1.Visible        = 'off';
bits1.Visible             = 'off';
component2.Visible        = 'off';
bits2.Visible             = 'off';
component3.Visible        = 'off';
bits3.Visible             = 'off';
component4.Visible        = 'off';
bits4.Visible             = 'off';
if strcmp(videoFormat,'Four character codes')
    fourCharacterLink.Visible   = 'on';
    fourcc.Visible              = 'on';
    bsFormatMaskParam.Visible   = 'off';
    numInMaskParam.Visible      = 'off';
    signedInput.Visible         = 'off';
    fileEndian.Visible          = 'off';
    componentOrder.Visible      = 'off';
    interlaced.Visible          = 'off';
    inheritBitsMaskParam.Visible= 'off';
else
    fourCharacterLink.Visible   = 'off';
    fourcc.Visible              = 'off';
    bsFormatMaskParam.Visible   = 'on';
    numInMaskParam.Visible      = 'on';
    signedInput.Visible         = 'on';
    fileEndian.Visible          = 'on';
    componentOrder.Visible      = 'on';
    interlaced.Visible          = 'on';
    inheritBitsMaskParam.Visible= 'on';
    if strcmp(inheritBits,'off')
        if strcmp(numInputs,'1')
            component1.Visible        = 'on';
            bits1.Visible             = 'on';
        elseif strcmp(numInputs,'2')
            component1.Visible        = 'on';
            bits1.Visible             = 'on';
            component2.Visible        = 'on';
            bits2.Visible             = 'on';
        elseif strcmp(numInputs,'3')
            component1.Visible        = 'on';
            bits1.Visible             = 'on';
            component2.Visible        = 'on';
            bits2.Visible             = 'on';
            component3.Visible        = 'on';
            bits3.Visible             = 'on';
        else
            component1.Visible        = 'on';
            bits1.Visible             = 'on';
            component2.Visible        = 'on';
            bits2.Visible             = 'on';
            component3.Visible        = 'on';
            bits3.Visible             = 'on';
            component4.Visible        = 'on';
            bits4.Visible             = 'on';
        end
    else
        if strcmp(numInputs,'1')
            component1.Visible        = 'on';
            bits1.Visible             = 'off';
        elseif strcmp(numInputs,'2')
            component1.Visible        = 'on';
            bits1.Visible             = 'off';
            component2.Visible        = 'on';
            bits2.Visible             = 'off';
        elseif strcmp(numInputs,'3')
            component1.Visible        = 'on';
            bits1.Visible             = 'off';
            component2.Visible        = 'on';
            bits2.Visible             = 'off';
            component3.Visible        = 'on';
            bits3.Visible             = 'off';
        else
            component1.Visible        = 'on';
            bits1.Visible             = 'off';
            component2.Visible        = 'on';
            bits2.Visible             = 'off';
            component3.Visible        = 'on';
            bits3.Visible             = 'off';
            component4.Visible        = 'on';
            bits4.Visible             = 'off';
        end
    end
end
    
    
