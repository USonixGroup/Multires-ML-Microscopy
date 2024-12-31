function vipblkbinaryfilereader(blk, action, varargin)
% binaryfilereader Computer Vision Toolbox Read Binary File  block
% mask helper function

%   Copyright 2018 The MathWorks, Inc.

% This function modifies settings of blocks in the subsystem. This
% should not dirty the model:

model = bdroot(blk);
dirtyFlag = get_param(model,'Dirty');
c = onCleanup(@()set_param(model,'Dirty',dirtyFlag));



switch action
    
    case 'browse'
        % Select file via browse button
        [filename, pathname] = uigetfile('*');
        if filename
            fname = fullfile(pathname,filename);
            set_param(blk,'Filename',fname);
        end
    case 'dynamic'
        dynamicBlockUpdate(blk)
end
function dynamicBlockUpdate(blk)
blkMask = Simulink.Mask.get(blk);

VideoFormat     = get_param(blk, 'VideoFormat');
BitStreamFormat = get_param(blk, 'BitStreamFormat');
NumOutputs      = get_param(blk, 'NumOutputs');
NumOutMaskParam = blkMask.getParameter('NumOutputs');
bsFormatMaskParam = blkMask.getParameter('BitStreamFormat');
YRows           = blkMask.getParameter('YRows');
YCols           = blkMask.getParameter('YCols');
Component1      = blkMask.getParameter('Component1');
Bits1           = blkMask.getParameter('Bits1');
Rows1           = blkMask.getParameter('Rows1');
Cols1           = blkMask.getParameter('Cols1');
Component2      = blkMask.getParameter('Component2');
Bits2           = blkMask.getParameter('Bits2');
Rows2           = blkMask.getParameter('Rows2');
Cols2           = blkMask.getParameter('Cols2');
Component3      = blkMask.getParameter('Component3');
Bits3           = blkMask.getParameter('Bits3');
Rows3           = blkMask.getParameter('Rows3');
Cols3           = blkMask.getParameter('Cols3');
Component4      = blkMask.getParameter('Component4');
Bits4           = blkMask.getParameter('Bits4');
Rows4           = blkMask.getParameter('Rows4');
Cols4           = blkMask.getParameter('Cols4');
SignedOutput    = blkMask.getParameter('SignedOutput');
FileEndian      = blkMask.getParameter('FileEndian');
Interlaced      = blkMask.getParameter('Interlaced');
ComponentOrder  = blkMask.getParameter('ComponentOrder');
fourcc          = blkMask.getParameter('FOURCC');

frameSize         = blkMask.getDialogControl('frameSize');
fourCharacterLink = blkMask.getDialogControl('fourCharacterLink');

frameSize.Visible = 'on';
YRows.Visible     = 'on';
YCols.Visible     = 'on';
Component1.Visible        = 'off';
Bits1.Visible             = 'off';
Rows1.Visible             = 'off';
Cols1.Visible             = 'off';
Component2.Visible        = 'off';
Bits2.Visible             = 'off';
Rows2.Visible             = 'off';
Cols2.Visible             = 'off';
Component3.Visible        = 'off';
Bits3.Visible             = 'off';
Rows3.Visible             = 'off';
Cols3.Visible             = 'off';
Component4.Visible        = 'off';
Bits4.Visible             = 'off';
Rows4.Visible             = 'off';
Cols4.Visible             = 'off';
if strcmp(VideoFormat, 'Four character codes')
    fourCharacterLink.Visible = 'on';
    fourcc.Visible            = 'on';
    bsFormatMaskParam.Visible = 'off';
    NumOutMaskParam.Visible   = 'off';
    SignedOutput.Visible      = 'off';
    FileEndian.Visible        = 'off';
    ComponentOrder.Visible    = 'off';
    Interlaced.Visible        = 'off';
else
    fourCharacterLink.Visible = 'off';
    fourcc.Visible            = 'off';
    bsFormatMaskParam.Visible = 'on';
    NumOutMaskParam.Visible   = 'on';
    SignedOutput.Visible      = 'on';
    FileEndian.Visible        = 'on';
    ComponentOrder.Visible    = 'on';
    Interlaced.Visible        = 'on';
    if strcmp(NumOutputs, '1')
        Component1.Visible        = 'on';
        Bits1.Visible             = 'on';
        Rows1.Visible             = 'on';
        Cols1.Visible             = 'on';
    elseif strcmp(NumOutputs, '2')
        Component1.Visible        = 'on';
        Bits1.Visible             = 'on';
        Rows1.Visible             = 'on';
        Cols1.Visible             = 'on';
        Component2.Visible        = 'on';
        Bits2.Visible             = 'on';
        Rows2.Visible             = 'on';
        Cols2.Visible             = 'on';
    elseif strcmp(NumOutputs, '3')
        Component1.Visible        = 'on';
        Bits1.Visible             = 'on';
        Rows1.Visible             = 'on';
        Cols1.Visible             = 'on';
        Component2.Visible        = 'on';
        Bits2.Visible             = 'on';
        Rows2.Visible             = 'on';
        Cols2.Visible             = 'on';
        Component3.Visible        = 'on';
        Bits3.Visible             = 'on';
        Rows3.Visible             = 'on';
        Cols3.Visible             = 'on';
    elseif strcmp(NumOutputs,'4')
        Component1.Visible        = 'on';
        Bits1.Visible             = 'on';
        Rows1.Visible             = 'on';
        Cols1.Visible             = 'on';
        Component2.Visible        = 'on';
        Bits2.Visible             = 'on';
        Rows2.Visible             = 'on';
        Cols2.Visible             = 'on';
        Component3.Visible        = 'on';
        Bits3.Visible             = 'on';
        Rows3.Visible             = 'on';
        Cols3.Visible             = 'on';
        Component4.Visible        = 'on';
        Bits4.Visible             = 'on';
        Rows4.Visible             = 'on';
        Cols4.Visible             = 'on';
    end
    if strcmp(BitStreamFormat, 'Planar')
        frameSize.Visible = 'off';
        YRows.Visible     = 'off';
        YCols.Visible     = 'off';
    end
        
end
