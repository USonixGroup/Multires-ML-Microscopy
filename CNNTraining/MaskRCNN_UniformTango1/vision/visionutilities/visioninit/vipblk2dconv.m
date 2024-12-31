function [dtInfo] = vipblk2dconv(action)
% VIPBLK2DCONV Mask dynamic dialog function for 2D Convolution block

% Copyright 1995-2019 The MathWorks, Inc.

blk = gcbh;   % Cache handle to block

switch action
    
    case 'init'
        dtInfo = dspGetFixptDataTypeInfo(blk,15);
    case 'dynamic'
        dynamicBlockUpdate(blk)
end

%----------------------------------------------------------------

function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);
% =====================================================
% Fixpt tab visibility
% =====================================================

accumMode      = get_param(blk, 'accumMode');
outputMode     = get_param(blk, 'outputMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
signedColumnLabel         = blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel     = blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel = blkMask.getDialogControl('fractionLengthColumnLabel');
accumSignedText           = blkMask.getDialogControl('accumSignedText');
outputSignedText          = blkMask.getDialogControl('outputSignedText');
prodOutputSignedText      = blkMask.getDialogControl('prodOutputSignedText');

signedColumnLabel.Visible          = 'on';
wordLengthColumnLabel.Visible      = 'on';
fractionLengthColumnLabel.Visible  = 'on';

if strcmp(accumMode, 'Binary point scaling')
    accumSignedText.Visible          = 'on';
    accumWordLengthMaskParam.Visible = 'on';
    accumFracLengthMaskParam.Visible = 'on';
else
    accumSignedText.Visible = 'off';
    accumWordLengthMaskParam.Visible = 'off';
    accumFracLengthMaskParam.Visible = 'off';
end


if strcmp(outputMode, 'Binary point scaling')
    outputSignedText.Visible          = 'on';
    outputWordLengthMaskParam.Visible = 'on';
    outputFracLengthMaskParam.Visible = 'on';
else
    outputSignedText.Visible = 'off';
    outputWordLengthMaskParam.Visible = 'off';
    outputFracLengthMaskParam.Visible = 'off';
end


if strcmp(prodOutputMode, 'Binary point scaling')
    prodOutputSignedText.Visible          = 'on';
    prodOutputWordLengthMaskParam.Visible = 'on';
    prodOutputFracLengthMaskParam.Visible = 'on';
else
    prodOutputSignedText.Visible          = 'off';
    prodOutputWordLengthMaskParam.Visible = 'off';
    prodOutputFracLengthMaskParam.Visible = 'off';
end


if strcmp(outputMode, 'Same as first input') && (strcmp(accumMode, 'Same as first input')||strcmp(accumMode, 'Same as product output'))...
        && strcmp(prodOutputMode, 'Same as first input')
    signedColumnLabel.Visible          = 'off';
    wordLengthColumnLabel.Visible      = 'off';
    fractionLengthColumnLabel.Visible  = 'off';
end
if (strcmp(outputMode, 'Same as first input')) && (strcmp(accumMode, 'Same as first input')||strcmp(accumMode, 'Same as product output'))...
        && ( strcmp(prodOutputMode, 'Same as first input'))
    fractionLengthColumnLabel.Visible = 'off';
end
