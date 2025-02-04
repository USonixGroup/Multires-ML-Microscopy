function vipblkgaussianpyramid
% VIPBLKGAUSSIANPYRAMID Mask dynamic dialog function for GAUSSIAN PYRAMID BLock

% Copyright 2018-2019 The MathWorks, Inc.

blk = gcbh;
blkMask = Simulink.Mask.get(gcb);

% =====================================================
% Fixpt tab visibility
% =====================================================
firstCoeffMode = get_param(blk, 'firstCoeffMode');
outputMode     = get_param(blk, 'outputMode');
accumMode      = get_param(blk, 'accumMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
firstCoeffWordLengthMaskParam = blkMask.getParameter('firstCoeffWordLength');
firstCoeffFracLengthMaskParam = blkMask.getParameter('firstCoeffFracLength');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
firstCoeffSignedText          = blkMask.getDialogControl('firstCoeffSignedText');
accumSignedText               = blkMask.getDialogControl('accumSignedText');
prodOutputModeSignedText      = blkMask.getDialogControl('prodOutputSignedText');
outputSignedText              = blkMask.getDialogControl('outputSignedText');
signedColumnLabel             = blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel         = blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel     = blkMask.getDialogControl('fractionLengthColumnLabel');
signedColumnLabel.Visible          = 'on';
wordLengthColumnLabel.Visible      = 'on';
fractionLengthColumnLabel.Visible  = 'on';
if strcmp(accumMode, 'Binary point scaling')
    accumSignedText.Visible               = 'on';
    accumWordLengthMaskParam.Visible      = 'on';
    accumFracLengthMaskParam.Visible      = 'on';
    firstCoeffFracLengthMaskParam.Visible = 'on';
else
    accumSignedText.Visible             = 'off';
    accumWordLengthMaskParam.Visible    = 'off';
    accumFracLengthMaskParam.Visible    = 'off';
end
if strcmp(prodOutputMode, 'Binary point scaling')
    prodOutputModeSignedText.Visible      = 'on';
    prodOutputWordLengthMaskParam.Visible = 'on';
    prodOutputFracLengthMaskParam.Visible = 'on';
    firstCoeffFracLengthMaskParam.Visible = 'on';
else
    prodOutputModeSignedText.Visible      = 'off';
    prodOutputWordLengthMaskParam.Visible = 'off';
    prodOutputFracLengthMaskParam.Visible = 'off';
end
if strcmp(firstCoeffMode, 'Binary point scaling')
    firstCoeffSignedText.Visible          = 'on';
    firstCoeffWordLengthMaskParam.Visible = 'on';
    firstCoeffFracLengthMaskParam.Visible = 'on';
elseif strcmp(firstCoeffMode, 'Specify word length')
    firstCoeffSignedText.Visible          = 'on';
    firstCoeffWordLengthMaskParam.Visible = 'on';
    firstCoeffFracLengthMaskParam.Visible = 'off';
else
    firstCoeffSignedText.Visible = 'off';
    firstCoeffWordLengthMaskParam.Visible = 'off';
    firstCoeffFracLengthMaskParam.Visible = 'off';
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
if strcmp(outputMode, 'Same as input') && (strcmp(accumMode, 'Same as input')||strcmp(accumMode, 'Same as product output'))...
        && strcmp(prodOutputMode, 'Same as input')&& (strcmp(firstCoeffMode, 'Same word length as input'))
    
    signedColumnLabel.Visible          = 'off';
    wordLengthColumnLabel.Visible      = 'off';
    fractionLengthColumnLabel.Visible  = 'off';
end
if(strcmp(outputMode, 'Same as input')) && (strcmp(accumMode, 'Same as input')||strcmp(accumMode, 'Same as product output'))...
        && ( strcmp(prodOutputMode, 'Same as input'))...
        &&(~strcmp(firstCoeffMode, 'Binary point scaling'))
    fractionLengthColumnLabel.Visible = 'off';
end
