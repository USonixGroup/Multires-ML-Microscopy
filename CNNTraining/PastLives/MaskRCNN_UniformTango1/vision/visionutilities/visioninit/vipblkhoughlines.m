function vipblkhoughlines(action)
% VIPBLKHOUGH Mask dynamic dialog function for Hough Lines BLock

% Copyright 2018-2020 The MathWorks, Inc.

blk = gcbh;
switch action
    case 'dynamic'
        dynamicBlockUpdate(blk)
end


function dynamicBlockUpdate(blk)
blkMask = Simulink.Mask.get(gcb);
sineCompMethod = get_param(blk, 'sineCompMethod');
thetaRes    = blkMask.getParameter('thetaRes');
if strcmp(sineCompMethod, 'Trigonometric function')
    thetaRes.Visible = 'off';
else
    thetaRes.Visible = 'on';
end
% =====================================================
% Fixpt tab visibility
% =====================================================
accumMode           = get_param(blk, 'accumMode');
prodOutputMode      = get_param(blk, 'prodOutputMode');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
accumSignedText               = blkMask.getDialogControl('accumSignedText');
prodOutputModeSignedText      = blkMask.getDialogControl('prodOutputSignedText');
firstCoeffFracLengthMaskParam = blkMask.getDialogControl('firstCoeffFracLengthMask');
fractionLengthColumnLabel     = blkMask.getDialogControl('fracLengthColumnLabel');
firstCoeffFracLengthMaskParam.Visible = 'off';
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

fractionLengthColumnLabel.Visible     = 'on';
if ~(strcmp(accumMode, 'Binary point scaling') || strcmp(prodOutputMode, 'Binary point scaling'))
    fractionLengthColumnLabel.Visible       = 'off';
end
