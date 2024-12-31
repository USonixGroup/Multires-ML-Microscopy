function vipblkautothreshold
% VIPBLKAUTOTHRESHOLD Mask dynamic dialog function for AUTO THRESHOLD block

% Copyright 2018-2019 The MathWorks, Inc.

blk = gcbh;   % Cache handle to block

blkMask = Simulink.Mask.get(gcb);

effMetricOut     = get_param(blk, 'effMetricOut');
userDefinedRange = get_param(blk, 'userDefinedRange');
scaleThreshold   = get_param(blk, 'scaleThreshold');
umin        = blkMask.getParameter('umin');
umax        = blkMask.getParameter('umax');
outOfRngOpt = blkMask.getParameter('outOfRngOpt');
scaleFactor = blkMask.getParameter('scaleFactor');

if strcmp(userDefinedRange,'on')
    umin.Visible       = 'on';
    umax.Visible       = 'on';
    outOfRngOpt.Visible= 'on';
else
    umin.Visible       = 'off';
    umax.Visible       = 'off';
    outOfRngOpt.Visible= 'off';
end

if strcmp(scaleThreshold,'on')
    scaleFactor.Visible = 'on';
else
    scaleFactor.Visible = 'off';
end

% =====================================================
% Fixpt tab visibility
% =====================================================

P1Mode  = get_param(blk, 'P1Mode');
A1Mode  = get_param(blk, 'A1Mode');
P2Mode  = get_param(blk, 'P2Mode');
A2Mode  = get_param(blk, 'A2Mode');
P3Mode  = get_param(blk, 'P3Mode');
A3Mode  = get_param(blk, 'A3Mode');
P4Mode  = get_param(blk, 'P4Mode');
A4Mode  = get_param(blk, 'A4Mode');
Q1Mode  = get_param(blk, 'Q1Mode');
EMMode  = get_param(blk, 'EMMode');
A3ModeMaskParam  = blkMask.getParameter('A3Mode');
EMModeMaskParam  = blkMask.getParameter('EMMode');
P1WordLength     = blkMask.getParameter('P1WordLength');
P1FracLength     = blkMask.getParameter('P1FracLength');
A1WordLength     = blkMask.getParameter('A1WordLength');
A1FracLength     = blkMask.getParameter('A1FracLength');
P2WordLength     = blkMask.getParameter('P2WordLength');
P2FracLength     = blkMask.getParameter('P2FracLength');
A2WordLength     = blkMask.getParameter('A2WordLength');
A2FracLength     = blkMask.getParameter('A2FracLength');
P3WordLength     = blkMask.getParameter('P3WordLength');
P3FracLength     = blkMask.getParameter('P3FracLength');
A3WordLength     = blkMask.getParameter('A3WordLength');
A3FracLength     = blkMask.getParameter('A3FracLength');
P4WordLength     = blkMask.getParameter('P4WordLength');
P4FracLength     = blkMask.getParameter('P4FracLength');
A4WordLength     = blkMask.getParameter('A4WordLength');
A4FracLength     = blkMask.getParameter('A4FracLength');
Q1WordLength     = blkMask.getParameter('Q1WordLength');
Q1FracLength     = blkMask.getParameter('Q1FracLength');
EMWordLength     = blkMask.getParameter('EMWordLength');
EMFracLength     = blkMask.getParameter('EMFracLength');

P1SignedText            = blkMask.getDialogControl('P1SignedText');
A1SignedText            = blkMask.getDialogControl('A1SignedText');
P2SignedText            = blkMask.getDialogControl('P2SignedText');
A2SignedText            = blkMask.getDialogControl('A2SignedText');
P3SignedText            = blkMask.getDialogControl('P3SignedText');
A3SignedText            = blkMask.getDialogControl('A3SignedText');
P4SignedText            = blkMask.getDialogControl('P4SignedText');
A4SignedText            = blkMask.getDialogControl('A4SignedText');
Q1SignedText            = blkMask.getDialogControl('Q1SignedText');
EMSignedText            = blkMask.getDialogControl('EMSignedText');
EMLabel                  = blkMask.getDialogControl('EMLabel');
A3Label                  = blkMask.getDialogControl('A3Label');
signedColumnLabel        = blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel    = blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel= blkMask.getDialogControl('fractionLengthColumnLabel');
signedColumnLabel.Visible          = 'on';
wordLengthColumnLabel.Visible      = 'on';
fractionLengthColumnLabel.Visible  = 'on';

if strcmp(P1Mode, 'Binary point scaling')
    P1SignedText.Visible = 'on';
    P1WordLength.Visible = 'on';
    P1FracLength.Visible = 'on';
elseif strcmp(P1Mode, 'Slope and bias scaling')
    P1SignedText.Visible = 'on';
    P1WordLength.Visible = 'on';
    P1FracLength.Visible = 'off';
else
    P1SignedText.Visible = 'on';
    P1WordLength.Visible = 'on';
    P1FracLength.Visible = 'off';
end
if strcmp(P2Mode, 'Binary point scaling')
    P2SignedText.Visible = 'on';
    P2WordLength.Visible = 'on';
    P2FracLength.Visible = 'on';
else
    P2SignedText.Visible = 'on';
    P2WordLength.Visible = 'on';
    P2FracLength.Visible = 'off';
end
if strcmp(P3Mode, 'Binary point scaling')
    P3SignedText.Visible = 'on';
    P3WordLength.Visible = 'on';
    P3FracLength.Visible = 'on';
else
    P3SignedText.Visible = 'on';
    P3WordLength.Visible = 'on';
    P3FracLength.Visible = 'off';
end
if strcmp(P4Mode, 'Binary point scaling')
    P4SignedText.Visible = 'on';
    P4WordLength.Visible = 'on';
    P4FracLength.Visible = 'on';
else
    P4SignedText.Visible = 'off';
    P4WordLength.Visible = 'off';
    P4FracLength.Visible = 'off';
end
if strcmp(A1Mode, 'Binary point scaling')
    A1SignedText.Visible = 'on';
    A1WordLength.Visible = 'on';
    A1FracLength.Visible = 'on';
elseif strcmp(A1Mode, 'Specify word length')
    A1SignedText.Visible = 'on';
    A1WordLength.Visible = 'on';
    A1FracLength.Visible = 'off';
else
    A1SignedText.Visible = 'off';
    A1WordLength.Visible = 'off';
    A1FracLength.Visible = 'off';
end
if strcmp(A2Mode, 'Binary point scaling')
    A2SignedText.Visible = 'on';
    A2WordLength.Visible = 'on';
    A2FracLength.Visible = 'on';
elseif strcmp(A2Mode, 'Specify word length')
    A2SignedText.Visible = 'on';
    A2WordLength.Visible = 'on';
    A2FracLength.Visible = 'off';
else
    A2SignedText.Visible = 'off';
    A2WordLength.Visible = 'off';
    A2FracLength.Visible = 'off';
end

if strcmp(A4Mode, 'Binary point scaling')
    A4SignedText.Visible = 'on';
    A4WordLength.Visible = 'on';
    A4FracLength.Visible = 'on';
else
    A4SignedText.Visible = 'off';
    A4WordLength.Visible = 'off';
    A4FracLength.Visible = 'off';
end
if strcmp(Q1Mode, 'Binary point scaling')
    Q1SignedText.Visible = 'on';
    Q1WordLength.Visible = 'on';
    Q1FracLength.Visible = 'on';
else
    Q1SignedText.Visible = 'on';
    Q1WordLength.Visible = 'on';
    Q1FracLength.Visible = 'off';
end

if strcmp(effMetricOut,'on')
    EMLabel.Visible         = 'on';
    A3Label.Visible         = 'on';
    A3ModeMaskParam.Visible = 'on';
    EMModeMaskParam.Visible = 'on';
    if strcmp(EMMode, 'Binary point scaling')
        EMSignedText.Visible = 'on';
        EMWordLength.Visible = 'on';
        EMFracLength.Visible = 'on';
    else
        EMSignedText.Visible = 'on';
        EMWordLength.Visible = 'on';
        EMFracLength.Visible = 'off';

    end
    if strcmp(A3Mode, 'Binary point scaling')
        A3SignedText.Visible = 'on';
        A3WordLength.Visible = 'on';
        A3FracLength.Visible = 'on';
    elseif strcmp(A3Mode, 'Specify word length')
        A3SignedText.Visible = 'on';
        A3WordLength.Visible = 'on';
        A3FracLength.Visible = 'off';
    else
        A3SignedText.Visible = 'off';
        A3WordLength.Visible = 'off';
        A3FracLength.Visible = 'off';
    end
    
    if (~strcmp(P1Mode, 'Binary point scaling')) && (~strcmp(P2Mode, 'Binary point scaling')) && (~strcmp(P3Mode, 'Binary point scaling'))...
            && (~strcmp(P4Mode, 'Binarypoint scaling')) && (~strcmp(Q1Mode, 'Binary point scaling')) && (~strcmp(EMMode, 'Binary point scaling'))...
            && (~strcmp(A1Mode, 'Binary point scaling')) && (~strcmp(A2Mode, 'Binary point scaling')) && (~strcmp(A3Mode, 'Binary point scaling'))...
            && (~strcmp(A4Mode, 'Binary point scaling'))
        
        fractionLengthColumnLabel.Visible = 'off';
    end
else
    A3ModeMaskParam.Visible = 'off';
    A3SignedText.Visible    = 'off';
    A3WordLength.Visible    = 'off';
    A3FracLength.Visible    = 'off';
    EMModeMaskParam.Visible = 'off';
    EMSignedText.Visible    = 'off';
    EMWordLength.Visible    = 'off';
    EMFracLength.Visible    = 'off';
    EMLabel.Visible         = 'off';
    A3Label.Visible         = 'off';
    if (~strcmp(P1Mode, 'Binary point scaling')) && (~strcmp(P2Mode, 'Binary point scaling')) && (~strcmp(P3Mode, 'Binary point scaling'))...
            && (~strcmp(P4Mode, 'Binary point scaling')) && (~strcmp(Q1Mode, 'Binary point scaling')) ...
            && (~strcmp(A1Mode, 'Binary point scaling')) && (~strcmp(A2Mode, 'Binary point scaling')) ...
            && (~strcmp(A4Mode, 'Binary point scaling'))
        
        fractionLengthColumnLabel.Visible = 'off';
    end
end
