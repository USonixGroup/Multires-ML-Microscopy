function [b] = vipblkrotate(varargin)
% MMBLKROTATE Mask dynamic dialog function for Rotation/Translation block

% Copyright 2003-2019 The MathWorks, Inc.

if nargin==0
    action = 'dynamic';   % mask callback
else
    action = 'icon';
end
blk = gcbh;

switch action
    case 'icon'
        b = get_labels(blk);
    case 'dynamic'
        dynamicBlockUpdate(blk);
end


% ----------------------------------------------------------
function ports = get_labels(blk)
ports.icon = 'Rotate';
anglesrc = get_param(blk, 'src_angle');
if strcmp(anglesrc,'Input port')
    ports.port1=1;
    ports.txt1='Image';
    ports.port2=2;
    ports.txt2='Angle';
    ports.port3=2;
    ports.txt3='';
else
    ports.port1=1;
    ports.txt1='';
    ports.port2=1;
    ports.txt2='';
    ports.port3=1;
    ports.txt3='';
end

% ------------------------------------------------------------------------
function dynamicBlockUpdate(blk)
blkMask = Simulink.Mask.get(gcb);
AngleSource = get_param(blk, 'src_angle');
angle       = blkMask.getParameter('angle');
maxAngle    = blkMask.getParameter('maxAngle');
dispImage   = blkMask.getParameter('dispImage');
% =====================================================
% Fixpt tab visibility
% =====================================================
firstCoeffMode = get_param(blk, 'firstCoeffMode');
accumMode      = get_param(blk, 'accumMode');
outputMode     = get_param(blk, 'outputMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
firstCoeffModeMaskParam       = blkMask.getParameter('firstCoeffMode');
firstCoeffWordLengthMaskParam = blkMask.getParameter('firstCoeffWordLength');
firstCoeffFracLengthMaskParam = blkMask.getParameter('firstCoeffFracLength');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
signedColumnLabel         = blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel     = blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel = blkMask.getDialogControl('fractionLengthColumnLabel');
firstCoeffSignedText      = blkMask.getDialogControl('firstCoeffSignedText');
accumSignedText           = blkMask.getDialogControl('accumSignedText');
outputSignedText          = blkMask.getDialogControl('outputSignedText');
prodOutputSignedText      = blkMask.getDialogControl('prodOutputSignedText');
firstCoeffLabel           = blkMask.getDialogControl('firstCoeffLabel');

signedColumnLabel.Visible          = 'on';
wordLengthColumnLabel.Visible      = 'on';
fractionLengthColumnLabel.Visible  = 'on';

if strcmp(AngleSource,'Input port')
    maxAngle.Visible = 'on';
    angle.Visible    = 'off';
    dispImage.Visible= 'on';
    firstCoeffModeMaskParam.Visible       = 'off';
    firstCoeffLabel.Visible               = 'off';
    firstCoeffSignedText.Visible          = 'off';
    firstCoeffWordLengthMaskParam.Visible = 'off';
    firstCoeffFracLengthMaskParam.Visible = 'off';
    
else
    dispImage.Visible= 'off';
    maxAngle.Visible = 'off';
    angle.Visible    = 'on';
    firstCoeffModeMaskParam.Visible = 'on';
    firstCoeffLabel.Visible = 'on';
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
end

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
if strcmp(AngleSource,'Input port')
    if strcmp(outputMode, 'Same as first input') && (strcmp(accumMode, 'Same as first input')||strcmp(accumMode, 'Same as product output'))...
            && strcmp(prodOutputMode, 'Same as first input')
        
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    elseif (strcmp(outputMode, 'Same as first input')) && (strcmp(accumMode, 'Same as first input')||strcmp(accumMode, 'Same as product output'))...
            && (strcmp(prodOutputMode, 'Same as first input'))
        fractionLengthColumnLabel.Visible = 'off';
    end
else
    if strcmp(outputMode, 'Same as first input') && (strcmp(accumMode, 'Same as first input')||strcmp(accumMode, 'Same as product output'))...
            && strcmp(prodOutputMode, 'Same as first input')&& (strcmp(firstCoeffMode, 'Same word length as input'))
        
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    end
    if(strcmp(outputMode, 'Same as first input')) && (strcmp(accumMode, 'Same as first input')||strcmp(accumMode, 'Same as product output'))...
            && (strcmp(prodOutputMode, 'Same as first input'))...
            &&(~strcmp(firstCoeffMode, 'Binary point scaling'))
        fractionLengthColumnLabel.Visible = 'off';
    end
end
% end of vipblkrotate.m
