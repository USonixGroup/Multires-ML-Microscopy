function [b] = vipblktranslate(varargin)
% MMBLKROTATE Mask dynamic dialog function for Translation block

% Copyright 2003-2019 The MathWorks, Inc.

if nargin==0
    action = 'dynamic';   % mask callback
else
    action = 'icon';
end

blk = gcbh;
switch action
    case 'icon'
        ports.icon = 'Translate';
        transsrc = get_param(blk, 'src_trans');
        if strcmp(transsrc,'Input port')
            ports.port1=1;
            ports.txt1='Image';
            ports.port2=2;
            ports.txt2='Offset';
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
        b = ports;
        
    case 'dynamic'
        dynamicBlockUpdate(blk);
end

function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);
offsetSource = get_param(blk, 'src_trans');
interpMethod = get_param(blk, 'interpMethod');
offset       = blkMask.getParameter('translation');
maxOffset    = blkMask.getParameter('maxOffset');
% =====================================================
% Fixpt tab visibility
% =====================================================
firstCoeffMode = get_param(blk, 'firstCoeffMode');
accumMode      = get_param(blk, 'accumMode');
outputMode     = get_param(blk, 'outputMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
firstCoeffModeMaskParam       = blkMask.getParameter('firstCoeffMode');
accumModeMaskParam            = blkMask.getParameter('accumMode');
outputModeMaskParam           = blkMask.getParameter('outputMode');
prodOutputModeMaskParam       = blkMask.getParameter('prodOutputMode');
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
prodOutputLabel           = blkMask.getDialogControl('prodOutputLabel');
accumLabel                = blkMask.getDialogControl('accumLabel');
outputLabel                = blkMask.getDialogControl('outputLabel');

signedColumnLabel.Visible          = 'on';
wordLengthColumnLabel.Visible      = 'on';
fractionLengthColumnLabel.Visible  = 'on';


if strcmp(offsetSource,'Input port')
    maxOffset.Visible = 'on';
    offset.Visible    = 'off';
    firstCoeffModeMaskParam.Visible       = 'off';
    firstCoeffLabel.Visible               = 'off';
    firstCoeffSignedText.Visible          = 'off';
    firstCoeffWordLengthMaskParam.Visible = 'off';
    firstCoeffFracLengthMaskParam.Visible = 'off';
else
    
    maxOffset.Visible = 'off';
    offset.Visible    = 'on';
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

if strcmp(interpMethod,'Nearest neighbor')
    
    accumModeMaskParam.Visible       = 'off';
    prodOutputModeMaskParam.Visible  = 'off';
    outputModeMaskParam.Visible      = 'on';
    prodOutputLabel.Visible          = 'off';
    accumLabel.Visible               = 'off';
    outputLabel.Visible              = 'on';
    prodOutputSignedText.Visible          = 'off';
    prodOutputWordLengthMaskParam.Visible = 'off';
    prodOutputFracLengthMaskParam.Visible = 'off';
    accumSignedText.Visible          = 'off';
    accumWordLengthMaskParam.Visible = 'off';
    accumFracLengthMaskParam.Visible = 'off';
    if strcmp(outputMode, 'Binary point scaling')
        outputSignedText.Visible          = 'on';
        outputWordLengthMaskParam.Visible = 'on';
        outputFracLengthMaskParam.Visible = 'on';

    else
        outputSignedText.Visible = 'off';
        outputWordLengthMaskParam.Visible = 'off';
        outputFracLengthMaskParam.Visible = 'off';
        
    end
else
    
    accumModeMaskParam.Visible       = 'on';
    prodOutputModeMaskParam.Visible  = 'on';
    outputModeMaskParam.Visible      = 'on';
    prodOutputLabel.Visible          = 'on';
    accumLabel.Visible               = 'on';
    outputLabel.Visible              = 'on';
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
if strcmp(interpMethod,'Nearest neighbor') && strcmp(offsetSource,'Specify via dialog')
    if (strcmp(outputMode, 'Same as first input')) && ...
            (strcmp(firstCoeffMode, 'Specify word length'))
        fractionLengthColumnLabel.Visible = 'off';
    end
    if strcmp(outputMode, 'Same as first input') && (strcmp(firstCoeffMode, 'Same word length as input'))
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    end
elseif strcmp(interpMethod,'Nearest neighbor') && strcmp(offsetSource,'Input port')
    if strcmp(outputMode, 'Same as first input')
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    end
end

% end of vipblktranslate.m
