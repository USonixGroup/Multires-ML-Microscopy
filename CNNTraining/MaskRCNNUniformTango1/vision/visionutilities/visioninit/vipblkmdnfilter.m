function [ports,dtInfo] = vipblkmdnfilter
% VIPBLK2DFIRFILTER Mask dynamic dialog function for Median Filter block

% Copyright 2003-2019 The MathWorks, Inc.

blk = gcbh;

ports = get_labels(blk);

dtInfo = dspGetFixptDataTypeInfo(gcbh,7);

dynamicBlockUpdate(blk);
try
    nghbood = get_param(blk,'nghbood');
    val = prod(slResolve(nghbood,blk,'expression'));
    isNHoddOdd = (rem(val,2) ~=0);
catch
    isNHoddOdd=0;
end

if isNHoddOdd
    % when nhoodsize=[r c], both r,c are odd (i.e., r*c is odd)
    % we need to make sure that
    % accumMode=prodOutputMode=outputMode=SAME_AS_INPUT
    dtInfo.accumMode=2;
    dtInfo.prodOutputMode=2;
    dtInfo.outputMode=2;
end



% ----------------------------------------------------------
function ports = get_labels(blk)
padMethod = get_param(blk, 'padType');
outsize  = get_param(blk,'outSize');
ports.type1='input';
ports.port1=1;
ports.txt1='I';

ports.type3='output';
ports.port3=1;
ports.txt3='';

if strcmp(outsize,'Valid')
    ports.type2='';
    ports.port2=1;
    ports.txt2='';
else
    if strcmp(padMethod,'Constant')
        padsrc = get_param(blk, 'padSrc');
        if strcmp(padsrc,'Input port')
            ports.type2='input';
            ports.port2=2;
            ports.txt2='PVal';
        else
            ports.type2='';
            ports.port2=1;
            ports.txt2='';
        end
    else
        ports.type2='';
        ports.port2=1;
        ports.txt2='';
    end
end
function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);

outsize  = get_param(blk, 'outSize');
padType  = get_param(blk, 'padType');
padSrc   = get_param(blk, 'padSrc');
padSrcMaskParam = blkMask.getParameter('padSrc');
padValMaskParam = blkMask.getParameter('padVal');
padTypeMaskParam = blkMask.getParameter('padType');
if strcmp(outsize,'Same as input port I')
    padTypeMaskParam.Visible = 'on';
else
    padTypeMaskParam.Visible = 'off';
end
if strcmp(outsize,'Same as input port I') && strcmp(padType,'Constant')
    padSrcMaskParam.Visible = 'on';
else
    padSrcMaskParam.Visible = 'off';
end
if strcmp(outsize,'Same as input port I') && strcmp(padType,'Constant') && strcmp(padSrc,'Specify via dialog')
    padValMaskParam.Visible = 'on';
else
    padValMaskParam.Visible = 'off';
end
nghbood = blkMask.getParameter('nghbood');
val = evalin('base',nghbood.Value);
val = prod(val(:));
isEven = (rem(val,2) == 0);
descriptionWhenOdd             = blkMask.getDialogControl ('whenBothAreOdd');
fixedPtDataTypesGroupBox       =  blkMask.getDialogControl('fixedPtDataTypesGroupBox');
fixptOperationalParamsGroupBox =  blkMask.getDialogControl('fixptOperationalParamsGroupBox');
floatingPointTrumpRule         = blkMask.getDialogControl('floatingPointTrumpRule');

if (isEven)
    accumMode      = get_param(blk, 'accumMode');
    outputMode     = get_param(blk, 'outputMode');
    accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
    accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
    outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
    outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
    signedColumnLabel             = blkMask.getDialogControl('signedColoumnLabel');
    wordLengthColumnLabel         = blkMask.getDialogControl('wordLegthColoumnlabel');
    fractionLengthColumnLabel     = blkMask.getDialogControl('fracLengthColumnLabel');
    accumSignedText               = blkMask.getDialogControl('accumSignedText');
    outputSignedText              = blkMask.getDialogControl('outputSignedText');
    
    fixedPtDataTypesGroupBox.Visible  = 'on';
    fixptOperationalParamsGroupBox.Visible = 'on';
    floatingPointTrumpRule.Visible    = 'on';
    signedColumnLabel.Visible         = 'on';
    wordLengthColumnLabel.Visible     = 'on';
    fractionLengthColumnLabel.Visible = 'on';
    
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
    
    if strcmp(accumMode, 'Same as input') && strcmp(outputMode, 'Same as input')
        signedColumnLabel.Visible         = 'off';
        wordLengthColumnLabel.Visible     = 'off';
        fractionLengthColumnLabel.Visible = 'off';     
    end
    
    if (strcmp(accumMode, 'Same as input'))...
            && (strcmp(outputMode, 'Same as input'))
        fractionLengthColumnLabel.Visible = 'off';
    end
    descriptionWhenOdd.Visible           = 'off';
else
    descriptionWhenOdd.Visible             = 'on';
    fixedPtDataTypesGroupBox.Visible       = 'off';
    fixptOperationalParamsGroupBox.Visible = 'off';
    floatingPointTrumpRule.Visible         = 'off';
end
% end of vipblk2dfirfilter.m
