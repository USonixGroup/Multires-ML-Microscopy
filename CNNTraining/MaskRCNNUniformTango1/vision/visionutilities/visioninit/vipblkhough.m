function [si, so, dtInfo] = vipblkhough(action)
% VIPBLKHOUGH Mask dynamic dialog function for Hough Transform BLock

% Copyright 1995-2019 The MathWorks, Inc.
if nargin==0, action = 'dynamic'; end
blkh = gcbh;
switch action
    case 'icon'
        [si, so] = getPortLabels(blkh);
        dtInfo = dspGetFixptDataTypeInfo(blkh,127);
    otherwise
        dynamicBlockUpdate(blkh)
end

% -----------------------------------------------
function [si, so] = getPortLabels(blkh)

si(1).port = 1;si(1).txt='BW';
so(1).port = 1;so(1).txt='Hough';

HasThetaRhoOutport =  strcmp(get_param(blkh,'out_theta_rho'),'on');
if HasThetaRhoOutport
    so(2).port = 2;so(2).txt='Theta';
    so(3).port = 3;so(3).txt='Rho';
else
    so(2).port = 1;so(2).txt='Hough';
    so(3).port = 1;so(3).txt='Hough';
end
% -----------------------------------------------
function dynamicBlockUpdate(blk)
blkMask = Simulink.Mask.get(gcb);
outputDataTypeMode = get_param(blk, 'outdtmode');
noDataTypesCase    = blkMask.getDialogControl('noDataTypesCase');
fixedptOpParams    = blkMask.getDialogControl('fixedptOpParams');
fixedptDataTypes   = blkMask.getDialogControl('fixedptDataTypes');
% =====================================================
% Fixpt tab visibility
% =====================================================
firstCoeffMode      = get_param(blk, 'firstCoeffMode');
secondCoeffMode     = get_param(blk, 'secondCoeffMode');
accumMode           = get_param(blk, 'accumMode');
outputMode          = get_param(blk, 'outputMode');
prodOutputMode      = get_param(blk, 'prodOutputMode');
memoryMode          = get_param(blk, 'memoryMode');
firstCoeffWordLengthMaskParam      = blkMask.getParameter('firstCoeffWordLength');
firstCoeffFracLengthMaskParam      = blkMask.getParameter('firstCoeffFracLength');
secondCoeffWordLengthMaskParam     = blkMask.getParameter('secondCoeffWordLength');
secondCoeffFracLengthMaskParam     = blkMask.getParameter('secondCoeffFracLength');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
memoryWordLengthMaskParam     = blkMask.getParameter('memoryWordLength');
memoryFracLengthMaskParam     = blkMask.getParameter('memoryFracLength');
accumSignedText               = blkMask.getDialogControl('accumSignedText');
outputSignedText              = blkMask.getDialogControl('outputSignedText');
prodOutputSignedText       = blkMask.getDialogControl('prodOutputSignedText');
memorySignedText           = blkMask.getDialogControl('memorySignedText');
secondCoeffSignedText      = blkMask.getDialogControl('secondCoeffSignedText');
firstCoeffSignedText       = blkMask.getDialogControl('firstCoeffSignedText');
dataTypesDescription       = blkMask.getDialogControl('dataTypesDescription');
fractionLengthColoumnLabel = blkMask.getDialogControl('fracLengthColumnLabel');
fractionLengthColoumnLabel.Visible = 'on';

if strcmp(outputDataTypeMode, 'Specify via Fixed-point tab')
    noDataTypesCase.Visible      = 'off';
    fixedptOpParams.Visible      = 'on';
    fixedptDataTypes.Visible     = 'on';
    dataTypesDescription.Visible = 'on';
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
        outputSignedText.Visible          = 'on';
        outputWordLengthMaskParam.Visible = 'on';
        outputFracLengthMaskParam.Visible = 'off';
    end
    if strcmp(prodOutputMode, 'Binary point scaling')
        prodOutputSignedText.Visible          = 'on';
        prodOutputWordLengthMaskParam.Visible = 'on';
        prodOutputFracLengthMaskParam.Visible = 'on';
    else
        prodOutputSignedText.Visible          = 'on';
        prodOutputWordLengthMaskParam.Visible = 'on';
        prodOutputFracLengthMaskParam.Visible = 'off';
    end
    if strcmp(memoryMode, 'Binary point scaling')
        memorySignedText.Visible          = 'on';
        memoryWordLengthMaskParam.Visible = 'on';
        memoryFracLengthMaskParam.Visible = 'on';
    else
        memorySignedText.Visible          = 'on';
        memoryWordLengthMaskParam.Visible = 'on';
        memoryFracLengthMaskParam.Visible = 'off';
    end
    if strcmp(firstCoeffMode, 'Binary point scaling')
        firstCoeffSignedText.Visible          = 'on';
        firstCoeffWordLengthMaskParam.Visible = 'on';
        firstCoeffFracLengthMaskParam.Visible = 'on';
    else
        firstCoeffSignedText.Visible          = 'on';
        firstCoeffWordLengthMaskParam.Visible = 'on';
        firstCoeffFracLengthMaskParam.Visible = 'off';
    end
    
    if strcmp(secondCoeffMode, 'Binary point scaling')
        secondCoeffSignedText.Visible          = 'on';
        secondCoeffWordLengthMaskParam.Visible = 'on';
        secondCoeffFracLengthMaskParam.Visible = 'on';
    else
        secondCoeffSignedText.Visible          = 'on';
        secondCoeffWordLengthMaskParam.Visible = 'on';
        secondCoeffFracLengthMaskParam.Visible = 'off';
    end
    
elseif  strcmp(outputDataTypeMode, 'single')|| strcmp(outputDataTypeMode, 'double')
    fixedptOpParams.Visible = 'off';
    fixedptDataTypes.Visible = 'off';
    noDataTypesCase.Visible = 'on';
    dataTypesDescription.Visible = 'off';
end

% [EOF] vipblkhough.m
