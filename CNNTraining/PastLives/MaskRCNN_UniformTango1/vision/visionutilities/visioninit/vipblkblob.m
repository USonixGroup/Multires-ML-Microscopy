function vipblkblob

% VIPBLK2DBLOB Mask dynamic dialog function for Blob Analysis block

% Copyright 2018-2019 The MathWorks, Inc.

blk = gcbh;
blkMask = Simulink.Mask.get(gcb);

equivDiameterSq     = get_param(blk, 'equivDiameterSq');
extent              = get_param(blk, 'extent');
perimeter           = get_param(blk, 'perimeter');
statsOutputDataType = get_param(blk, 'outDT');
useMinArea          = get_param(blk, 'useMinArea');
useMaxArea          = get_param(blk, 'useMaxArea');
isFill              = get_param(blk,'isFill');
isOutVarDim         = get_param(blk,'isOutVarDim');
angle               = blkMask.getParameter('angle');
eccentricity        = blkMask.getParameter('eccentricity');
majorAxis           = blkMask.getParameter('majorAxis');
minorAxis           = blkMask.getParameter('minorAxis');
minArea             = blkMask.getParameter('minArea');
maxArea             = blkMask.getParameter('maxArea');
isFillMaskParam     = blkMask.getParameter('isFill');
fillValues          = blkMask.getParameter('fillValues');
note1               = blkMask.getDialogControl('note1');
note2               = blkMask.getDialogControl('note2');

if strcmp(useMinArea, 'on')
    minArea.Visible           = 'on';
else
    minArea.Visible           = 'off';
end

if strcmp(useMaxArea, 'on')
    maxArea.Visible           = 'on';
else
    maxArea.Visible           = 'off';
end

if strcmp(isOutVarDim, 'off')
    isFillMaskParam.Visible   = 'on';
    if strcmp(isFill,'on')
        fillValues.Visible        = 'on';
    else
        fillValues.Visible        = 'off';
    end
else
    isFillMaskParam.Visible   = 'off';
    fillValues.Visible        = 'off';
end

% =====================================================
% Fixpt tab visibility
% =====================================================

firstCoeffMode                 =  get_param(blk, 'firstCoeffMode');
secondCoeffMode                =  get_param(blk, 'secondCoeffMode');
accumMode                      =  get_param(blk, 'accumMode');
outputMode                     =  get_param(blk, 'outputMode');
prodOutputMode                 =  get_param(blk, 'prodOutputMode');
memoryMode                     =  get_param(blk, 'memoryMode');
firstCoeffModeMaskParam        =  blkMask.getParameter('firstCoeffMode');
secondCoeffModeMaskParam       =  blkMask.getParameter('secondCoeffMode');
accumModeModeMaskParam         =  blkMask.getParameter('accumMode');
outputModeModeMaskParam        =  blkMask.getParameter('outputMode');
prodOutputModeMaskParam        =  blkMask.getParameter('prodOutputMode');
memoryModeMaskParam            =  blkMask.getParameter('memoryMode');
firstCoeffWordLengthMaskParam  =  blkMask.getParameter('firstCoeffWordLength');
firstCoeffFracLengthMaskParam  =  blkMask.getParameter('firstCoeffFracLength');
secondCoeffWordLengthMaskParam =  blkMask.getParameter('secondCoeffWordLength');
secondCoeffFracLengthMaskParam =  blkMask.getParameter('secondCoeffFracLength');
accumWordLengthMaskParam       =  blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam       =  blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam      =  blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam      =  blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam  =  blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam  =  blkMask.getParameter('prodOutputFracLength');
memoryWordLengthMaskParam      =  blkMask.getParameter('memoryWordLength');
memoryFracLengthMaskParam      =  blkMask.getParameter('memoryFracLength');
prodOutputLabel                =  blkMask.getDialogControl('prodOutputLabel');
memoryLabel                    =  blkMask.getDialogControl('memoryLabel');
firstCoeffLabel                =  blkMask.getDialogControl('firstCoeffLabel');
secondCoeffLabel               =  blkMask.getDialogControl('secondCoeffLabel');
accumSignedText                =  blkMask.getDialogControl('accumSignedText');
outputSignedText               =  blkMask.getDialogControl('outputSignedText');
prodOutputSignedText           =  blkMask.getDialogControl('prodOutputSignedText');
memorySignedText               =  blkMask.getDialogControl('memorySignedText');
secondCoeffSignedText          =  blkMask.getDialogControl('secondCoeffSignedText');
firstCoeffSignedText           =  blkMask.getDialogControl('firstCoeffSignedText');
dataTypesDescription           =  blkMask.getDialogControl('dataTypesDescription');
noDataTypes                    =  blkMask.getDialogControl('noDataTypes');
fractionLengthColumnLabel      =  blkMask.getDialogControl('fracLengthColumnLabel');
fixPtOpParams                  =  blkMask.getDialogControl('fixPtOpParams');
fixPtDataTypes                 =  blkMask.getDialogControl('fixPtDataTypes');
fractionLengthColumnLabel.Visible  = 'on';

if strcmp(statsOutputDataType,'Specify via Data Types tab')
    note1.Visible                    = 'off';
    note2.Visible                    = 'on';
    dataTypesDescription.Visible     = 'on';
    noDataTypes.Visible              = 'off';
    fixPtOpParams.Visible            = 'on';
    fixPtDataTypes.Visible           = 'on';
    firstCoeffModeMaskParam.Visible  = 'off';
    secondCoeffModeMaskParam.Visible = 'off';
    accumModeModeMaskParam.Visible   = 'on';
    outputModeModeMaskParam.Visible  = 'on';
    prodOutputModeMaskParam.Visible  = 'off';
    memoryModeMaskParam.Visible      = 'off';
    angle.Visible                    = 'off';
    eccentricity.Visible             = 'off';
    majorAxis.Visible                = 'off';
    minorAxis.Visible                = 'off';
    prodOutputSignedText.Visible           = 'off';
    prodOutputWordLengthMaskParam.Visible  = 'off';
    prodOutputFracLengthMaskParam.Visible  = 'off';
    memorySignedText.Visible               = 'off';
    memoryWordLengthMaskParam.Visible      = 'off';
    memoryFracLengthMaskParam.Visible      = 'off';
    firstCoeffSignedText.Visible           = 'off';
    firstCoeffWordLengthMaskParam.Visible  = 'off';
    firstCoeffFracLengthMaskParam.Visible  = 'off';
    secondCoeffSignedText.Visible          = 'off';
    secondCoeffWordLengthMaskParam.Visible = 'off';
    secondCoeffFracLengthMaskParam.Visible = 'off';
    prodOutputLabel.Visible                = 'off';
    memoryLabel.Visible                    = 'off';
    firstCoeffLabel.Visible                = 'off';
    secondCoeffLabel.Visible               = 'off';
    
    if strcmp(accumMode, 'Binary point scaling')
        accumSignedText.Visible          = 'on';
        accumWordLengthMaskParam.Visible = 'on';
        accumFracLengthMaskParam.Visible = 'on';
    else
        accumSignedText.Visible          = 'on';
        accumWordLengthMaskParam.Visible = 'on';
        accumFracLengthMaskParam.Visible = 'off';
    end
    if strcmp(outputMode, 'Binary point scaling')
        outputSignedText.Visible          = 'on';
        outputWordLengthMaskParam.Visible = 'on';
        outputFracLengthMaskParam.Visible = 'on';
    else
        outputSignedText.Visible          = 'off';
        outputWordLengthMaskParam.Visible = 'off';
        outputFracLengthMaskParam.Visible = 'off';
    end
    if strcmp(equivDiameterSq,'on')
        prodOutputLabel.Visible          = 'on';
        memoryLabel.Visible              = 'on';
        prodOutputModeMaskParam.Visible  = 'on';
        memoryModeMaskParam.Visible      = 'on';
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
            memorySignedText.Visible          = 'off';
            memoryWordLengthMaskParam.Visible = 'off';
            memoryFracLengthMaskParam.Visible = 'off';
        end
        
    end
    if strcmp(extent,'on')
        firstCoeffModeMaskParam.Visible  = 'on';
        firstCoeffLabel.Visible          = 'on';
        if strcmp(firstCoeffMode, 'Binary point scaling')
            firstCoeffSignedText.Visible          = 'on';
            firstCoeffWordLengthMaskParam.Visible = 'on';
            firstCoeffFracLengthMaskParam.Visible = 'on';
        else
            firstCoeffSignedText.Visible          = 'off';
            firstCoeffWordLengthMaskParam.Visible = 'off';
            firstCoeffFracLengthMaskParam.Visible = 'off';
        end
    end
    if strcmp(perimeter,'on')
        secondCoeffModeMaskParam.Visible       = 'on';
        secondCoeffLabel.Visible               = 'on';
        if strcmp(secondCoeffMode, 'Binary point scaling')
            secondCoeffSignedText.Visible          = 'on';
            secondCoeffWordLengthMaskParam.Visible = 'on';
            secondCoeffFracLengthMaskParam.Visible = 'on';
        else
            secondCoeffSignedText.Visible          = 'off';
            secondCoeffWordLengthMaskParam.Visible = 'off';
            secondCoeffFracLengthMaskParam.Visible = 'off';
        end
    end
    if strcmp(equivDiameterSq,'on') && strcmp(extent,'off') && strcmp(perimeter,'off')
        if (~strcmp(outputMode, 'Binary point scaling') && ~strcmp(accumMode, 'Binary point scaling')...
                && ~strcmp(memoryMode,'Binary point scaling') && ~strcmp(prodOutputMode, 'Binary point scaling'))
            fractionLengthColumnLabel.Visible = 'off';
        end
    end
    if strcmp(equivDiameterSq,'off') && strcmp(extent,'on') && strcmp(perimeter,'off')
        if (~strcmp(outputMode, 'Binary point scaling') && ~strcmp(accumMode, 'Binary point scaling')...
                && ~strcmp(firstCoeffMode,'Binary point scaling'))
            fractionLengthColumnLabel.Visible = 'off';
        end
    end
    if strcmp(equivDiameterSq,'off') && strcmp(extent,'off') && strcmp(perimeter,'on')
        if (~strcmp(outputMode, 'Binary point scaling') && ~strcmp(accumMode, 'Binary point scaling')...
                && ~strcmp(secondCoeffMode,'Binary point scaling'))
            fractionLengthColumnLabel.Visible = 'off';
        end
    end
    if strcmp(equivDiameterSq,'on') && strcmp(extent,'on') && strcmp(perimeter,'off')
        if (~strcmp(outputMode, 'Binary point scaling') && ~strcmp(accumMode, 'Binary point scaling')...
                && ~strcmp(memoryMode,'Binary point scaling') && ~strcmp(prodOutputMode, 'Binary point scaling'))...
                && ~strcmp(firstCoeffMode, 'Binary point scaling')
            fractionLengthColumnLabel.Visible = 'off';
        end
    end
    if strcmp(equivDiameterSq,'off') && strcmp(extent,'on') && strcmp(perimeter,'on')
        if (~strcmp(outputMode, 'Binary point scaling') && ~strcmp(accumMode, 'Binary point scaling')...
                && ~strcmp(secondCoeffMode, 'Binary point scaling') && ~strcmp(firstCoeffMode, 'Binary point scaling'))
            fractionLengthColumnLabel.Visible = 'off';
        end
    end
    if strcmp(equivDiameterSq,'on') && strcmp(extent,'off') && strcmp(perimeter,'on')
        if (~strcmp(outputMode, 'Binary point scaling') && ~strcmp(accumMode, 'Binary point scaling')...
                && ~strcmp(memoryMode,'Binary point scaling') && ~strcmp(prodOutputMode, 'Binary point scaling'))...
                && ~strcmp(secondCoeffMode, 'Binary point scaling')
            fractionLengthColumnLabel.Visible = 'off';
        end
    end
    if strcmp(equivDiameterSq,'on') && strcmp(extent,'on') && strcmp(perimeter,'on')
        
        if ~strcmp(outputMode, 'Binary point scaling') && ~strcmp(accumMode, 'Binary point scaling')...
                && ~strcmp(memoryMode,'Binary point scaling') && ~strcmp(prodOutputMode, 'Binary point scaling')...
                && ~strcmp(secondCoeffMode, 'Binary point scaling') && ~strcmp(firstCoeffMode, 'Binary point scaling')
            fractionLengthColumnLabel.Visible = 'off';
        end
        
    end
    if strcmp(equivDiameterSq,'off') && strcmp(extent,'off') && strcmp(perimeter,'off')
        
        if (~strcmp(outputMode, 'Binary point scaling')) && (~strcmp(accumMode, 'Binary point scaling'))
            fractionLengthColumnLabel.Visible = 'off';
        end
        
    end
else
    note1.Visible                    = 'on';
    note2.Visible                    = 'off';
    angle.Visible                    = 'on';
    eccentricity.Visible             = 'on';
    majorAxis.Visible                = 'on';
    minorAxis.Visible                = 'on';
    noDataTypes.Visible              = 'on';
    fixPtOpParams.Visible            = 'off';
    fixPtDataTypes.Visible           = 'off';
    dataTypesDescription.Visible     = 'off';
end