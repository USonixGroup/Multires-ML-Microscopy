function vipblkcddynamic

% VIPBLKCDDYNAMIC is Mask editor dynamic block function for Corner Detection block

% Copyright 2018-2019 The MathWorks, Inc.

blk = gcbh;
blkMask = Simulink.Mask.get(gcb);

method         = get_param(blk, 'method');
outputInUse    = get_param(blk, 'outputInUse');
factor         = blkMask.getParameter('factor');
filter         = blkMask.getParameter('filter');
thrInten       = blkMask.getParameter('thrInten');
thrAngle       = blkMask.getParameter('thrAngle');
maxNum         = blkMask.getParameter('maxNum');
thrMetric      = blkMask.getParameter('thrMetric');
neighborSize   = blkMask.getParameter('neighborSize');
isOutVarDim    = blkMask.getParameter('isOutVarDim');

if strcmp(method,'Local intensity comparison (Rosen & Drummond)')
    thrInten.Visible = 'on';
    thrAngle.Visible = 'on';
    filter.Visible   = 'off';
    factor.Visible   = 'off';
elseif strcmp(method, 'Minimum eigenvalue (Shi & Tomasi)')
    thrInten.Visible = 'off';
    thrAngle.Visible = 'off';
    filter.Visible   = 'on';
    factor.Visible   = 'off';
else
    thrInten.Visible = 'off';
    thrAngle.Visible = 'off';
    filter.Visible   = 'on';
    factor.Visible   = 'on';
end

if strcmp(outputInUse, 'Metric matrix')
    maxNum.Visible       = 'off';
    thrMetric.Visible    = 'off';
    neighborSize.Visible = 'off';
    isOutVarDim.Visible  = 'off';
else
    maxNum.Visible       = 'on';
    thrMetric.Visible    = 'on';
    neighborSize.Visible = 'on';
    isOutVarDim.Visible  = 'on';
end

% =====================================================
% Fixpt tab visibility
% =====================================================

accumMode      = get_param(blk, 'accumMode');
outputMode     = get_param(blk, 'outputMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
firstCoeffMode = get_param(blk, 'firstCoeffMode');
memoryMode     = get_param(blk, 'memoryMode');
firstCoeffModeMaskParam      = blkMask.getParameter('firstCoeffMode');
prodOutputModeMaskParam      = blkMask.getParameter('prodOutputMode');
memoryModeMaskParam           = blkMask.getParameter('memoryMode');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
firstCoeffWordLengthMaskParam = blkMask.getParameter('firstCoeffWordLength');
firstCoeffFracLengthMaskParam = blkMask.getParameter('firstCoeffFracLength');
memoryWordLengthMaskParam     = blkMask.getParameter('memoryWordLength');
memoryFracLengthMaskParam     = blkMask.getParameter('memoryFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');


signedColumnLabel         = blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel     = blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel = blkMask.getDialogControl('fractionLengthColumnLabel');
prodOutputLabel           = blkMask.getDialogControl('prodOutputLabel');
memoryLabel               = blkMask.getDialogControl('memoryLabel');
firstCoeffLabel           = blkMask.getDialogControl('firstCoeffLabel');
accumSignedText           = blkMask.getDialogControl('accumSignedText');
outputSignedText          = blkMask.getDialogControl('outputSignedText');
prodOutputSignedText      = blkMask.getDialogControl('prodOutputSignedText');
firstCoeffSignedText      = blkMask.getDialogControl('firstCoeffSignedText');
memorySignedText          = blkMask.getDialogControl('memorySignedText');

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
    outputSignedText.Visible          = 'off';
    outputWordLengthMaskParam.Visible = 'off';
    outputFracLengthMaskParam.Visible = 'off';
end
if strcmp(method,'Local intensity comparison (Rosen & Drummond)')
    firstCoeffLabel.Visible               = 'off';
    memoryLabel.Visible                   = 'off';
    prodOutputLabel.Visible               = 'off';
    firstCoeffModeMaskParam.Visible       = 'off';
    prodOutputModeMaskParam.Visible       = 'off';
    memoryModeMaskParam.Visible           = 'off';
    prodOutputSignedText.Visible          = 'off';
    prodOutputWordLengthMaskParam.Visible = 'off';
    prodOutputFracLengthMaskParam.Visible = 'off';
    firstCoeffSignedText.Visible          = 'off';
    firstCoeffWordLengthMaskParam.Visible = 'off';
    firstCoeffFracLengthMaskParam.Visible = 'off';
    memorySignedText.Visible              = 'off';
    memoryWordLengthMaskParam.Visible     = 'off';
    memoryFracLengthMaskParam.Visible     = 'off';
    if  ~strcmp(accumMode, 'Binary point scaling') && ~strcmp(outputMode, 'Binary point scaling')
        fractionLengthColumnLabel.Visible  = 'on';
    end
    if strcmp(accumMode,'Same as input') &&  (strcmp(outputMode, 'Same as accumulator')|| strcmp(outputMode,'same as input'))
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    end
else
    firstCoeffLabel.Visible               = 'on';
    memoryLabel.Visible                   = 'on';
    prodOutputLabel.Visible               = 'on';
    firstCoeffModeMaskParam.Visible       = 'on';
    prodOutputModeMaskParam.Visible       = 'on';
    memoryModeMaskParam.Visible           = 'on';
    if strcmp(firstCoeffMode, 'Binary point scaling')
        firstCoeffSignedText.Visible          = 'on';
        firstCoeffWordLengthMaskParam.Visible = 'on';
        firstCoeffFracLengthMaskParam.Visible = 'on';
    elseif strcmp(firstCoeffMode, 'Specify word length')
        firstCoeffSignedText.Visible          = 'on';
        firstCoeffWordLengthMaskParam.Visible = 'on';
        firstCoeffFracLengthMaskParam.Visible = 'off';
    else
        firstCoeffSignedText.Visible          = 'off';
        firstCoeffWordLengthMaskParam.Visible = 'off';
        firstCoeffFracLengthMaskParam.Visible = 'off';
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
    if strcmp(memoryMode,'Binary point scaling')
        memorySignedText.Visible              = 'on';
        memoryWordLengthMaskParam.Visible     = 'on';
        memoryFracLengthMaskParam.Visible     = 'on';
    else
        memorySignedText.Visible              = 'off';
        memoryWordLengthMaskParam.Visible     = 'off';
        memoryFracLengthMaskParam.Visible     = 'off';
    end
    
    if ~strcmp(accumMode, 'Binary point scaling') && ~strcmp(outputMode, 'Binary point scaling')...
            && ~strcmp(prodOutputMode, 'Binary point scaling') && ~strcmp(firstCoeffMode, 'Binary point scaling')&& ~strcmp(memoryMode, 'Binary point scaling')
        fractionLengthColumnLabel.Visible  = 'off';
    end
    if strcmp(accumMode,'Same as input') &&  (strcmp(outputMode, 'Same as accumulator')|| strcmp(outputMode,'Same as input'))...
            && strcmp(prodOutputMode, 'Same as input') && strcmp(memoryMode, 'Same as input') && strcmp(firstCoeffMode, 'Same word length as input')
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    end
end

end

