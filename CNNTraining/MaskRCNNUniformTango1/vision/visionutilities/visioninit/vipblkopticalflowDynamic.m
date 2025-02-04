function vipblkopticalflowDynamic
% VIPBLKOPTICALFLOWDYNAMIC is  Optical Flow Block dynamicBlockUpdate function.

% Copyright 2018-2019 The MathWorks, Inc.
%

blk = gcbh;
blkMask = Simulink.Mask.get(gcb);

method          = get_param(blk, 'method');
gradMethod      = get_param(blk, 'gradMethod');
stopCriteria    = get_param(blk, 'stop_criteria');
which2img       = get_param(blk, 'which2img');
outVelForm      = get_param(blk, 'outVelForm');
gradMethodMask     = blkMask.getParameter('gradMethod');
which2imgMask      = blkMask.getParameter('which2img');
N                  = blkMask.getParameter('N');
lambda             = blkMask.getParameter('lambda');
maxIter            = blkMask.getParameter('maxIter');
maxAllowAbsDiff    = blkMask.getParameter('maxAllowAbsDiff');
stopCriteriaMask   = blkMask.getParameter('stop_criteria');
numInports         = blkMask.getParameter('numInports');
sigmaS             = blkMask.getParameter('sigmaS');
sigmaW             = blkMask.getParameter('sigmaW');
discardNormalFlow  = blkMask.getParameter('discardNormalFlow');
outputCurrentImage = blkMask.getParameter('outputCurrentImage');
eigTh              = blkMask.getParameter('eigTh');
if strcmp(which2img,'Two images')
    N.Visible    = 'off';
else
    N.Visible    = 'on';
end
if strcmp(method,'Horn-Schunck')
    numInports.Visible         = 'off';
    sigmaS.Visible             = 'off';
    sigmaW.Visible             = 'off';
    discardNormalFlow.Visible  = 'off';
    outputCurrentImage.Visible = 'off';
    lambda.Visible           = 'on';
    stopCriteriaMask.Visible = 'on';
    eigTh.Visible            = 'off';
    gradMethodMask.Visible   = 'off';
    which2imgMask.Visible    = 'on';
    if strcmp(stopCriteria,'When velocity difference falls below threshold')
        maxIter.Visible         = 'off';
        maxAllowAbsDiff.Visible = 'on';
    elseif strcmp(stopCriteria,'When maximum number of iterations is reached')
        maxIter.Visible         = 'on';
        maxAllowAbsDiff.Visible = 'off';
    else
        maxIter.Visible         = 'on';
        maxAllowAbsDiff.Visible = 'on';
    end
else
    gradMethodMask.Visible   = 'on';
    eigTh.Visible            = 'on';
    lambda.Visible           = 'off';
    stopCriteriaMask.Visible = 'off';
    maxIter.Visible         = 'off';
    maxAllowAbsDiff.Visible = 'off';
    if strcmp(gradMethod,'Derivative of Gaussian')
        numInports.Visible         = 'on';
        sigmaS.Visible             = 'on';
        sigmaW.Visible             = 'on';
        discardNormalFlow.Visible  = 'on';
        outputCurrentImage.Visible = 'on';
        which2imgMask.Visible      = 'off';
        N.Visible                  = 'off';
    else
        numInports.Visible         = 'off';
        sigmaS.Visible             = 'off';
        sigmaW.Visible             = 'off';
        discardNormalFlow.Visible  = 'off';
        outputCurrentImage.Visible = 'off';
        which2imgMask.Visible      = 'on';
    end
end


% =====================================================
% Fixpt tab visibility
% =====================================================
firstCoeffMode      = get_param(blk, 'firstCoeffMode');
accumMode           = get_param(blk, 'accumMode');
outputMode          = get_param(blk, 'outputMode');
prodOutputMode      = get_param(blk, 'prodOutputMode');
memoryMode          = get_param(blk, 'memoryMode');
firstCoeffWordLengthMaskParam = blkMask.getParameter('firstCoeffWordLength');
firstCoeffFracLengthMaskParam = blkMask.getParameter('firstCoeffFracLength');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
memoryWordLengthMaskParam     = blkMask.getParameter('memoryWordLength');
memoryFracLengthMaskParam     = blkMask.getParameter('memoryFracLength');
accumSignedText            = blkMask.getDialogControl('accumSignedText');
outputSignedText           = blkMask.getDialogControl('outputSignedText');
prodOutputSignedText       = blkMask.getDialogControl('prodOutputSignedText');
firstCoeffSignedText       = blkMask.getDialogControl('firstCoeffSignedText');
memorySignedText           = blkMask.getDialogControl('memorySignedText');
noDataTypesCase            = blkMask.getDialogControl('noDataTypes');
fixedptOpParams            = blkMask.getDialogControl('fixedptOpParams');
fixedptDataTypes           = blkMask.getDialogControl('fixedptDataTypes');
dataTypesDescription       = blkMask.getDialogControl('dataTypesDescription');
fractionLengthColoumnLabel = blkMask.getDialogControl('fracLengthColumnLabel');
fractionLengthColoumnLabel.Visible = 'on';

if strcmp(method, 'Lucas-Kanade') && strcmp(gradMethod,'Difference filter [-1 1]')
    noDataTypesCase.Visible      = 'off';
    fixedptOpParams.Visible      = 'on';
    fixedptDataTypes.Visible     = 'on';
    dataTypesDescription.Visible = 'on';
    if strcmp(outVelForm,'Magnitude-squared')
        outputSignedText.Prompt = 'Simulink:dialog:No_CB';
    else
        outputSignedText.Prompt = 'Simulink:dialog:Yes_CB';
    end
    if strcmp(accumMode, 'Binary point scaling')
        accumSignedText.Visible          = 'on';
        accumWordLengthMaskParam.Visible = 'on';
        accumFracLengthMaskParam.Visible = 'on';
    else
        accumSignedText.Visible          = 'off';
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
        memorySignedText.Visible          = 'off';
        memoryWordLengthMaskParam.Visible = 'off';
        memoryFracLengthMaskParam.Visible = 'off';
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
        firstCoeffSignedText.Visible          = 'off';
        firstCoeffWordLengthMaskParam.Visible = 'off';
        firstCoeffFracLengthMaskParam.Visible = 'off';
    end
else
    fixedptOpParams.Visible      = 'off';
    fixedptDataTypes.Visible     = 'off';
    noDataTypesCase.Visible      = 'on';
    dataTypesDescription.Visible = 'off';
end
end