function [si,so,blkname,dtInfo]= vipblkedge(action)

% VIPBLKEDGE Mask initialization function for Edge Detection block

% Copyright 2003-2019 The MathWorks, Inc.
if nargin==0, action = 'dynamic'; end
blk = gcbh;   % Cache handle to block

switch action
    case 'init'
        [si,so,blkname,dtInfo] = init_blk(blk);
        dynamicBlockUpdate(blk);
    otherwise
        dynamicBlockUpdate(blk);
end
end

function [si,so,blkname,dtInfo]= init_blk(blk)
% get elements of the mask that affect dynamic elements
outputTypeStr = get_param(blk,'outputType');
methodStr = get_param(blk,'method');

isBlkInLibrary = strcmp(get_param(bdroot(blk),'BlockDiagramType'),'library');
if isBlkInLibrary
    blkname = 'Edge\nDetection';
else
    blkname = methodStr;
end

userDefinedThresholdStr = get_param(blk,'userDefinedThreshold');
thresholdSourceStr = get_param(blk,'thresholdSource');

dtInfo = dspGetFixptDataTypeInfo(blk,15);

si(1).port = 1; si(1).txt  = 'I';
if ~strcmp(methodStr,'Canny')
    if    strcmp(thresholdSourceStr, 'Input port') && ...
            ~strcmp(outputTypeStr,'Gradient components') && ...
            strcmp(userDefinedThresholdStr, 'on')
        si(2).port = 2; si(2).txt = 'Th';
    else
        si(2).port = 1; si(2).txt = 'I';
    end
else
    if    strcmp(thresholdSourceStr, 'Input port') && ...
            strcmp(userDefinedThresholdStr, 'on')
        si(2).port = 2; si(2).txt = 'Th';
    else
        si(2).port = 1; si(2).txt = 'I';
    end
end

if strcmp(methodStr,'Canny') || strcmp(outputTypeStr,'Binary image')
    [so(1:3).port] = deal(1,1,1);
    [so(1:3).txt]  = deal('Edge','Edge','Edge');
elseif strcmp(outputTypeStr,'Gradient components')
    [so(1:3).port] = deal(1,2,1);
    if strcmp(methodStr,'Roberts')
        [so(1:3).txt]  = deal('G45','G135','G45');
    else
        [so(1:3).txt]  = deal('Gv','Gh','Gv');
    end
else % both
    [so(1:3).port] = deal(1,2,3);
    so(1).txt  = 'Edge';
    if strcmp(methodStr,'Roberts')
        [so(2:3).txt]  = deal('G45','G135');
    else
        [so(2:3).txt]  = deal('Gv','Gh');
    end
end
end


function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);
mask_enables              = get_param(blk, 'MaskEnables');
old_mask_enables          = mask_enables;
maskParamNames            = get_param(blk,'MaskNames');

methodValue               = get_param(blk, 'method');

outputTypeParam           = blkMask.getParameter('outputType');
userDefinedThresholdParam = blkMask.getParameter('userDefinedThreshold');
thresholdSourceParam      = blkMask.getParameter('thresholdSource');
thresholdParam            = blkMask.getParameter('threshold');
thresholdTuningParam      = blkMask.getParameter('thresholdTuning');
edgeThinningParam         = blkMask.getParameter('edgeThinning');
thresholdCannyParam       = blkMask.getParameter('threshold_canny');
autoPercentParam          = blkMask.getParameter('autoPercent');
sigmaParam                = blkMask.getParameter('sigma');

if(strcmp(methodValue, 'Canny'))
    outputTypeParam.Visible = 'off'; %% only one output and it is Binary Edge
else
    outputTypeParam.Visible = 'on';
end

outputTypeValue = get_param(blk, 'outputType');

%userDefinedThreshold
if(~strcmp(methodValue, 'Canny'))
    if(strcmp(outputTypeValue, 'Gradient components'))
        userDefinedThresholdParam.Visible = 'off';
    else
        userDefinedThresholdParam.Visible = 'on';
    end
else
    userDefinedThresholdParam.Visible = 'on';
end

userThdValue    = get_param(blk, 'userDefinedThreshold');

% ThresholdSource type

if(~strcmp(methodValue, 'Canny'))
    if(strcmp(outputTypeValue, 'Gradient components'))
        thresholdSourceParam.Visible = 'off';
    else
        if(strcmp(userThdValue, 'on'))
            thresholdSourceParam.Visible = 'on';
        else
            thresholdSourceParam.Visible = 'off';
        end
    end
else
    if(strcmp(userThdValue, 'on'))
        thresholdSourceParam.Visible = 'on';
    else
        thresholdSourceParam.Visible = 'off';
    end
end

thdSourceValue  = get_param(blk, 'thresholdSource');

%threshold
if(~strcmp(methodValue, 'Canny'))
    if(strcmp(outputTypeValue, 'Gradient components'))
        thresholdParam.Visible = 'off';
    else
        if(strcmp(userThdValue, 'on') && strcmp(thdSourceValue, 'Specify via dialog'))
            thresholdParam.Visible = 'on';
            thresholdParamEnable   = 'on';
        else
            thresholdParam.Visible = 'off';
            thresholdParamEnable   = 'off';
        end
        mask_enables{strcmp(maskParamNames,'threshold')}       = thresholdParamEnable;
    end
else %% Canny method
    thresholdParam.Visible = 'off';
    if(strcmp(userThdValue, 'on') && strcmp(thdSourceValue, 'Specify via dialog'))
        thresholdCannyParam.Visible = 'on';
    else
        thresholdCannyParam.Visible = 'off';
    end
end

%thresholdTuning
if(~strcmp(methodValue, 'Canny'))
    if(strcmp(outputTypeValue, 'Gradient components'))
        thresholdTuningParam.Visible = 'off';
    else
        if(strcmp(userThdValue, 'on'))
            thresholdTuningParam.Visible = 'off';
        else
            thresholdTuningParam.Visible = 'on';
        end
    end
else
    thresholdTuningParam.Visible = 'off';
end

%edgeThinning
if(~strcmp(methodValue, 'Canny'))
    if(strcmp(outputTypeValue, 'Gradient components'))
        edgeThinningParam.Visible = 'off';
    else
        edgeThinningParam.Visible = 'on';
    end
else
    edgeThinningParam.Visible = 'off';
end

%threshold for Canny
if(strcmp(methodValue, 'Canny'))
    if(strcmp(userThdValue, 'on') && strcmp(thdSourceValue, 'Specify via dialog'))
        thresholdCannyParam.Visible = 'on';
        thresholdCannyParamEnable   = 'on';
    else
        thresholdCannyParam.Visible = 'off';
        thresholdCannyParamEnable   = 'off';
        
    end
    mask_enables{strcmp(maskParamNames,'threshold_canny')} = thresholdCannyParamEnable;
else
    thresholdCannyParam.Visible = 'off';
end

%'Percent of pixels not edges' for Canny
if(strcmp(methodValue, 'Canny'))
    if(~strcmp(userThdValue, 'on'))
        autoPercentParam.Visible = 'on';
    else
        autoPercentParam.Visible = 'off';
    end
else
    autoPercentParam.Visible = 'off';
end

%sigma for Canny
if(strcmp(methodValue, 'Canny'))
    sigmaParam.Visible = 'on';
else
    sigmaParam.Visible = 'off';
end

% =====================================================
% Fixpt tab visibility
% =====================================================

accumMode      = get_param(blk, 'accumMode');
outputMode     = get_param(blk, 'outputMode');
prodOutputMode = get_param(blk, 'prodOutputMode');

accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
outputModeMaskParam           = blkMask.getParameter('outputMode');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');


accumSignedText       = blkMask.getDialogControl('accumSignedText');
outputLabel           = blkMask.getDialogControl('outputLabel');
outputSignedText      = blkMask.getDialogControl('outputSignedText');
prodOutputSignedText  = blkMask.getDialogControl('prodOutputSignedText');
SignedColumnLabel     = blkMask.getDialogControl('SignedColumnLabel');
WordLengthColumnLabel = blkMask.getDialogControl('WordLengthColumnLabel');
FracLengthColumnLabel = blkMask.getDialogControl('FractionLengthColumnLabel');

signedColumnLabelVisibility     = 'off';
wordLengthColumnLabelVisibility = 'off';
fracLengthColumnLabelVisibility = 'off';

if strcmp(prodOutputMode, 'Binary point scaling')
    prodOutputSignedText.Visible          = 'on';
    prodOutputWordLengthMaskParam.Visible = 'on';
    prodOutputFracLengthMaskParam.Visible = 'on';
    signedColumnLabelVisibility           = 'on';
    wordLengthColumnLabelVisibility       = 'on';
    fracLengthColumnLabelVisibility       = 'on';
else
    prodOutputSignedText.Visible          = 'off';
    prodOutputWordLengthMaskParam.Visible = 'off';
    prodOutputFracLengthMaskParam.Visible = 'off';
end

if strcmp(accumMode, 'Binary point scaling')
    accumSignedText.Visible          = 'on';
    accumWordLengthMaskParam.Visible = 'on';
    accumFracLengthMaskParam.Visible = 'on';
    signedColumnLabelVisibility      = 'on';
    wordLengthColumnLabelVisibility  = 'on';
    fracLengthColumnLabelVisibility  = 'on';
else
    accumSignedText.Visible          = 'off';
    accumWordLengthMaskParam.Visible = 'off';
    accumFracLengthMaskParam.Visible = 'off';
end

if(strcmp(outputTypeValue, 'Binary image'))
    outputLabel.Visible               = 'off';
    outputModeMaskParam.Visible       = 'off';
    outputSignedText.Visible          = 'off';
    outputWordLengthMaskParam.Visible = 'off';
    outputFracLengthMaskParam.Visible = 'off';
else
    outputLabel.Visible         = 'on';
    outputModeMaskParam.Visible = 'on';
    if strcmp(outputMode, 'Binary point scaling')
        outputSignedText.Visible          = 'on';
        outputWordLengthMaskParam.Visible = 'on';
        outputFracLengthMaskParam.Visible = 'on';
        signedColumnLabelVisibility       = 'on';
        wordLengthColumnLabelVisibility   = 'on';
        fracLengthColumnLabelVisibility   = 'on';
    else
        outputSignedText.Visible          = 'off';
        outputWordLengthMaskParam.Visible = 'off';
        outputFracLengthMaskParam.Visible = 'off';
    end
end
SignedColumnLabel.Visible     = signedColumnLabelVisibility;
WordLengthColumnLabel.Visible = wordLengthColumnLabelVisibility;
FracLengthColumnLabel.Visible = fracLengthColumnLabelVisibility;
%% discard if canny method supports fixed point
emptyFixptLabel            = blkMask.getDialogControl('emptyFixpt');
FixPtOpParamsGroupBox      = blkMask.getDialogControl('FixPtOpParamsGroupBox');
FixptDatatypesGroupBox     = blkMask.getDialogControl('FixptDatatypesGroupBox');
FloatingPointTrumpRuleText = blkMask.getDialogControl('FloatingPointTrumpRule');
if(strcmp(methodValue, 'Canny'))
    emptyFixptLabel.Visible            = 'on';
    FixPtOpParamsGroupBox.Visible      = 'off';
    FixptDatatypesGroupBox.Visible     = 'off';
    FloatingPointTrumpRuleText.Visible = 'off';
else
    emptyFixptLabel.Visible            = 'off';
    FixPtOpParamsGroupBox.Visible      = 'on';
    FixptDatatypesGroupBox.Visible     = 'on';
    FloatingPointTrumpRuleText.Visible = 'on';
end

% This is needed to disable invisible text field in the mask,
% to prevent their contents from being evaluated
if (~isequal(mask_enables, old_mask_enables))
    set_param(blk, 'MaskEnables', mask_enables);
end
end

% [EOF]

