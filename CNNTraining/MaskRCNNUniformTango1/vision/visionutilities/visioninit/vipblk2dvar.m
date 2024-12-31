function vipblk2dvar
% VIPBLK2DVAR Mask dynamic dialog function for 2D Variance block

% Copyright 2019 The MathWorks, Inc.

blk = gcbh;   % Cache handle to block
blkMask = Simulink.Mask.get(gcb);

run                    =   get_param(blk, 'run');
directionMode          =   get_param(blk, 'directionMode');
roiEnable              =   get_param(blk, 'roiEnable');
roiType                =   get_param(blk, 'roiType');
dimension              =   blkMask.getParameter('dimension');
resetPort              =   blkMask.getParameter('reset_popup');
directionModeMaskParam =   blkMask.getParameter('directionMode');
roiEnableMaskParam     =   blkMask.getParameter('roiEnable');
roiTypeMaskParam       =   blkMask.getParameter('roiType');
roiPortion             =   blkMask.getParameter('roiPortion');
roiOutput              =   blkMask.getParameter('roiOutput');
roiFlag                =   blkMask.getParameter('roiFlag');
roiProcessing          =   blkMask.getDialogControl('roiProcessing');

if strcmp(run,'off')
    directionModeMaskParam.Visible  = 'on';
    resetPort.Visible               = 'off';
    if strcmp(directionMode, 'Entire input')
        roiProcessing.Visible      = 'on';
        dimension.Visible          = 'off';
        roiEnableMaskParam.Visible = 'on';
        if strcmp(roiEnable,'on')
            roiTypeMaskParam.Visible = 'on';
            if strcmp(roiType,'Rectangles')
                roiPortion.Visible       = 'on';
                roiOutput.Visible        = 'on';
                roiFlag.Visible          = 'on';
                roiFlag.Prompt           = 'vision:masks:OutputFlagROIImageBound';
            elseif strcmp(roiType,'Lines')
                roiPortion.Visible       = 'off';
                roiOutput.Visible        = 'on';
                roiFlag.Visible          = 'on';
                roiFlag.Prompt           = 'vision:masks:OutputFlagROIImageBound';
            elseif strcmp(roiType,'Label matrix')
                roiPortion.Visible       = 'off';
                roiOutput.Visible        = 'on';
                roiFlag.Visible          = 'on';
                roiFlag.Prompt           = 'vision:masks:OutputFlagIndicatingIfInputLabelNumAreValid';
            else
                roiPortion.Visible       = 'off';
                roiOutput.Visible        = 'off';
                roiFlag.Visible          = 'off';
            end
        else
            roiTypeMaskParam.Visible    = 'off';
            roiPortion.Visible          = 'off';
            roiOutput.Visible           = 'off';
            roiFlag.Visible             = 'off';
        end  
        
    elseif strcmp(directionMode, 'Specified dimension')
        dimension.Visible        = 'on';
        roiProcessing.Visible    = 'off';
    else
        dimension.Visible        = 'off';
        roiProcessing.Visible    = 'off';
    end
else
    directionModeMaskParam.Visible  = 'off';
    resetPort.Visible               = 'on';
    roiProcessing.Visible           = 'off';
    dimension.Visible               = 'off';
end

% =====================================================
% Fixpt tab visibility
% =====================================================

accumMode      = get_param(blk, 'accumMode');
memoryMode     = get_param(blk, 'memoryMode');
outputMode     = get_param(blk, 'outputMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
accumWordLengthMaskParam      =   blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      =   blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam     =   blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     =   blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam =   blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam =   blkMask.getParameter('prodOutputFracLength');
memoryWordLengthMaskParam     =   blkMask.getParameter('memoryWordLength');
memoryFracLengthMaskParam     =   blkMask.getParameter('memoryFracLength');
signedColumnLabel             =   blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel         =   blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel     =   blkMask.getDialogControl('fractionLengthColumnLabel');
accumSignedText               =   blkMask.getDialogControl('accumSignedText');
outputSignedText              =   blkMask.getDialogControl('outputSignedText');
prodOutputSignedText          =   blkMask.getDialogControl('prodOutputSignedText');
memorySignedText              =   blkMask.getDialogControl('memorySignedText');

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
if strcmp(memoryMode, 'Binary point scaling')
    memorySignedText.Visible          = 'on';
    memoryWordLengthMaskParam.Visible = 'on';
    memoryFracLengthMaskParam.Visible = 'on';
else
    memorySignedText.Visible          = 'off';
    memoryWordLengthMaskParam.Visible = 'off';
    memoryFracLengthMaskParam.Visible = 'off';
end
if (strcmp(outputMode, 'Same as input') || strcmp(outputMode,'Same as input-squared product')|| strcmp(outputMode, 'Same as accumulator'))...
        && (strcmp(accumMode, 'Same as input')||strcmp(accumMode, 'Same as input-squared product'))...
        && strcmp(prodOutputMode, 'Same as input')&& strcmp(memoryMode, 'Same as input-squared product')
    signedColumnLabel.Visible          = 'off';
    wordLengthColumnLabel.Visible      = 'off';
    fractionLengthColumnLabel.Visible  = 'off';
end
if (~strcmp(outputMode, 'Binary point scaling')) && (~(strcmp(accumMode, 'Binary point scaling'))...
        && (~strcmp(prodOutputMode, 'Binary point scaling')) && (~strcmp(memoryMode,'Binary point scaling')))
    fractionLengthColumnLabel.Visible = 'off';
end

