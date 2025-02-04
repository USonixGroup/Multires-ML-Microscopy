function varargout = vipblkresize(action)
% DSPBLKRESIZE Video Processing Blockset resize block helper function.

% Copyright 1995-2021 The MathWorks, Inc.

if nargin==0, action = 'dynamic'; end
blk = gcbh;
useROIChecked = strcmp(get_param(blk,'useROI'),'on');
useAntialias  = strcmp(get_param(blk,'antialias'),'on');
mode = get_param(blk, 'specify');
interpMethod = get_param(blk, 'interp_method');
isRoiEnabled = (strcmp(mode,'Number of output rows and columns') ...
    && (strcmp(interpMethod,'Nearest neighbor') ...
    || strcmp(interpMethod,'Bilinear') ...
    || strcmp(interpMethod,'Bicubic')) ...
    && (~useAntialias) && useROIChecked);


switch action
    case 'init'
        varargout = {dspGetFixptDataTypeInfo(blk,47)};
        
    case 'icon'
        % Port labels
        if isRoiEnabled
            s.i1 = 1; s.i1s = 'Image';
            s.i2 = 2; s.i2s = 'ROI';
            roiFlagExists = strcmp(get_param(blk,'roiFlag'),'on');
            if (roiFlagExists)
                s.o1 = 1; s.o1s = 'Out';
                s.o2 = 2; s.o2s = 'Flag';
            else
                s.o1 = 1; s.o1s = '';
                s.o2 = 1; s.o2s = '';
            end
        else
            s.i1 = 1; s.i1s = '';
            s.i2 = 1; s.i2s = '';
            s.o1 = 1; s.o1s = '';
            s.o2 = 1; s.o2s = '';
        end
        varargout = {s};
    case 'dynamic'
        dynamicBlockUpdate(blk);
    otherwise
        error(message('vision:internal:unhandledCase'));
end
function dynamicBlockUpdate(blk)
blkMask = Simulink.Mask.get(gcb);

specify      = get_param(blk, 'specify');
interpMethod = get_param(blk, 'interp_method');
rfactor      = blkMask.getParameter('rfactor');
outCols      = blkMask.getParameter('outCols');
outRows      = blkMask.getParameter('outRows');
outRowsCols  = blkMask.getParameter('outRowsCols');
useROIMaskParameter = blkMask.getParameter('useROI');
roiFlagMaskParameter= blkMask.getParameter('roiFlag');

useROI    = get_param(blk,'useROI');
antialias = get_param(blk,'antialias');

ROIProcessing = blkMask.getDialogControl('ROIProcessing');

if strcmp(specify,'Output size as a percentage of input size')
    rfactor.Visible     = 'on';
    outCols.Visible     = 'off';
    outRows.Visible     = 'off';
    outRowsCols.Visible = 'off';
elseif strcmp(specify,'Number of output columns and preserve aspect ratio')
    rfactor.Visible     = 'off';
    outCols.Visible     = 'on';
    outRows.Visible     = 'off';
    outRowsCols.Visible = 'off';
elseif strcmp(specify,'Number of output rows and preserve aspect ratio')
    rfactor.Visible     = 'off';
    outCols.Visible     = 'off';
    outRows.Visible     = 'on';
    outRowsCols.Visible = 'off';
else
    rfactor.Visible     = 'off';
    outCols.Visible     = 'off';
    outRows.Visible     = 'off';
    outRowsCols.Visible = 'on';
end

if strcmp(specify,'Number of output rows and columns') && (strcmp(interpMethod,'Nearest neighbor')||...
        strcmp(interpMethod,'Bilinear')||strcmp(interpMethod, 'Bicubic'))...
        && strcmp(antialias, 'off')
    ROIProcessing.Visible = 'on';
    useROIMaskParameter.Visible = 'on';
    
else
     ROIProcessing.Visible = 'off';
     useROIMaskParameter.Visible = 'off';
     roiFlagMaskParameter.Visible = 'off';
     
end

    if strcmp(useROIMaskParameter.Value,'on')
        roiFlagMaskParameter.Visible = 'on';
    else
        roiFlagMaskParameter.Visible = 'off';
    end
% =====================================================
% Fixpt tab visibility
% =====================================================
firstCoeffMode = get_param(blk, 'firstCoeffMode');
accumMode      = get_param(blk, 'accumMode');
outputMode     = get_param(blk, 'outputMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
firstCoeffModeMaskParam       = blkMask.getParameter('firstCoeffMode');
firstCoeffWordLengthMaskParam = blkMask.getParameter('firstCoeffWordLength');
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


if strcmp(specify,'Output size as a percentage of input size') && ...
        (strcmp(interpMethod,'Nearest neighbor')||strcmp(interpMethod,'Bilinear'))
    firstCoeffModeMaskParam.Visible       = 'off';
    firstCoeffLabel.Visible               = 'off';
    firstCoeffSignedText.Visible          = 'off';
    firstCoeffWordLengthMaskParam.Visible = 'off';
    
    
else
    firstCoeffModeMaskParam.Visible = 'on';
    firstCoeffLabel.Visible = 'on';
    if strcmp(firstCoeffMode, 'Specify word length')
        firstCoeffSignedText.Visible          = 'on';
        firstCoeffWordLengthMaskParam.Visible = 'on';
        
    else
        firstCoeffSignedText.Visible = 'off';
        firstCoeffWordLengthMaskParam.Visible = 'off';
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
if strcmp(specify,'Output size as a percentage of input size') && ...
        (strcmp(interpMethod,'Nearest neighbor')||strcmp(interpMethod,'Bilinear'))
    if strcmp(outputMode, 'Same as input') && (strcmp(accumMode, 'Same as input')||strcmp(accumMode, 'Same as product output'))...
            && strcmp(prodOutputMode, 'Same as input')
        
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    elseif (strcmp(outputMode, 'Same as input')) && (strcmp(accumMode, 'Slope and bias scaling')||strcmp(accumMode, 'Same as input')||strcmp(accumMode, 'Same as product output'))...
            && (strcmp(prodOutputMode, 'Same as input'))
        fractionLengthColumnLabel.Visible = 'off';
    end
else
    if strcmp(outputMode, 'Same as input') && (strcmp(accumMode, 'Same as input')||strcmp(accumMode, 'Same as product output'))...
            && strcmp(prodOutputMode, 'Same as input')&& (strcmp(firstCoeffMode, 'Same word length as input'))
        
        signedColumnLabel.Visible          = 'off';
        wordLengthColumnLabel.Visible      = 'off';
        fractionLengthColumnLabel.Visible  = 'off';
    end
    if(strcmp(outputMode, 'Same as input')) && (strcmp(accumMode, 'Slope and bias scaling')||strcmp(accumMode, 'Same as input')||strcmp(accumMode, 'Same as product output'))...
            && ( strcmp(prodOutputMode, 'Same as input'))
        fractionLengthColumnLabel.Visible = 'off';
    end
end
% end of vipblkresize.m