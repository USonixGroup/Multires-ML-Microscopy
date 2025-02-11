function [b] = vipblkcomposite(varargin)
% VIPBLKCOMPOSITE Mask dynamic dialog function for Compositing block

% Copyright 2003-2019 The MathWorks, Inc.

if nargin==0
    action = 'dynamic';   % mask callback
else
    action = 'icon';
end

blk = gcbh;

switch action
    case 'icon'
        operatn = get_param(blk,'operation');
        inLibrary = strcmp(get_param(bdroot(blk),'BlockDiagramType'),'library');
        if inLibrary
            ports.icon = 'Compositing';
        else
            if (strcmp(operatn,'Blend'))
                ports.icon = 'Blend';
            elseif (strcmp(operatn,'Binary mask'))
                ports.icon = 'Binary\nmask';
            else
                ports.icon = 'Highlight';
            end
        end
        ports.port1=1;
        ports.txt1='Image1';
        ports.port2=2;
        ports.txt2='Image2';
        extraPort = 0;
        if (strcmp(operatn,'Blend'))
            bFacSource = get_param(blk,'bFacSrc');
            if (strcmp(bFacSource,'Input port'))
                ports.port3=3;
                ports.txt3='Factor';
                extraPort = 1;
            end
        elseif (strcmp(operatn,'Binary mask'))
            mFacSource = get_param(blk,'mFacSrc');
            if (strcmp(mFacSource,'Input port'))
                extraPort = 1;
                ports.port3=3;
                ports.txt3='Mask';
            end
        else
            ports.port2=2;
            ports.txt2='Mask';
            extraPort = 0;
        end
        co_src = get_param(blk, 'source');
        if  strcmp(co_src,'Input port')
            if (extraPort == 1)
                ports.port4=4;
                ports.txt4='Location';
            else
                ports.port3=3;
                ports.txt3='Location';
                ports.port4=3;
                ports.txt4='';
            end
        else
            if (extraPort == 1)
                ports.port4=3;
                ports.txt4='';
            else
                ports.port3=2;
                ports.txt3='';
                ports.port4=2;
                ports.txt4='';
            end
        end
        b = ports;
    case 'dynamic'
        dynamicBlockUpdate(blk);
    otherwise
        error(message('vision:internal:unhandledCase'));
end
function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);

operation           = get_param(blk, 'operation');
opacityFactorSource = get_param(blk, 'bFacSrc');
maskFactorSource    = get_param(blk, 'mFacSrc');
locationSource      = get_param(blk, 'source');

opacityFactor             = blkMask.getParameter('bFactor');
maskFactor                = blkMask.getParameter('mFactor');
coordinates               = blkMask.getParameter('coordinates');
opacityFactorSrcMaskParam = blkMask.getParameter('bFacSrc');
maskFactorSrcMaskParam    = blkMask.getParameter('mFacSrc');

if strcmp(operation,'Blend')
    opacityFactorSrcMaskParam.Visible = 'on';
    maskFactorSrcMaskParam.Visible    = 'off';
    maskFactor.Visible                = 'off';
    if strcmp(opacityFactorSource,'Specify via dialog')
        opacityFactor.Visible = 'on';
    else
        opacityFactor.Visible = 'off';
    end
    if strcmp(locationSource,'Specify via dialog')
        coordinates.Visible = 'on';
    else
        coordinates.Visible = 'off';
    end
elseif strcmp(operation,'Binary mask')
    opacityFactorSrcMaskParam.Visible = 'off';
    maskFactorSrcMaskParam.Visible    = 'on';
    opacityFactor.Visible             = 'off';
    if strcmp(maskFactorSource,'Specify via dialog')
        maskFactor.Visible = 'on';
        
    else
        maskFactor.Visible = 'off';
    end
    if strcmp(locationSource,'Specify via dialog')
        coordinates.Visible = 'on';
    else
        coordinates.Visible = 'off';
    end
    
else
    opacityFactorSrcMaskParam.Visible = 'off';
    maskFactorSrcMaskParam.Visible    = 'off';
    coordinates.Visible               = 'off';
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
firstCoeffFracLengthMaskParam = blkMask.getParameter('firstCoeffFracLength');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
outputWordLengthMaskParam     = blkMask.getParameter('outputWordLength');
outputFracLengthMaskParam     = blkMask.getParameter('outputFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
dataTypesDescription      = blkMask.getDialogControl('dataTypesDescription');
fixedptOpParams           = blkMask.getDialogControl('fixedptOpParams');
fixedptDataTypes          = blkMask.getDialogControl('fixedptDataTypes');
signedColumnLabel         = blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel     = blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel = blkMask.getDialogControl('fracLengthColumnLabel');
firstCoeffLabel           = blkMask.getDialogControl('firstCoeffLabel');
firstCoeffSignedText      = blkMask.getDialogControl('firstCoeffSignedText');
accumSignedText           = blkMask.getDialogControl('accumSignedText');
outputSignedText          = blkMask.getDialogControl('outputSignedText');
prodOutputSignedText      = blkMask.getDialogControl('prodOutputSignedText');
noDataTypesCase           = blkMask.getDialogControl('noDataTypesCase');
signedColumnLabel.Visible          = 'on';
wordLengthColumnLabel.Visible      = 'on';
fractionLengthColumnLabel.Visible  = 'on';
dataTypesDescription.Visible       = 'on';
if strcmp(operation,'Blend')
    
    if strcmp(opacityFactorSource,'Specify via dialog')
        fixedptOpParams.Visible          = 'on';
        fixedptDataTypes.Visible         = 'on';
        noDataTypesCase.Visible          = 'off';
        firstCoeffLabel.Visible          = 'on';
        firstCoeffModeMaskParam.Visible  = 'on';
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
        
        if strcmp(outputMode, 'Same as first input') &&...
                (strcmp(accumMode, 'Same as first input')||...
                strcmp(accumMode, 'Same as product output'))&&...
                strcmp(prodOutputMode, 'Same as first input')&&...
                (strcmp(firstCoeffMode, 'Same word length as input'))
            signedColumnLabel.Visible          = 'off';
            wordLengthColumnLabel.Visible      = 'off';
            fractionLengthColumnLabel.Visible  = 'off';
        end
        if (~strcmp(outputMode, 'Binary point scaling')) && ...
                (~strcmp(accumMode, 'Binary point scaling'))...
                && (~strcmp(prodOutputMode, 'Binary point scaling'))&&...
                (~strcmp(firstCoeffMode, 'Binary point scaling'))
            fractionLengthColumnLabel.Visible = 'off';
        end
        
    else
        firstCoeffLabel.Visible               = 'off';
        firstCoeffModeMaskParam.Visible       = 'off';
        firstCoeffSignedText.Visible          = 'off';
        firstCoeffWordLengthMaskParam.Visible = 'off';
        firstCoeffFracLengthMaskParam.Visible = 'off';
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
        elseif strcmp(prodOutputMode, 'Slope and bias scaling')
            prodOutputSignedText.Visible          = 'on';
            prodOutputWordLengthMaskParam.Visible = 'on';
            prodOutputFracLengthMaskParam.Visible = 'off';
        else
            prodOutputSignedText.Visible          = 'off';
            prodOutputWordLengthMaskParam.Visible = 'off';
            prodOutputFracLengthMaskParam.Visible = 'off';
        end
        if strcmp(outputMode, 'Same as first input') &&...
                (strcmp(accumMode, 'Same as first input')||...
                strcmp(accumMode, 'Same as product output'))&&...
                strcmp(prodOutputMode, 'Same as first input')
            signedColumnLabel.Visible          = 'off';
            wordLengthColumnLabel.Visible      = 'off';
            fractionLengthColumnLabel.Visible  = 'off';
        end
        if (strcmp(outputMode, 'Same as first input')) &&...
                (strcmp(accumMode, 'Same as input')||...
                strcmp(accumMode, 'Same as product output'))&&...
                (strcmp(prodOutputMode, 'Same as first input'))
            fractionLengthColumnLabel.Visible = 'off';
        end
    end
else
    fixedptOpParams.Visible      = 'off';
    fixedptDataTypes.Visible     = 'off';
    noDataTypesCase.Visible      = 'on';
    dataTypesDescription.Visible = 'off';
end
