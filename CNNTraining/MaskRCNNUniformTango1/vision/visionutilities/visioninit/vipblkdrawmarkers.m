function [b] = vipblkdrawmarkers(varargin)
% VIPBLKCROSSHAIRS Mask dynamic dialog function for Draw Cross-hairs block

% Copyright 2003-2019 The MathWorks, Inc.

if nargin==0
    action = 'dynamic';   % mask callback
else
    action = 'icon';
end
blk = gcbh;
switch action
    case 'icon'
        shapeToDraw = get_param(blk, 'shape');
        inLibrary = strcmp(get_param(bdroot(blk),'BlockDiagramType'),'library');
        if inLibrary
            ports.icon = 'Draw\nMarkers';
        else
            if (strcmp(shapeToDraw,'Circle'))
                ports.icon = 'Draw markers\n(Circle)';
            elseif (strcmp(shapeToDraw,'X-mark'))
                ports.icon = 'Draw markers\n(X-mark)';
            elseif (strcmp(shapeToDraw,'Plus'))
                ports.icon = 'Draw markers\n(Plus)';
            elseif (strcmp(shapeToDraw,'Star'))
                ports.icon = 'Draw markers\n(Star)';
            else % draw Square marker
                ports.icon = 'Draw markers\n(Square)';
            end
        end
        oldVerObsolete = strcmp(get_param(blk, 'inType'), 'Obsolete') == 1;
        if (oldVerObsolete)
            isSinglePort = strcmp(get_param(blk, 'imagePorts'), 'One multidimensional signal') == 1;
        else
            isSinglePort = strcmp(get_param(blk, 'inType'), 'Intensity') == 1;
        end
        
        if (isSinglePort)
            nextPort = 1;
            ports.iport1=nextPort;
            ports.itxt1='';
            
            ports.iport2=nextPort;
            ports.itxt2='';
            
            ports.iport3=nextPort;
            ports.itxt3='Image';
            
            nextPort = nextPort + 1;
            ports.iport4=nextPort;
            ports.itxt4='Pts';
            
            if (strcmp(get_param(blk,'viewport'),'Specify region of interest via port'))
                nextPort = nextPort + 1;
                ports.itxt5='ROI';
            else
                ports.itxt5='';
            end
            ports.iport5=nextPort;
            
            if (strcmp(get_param(blk,'fillClrSource'),'Input port'))
                nextPort = nextPort + 1;
                ports.itxt6 = 'Clr';
            else
                ports.itxt6 = '';
            end
            ports.iport6 = nextPort;
            
            ports.oport1=1;
            ports.otxt1='';
            ports.oport2=1;
            ports.otxt2='';
            ports.oport3=1;
            ports.otxt3='';
        else
            ports.iport1=1;
            ports.itxt1='R';
            ports.iport2=2;
            ports.itxt2='G';
            ports.iport3=3;
            ports.itxt3='B';
            
            ports.iport4=4;
            ports.itxt4='Pts';
            nextPort = 4;
            
            if (strcmp(get_param(blk,'viewport'),'Specify region of interest via port'))
                nextPort = nextPort + 1;
                ports.itxt5='ROI';
            else
                ports.itxt5='';
            end
            ports.iport5=nextPort;
            
            if (strcmp(get_param(blk,'fillClrSource'),'Input port'))
                nextPort = nextPort + 1;
                ports.itxt6 = 'Clr';
            else
                ports.itxt6 = '';
            end
            ports.iport6 = nextPort;
            
            ports.oport1=1;
            ports.otxt1='R';
            ports.oport2=2;
            ports.otxt2='G';
            ports.oport3=3;
            ports.otxt3='B';
        end
        b = ports;
    case 'dynamic'
        dynamicBlockUpdate(blk)
    otherwise
        error(message('vision:internal:unhandledCase'));
end
function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);

markerShape   = get_param(blk, 'shape');
filled        = get_param(blk, 'fill');
antialiasing  = get_param(blk, 'antialiasing');
fillClrSource = get_param(blk, 'fillClrSource');
fillColor     = get_param(blk, 'display');
filledMaskParam        = blkMask.getParameter('fill');
antialiasingMaskParam  = blkMask.getParameter('antialiasing');
opacity                = blkMask.getParameter('opacity');
fillColorMaskParam     = blkMask.getParameter('display');
color                  = blkMask.getParameter('color');
fillClrSourceLabel     = blkMask.getDialogControl('fillClrSourceLabel');
fillColorLabel         = blkMask.getDialogControl('fillColorLabel');
if strcmp(markerShape,'Circle')
    filledMaskParam.Visible       = 'on';
    antialiasingMaskParam.Visible = 'on';
elseif strcmp(markerShape,'X-mark')||strcmp(markerShape,'Star')
    filledMaskParam.Visible       = 'off';
    antialiasingMaskParam.Visible = 'on';
elseif strcmp(markerShape,'Square')
    filledMaskParam.Visible       = 'on';
    antialiasingMaskParam.Visible = 'off';
elseif strcmp(markerShape,'Plus')
    filledMaskParam.Visible       = 'off';
    antialiasingMaskParam.Visible = 'off';
end
if strcmp(fillClrSource, 'Specify via dialog')
    fillColorMaskParam.Visible = 'on';
    fillColorLabel.Visible     = 'on';
else
    fillColorLabel.Visible     = 'off';
    fillColorMaskParam.Visible = 'off';
end
if strcmp(filled, 'on')
    fillClrSourceLabel.Prompt       = 'vision:masks:FillColorSource';
    fillColorLabel.Prompt           = 'vision:masks:FillColor';
    opacity.Visible                 = 'on';
else
    fillClrSourceLabel.Prompt       = 'vision:masks:BorderColorSource';
    fillColorLabel.Prompt           = 'vision:masks:BorderColor';
    opacity.Visible                 = 'off';
end
if strcmp(fillColor,'User-specified value')
    color.Visible = 'on';
    
else
    color.Visible = 'off';
    
end
% =====================================================
% Fixpt tab visibility
% =====================================================
memoryMode     = get_param(blk,'memoryMode');
accumMode      = get_param(blk, 'accumMode');
prodOutputMode = get_param(blk, 'prodOutputMode');
accumWordLengthMaskParam      = blkMask.getParameter('accumWordLength');
accumFracLengthMaskParam      = blkMask.getParameter('accumFracLength');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
accumModeMaskParam            = blkMask.getParameter('accumMode');
prodOutputModeMaskParam       = blkMask.getParameter('prodOutputMode');
memoryModeMaskParam           = blkMask.getParameter('memoryMode');
memoryWordLength              = blkMask.getParameter('memoryWordLength');
memoryFracLength              = blkMask.getParameter('memoryFracLength');
signedColumnLabel         = blkMask.getDialogControl('signedColumnLabel');
wordLengthColumnLabel     = blkMask.getDialogControl('wordLengthColumnLabel');
fractionLengthColumnLabel = blkMask.getDialogControl('fractionLengthColumnLabel');
accumSignedText           = blkMask.getDialogControl('accumSignedText');
prodOutputSignedText      = blkMask.getDialogControl('prodOutputSignedText');
memoryModeSignedText      = blkMask.getDialogControl('memorySignedText');
accumLabel                = blkMask.getDialogControl('accumLabel');
prodOutputlabel           = blkMask.getDialogControl('prodOutputLabel');
memoryLabel               = blkMask.getDialogControl('memoryLabel');
floatingPointTrumpRule    = blkMask.getDialogControl('floatingPointTrumpRule');
fixptOpParams             = blkMask.getDialogControl('fixptOpParams');
fixptDataTypes            = blkMask.getDialogControl('fixPtDataTypes');
dataTypesParametersAreApplicable = blkMask.getDialogControl('dataTypesParametersAreApplicable');
dataTypesParametersAreApplicable.Visible = 'off';
signedColumnLabel.Visible         = 'on';
wordLengthColumnLabel.Visible     = 'on';
fractionLengthColumnLabel.Visible = 'on';

if  strcmp(markerShape,'Circle')
    if strcmp(filled, 'on')
        floatingPointTrumpRule.Visible    = 'on';
        fixptOpParams.Visible           = 'on';
        fixptDataTypes.Visible          = 'on';
        memoryLabel.Visible            =  'on';
        accumLabel.Visible             =  'on';
        prodOutputlabel.Visible        =  'on';
        accumModeMaskParam.Visible     =  'on';
        prodOutputModeMaskParam.Visible=  'on';
        memoryModeMaskParam.Visible    =  'on';
        if strcmp(accumMode, 'Binary point scaling')
            accumSignedText.Visible          = 'on';
            accumWordLengthMaskParam.Visible = 'on';
            accumFracLengthMaskParam.Visible = 'on';
        else
            accumSignedText.Visible = 'off';
            accumWordLengthMaskParam.Visible = 'off';
            accumFracLengthMaskParam.Visible = 'off';
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
        if strcmp(memoryMode, 'Specify word length')
            memoryModeSignedText.Visible = 'on';
            memoryWordLength.Visible     = 'on';
            memoryFracLength.Visible     = 'off';
        else
            memoryModeSignedText.Visible  = 'off';
            memoryWordLength.Visible      = 'off';
            memoryFracLength.Visible      = 'off';
        end
    elseif strcmp(filled, 'off')&& strcmp(antialiasing, 'on')
        floatingPointTrumpRule.Visible    = 'on';
        fixptOpParams.Visible           = 'on';
        fixptDataTypes.Visible          = 'on';
        accumLabel.Visible             =  'on';
        prodOutputlabel.Visible        =  'on';
        accumModeMaskParam.Visible     =  'on';
        prodOutputModeMaskParam.Visible=  'on';
        if strcmp(accumMode, 'Binary point scaling')
            accumSignedText.Visible          = 'on';
            accumWordLengthMaskParam.Visible = 'on';
            accumFracLengthMaskParam.Visible = 'on';
        else
            accumSignedText.Visible = 'off';
            accumWordLengthMaskParam.Visible = 'off';
            accumFracLengthMaskParam.Visible = 'off';
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
        memoryModeMaskParam.Visible   =  'off';
        memoryModeSignedText.Visible  = 'off';
        memoryWordLength.Visible      = 'off';
        memoryFracLength.Visible      = 'off';
        memoryLabel.Visible           =  'off';
    else
        fixptOpParams.Visible                 = 'off';
        fixptDataTypes.Visible                = 'off';
        dataTypesParametersAreApplicable.Visible = 'on';
        floatingPointTrumpRule.Visible = 'off';
    end
    
elseif (strcmp(markerShape,'X-mark')||strcmp(markerShape,'Star'))
    
    if  strcmp(antialiasing, 'on')
        floatingPointTrumpRule.Visible    = 'on';
        fixptOpParams.Visible           = 'on';
        fixptDataTypes.Visible          = 'on';
        accumLabel.Visible             =  'on';
        prodOutputlabel.Visible        =  'on';
        memoryLabel.Visible            =  'off';
        accumLabel.Visible             =  'on';
        prodOutputlabel.Visible        =  'on';
        accumModeMaskParam.Visible     =  'on';
        prodOutputModeMaskParam.Visible=  'on';
        memoryModeMaskParam.Visible    =  'off';
        memoryModeSignedText.Visible  = 'off';
        memoryWordLength.Visible      = 'off';
        memoryFracLength.Visible      = 'off';
        memoryLabel.Visible           =  'off';
        if strcmp(accumMode, 'Binary point scaling')
            accumSignedText.Visible          = 'on';
            accumWordLengthMaskParam.Visible = 'on';
            accumFracLengthMaskParam.Visible = 'on';
        else
            accumSignedText.Visible = 'off';
            accumWordLengthMaskParam.Visible = 'off';
            accumFracLengthMaskParam.Visible = 'off';
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
    else
        fixptOpParams.Visible                 = 'off';
        fixptDataTypes.Visible                = 'off';
        dataTypesParametersAreApplicable.Visible = 'on';
        floatingPointTrumpRule.Visible = 'off';
    end
    
    
elseif strcmp(markerShape,'Square')
    if strcmp(filled,'on')
        floatingPointTrumpRule.Visible    = 'on';
        fixptOpParams.Visible             = 'on';
        fixptDataTypes.Visible            = 'on';
        memoryLabel.Visible               = 'on';
        accumLabel.Visible                = 'on';
        prodOutputlabel.Visible           = 'on';
        accumModeMaskParam.Visible        = 'on';
        prodOutputModeMaskParam.Visible   = 'on';
        memoryModeMaskParam.Visible       = 'on';
        if strcmp(accumMode, 'Binary point scaling')
            accumSignedText.Visible          = 'on';
            accumWordLengthMaskParam.Visible = 'on';
            accumFracLengthMaskParam.Visible = 'on';
        else
            accumSignedText.Visible = 'off';
            accumWordLengthMaskParam.Visible = 'off';
            accumFracLengthMaskParam.Visible = 'off';
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
        
        if strcmp(memoryMode, 'Specify word length')
            memoryModeSignedText.Visible = 'on';
            memoryWordLength.Visible     = 'on';
            memoryFracLength.Visible     = 'off';
        else
            memoryModeSignedText.Visible  = 'off';
            memoryWordLength.Visible      = 'off';
            memoryFracLength.Visible      = 'off';
        end
    else
        fixptOpParams.Visible                    = 'off';
        fixptDataTypes.Visible                   = 'off';
        dataTypesParametersAreApplicable.Visible = 'on';
        floatingPointTrumpRule.Visible           = 'off';
    end
else
    fixptOpParams.Visible                 = 'off';
    fixptDataTypes.Visible                = 'off';
    dataTypesParametersAreApplicable.Visible = 'on';
    floatingPointTrumpRule.Visible = 'off';
end

if (strcmp(accumMode, 'Same as first input')||strcmp(accumMode, 'Same as product output'))...
        && strcmp(prodOutputMode, 'Same as first input') && strcmp(memoryMode, 'Same word length as input')
    signedColumnLabel.Visible          = 'off';
    wordLengthColumnLabel.Visible      = 'off';
    fractionLengthColumnLabel.Visible  = 'off';
end
if (~strcmp(accumMode, 'Binary point scaling'))...
        && (~strcmp(prodOutputMode, 'Binary point scaling'))
    fractionLengthColumnLabel.Visible = 'off';
end
