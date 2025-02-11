function [b] = vipblkdrawshapes(action)
% VIPBLKROIDISPLAY Mask dynamic dialog function for Draw Shapes blocku

% Copyright 2003-2019 The MathWorks, Inc.
blk = gcbh;

switch action
    case 'icon'
        shapeToDraw = get_param(blk, 'shape');
        inLibrary = strcmp(get_param(bdroot(blk),'BlockDiagramType'),'library');
        if inLibrary
            ports.icon = 'Draw\nShapes';
        else
            if (strcmp(shapeToDraw,'Rectangles'))
                ports.icon = 'Draw\nRectangles';
            elseif (strcmp(shapeToDraw,'Lines'))
                ports.icon = 'Draw\nLines';
            elseif (strcmp(shapeToDraw,'Polygons'))
                ports.icon = 'Draw\nPolygons';
            else
                ports.icon = 'Draw\nCircles';
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
        dynamicBlockUpdate(blk);
end


% ----------------------------------------------------------
function ports = get_labels(blk)
% Change this to be either I or R,G and B.
% end of vipblkdrawshapes.m

%-----------------------------------------------------------
function dynamicBlockUpdate(blk)
blkMask = Simulink.Mask.get(gcb);


shapeToDraw              = get_param(blk, 'shape');
filled                   = get_param(blk, 'fill');
antialiasing             = get_param(blk, 'antialiasing');
borderClrSourceGetParam  = get_param(blk,'fillClrSource');
display                  = get_param(blk,'display');

borderClrSource          =  blkMask.getParameter('fillClrSource');
color                    =  blkMask.getParameter('color');
filledMaskParam          =  blkMask.getParameter('fill');
antialiasingMaskParam    =  blkMask.getParameter('antialiasing');
opacity                  =  blkMask.getParameter('opacity');
borderColor              =  blkMask.getParameter('display');
lineWidth                =  blkMask.getParameter('lineWidth');
lineLabel                = blkMask.getDialogControl('lineLabel');
rectangleLabel           = blkMask.getDialogControl('rectangleLabel');
polygonLabel             = blkMask.getDialogControl('polygonLabel');
circleLabel              = blkMask.getDialogControl('circleLabel');
fillClrSourceLabel       = blkMask.getDialogControl('fillClrSourceLabel');
fillColorLabel           = blkMask.getDialogControl('fillColorLabel');
if strcmp(shapeToDraw,'Circles')||strcmp(shapeToDraw,'Polygons')
    filledMaskParam.Visible       = 'on';
    antialiasingMaskParam.Visible = 'on';
elseif strcmp(shapeToDraw,'Lines')
    filledMaskParam.Visible       = 'off';
    antialiasingMaskParam.Visible = 'on';
    lineLabel.Visible             = 'on';
    rectangleLabel.Visible        = 'off';
    circleLabel.Visible           = 'off';
    polygonLabel.Visible          = 'off';
elseif strcmp(shapeToDraw,'Rectangles')
    filledMaskParam.Visible       = 'on';
    antialiasingMaskParam.Visible = 'off';
    rectangleLabel.Visible        = 'on';
    lineLabel.Visible             = 'off';
    circleLabel.Visible           = 'off';
    polygonLabel.Visible          = 'off';
end
if strcmp(shapeToDraw,'Circles')
    circleLabel.Visible   = 'on';
    lineLabel.Visible             = 'off';
    rectangleLabel.Visible        = 'off';
    polygonLabel.Visible          = 'off';
end
if strcmp(shapeToDraw,'Polygons')
    polygonLabel.Visible  = 'on';
    lineLabel.Visible             = 'off';
    rectangleLabel.Visible        = 'off';
    circleLabel.Visible           = 'off';
end

borderClrSource.Visible = 'on';
borderColor.Visible     = 'on';
if strcmp(filled, 'on')
    lineWidth.Visible               =  'off';
    fillClrSourceLabel.Prompt       = 'vision:masks:FillColorSource';
    fillColorLabel.Prompt           = 'vision:masks:FillColor';
    opacity.Visible                 = 'on';
else
    fillClrSourceLabel.Prompt          = 'vision:masks:BorderColorSource';
    fillColorLabel.Prompt              = 'vision:masks:BorderColor';
    lineWidth.Visible                  = 'on';
    opacity.Visible                    = 'off';
end

if strcmp(borderClrSourceGetParam, 'Specify via dialog')
    borderColor.Visible = 'on';
    fillColorLabel.Visible = 'on';
else
    borderColor.Visible = 'off';
    fillColorLabel.Visible = 'off';
end

if strcmp(display,'User-specified value')
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
memoryFractionLength      = blkMask.getDialogControl('memoryFractionLength');
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
fixptDataTypes            = blkMask.getDialogControl('fixedPtDataTypes');
dataTypesParametersAreApplicable = blkMask.getDialogControl('dataTypesParametersAreApplicable');
dataTypesParametersAreApplicable.Visible = 'off';
signedColumnLabel.Visible         = 'on';
wordLengthColumnLabel.Visible     = 'on';
fractionLengthColumnLabel.Visible = 'on';

if  strcmp(shapeToDraw,'Circles')
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
            memoryFractionLength.Visible = 'on';
        else
            memoryFractionLength.Visible  = 'off';
            memoryModeSignedText.Visible  = 'off';
            memoryWordLength.Visible      = 'off';
        end
    elseif strcmp(filled, 'off')&& strcmp(antialiasing, 'on')
        floatingPointTrumpRule.Visible  = 'on';
        fixptOpParams.Visible           = 'on';
        fixptDataTypes.Visible          = 'on';
        accumLabel.Visible              =  'on';
        prodOutputlabel.Visible         =  'on';
        accumModeMaskParam.Visible      =  'on';
        prodOutputModeMaskParam.Visible =  'on';
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
        memoryFractionLength.Visible  = 'off';
        memoryLabel.Visible           =  'off';
    else
        fixptOpParams.Visible                    = 'off';
        fixptDataTypes.Visible                   = 'off';
        dataTypesParametersAreApplicable.Visible = 'on';
        floatingPointTrumpRule.Visible           = 'off';
    end
elseif strcmp(shapeToDraw,'Polygons')
    if strcmp(filled, 'on')
        floatingPointTrumpRule.Visible  = 'on';
        fixptOpParams.Visible           = 'on';
        fixptDataTypes.Visible          = 'on';
        memoryLabel.Visible             =  'on';
        accumLabel.Visible              =  'on';
        prodOutputlabel.Visible         =  'on';
        accumModeMaskParam.Visible      =  'on';
        prodOutputModeMaskParam.Visible =  'on';
        memoryModeMaskParam.Visible     =  'on';
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
        else
            memoryModeSignedText.Visible  = 'off';
            memoryWordLength.Visible      = 'off';
        end
    else
        floatingPointTrumpRule.Visible    =  'on';
        fixptOpParams.Visible             =  'on';
        fixptDataTypes.Visible            =  'on';
        accumLabel.Visible                =  'on';
        prodOutputlabel.Visible           =  'on';
        accumModeMaskParam.Visible        =  'on';
        prodOutputModeMaskParam.Visible   =  'on';
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
        memoryModeMaskParam.Visible   = 'off';
        memoryModeSignedText.Visible  = 'off';
        memoryWordLength.Visible      = 'off';
        memoryFractionLength.Visible  = 'off';
        memoryLabel.Visible           = 'off';
    end
elseif strcmp(shapeToDraw,'Lines')
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
    memoryFractionLength.Visible  = 'off';
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
elseif strcmp(shapeToDraw,'Rectangles')
    if strcmp(filled,'on')
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
        else
            memoryModeSignedText.Visible  = 'off';
            memoryWordLength.Visible      = 'off';
        end
    else
        fixptOpParams.Visible                 = 'off';
        fixptDataTypes.Visible                = 'off';
        dataTypesParametersAreApplicable.Visible = 'on';
        floatingPointTrumpRule.Visible = 'off';
    end
end

memoryFractionLength.Visible     = 'off';
fractionLengthColumnLabel.Visible  = 'on';
if (strcmp(accumMode, 'Same as first input')||...
        strcmp(accumMode, 'Same as product output'))&& ...
        strcmp(prodOutputMode, 'Same as first input') &&...
        strcmp(memoryMode, 'Same word length as input')
    signedColumnLabel.Visible          = 'off';
    wordLengthColumnLabel.Visible      = 'off';
end
if (~(strcmp(accumMode, 'Binary point scaling')||...
        strcmp(prodOutputMode, 'Binary point scaling')))
    fractionLengthColumnLabel.Visible = 'off';
end

if (strcmp(accumMode, 'Binary point scaling')||...
        strcmp(prodOutputMode, 'Binary point scaling'))&&...
        (strcmp(filled,'on') && strcmp(memoryMode, 'Specify word length'))
    memoryFractionLength.Visible     = 'on';
end
