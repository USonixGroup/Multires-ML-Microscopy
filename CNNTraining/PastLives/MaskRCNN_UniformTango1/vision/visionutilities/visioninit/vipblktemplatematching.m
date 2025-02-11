function varargout = vipblktemplatematching(action)
% VIPBLKTEMPLATEMATCHING Mask dynamic dialog function for Template Matching Block

% Copyright 1995-2019 The MathWorks, Inc.

if nargin==0, action = 'dynamic'; end
blk = gcbh;   % Cache handle to block

switch action
    case 'icon'
        [iconStruct,blkname] = get_labels(blk);
        varargout(1) = {iconStruct};
        varargout(2) = {blkname};
        
    case 'init'
        dtInfo = dspGetFixptDataTypeInfo(blk,15);
        varargout(1) = {dtInfo};
        
    otherwise
        dynamicBlockUpdate(blk);
        
end

% ----------------------------------------------------------
function [ports,blkname]  = get_labels(blk)

metricStr = get_param(blk,'metric');

isBlkInLibrary = strcmp(get_param(bdroot(blk),'BlockDiagramType'),'library');
if isBlkInLibrary
    blkname = 'Template\nMatching';
else
    switch metricStr
        case 'Sum of absolute differences'
            blkname = 'Sum of\nAbsolute\nDifferences';
        case 'Sum of squared differences'
            blkname = 'Sum of\nSquared\nDifferences';
        case 'Maximum absolute difference'
            blkname = 'Maximum\nAbsolute\nDifference';
        otherwise
            blkname = 'Template\nMatching';
    end
end

outputMode      = get_param(blk,'output');
isBestMatchLoc  = strcmp(outputMode,'Best match location') ;
isMetricMatrix  = strcmp(outputMode,'Metric matrix') ;
isROI           = strcmp(get_param(blk,'roi'),'on');
isROIValidPort  = strcmp(get_param(blk, 'roiValid'),'on');

ports.iport1=1;
ports.itxt1='I';

ports.iport2=2;
ports.itxt2='T';

if (isBestMatchLoc && isROI)
    ports.iport3=3;
    ports.itxt3='ROI';
else
    ports.iport3=2;
    ports.itxt3='';
end

if (isMetricMatrix)
    ports.oport1=1;
    ports.otxt1='Metric';
    
    ports.oport2=1;
    ports.otxt2='';
    
    ports.oport3=1;
    ports.otxt3='';
    
    ports.oport4=1;
    ports.otxt4='';
elseif (isBestMatchLoc)
    ports.oport1=1;
    ports.otxt1='Loc';
    isNMetric = strcmp(get_param(blk,'nMetric'), 'on');
    if (isNMetric)
        ports.oport2=2;
        ports.otxt2='NMetric';
        ports.oport3=3;
        ports.otxt3='NValid';
        if (isROI && isROIValidPort)
            ports.oport4=4;
            ports.otxt4='ROIValid';
        else
            ports.oport4=1;
            ports.otxt4='';
        end
    else
        if (isROI && isROIValidPort)
            ports.oport4=2;
            ports.otxt4='ROIValid';
            ports.oport2=2;
            ports.otxt2='';
            ports.oport3=2;
            ports.otxt3='';
        else
            ports.oport4=1;
            ports.otxt4='';
            ports.oport2=1;
            ports.otxt2='';
            ports.oport3=1;
            ports.otxt3='';
        end
    end
else
    error(message('vision:internal:unhandledCase'));
end

function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(gcb);

outputValue = get_param(blk, 'output');
metricValue = get_param(blk, 'metric');

methodMaskParam    = blkMask.getParameter('method');
nMetricMaskParam   = blkMask.getParameter('nMetric');
roiMaskParam       = blkMask.getParameter('roi');
roiValidMaskParam  = blkMask.getParameter('roiValid');
nSizeMaskParam     = blkMask.getParameter('nSize');

ROIProcessing = blkMask.getDialogControl('ROIProcessing');

% =====================================================
% Visibility based on 'Metric Matrix' output
% =====================================================

if strcmp(outputValue,'Metric matrix')
    methodMaskParam.Visible  = 'off';
    nMetricMaskParam.Visible = 'off';
    roiMaskParam.Visible     = 'off';
    ROIProcessing.Visible    = 'off';
else
    methodMaskParam.Visible  = 'on';
    nMetricMaskParam.Visible = 'on';
    roiMaskParam.Visible     = 'on';
    ROIProcessing.Visible    = 'on';
end

% =====================================================
% Visibility based on 'Best match location' output
% =====================================================

nMetricValue = get_param(blk, 'nMetric');

if strcmp(outputValue,'Best match location') && strcmp(nMetricValue, 'on')
    nSizeMaskParam.Visible = 'on';
else
    nSizeMaskParam.Visible = 'off';
end

% =====================================================
% Visibility based on roi
% =====================================================

roiValue = get_param(blk, 'roi');

if strcmp(roiMaskParam.Visible,'on') && strcmp(roiValue, 'on')
    roiValidMaskParam.Visible = 'on';
else
    roiValidMaskParam.Visible = 'off';
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
prodOutputModeMaskParam       = blkMask.getParameter('prodOutputMode');
prodOutputWordLengthMaskParam = blkMask.getParameter('prodOutputWordLength');
prodOutputFracLengthMaskParam = blkMask.getParameter('prodOutputFracLength');
SignedColumnLabel          = blkMask.getDialogControl('SignedColumnLabel');
WordLengthColumnLabel     = blkMask.getDialogControl('WordLengthColumnLabel');
FractionLengthColumnLabel = blkMask.getDialogControl('FractionLengthColumnLabel');
accumSignedText           = blkMask.getDialogControl('accumSignedText');
outputLabel               = blkMask.getDialogControl('outputLabel');
outputSignedText          = blkMask.getDialogControl('outputSignedText');
prodOutputLabel           = blkMask.getDialogControl('prodOutputLabel');
prodOutputSignedText      = blkMask.getDialogControl('prodOutputSignedText');

SignedColumnLabel.Visible = 'on';
WordLengthColumnLabel.Visible = 'on';
FractionLengthColumnLabel.Visible = 'on';
if strcmp(accumMode, 'Binary point scaling')
    accumSignedText.Visible          = 'on';
    accumWordLengthMaskParam.Visible = 'on';
    accumFracLengthMaskParam.Visible = 'on';
else
    accumSignedText.Visible = 'off';
    accumWordLengthMaskParam.Visible = 'off';
    accumFracLengthMaskParam.Visible = 'off';
end

if strcmp(outputValue,'Best match location') && strcmp(nMetricValue, 'off')
    outputLabel.Visible               = 'off';
    outputModeMaskParam.Visible       = 'off';
    outputSignedText.Visible          = 'off';
    outputWordLengthMaskParam.Visible = 'off';
    outputFracLengthMaskParam.Visible = 'off';
else
    outputLabel.Visible = 'on';
    outputModeMaskParam.Visible = 'on';
    
    if strcmp(outputMode, 'Binary point scaling')
        outputSignedText.Visible          = 'on';
        outputWordLengthMaskParam.Visible = 'on';
        outputFracLengthMaskParam.Visible = 'on';
    else
        outputSignedText.Visible = 'off';
        outputWordLengthMaskParam.Visible = 'off';
        outputFracLengthMaskParam.Visible = 'off';
        
    end
end

if (strcmp(metricValue,'Sum of squared differences'))
    
    prodOutputLabel.Visible         = 'on';
    prodOutputModeMaskParam.Visible = 'on';
    
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
    prodOutputLabel.Visible               = 'off';
    prodOutputModeMaskParam.Visible       = 'off';
    prodOutputSignedText.Visible          = 'off';
    prodOutputWordLengthMaskParam.Visible = 'off';
    prodOutputFracLengthMaskParam.Visible = 'off';
    
end

if (strcmp(metricValue,'Sum of squared differences')) &&...
        (strcmp(outputValue,'Best match location') &&...
        strcmp(nMetricValue, 'on'))
    
    if  ~strcmp(prodOutputMode, 'Binary point scaling') &&...
            ~strcmp(outputMode, 'Binary point scaling') &&...
            ~strcmp(accumMode, 'Binary point scaling')
        SignedColumnLabel.Visible = 'off';
        WordLengthColumnLabel.Visible = 'off';
        FractionLengthColumnLabel.Visible = 'off';
    end
elseif (~strcmp(metricValue,'Sum of squared differences')) &&...
        (strcmp(outputValue,'Best match location') &&...
        strcmp(nMetricValue, 'off'))
    
    if ~strcmp(outputMode, 'Binary point scaling') &&...
            ~strcmp(accumMode, 'Binary point scaling')
        SignedColumnLabel.Visible = 'off';
        WordLengthColumnLabel.Visible = 'off';
        FractionLengthColumnLabel.Visible = 'off';
    end
elseif  (~strcmp(metricValue,'Sum of squared differences')) &&...
        (~(strcmp(outputValue,'Best match location') &&...
        strcmp(nMetricValue, 'off')))
    
    if ~strcmp(accumMode, 'Binary point scaling')
        SignedColumnLabel.Visible = 'off';
        WordLengthColumnLabel.Visible = 'off';
        FractionLengthColumnLabel.Visible = 'off';
    end
end
% =====================================================
% Fixpt tab Signed
% =====================================================
accumSignedText = blkMask.getDialogControl('accumSignedText');

if (strcmp(metricValue,'Sum of squared differences'))
    accumSignedText.Prompt = 'Simulink:dialog:yes_CB';
else
    accumSignedText.Prompt = 'Simulink:dialog:UDTSameAsInputRule';
end
