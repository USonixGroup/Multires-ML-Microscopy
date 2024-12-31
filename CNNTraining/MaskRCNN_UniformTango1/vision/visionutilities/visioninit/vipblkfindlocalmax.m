function s = vipblkfindlocalmax(action)
% VIPBLKHOUGHPEAKS Video Processing Blockset Find Local Maxima mask helper function.

% Copyright 1995-2018 The MathWorks, Inc.
%

blk = gcbh;


isR2009bOrLater = strcmp(get_param(blk,'src_thresh'), 'Obsolete9b');

if isR2009bOrLater
    puSRCTHCstr = get_param(blk, 'src_thresh_inuse');
else
    puSRCTHCstr = get_param(blk, 'src_thresh');
end

isOutVarDim = strcmp(get_param(blk,'isOutVarDim'),'on');

switch action
    case 'icon'
        
        % input port label
        if strcmp(get_param(blk,'inputIsHough'),'on')
            inPort1label = 'Hough';
        else
            inPort1label = 'I';
        end
        
        s(1).port = 1;
        s(1).txt = inPort1label;
        
        if strncmp(puSRCTHCstr, 'Input ports',1)
            s(2).port = 2;
            s(2).txt = 'Th';
        else
            s(2).port = 1;
            s(2).txt = inPort1label;
        end
        
        s(3).port = 1;
        s(3).txt = 'Idx';
        
        if isR2009bOrLater && isOutVarDim
            s(4).port = 1;
            s(4).txt = 'Idx';
        else
            s(4).port = 2;
            s(4).txt = 'Count';
        end
        
    case 'backwardcompatibility'
        if ~isR2009bOrLater
            set_param(blk, 'isOutVarDim', 'off');
            set_param(blk, 'src_thresh_inuse', get_param(blk, 'src_thresh'));
            set_param(blk, 'src_thresh', 'Obsolete9b');
        end
    case 'dynamic'
        dynamicBlockUpdate(blk);
end % end switch

function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(blk);

sourceOfThresholdValue   = get_param(blk, 'src_thresh_inuse');
outputVariableSizeSignal = get_param(blk, 'isOutVarDim');
threshold                = blkMask.getParameter('threshold');
countOutputDataType      = blkMask.getParameter('dt_count');

if strcmp(sourceOfThresholdValue, 'Specify via dialog')
    threshold.Visible = 'on';
else
    threshold.Visible = 'off';
end
if strcmp(outputVariableSizeSignal, 'off')
    countOutputDataType.Visible = 'on';
else
    countOutputDataType.Visible = 'off';
end



% [EOF] vipblkfindlocalmax.m
