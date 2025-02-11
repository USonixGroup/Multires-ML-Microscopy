function vipblkimfile(blk, action)
% VIPBLKHOUGH Mask dynamic dialog function for Hough Transform BLock

% Copyright 2018-2020 The MathWorks, Inc.


% This function modifies settings of blocks in the subsystem. This
% should not dirty the model:

model = bdroot(blk);
dirtyFlag = get_param(model,'Dirty');
c = onCleanup(@()set_param(model,'Dirty',dirtyFlag));



switch action
      case 'browse'
        % Select file via browse button
        [filename, pathname] = uigetfile('*');
        if filename
            fname = fullfile(pathname,filename);
            set_param(blk,'Filename',fname);
        end
    case 'dynamic'
        dynamicBlockUpdate(blk)
end
function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(blk);
imagePorts         = get_param(blk, 'imagePorts');
imageDataType      = get_param(blk, 'ImageDataType');
fractionLengthMode = get_param(blk, 'FractionLengthMode');

sdImageDataType    = blkMask.getParameter('sdImageDataType');
outPortLabels      = blkMask.getParameter('OutPortLabels');
signed             = blkMask.getParameter('Signed');
wordLength         = blkMask.getParameter('WordLength');
flModeMaskParam    = blkMask.getParameter('FractionLengthMode');
fractionLength     = blkMask.getParameter('FractionLength');
if strcmp(imagePorts,'Separate color signals')
    outPortLabels.Visible = 'on';
else
    outPortLabels.Visible = 'off';
end
if strcmp(imageDataType,'Fixed-point')
    sdImageDataType.Visible   = 'off';
    signed.Visible            = 'on';
    wordLength.Visible        = 'on';
    flModeMaskParam.Visible   = 'on';
    if strcmp(fractionLengthMode,'Best precision')
     fractionLength.Visible   = 'off';
    else
     fractionLength.Visible   = 'on';
    end
elseif strcmp(imageDataType,'User-defined')
    sdImageDataType.Visible = 'on';
    signed.Visible          = 'off';
    wordLength.Visible      = 'off';
    flModeMaskParam.Visible = 'off';
    fractionLength.Visible  = 'off';
else
    sdImageDataType.Visible = 'off';
    signed.Visible          = 'off';
    wordLength.Visible      = 'off';
    flModeMaskParam.Visible = 'off';
    fractionLength.Visible  = 'off';
end

    
