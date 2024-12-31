function vipblkvideoToWsMask
% VIPBLK2DAUTOCORR Mask dynamic dialog function for 2D AutoCorrelation block

% Copyright 2020 The MathWorks, Inc.

  blk = gcbh;   % Cache handle to block
  dynamicBlockUpdate(blk)
end
function dynamicBlockUpdate(blk)

blkMask = Simulink.Mask.get(blk);

InPortLabelsValue =  blkMask.getParameter ('InPortLabels');
NumInputsValue = blkMask.getParameter('NumInputs');

if strcmp(NumInputsValue.Value, '1')
    InPortLabelsValue.Visible = 'off';
else
    InPortLabelsValue.Visible = 'on';
end
end
    