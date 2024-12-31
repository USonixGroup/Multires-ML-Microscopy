function vipblk2dhistogram
% VIPBLK2DHISTOGRAM is a Mask dynamic function for 2-D Histogram
% Block of visionstatistics library

% Copyright 2018 The MathWorks, Inc.


blk = gcbh;
blkMask = Simulink.Mask.get(blk);

run  =   get_param(blk, 'run');
trigtype =   blkMask.getParameter('trigtype');
if strcmp(run, 'on')
    trigtype.Visible = 'on';
else
    trigtype.Visible = 'off';
end