function vipblk2dmedian

% VIPBLK2DMEDIAN is a Mask dynamic function for 2-D Median
% Block of visionstatistics library

% Copyright 2018 The MathWorks, Inc.

blk = gcbh;
blkMask = Simulink.Mask.get(blk);

directionMode          =   get_param(blk, 'directionMode');
dimension              =   blkMask.getParameter('dimension');
if strcmp(directionMode, 'Specified dimension')
    dimension.Visible   = 'on';
else
    dimension.Visible   = 'off';
end