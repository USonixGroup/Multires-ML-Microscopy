function Opensubsystem(gcbh)
% When the button on the block mask is pressed this function is called.
% This function opens the subsystem where the user will place his sub matrix
% processing system.

% Copyright 2018 The MathWorks, Inc.

blk = gcbh;
bname = [get(blk,'Parent') '/' get(blk,'Name') '/Block iterator/sub-block process'];

open_system(bname);

end