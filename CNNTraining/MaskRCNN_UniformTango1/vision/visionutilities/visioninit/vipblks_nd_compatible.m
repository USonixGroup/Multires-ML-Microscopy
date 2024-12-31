function vipblk_nd_compatible
%

%   Copyright 2010-2020 The MathWorks, Inc.

if ~strcmp(get_param(gcb,'tag'),'vipblks_nd')
    set_param(gcb,'tag','vipblks_nd');
    set_param(gcbh,'imagePorts','Separate color signals');
end
