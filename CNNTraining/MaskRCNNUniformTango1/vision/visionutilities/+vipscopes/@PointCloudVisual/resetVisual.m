function resetVisual(this)
%RESETVISUAL Blank out the Point Cloud on the visual.

% %   Copyright 2022 The MathWorks, Inc.
% 
[X, Y, Z] = sphere(1);
this.XData= [0.5*X(:); 0.75*X(:); X(:)];
this.YData= [0.5*Y(:); 0.75*Y(:); Y(:)];
this.ZData= [0.5*Z(:); 0.75*Z(:); Z(:)];
view(this.PcplayerObj, [this.XData, this.YData, this.ZData]);

% [EOF]