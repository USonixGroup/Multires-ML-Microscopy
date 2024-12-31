function registerVisionUtilityExtensions(ext)
%registerVisionUtilityExtensions

%   Copyright 2016-2022 The MathWorks, Inc.

uiscopes.addDataHandler(ext,'Streaming','Video','scopeextensions.VideoMLStreamingHandler');

r = ext.add('Visuals', ...
            'Point Cloud', ...
            'vipscopes.PointCloudVisual',...
            'Point Cloud Visualization', ...
            'Point Cloud');

% [EOF]
