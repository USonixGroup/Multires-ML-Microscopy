function bbox = xywh2minmax(bbox)
% Convert bbox [x y w h] to minmax format [xmin ymin xmax ymax].
%
% % N.B. By convention xywh box are in spatial coordinates. minmax box are
% in spatial coordinates.

%   Copyright 2019-2021 The MathWorks, Inc.

% [xmin ymin xmax ymax] in spatial coordinates.
bbox(:,[1 2]) = bbox(:,[1 2]);
bbox(:,[3 4]) = bbox(:,[1 2]) + bbox(:,[3 4]);
