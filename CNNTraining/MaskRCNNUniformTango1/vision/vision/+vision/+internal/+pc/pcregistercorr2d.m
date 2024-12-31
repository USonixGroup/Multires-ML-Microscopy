function [tform, peak] = pcregistercorr2d(occupancyGridMoving,...
                                occupancyGridFixed, windowTF)
%pcregistercorr2d Register occupancy grids of point cloud objects
%   tform = pcregistercorr2d(occupancyGridMoving, occupancyGridFixed)
%   returns a rigid2d transformation object specifying the transformation
%   between two occupancyGrids. occupancyGridMoving and occupancyGridFixed
%   are 2-d occupancy grids obtained from the moving and fixed point
%   clouds.

% Copyright 2020-2024 The Mathworks, Inc.
%
% References:
% -----------
% M. Dimitrievski, D. Van Hamme, P. Veelaert and W. Philips, "Robust
% matching of occupancy maps for odometry in autonomous vehicles", (2016)
% Proceedings of 2016 VISAPP. p.626-633

%#codegen

% imregcorr reports the transformation from the origin. Hence provide a
% referencing such that the sensor is at the center of the images
Rgrid = imref2d(size(occupancyGridMoving));
offsetX = mean(Rgrid.XWorldLimits);
Rgrid.XWorldLimits = Rgrid.XWorldLimits - offsetX;
offsetY = mean(Rgrid.YWorldLimits);
Rgrid.YWorldLimits = Rgrid.YWorldLimits - offsetY;

[tform, peak] = imregcorr(occupancyGridMoving, Rgrid, occupancyGridFixed,...
                    Rgrid, 'Window', windowTF, 'Method', 'phasecorr');

end

