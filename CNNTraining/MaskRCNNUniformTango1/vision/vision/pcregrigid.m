function [tform, movingReg, rmse] = pcregrigid(moving, fixed, varargin)
%PCREGRIGID Register two point clouds with ICP algorithm.
%   PCREGRIGID is not recommended. Use PCREGISTERICP instead.
%
%   tform = PCREGRIGID(moving, fixed) returns the rigid transformation that
%   registers the moving point cloud with the fixed point cloud. moving and
%   fixed are pointCloud object. tform is an affine3d object that describes
%   the rigid 3-D transform. The rigid transformation between the moving
%   and fixed are estimated by Iterative Closest Point (ICP) algorithm.
%   Consider downsampling point clouds using pcdownsample before using
%   PCREGRIGID to improve accuracy and efficiency of registration.
%
%   [tform, movingReg] = PCREGRIGID(moving, fixed) additionally
%   returns the transformed  point cloud, movingReg, that is aligned with
%   the fixed point cloud.
%
%   [..., rmse] = PCREGRIGID(moving, fixed) additionally returns the root
%   mean squared error of the Euclidean distance between the inlier points
%   of the aligned point clouds.
% 
%   [...] = PCREGRIGID(...,Name, Value) specifies additional
%   name-value pairs described below:
%
%   'Metric'            A string used to specify the metric of the
%                       minimization function. The ICP algorithm minimized
%                       the distance between the two point clouds according
%                       to the given metric. Valid strings are
%                       'pointToPoint' and 'pointToPlane'. Setting Metric
%                       to 'pointToPlane' can reduce the number of
%                       iterations to process. However, this metric
%                       requires extra algorithmic steps within each
%                       iteration. The 'pointToPlane' metric helps
%                       registration of planar surfaces.
%
%                       Default: 'pointToPoint'
%
%   'Extrapolate'       A boolean to turn on/off the extrapolation step
%                       that traces out a path in the registration state
%                       space, described in the original paper of ICP by
%                       Besl and McKay (1992). This may reduce the number
%                       of iterations to converge.
%                           
%                       Default: false
%
%   'InlierRatio'       A scalar to specify the percentage of inliers.
%                       During an ICP iteration, every point in the moving
%                       point cloud is matched to its nearest neighbor in
%                       the fixed point cloud. The pair of matched points
%                       is considered as an inlier if its Euclidean
%                       distance falls into the given percentage of the
%                       distribution of matching distance. By default, all
%                       matching pairs are used.
%
%                       Default: 1
%
%   'InitialTransform'  An affine3d object to specify the initial rigid
%                       transformation. This is useful when a coarse
%                       estimation can be provided externally.
%
%                       Default: affine3d()
%
%   'MaxIterations'     A positive integer to specify the maximum number 
%                       of iterations before ICP stops.
%
%                       Default: 20
%
%   'Tolerance'         A 2-element vector, [Tdiff, Rdiff], to specify
%                       the tolerance of absolute difference in translation
%                       and rotation estimated in consecutive ICP
%                       iterations. Tdiff measures the Euclidean distance
%                       between two translation vectors, while Rdiff
%                       measures the angular difference in radians. The
%                       algorithm stops when the average difference between
%                       estimated rigid transformations in the three most
%                       recent consecutive iterations falls below the
%                       specified tolerance value.
%
%                       Default: [0.01, 0.009] 
%
%   'Verbose'           Set true to display progress information.
%
%                       Default: false
%
%   Notes
%   -----
%   - The registration algorithm is based on Iterative Closest Point (ICP)
%     algorithm, which is an iterative process. Best performance might
%     require adjusting the options for different data.
%
%   - Point cloud normals are required by the registration algorithm when
%    'pointToPlane' metric is chosen. If the Normal property of the second
%    input is empty, the function fills it.
%
%   - When the Normal property is filled automatically, the number of
%     points, K, to fit local plane is set to 6. This default may not work
%     under all circumstances. If the registration with 'pointToPlane'
%     metric fails, consider calling pcnormals function with a custom value
%     of K.
%
%   Class Support 
%   ------------- 
%   moving and fixed must be pointCloud object.
%
%   Example: Align two point clouds
%   --------------------------------
%   ptCloud = pcread('teapot.ply');
%   figure
%   pcshow(ptCloud) 
%   title('Teapot')
%
%   % Create a transform object with 30 degree rotation along z-axis and
%   % translation [5, 5, 10]
%   A = [cos(pi/6) sin(pi/6) 0 0; ...
%       -sin(pi/6) cos(pi/6) 0 0; ...
%               0         0  1 0; ...
%               5         5 10 1];
%   tform1 = affine3d(A);
%
%   % Transform the point cloud
%   ptCloudTformed = pctransform(ptCloud, tform1);
%
%   figure
%   pcshow(ptCloudTformed)
%   title('Transformed Teapot')
%
%   % Apply the rigid registration
%   [tform, ptCloudReg] = pcregrigid(ptCloudTformed, ptCloud, 'Extrapolate', true);
%
%   % Visualize the alignment
%   pcshowpair(ptCloud, ptCloudReg)
%
%   % Compare the result with the true transformation
%   disp(tform1.T);
%   tform2 = invert(tform);
%   disp(tform2.T);
%
% See also pointCloud, pcregistericp, pctransform, affine3d, pcshow, 
%          pcdownsample, pcshowpair, pcdenoise, pcmerge
 
% Copyright 2014-2023 The MathWorks, Inc.
%
% References
% ----------
% Besl, Paul J.; N.D. McKay (1992). "A Method for Registration of 3-D
% Shapes". IEEE Trans. on Pattern Analysis and Machine Intelligence (Los
% Alamitos, CA, USA: IEEE Computer Society) 14 (2): 239-256.
%
% Chen, Yang; Gerard Medioni (1991). "Object modelling by registration of
% multiple range images". Image Vision Comput. (Newton, MA, USA:
% Butterworth-Heinemann): 145-155

warning(message('vision:pointcloud:regRigidDeprecation'));

% Validate inputs
narginchk(2, 16)

varargin{end+1} = 'UseDegree';
varargin{end+1} = false;

[unorgMoving, unorgFixed, configOptions] = vision.internal.pc.parseICPInputs( ...
    moving, fixed, mfilename, varargin{:});

icpReg = vision.internal.pc.ICPRegistrationGPU(unorgMoving, unorgFixed, configOptions);

icpReg.initializeRegistration;

[tform, rmse] = icpReg.registerPointClouds;

if nargout > 1
    movingReg = pctransform(moving, tform);
end

end
