% relativeCameraPose Compute relative up-to-scale pose of calibrated camera
%
%--------------------------------------------------------------------------
%   relativeCameraPose is not recommended. Use estrelpose instead.
%--------------------------------------------------------------------------
%
%   [relativeOrientation, relativeLocation] = relativeCameraPose(M,
%   cameraParams, inlierPoints1, inlierPoints2) returns the orientation and
%   up-to-scale location of a calibrated camera relative to its previous
%   pose. relativeLocation is always a unit vector.
%
%   M is an essential or fundamental 3-by-3 matrix, or a projective2d
%   object containing a homography matrix. cameraParams is a
%   cameraParameters or cameraIntrinsics object. inlierPoints1 and
%   inlierPoints2 are matching inlier points from the two views
%   corresponding to the two poses. M, inlierPoints1, and inlierPoints2 are
%   returned by the estimateEssentialMatrix, estimateFundamentalMatrix,
%   or estimateGeometricTransform2D functions. relativeOrientation is a
%   3-by-3-by-N rotation matrix. relativeLocation is a N-by-3 matrix with a
%   unit vector at each row. N is one if M is an essential or fundamental
%   matrix, and is up to two if M is a projective2d object.
%
%   [...] = relativeCameraPose(M, cameraParams1, cameraParams2, inlierPoints1, inlierPoints2)
%   returns the orientation and location of camera 2 relative to camera 1.
%   cameraParams1 and cameraParams2 are cameraParameters or
%   cameraIntrinsics objects containing the parameters of camera 1 and
%   camera 2 respectively.
%
%   [..., validPointsFraction] = relativeCameraPose(...) additionally
%   returns the fraction of the inlier points that project in front of both
%   cameras. If this fraction is too small (e. g. less than 0.9), that can
%   indicate that the input matrix M is unreliable.
%
%   Notes
%   -----
%   - You can compute the camera extrinsics as follows:
%     [rotationMatrix, translationVector] = cameraPoseToExtrinsics(
%       relativeOrientation, relativeLocation)
%
%   - The relativeCameraPose function uses inlierPoints1 and inlierPoints2 to
%     determine which one of the multiple possible solutions is physically
%     realizable. If the input M is a projective2d object, there could be
%     up to two solutions that are equally realizable.
%
%    Class Support
%    -------------
%    M must be double or single. cameraParams must be a cameraParameters or
%    cameraIntrinsics object. inlierPoints1 and inlierPoints2 can be
%    double, single, or any of the point feature types. location and
%    orientation are the same class as M.
%
%  See also estworldpose, cameraCalibrator, estimateCameraParameters, 
%    estimateEssentialMatrix, estimateFundamentalMatrix, cameraProjection,
%    plotCamera, triangulate, triangulateMultiview, pose2extr,
%    estimateExtrinsics, estgeotform2d.

% Copyright 2015-2022 The MathWorks, Inc.

% References:
% -----------
% [1] R. Hartley, A. Zisserman, "Multiple View Geometry in Computer
% Vision," Cambridge University Press, 2003.
%
% [2] R. Hartley, P. Sturm. "Triangulation." Computer Vision and
% Image Understanding. Vol 68, No. 2, November 1997, pp. 146-157
%
% [3] O. Faugeras and F. Lustman, “Motion and structure from motion in a
% piecewise planar environment”, in International Journal of Pattern
% Recognition and Artificial Intelligence, 2(3):485–508, 1988.

%#codegen

function [orientation, location, validPointsFraction] = ...
    relativeCameraPose(F, varargin)

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = true;

[orientation, location, validPointsFraction] = vision.internal.calibration.estRelPoseImpl( ...
    F, isCameraParamsSupported, mfilename, varargin{:});