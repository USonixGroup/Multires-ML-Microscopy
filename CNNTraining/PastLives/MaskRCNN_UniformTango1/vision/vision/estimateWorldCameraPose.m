% estimateWorldCameraPose Estimate camera pose from 3-D to 2-D point correspondences
%
%--------------------------------------------------------------------------
%   estimateWorldCameraPose is not recommended. Use estworldpose instead.
%--------------------------------------------------------------------------
%
%   [worldOrientation, worldLocation] = estimateWorldCameraPose(imagePoints, worldPoints, cameraParams)
%   returns the orientation and location of a calibrated camera in the world coordinate
%   system in which worldPoints are defined. 
%
%   The function solves the Perspective-n-Point (PnP) problem using the P3P algorithm. 
%   The function eliminates spurious correspondences using the M-estimator SAmple Consensus 
%   (MSAC) algorithm
%
%   Inputs            Description
%   ------            -----------
%   imagePoints       M-by-2 array of [x,y] coordinates of undistorted image points,
%                     with M >= 4.  
%  
%   worldPoints       M-by-3 array of [x,y,z] coordinates of world points.
%  
%   cameraParams      a cameraParameters or cameraIntrinsics object
%  
%   Outputs           Description
%   -------            -----------
%   worldOrientation  orientation of the camera in the world coordinates specified
%                     as a 3-by-3 matrix
%   worldLocation     location of the camera in the world coordinates specified as a
%                     1-by-3 vector
%   
%   [..., inlierIdx] = estimateWorldCameraPose(...) additionally returns the indices of the 
%   inliers used to compute the camera pose. inlierIdx is an M-by-1 logical vector, 
%   with the values of true corresponding to the inliers.
% 
%   [..., status] = estimateWorldCameraPose(...) additionally returns a status code. If the 
%   status output is not specified, the function will issue an error if the number of 
%   input points or the number of inliers is less than 4. The status can have the following 
%   values:
%  
%       0: No error.
%       1: imagePoints and worldPoints do not contain enough points.
%          At least 4 points are required.
%       2: Not enough inliers found. At least 4 inliers are required.
% 
%   [...] = estimateWorldCameraPose(..., Name, Value) specifies additional name-value pair 
%   arguments described below:
% 
%   'MaxNumTrials'          Positive integer scalar.
%                           Specifies the number of random trials for finding
%                           the outliers. The actual number of trials depends
%                           on imagePoints, worldPoints, and the values of the 
%                           MaxReprojectionError and Confidence parameters. Increasing 
%                           this value will improve the robustness of the output 
%                           at the expense of additional computation.
%  
%                           Default: 1000
%  
%   'Confidence'            Scalar value greater than 0 and less than 100.
%                           Specifies the desired confidence (in percentage)
%                           for finding the maximum number of inliers. Increasing 
%                           this value will improve the robustness of the output 
%                           at the expense of additional computation.
%  
%                           Default: 99
%  
%   'MaxReprojectionError'  Positive numeric scalar specifying the reprojection
%                           error threshold in pixels for finding outliers. Increasing 
%                           this value will make the algorithm converge faster, 
%                           but may reduce the accuracy of the result.
%  
%                           Default: 1
%  
%    Notes
%    -----
%    The function does not account for lens distortion. You can either
%    undistort the images using the undistortImage function before 
%    detecting the image points, or you can undistort the image points 
%    themselves using the undistortPoints function.
% 
%    Class Support
%    -------------
%    imagePoints and worldPoints must be of the same class, which can be 
%    double or single. cameraParams must be a cameraParameters or
%    cameraIntrinsics object. orientation and location are of class double.
% 
%    Example: Determine camera pose from world-to-image correspondences
%    ------------------------------------------------------------------
%    data = load('worldToImageCorrespondences.mat');
%    [worldOrientation, worldLocation] = estimateWorldCameraPose(...
%        data.imagePoints, data.worldPoints, data.cameraParams);
%    pcshow(data.worldPoints, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'down', ...
%        'MarkerSize', 30);
%    hold on
%    plotCamera('Size', 10, 'Orientation', worldOrientation, 'Location',...
%        worldLocation);
%    
%    See also estrelpose, viewSet, triangulateMultiview, bundleAdjustment, 
%             plotCamera, pcshow, estimateExtrinsics, triangulate, pose2extr.

%  Copyright 2015-2022 The MathWorks, Inc.

% References:
% ----------
% [1] X.-S. Gao, X.-R. Hou, J. Tang, and H.-F. Cheng, "Complete Solution 
%     Classification for the Perspective-Three-Point Problem", IEEE Trans. 
%     Pattern Analysis and Machine Intelligence, vol. 25, no. 8, pp. 930-943, 
%     2003.

%#codegen

function [orientation, location, inlierIdx, status] = ...
    estimateWorldCameraPose(imagePoints, worldPoints, cameraParams, varargin)

% True if cameraParameters is a supported class for cameraParams
isCameraParamsSupported = true;

[orientation, location, inlierIdx, status, statusCode] = vision.internal.calibration.estWorldPoseImpl( ...
    imagePoints, worldPoints, cameraParams, isCameraParamsSupported, mfilename, varargin{:});

if nargout < 4
    checkRuntimeStatus(statusCode, status);
end

end

%==========================================================================
% Check runtime status and report error if there is one
%==========================================================================
function checkRuntimeStatus(statusCode, status)
coder.internal.errorIf(status==statusCode.NotEnoughPts, ...
    'vision:points:notEnoughMatchedPts', 'imagePoints', 'worldPoints', 4);

coder.internal.errorIf(status==statusCode.NotEnoughInliers, ...
    'vision:points:notEnoughInlierMatches', 'imagePoints', ...
    'worldPoints');
end