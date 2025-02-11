function [t1,t2] = estimateUncalibratedRectification(f,inlierPoints1,inlierPoints2,imageSize)
%estimateUncalibratedRectification Uncalibrated stereo rectification
%
%--------------------------------------------------------------------------
%  estimateUncalibratedRectification is not recommended.
%  Use estimateStereoRectification instead.
%--------------------------------------------------------------------------
%
%   [T1,T2] = estimateUncalibratedRectification(F, inlierPoints1, inlierPoints2, imageSize)
%   returns projective transformations for rectifying stereo images. This
%   function does not require specification of intrinsic and extrinsic 
%   parameters.
%
%   Inputs:
%   -------
%   inlierPoints1 - Corresponding points in stereo images I1 and I2    
%   inlierPoints2   respectively, specified as cornerPoints objects, 
%                   SIFTPoints objects, SURFPoints objects, MSERRegions 
%                   objects, BRISKPoints objects, ORBPoints objects or 
%                   M-by-2 matrices of [x,y] coordinates.
%
%   F             - Fundamental matrix for the stereo images, specified as
%                   a 3-by-3 single or double matrix. If P1, a point in 
%                   image I1, corresponds to P2 in image I2, then
%                       [P2,1] * F * [P1,1]' = 0.
%
%   imageSize     - Size of image I2, as returned by the size function.
%
%   Outputs
%   -------
%   T1, T2        - T1 and T2 are 3-by-3 matrices describing the projective
%                   transformations for I1 and I2, respectively. T1 and T2
%                   are the same class as F.
%
%   Notes
%   -----
%   Applying T1 (or T2) to I1 (or I2) may result in an undesired
%   distortion, if the epipole is located within I1 (or I2). You can use
%   function isEpipoleInImage to check if the epipole is inside the image.
%
%
%   Example
%   -------
%   % Load the stereo images and feature points which are already matched.
%   I1 = imread('yellowstone_left.png');
%   I2 = imread('yellowstone_right.png');
%   load yellowstone_inlier_points;
%
%   % Display point correspondences. Notice that the inlier points are in
%   % different rows, indicating that the stereo pair is not rectified.
%   showMatchedFeatures(I1, I2, inlier_points1, inlier_points2, 'montage');
%   title('Original images and inlier feature points');
%
%   % Compute the fundamental matrix from the corresponding points.
%   f = estimateFundamentalMatrix(inlier_points1, inlier_points2,...
%     'Method', 'Norm8Point');
%
%   % Compute the rectification transformations.
%   [t1, t2] = estimateUncalibratedRectification(f, inlier_points1, ...
%                inlier_points2, size(I2));
%
%   % Rectify the stereo images using projective transformations t1 and t2.
%   [I1Rect, I2Rect] = rectifyStereoImages(I1, I2, t1, t2);
%
%   % Display the stereo anaglyph, which can be viewed with 3-D glasses.
%   figure;
%   imshow(stereoAnaglyph(I1Rect, I2Rect));
%
% See also estimateFundamentalMatrix, rectifyStereoImages,
%   detectHarrisFeatures, detectMinEigenFeatures, detectFASTFeatures,
%   detectSURFFeatures, detectMSERFeatures, detectBRISKFeatures,
%   detectORBFeatures, extractFeatures, matchFeatures.

% References:
%   R. Hartley, A. Zisserman, "Multiple View Geometry in Computer Vision,"
%   Cambridge University Press, 2003.

% Copyright 2010-2022 The MathWorks, Inc.

%#codegen

[t1, t2] = vision.internal.calibration.estimateStereoRectificationImpl(f, ...
    inlierPoints1, inlierPoints2, imageSize, mfilename, false);

end