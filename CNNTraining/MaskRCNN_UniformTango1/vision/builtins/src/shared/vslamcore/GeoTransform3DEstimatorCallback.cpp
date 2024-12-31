/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/*
 * MW: This file contains the code derived from //3rdparty/R2023a/OpenCV/modules/calib3d/src/ptsetreg.cpp
 */

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/GeoTransform3DEstimatorCallback.hpp"
#else
    #include "GeoTransform3DEstimatorCallback.hpp"
#endif

using namespace cv;

namespace vision
{
    namespace vslam
    {

        int MW_RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters )
        {
            if( modelPoints <= 0 )
                CV_Error( Error::StsOutOfRange, "the number of model points should be positive" );
        
            p = MAX(p, 0.);
            p = MIN(p, 1.);
            ep = MAX(ep, 0.);
            ep = MIN(ep, 1.);
        
            // avoid inf's & nan's
            double num = MAX(1. - p, DBL_MIN);
            double denom = 1. - std::pow(1. - ep, modelPoints);
            if( denom < DBL_MIN )
                return 0;
        
            num = std::log(num);
            denom = std::log(denom);
        
            return denom >= 0 || -num >= maxIters*(-denom) ? maxIters : cvRound(num/denom);
        }

        Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb,
                                                         int _modelPoints, double _threshold,
                                                         double _confidence, int _maxIters)
        {
            return Ptr<PointSetRegistrator>(
                new MW_RANSACPointSetRegistrator(_cb, _modelPoints, _threshold, _confidence, _maxIters));
        }

        ///////////////////////////////////////////////////////////////////////////////
        // MW: Only the function below has been added. All the other code is derived
        //     from //3rdparty/R2023a/OpenCV/modules/calib3d/src/ptsetreg.cpp
        ///////////////////////////////////////////////////////////////////////////////
        
        /**
         * @brief Estimate similarity transformation matrix between 2 sets of matched 3-D points
         *
         * @param[in]  _from            points in frame 1, size Nx3 in mat
         * @param[in]  _to              points in frame 2, size Nx3 in mat
         * @param[in]  _imagePoints1    projected points in frame 1, specified as a vector of cv::KeyPoint
         * @param[in]  _imagePoints2    projected points in frame 2, specified as a vector of cv::KeyPoint
         * @param[in]  _intrinsics      camera intrinsics matrix, specified as a 3-by-3 matrix
         * @param[out]  _out            rotation matrix, size 3x3 in mat
         * @param[out]  _inliers        N-by-1 logical vector containing 1 for inliers and 0 for outliers
         * @param[in]  ransacThreshold  Maximum distance, in pixel, that a point can differ from the 
         *                              projected location of its associated point to be considered an inlier
         * @param[in]  confidence       Desired confidence (in percentage [0,1]) for finding the maximum 
         *                              number of inliers
         */
        int estimateSimilarity3D(InputArray _from, InputArray _to,
                                 const std::vector<cv::KeyPoint>& _imagePoints1, const std::vector<cv::KeyPoint>& _imagePoints2, 
                                 const Matx33d& _intrinsics,
                                 OutputArray _out, OutputArray _inliers,
                                 double ransacThreshold, double confidence)
        {
            Mat from = _from.getMat(), to = _to.getMat();
            int count = from.checkVector(3);
        
            CV_Assert( count >= 0 && to.checkVector(3) == count );
            
            Mat dFrom, dTo;
            from.convertTo(dFrom, CV_32F);
            to.convertTo(dTo, CV_32F);
            dFrom = dFrom.reshape(3, count);
            dTo = dTo.reshape(3, count);
            
            const double epsilon = DBL_EPSILON;
            ransacThreshold = ransacThreshold <= 0 ? 3 : ransacThreshold;
            confidence = (confidence < epsilon) ? 0.99 : (confidence > 1 - epsilon) ? 0.99 : confidence;
        
            return createRANSACPointSetRegistrator(makePtr<Similarity3DEstimatorCallback>(_imagePoints1, _imagePoints2, _intrinsics), 
                                                   4, ransacThreshold, confidence)->run(dFrom, dTo, _out, _inliers);
        }

         /**
         * @brief Estimate rigid transformation matrix between 2 sets of matched 3-D points
         *
         * @param[in]  _from            points in frame 1, size Nx3 in mat
         * @param[in]  _to              points in frame 2, size Nx3 in mat
         * @param[in]  _imagePoints1    projected points in frame 1, specified as a vector of cv::KeyPoint
         * @param[in]  _imagePoints2    projected points in frame 2, specified as a vector of cv::KeyPoint
         * @param[in]  _intrinsics      camera intrinsics matrix, specified as a 3-by-3 matrix
         * @param[out]  _out            rotation matrix, size 3x3 in mat
         * @param[out]  _inliers        N-by-1 logical vector containing 1 for inliers and 0 for outliers
         * @param[in]  ransacThreshold  Maximum distance, in pixel, that a point can differ from the 
         *                              projected location of its associated point to be considered an inlier
         * @param[in]  confidence       Desired confidence (in percentage [0,1]) for finding the maximum 
         *                              number of inliers
         */
        int estimateRigid3D(InputArray _from, InputArray _to,
                                 const std::vector<cv::KeyPoint>& _imagePoints1, const std::vector<cv::KeyPoint>& _imagePoints2, 
                                 const Matx33d& _intrinsics,
                                 OutputArray _out, OutputArray _inliers,
                                 double ransacThreshold, double confidence)
        {
            Mat from = _from.getMat(), to = _to.getMat();
            int count = from.checkVector(3);
        
            CV_Assert( count >= 0 && to.checkVector(3) == count );
            
            Mat dFrom, dTo;
            from.convertTo(dFrom, CV_32F);
            to.convertTo(dTo, CV_32F);
            dFrom = dFrom.reshape(3, count);
            dTo = dTo.reshape(3, count);
            
            const double epsilon = DBL_EPSILON;
            ransacThreshold = ransacThreshold <= 0 ? 3 : ransacThreshold;
            confidence = (confidence < epsilon) ? 0.99 : (confidence > 1 - epsilon) ? 0.99 : confidence;
        
            return createRANSACPointSetRegistrator(makePtr<Rigid3DEstimatorCallback>(_imagePoints1, _imagePoints2, _intrinsics), 
                                                   4, ransacThreshold, confidence)->run(dFrom, dTo, _out, _inliers);
        }

    } // namespace 
}