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

#ifndef __GEOTRANSFORM_3D_ESTIMATOR_CALLBACK__
#define __GEOTRANSFORM_3D_ESTIMATOR_CALLBACK__

#include "mw_precomp.hpp"
#include "opencv2/core.hpp"

#include <algorithm>
#include <iterator>
#include <limits>
#include <cmath>

#include "mw_usac.hpp"
#include "converter.hpp"

using namespace cv;

namespace vision
{
    namespace vslam
    {

        int MW_RANSACUpdateNumIters(double p, double ep, int modelPoints, int maxIters);

        class MW_RANSACPointSetRegistrator : public PointSetRegistrator
        {
        public:
            MW_RANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb = Ptr<PointSetRegistrator::Callback>(),
                int _modelPoints = 0, double _threshold = 0, double _confidence = 0.99, int _maxIters = 1000)
                : cb(_cb), modelPoints(_modelPoints), threshold(_threshold), confidence(_confidence), maxIters(_maxIters) {}

            int findInliers(const Mat& m1, const Mat& m2, const Mat& model, Mat& err, Mat& mask, double thresh) const
            {
                cb->computeError(m1, m2, model, err);
                mask.create(err.size(), CV_8U);

                CV_Assert(err.isContinuous() && err.type() == CV_32F && mask.isContinuous() && mask.type() == CV_8U);
                const float* errptr = err.ptr<float>();
                uchar* maskptr = mask.ptr<uchar>();
                float t = (float)(thresh * thresh);
                int i, n = (int)err.total(), nz = 0;
                for (i = 0; i < n; i++)
                {
                    int f = errptr[i] <= t;
                    maskptr[i] = (uchar)f;
                    nz += f;
                }

                return nz;
            }

            bool getSubset(const Mat& m1, const Mat& m2, Mat& ms1, Mat& ms2, RNG& rng, int maxAttempts = 1000) const
            {
                cv::AutoBuffer<int> _idx(modelPoints);
                int* idx = _idx.data();

                const int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
                const int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;

                int esz1 = (int)m1.elemSize1() * d1;
                int esz2 = (int)m2.elemSize1() * d2;
                CV_Assert((esz1 % sizeof(int)) == 0 && (esz2 % sizeof(int)) == 0);
                esz1 /= sizeof(int);
                esz2 /= sizeof(int);

                const int count = m1.checkVector(d1);
                const int count2 = m2.checkVector(d2);
                CV_Assert(count >= modelPoints && count == count2);

                const int* m1ptr = m1.ptr<int>();
                const int* m2ptr = m2.ptr<int>();

                ms1.create(modelPoints, 1, CV_MAKETYPE(m1.depth(), d1));
                ms2.create(modelPoints, 1, CV_MAKETYPE(m2.depth(), d2));

                int* ms1ptr = ms1.ptr<int>();
                int* ms2ptr = ms2.ptr<int>();

                for (int iters = 0; iters < maxAttempts; ++iters)
                {
                    int i;

                    for (i = 0; i < modelPoints; ++i)
                    {
                        int idx_i;

                        for (idx_i = rng.uniform(0, count);
                            std::find(idx, idx + i, idx_i) != idx + i;
                            idx_i = rng.uniform(0, count))
                        {
                        }

                        idx[i] = idx_i;

                        for (int k = 0; k < esz1; ++k)
                            ms1ptr[i * esz1 + k] = m1ptr[idx_i * esz1 + k];

                        for (int k = 0; k < esz2; ++k)
                            ms2ptr[i * esz2 + k] = m2ptr[idx_i * esz2 + k];
                    }

                    if (cb->checkSubset(ms1, ms2, i))
                        return true;
                }

                return false;
            }

            bool run(InputArray _m1, InputArray _m2, OutputArray _model, OutputArray _mask) const CV_OVERRIDE
            {
                bool result = false;
                Mat m1 = _m1.getMat(), m2 = _m2.getMat();
                Mat err, mask, model, bestModel, ms1, ms2;

                int iter, niters = MAX(maxIters, 1);
                int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
                int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
                int count = m1.checkVector(d1), count2 = m2.checkVector(d2), maxGoodCount = 0;

                RNG rng((uint64)-1);

                CV_Assert(cb);
                CV_Assert(confidence > 0 && confidence < 1);

                CV_Assert(count >= 0 && count2 == count);
                if (count < modelPoints)
                    return false;

                Mat bestMask0, bestMask;

                if (_mask.needed())
                {
                    _mask.create(count, 1, CV_8U, -1, true);
                    bestMask0 = bestMask = _mask.getMat();
                    CV_Assert((bestMask.cols == 1 || bestMask.rows == 1) && (int)bestMask.total() == count);
                }
                else
                {
                    bestMask.create(count, 1, CV_8U);
                    bestMask0 = bestMask;
                }

                if (count == modelPoints)
                {
                    if (cb->runKernel(m1, m2, bestModel) <= 0)
                        return false;
                    bestModel.copyTo(_model);
                    bestMask.setTo(Scalar::all(1));
                    return true;
                }

                for (iter = 0; iter < niters; iter++)
                {
                    int i, nmodels;
                    if (count > modelPoints)
                    {
                        bool found = getSubset(m1, m2, ms1, ms2, rng, 10000);
                        if (!found)
                        {
                            if (iter == 0)
                                return false;
                            break;
                        }
                    }

                    nmodels = cb->runKernel(ms1, ms2, model);
                    if (nmodels <= 0)
                        continue;
                    CV_Assert(model.rows % nmodels == 0);
                    Size modelSize(model.cols, model.rows / nmodels);

                    for (i = 0; i < nmodels; i++)
                    {
                        Mat model_i = model.rowRange(i * modelSize.height, (i + 1) * modelSize.height);
                        int goodCount = findInliers(m1, m2, model_i, err, mask, threshold);

                        if (goodCount > MAX(maxGoodCount, modelPoints - 1))
                        {
                            std::swap(mask, bestMask);
                            model_i.copyTo(bestModel);
                            maxGoodCount = goodCount;
                            niters = MW_RANSACUpdateNumIters(confidence, (double)(count - goodCount) / count, modelPoints, niters);
                        }
                    }
                }

                if (maxGoodCount > 0)
                {
                    if (bestMask.data != bestMask0.data)
                    {
                        if (bestMask.size() == bestMask0.size())
                            bestMask.copyTo(bestMask0);
                        else
                            transpose(bestMask, bestMask0);
                    }
                    bestModel.copyTo(_model); 
                    result = true;
                }
                else
                    _model.release();

                return result;
            }

            void setCallback(const Ptr<PointSetRegistrator::Callback>& _cb) CV_OVERRIDE { cb = _cb; }

            Ptr<PointSetRegistrator::Callback> cb;
            int modelPoints;
            double threshold;
            double confidence;
            int maxIters;
        };

        Ptr<PointSetRegistrator> createRANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb,
            int _modelPoints, double _threshold,
            double _confidence, int _maxIters);
        /**
         *  @brief Estimate 3-D similarity transformation from matching point pairs.
         *  - every element in _m1 contains (X,Y,Z), which are called source points
         *  - every element in _m2 contains (x,y,z), which are called destination points
         *  - _model is of size 4x4, which contains scale, translation and rotation coeffs
         *
         *   References
         *   ----------
         *   Berthold K. P. Horn, "Closed-form solution of absolute orientation
         *   using unit quaternions," J. Opt. Soc. Am. A 4, 629-642 (1987)
         */
        class Similarity3DEstimatorCallback : public PointSetRegistrator::Callback
        {
        public:

            Similarity3DEstimatorCallback(
                const std::vector<cv::KeyPoint> _imagePoints1, const std::vector<cv::KeyPoint> _imagePoints2, const Matx33d& _intrinsics) :
                imagePoints1(_imagePoints1), imagePoints2(_imagePoints2), K(_intrinsics) {}

            /**
             * Reference: Mur-Artal, R., Montiel, J.M.M. and Tardos, J.D., 2015. ORB-SLAM: a versatile and
             * accurate monocular SLAM system. IEEE transactions on robotics, 31(5), pp.1147-1163.
             */
            void computeError(InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err) const CV_OVERRIDE
            {
                Mat m1 = _m1.getMat(), m2 = _m2.getMat(), model = _model.getMat();
                const Point3f* from = m1.ptr<Point3f>();
                const Point3f* to = m2.ptr<Point3f>();
                const double* F = model.ptr<double>();

                int count = m1.checkVector(3);
                CV_Assert(count > 0);

                _err.create(count, 1, CV_32F);
                Mat err = _err.getMat();
                float* errptr = err.ptr<float>();

                for (int i = 0; i < count; i++)
                {
                    const Point3f& pt3dIn1 = from[i];
                    const Point3f& pt3dIn2 = to[i];

                    // Transform 3-D points in camera 1 to camera 2
                    double X2 = F[0] * pt3dIn1.x + F[1] * pt3dIn1.y + F[2] * pt3dIn1.z + F[3];
                    double Y2 = F[4] * pt3dIn1.x + F[5] * pt3dIn1.y + F[6] * pt3dIn1.z + F[7];
                    double Z2 = F[8] * pt3dIn1.x + F[9] * pt3dIn1.y + F[10] * pt3dIn1.z + F[11];

                    // Project 3-D points in camera 1 to camera 2 image plane and compute error
                    double u2_e = K(0, 0) * X2 / Z2 + K(0, 2) - imagePoints2[i].pt.x;
                    double v2_e = K(1, 1) * Y2 / Z2 + K(1, 2) - imagePoints2[i].pt.y;

                    // Transform 3-D points in camera 2 to camera 1
                    double X1 = F[0] * (pt3dIn2.x - F[3]) + F[4] * (pt3dIn2.y - F[7]) + F[8] * (pt3dIn2.z - F[11]);
                    double Y1 = F[1] * (pt3dIn2.x - F[3]) + F[5] * (pt3dIn2.y - F[7]) + F[9] * (pt3dIn2.z - F[11]);
                    double Z1 = F[2] * (pt3dIn2.x - F[3]) + F[6] * (pt3dIn2.y - F[7]) + F[10] * (pt3dIn2.z - F[11]);

                    // Project 3-D points in camera 2 to camera 1 image plane and compute error
                    double u1_e = K(0, 0) * X1 / Z1 + K(0, 2) - imagePoints1[i].pt.x;
                    double v1_e = K(1, 1) * Y1 / Z1 + K(1, 2) - imagePoints1[i].pt.y;

                    // Pick the larger reprojection error
                    errptr[i] = std::max((float)(u2_e * u2_e + v2_e * v2_e), (float)(u1_e * u1_e + v1_e * v1_e));
                }
            }

            bool checkSubset(InputArray _ms1, InputArray _ms2, int count) const CV_OVERRIDE
            {
                const float threshold = 0.996f;
                Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();

                for (int inp = 1; inp <= 2; inp++)
                {
                    int j, k, i = count - 1;
                    const Mat* msi = inp == 1 ? &ms1 : &ms2;
                    const Point3f* ptr = msi->ptr<Point3f>();

                    CV_Assert(count <= msi->rows);

                    // check that the i-th selected point does not belong
                    // to a line connecting some previously selected points
                    for (j = 0; j < i; ++j)
                    {
                        Point3f d1 = ptr[j] - ptr[i];
                        float n1 = d1.x * d1.x + d1.y * d1.y + d1.z * d1.z;

                        for (k = 0; k < j; ++k)
                        {
                            Point3f d2 = ptr[k] - ptr[i];
                            float denom = (d2.x * d2.x + d2.y * d2.y + d2.z * d2.z) * n1;
                            float num = d1.x * d2.x + d1.y * d2.y + d1.z * d2.z;

                            if (num * num > threshold * threshold * denom)
                                return false;
                        }
                    }
                }
                return true;
            }

            ///////////////////////////////////////////////////////////////////////////////
            // MW: Only the function below has been added. All the other code is derived
            //     from //3rdparty/R2023a/OpenCV/modules/calib3d/src/ptsetreg.cpp
            ///////////////////////////////////////////////////////////////////////////////

            virtual int runKernel(InputArray _m1, InputArray _m2, OutputArray _model) const CV_OVERRIDE
            {
                cv::Mat m1(_m1.getMat()), m2(_m2.getMat());
                _model.create(4, 4, CV_64F);

                // Compute centroid of both inputs
                cv::Mat cm1, cm2;
                cv::reduce(m1, cm1, 0, cv::REDUCE_AVG, CV_64F);
                cv::reduce(m2, cm2, 0, cv::REDUCE_AVG, CV_64F);
                cm1 = cm1.reshape(1);
                cm2 = cm2.reshape(1);

                // Shift points by their centroid
                cv::Mat pointsCentroid1(m1 - cm1), pointsCentroid2(m2 - cm2);
                pointsCentroid1 = pointsCentroid1.reshape(1);
                pointsCentroid2 = pointsCentroid2.reshape(1);

                // Compute covariance matrix
                cv::Mat M(pointsCentroid2.t() * pointsCentroid1);

                // Compute N: real symmetric 4x4 matrix
                cv::Mat N(4, 4, CV_64F);

                N.at<double>(0, 0) = M.at<float>(0, 0) + M.at<float>(1, 1) + M.at<float>(2, 2);
                N.at<double>(0, 1) = M.at<float>(1, 2) - M.at<float>(2, 1);
                N.at<double>(0, 2) = M.at<float>(2, 0) - M.at<float>(0, 2);
                N.at<double>(0, 3) = M.at<float>(0, 1) - M.at<float>(1, 0);

                N.at<double>(1, 0) = N.at<double>(0, 1);
                N.at<double>(1, 1) = M.at<float>(0, 0) - M.at<float>(1, 1) - M.at<float>(2, 2);
                N.at<double>(1, 2) = M.at<float>(0, 1) + M.at<float>(1, 0);
                N.at<double>(1, 3) = M.at<float>(2, 0) + M.at<float>(0, 2);

                N.at<double>(2, 0) = N.at<double>(0, 2);
                N.at<double>(2, 1) = N.at<double>(1, 2);
                N.at<double>(2, 2) = -M.at<float>(0, 0) + M.at<float>(1, 1) - M.at<float>(2, 2);
                N.at<double>(2, 3) = M.at<float>(1, 2) + M.at<float>(2, 1);

                N.at<double>(3, 0) = N.at<double>(0, 3);
                N.at<double>(3, 1) = N.at<double>(1, 3);
                N.at<double>(3, 2) = N.at<double>(2, 3);
                N.at<double>(3, 3) = -M.at<float>(0, 0) - M.at<float>(1, 1) + M.at<float>(2, 2);

                // Compute eigen vectors of N
                Mat V, D;
                cv::eigen(N, D, V);

                // Since eigen values are in descending order, we will choose the first eigen vector
                // And use it as a quaternion
                Mat quat(V.row(0));
                double q0(quat.at<double>(0, 0)),
                    qx(quat.at<double>(0, 1)),
                    qy(quat.at<double>(0, 2)),
                    qz(quat.at<double>(0, 3));

                // Convert quat to rot
                Mat rotMat;
                quatrot2rotM<double>(q0, qx, qy, qz, rotMat);
                rotMat = rotMat.reshape(1);

                // Compute scale
                pointsCentroid1 = pointsCentroid1.mul(pointsCentroid1);
                pointsCentroid2 = pointsCentroid2.mul(pointsCentroid2);
                double scale(std::sqrt(cv::sum(pointsCentroid2)[0] / cv::sum(pointsCentroid1)[0]));

                // Include scaling in the rotation matrix
                rotMat = rotMat.t() * (scale * cv::Mat::eye(3, 3, CV_64F));

                // Compute translation
                Mat trans(cm2.t() - (rotMat * cm1.t()));

                // Return the 4x4 Homogenous matrix
                Mat homgMat(_model.getMat());
                double* H = homgMat.ptr<double>();

                H[0] = rotMat.at<double>(0, 0);
                H[1] = rotMat.at<double>(0, 1);
                H[2] = rotMat.at<double>(0, 2);
                H[3] = trans.at<double>(0, 0);

                H[4] = rotMat.at<double>(1, 0);
                H[5] = rotMat.at<double>(1, 1);
                H[6] = rotMat.at<double>(1, 2);
                H[7] = trans.at<double>(1, 0);

                H[8] = rotMat.at<double>(2, 0);
                H[9] = rotMat.at<double>(2, 1);
                H[10] = rotMat.at<double>(2, 2);
                H[11] = trans.at<double>(2, 0);

                H[12] = 0;
                H[13] = 0;
                H[14] = 0;
                H[15] = 1;

                return 1;
            }

            std::vector<cv::KeyPoint> imagePoints1;
            std::vector<cv::KeyPoint> imagePoints2;
            Matx33d K;
        };


        class Rigid3DEstimatorCallback : public Similarity3DEstimatorCallback
        {
        public:

            Rigid3DEstimatorCallback(
                const std::vector<cv::KeyPoint> _imagePoints1, const std::vector<cv::KeyPoint> _imagePoints2, const Matx33d& _intrinsics) :
                Similarity3DEstimatorCallback(_imagePoints1, _imagePoints2, _intrinsics) {}

            /**
            * References: Arun KS, Huang TS, Blostein SD (1987) Least-squares fitting of two 3-D point sets.
            * IEEE Trans Pattern Anal Machine Intell 9:698-700
            */
            int runKernel(InputArray _m1, InputArray _m2, OutputArray _model) const CV_OVERRIDE
            {
                cv::Mat m1(_m1.getMat()), m2(_m2.getMat());
                _model.create(4, 4, CV_64F);

                // Compute centroid of both inputs
                cv::Mat cm1, cm2;
                cv::reduce(m1, cm1, 0, cv::REDUCE_AVG, CV_64F);
                cv::reduce(m2, cm2, 0, cv::REDUCE_AVG, CV_64F);
                cm1 = cm1.reshape(1);
                cm2 = cm2.reshape(1);

                // Shift points by their centroid
                cv::Mat pointsCentroid1(m1 - cm1), pointsCentroid2(m2 - cm2);
                pointsCentroid1 = pointsCentroid1.reshape(1);
                pointsCentroid2 = pointsCentroid2.reshape(1);

                // Compute covariance matrix
                cv::Mat M(pointsCentroid1.t() * pointsCentroid2);

                cv::Mat W, U, Vt;
                cv::SVD::compute(M,W,U,Vt);

                // Compute rotation
                cv::Mat tempM = cv::Mat::eye(3, 3, CV_64F);

                // Handle the reflection case
                tempM.at<double>(2, 2) = cv::determinant(U * Vt) > 0 ? 1.0 : -1.0;

                Vt.convertTo(Vt, CV_64F);
                U.convertTo(U, CV_64F);
                cv::Mat rotMat = Vt.t() * tempM * U.t();

                // Compute translation
                cv::Mat trans(cm2.t() - (rotMat * cm1.t()));

                // Return the 4x4 Homogenous matrix
                cv::Mat homgMat(_model.getMat());
                double *H = homgMat.ptr<double>();
            
                H[0] = rotMat.at<double>(0,0);
                H[1] = rotMat.at<double>(0,1);
                H[2] = rotMat.at<double>(0,2);
                H[3] = trans.at<double>(0,0);
            
                H[4] = rotMat.at<double>(1,0);
                H[5] = rotMat.at<double>(1,1);
                H[6] = rotMat.at<double>(1,2);
                H[7] = trans.at<double>(1,0);
            
                H[8] = rotMat.at<double>(2,0);
                H[9] = rotMat.at<double>(2,1);
                H[10] = rotMat.at<double>(2,2);
                H[11] = trans.at<double>(2,0);
            
                H[12] = 0;
                H[13] = 0;
                H[14] = 0;
                H[15] = 1;
                
                return 1;
            }
        };

        /**
         * @brief Estimate similarity transformation matrix between 2 sets of matched 3-D points by
         *          minimizing the reprojection errors of two frames
         *
         * @param[in]  _from            3-D points observed in frame 1, size Nx3 in mat
         * @param[in]  _to              3-D points observed in frame 2, size Nx3 in mat
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
            const Matx33d& _intrinsics, OutputArray _out, OutputArray _inliers,
            double ransacThreshold = 6, double confidence = 0.99);

         /**
         * @brief Estimate rigid transformation matrix between 2 sets of matched 3-D points by
         *          minimizing the reprojection errors of two frames
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
            const Matx33d& _intrinsics,  OutputArray _out, OutputArray _inliers,
            double ransacThreshold = 6, double confidence = 0.99);
    } // namespace 
}
#endif