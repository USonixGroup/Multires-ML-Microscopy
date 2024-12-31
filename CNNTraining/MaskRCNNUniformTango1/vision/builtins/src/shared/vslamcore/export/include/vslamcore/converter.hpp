////////////////////////////////////////////////////////////////////////////////
// Utility functions to convert data formats
// 
// Copyright 2022-23 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include <vector>
#include <sstream>
#include <string>
#include <cstdio>
#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"
#include "QueueInstance.hpp"
#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"

namespace vision {
    namespace vslam {
        template <typename T>
        void tform2quatpose(const cv::Matx<T,4,4>& tform, std::vector<T>& pose) {

            T Qxx = tform(0, 0);
            T Qyx = tform(1, 0);
            T Qzx = tform(2, 0);
            T Qxy = tform(0, 1);
            T Qyy = tform(1, 1);
            T Qzy = tform(2, 1);
            T Qxz = tform(0, 2);
            T Qyz = tform(1, 2);
            T Qzz = tform(2, 2);

            T t = Qxx + Qyy + Qzz;

            T r, s, w, x, y, z;
            if (t >= 0) {
                r = sqrt(1 + t);
                s = 0.5 / r;
                w = 0.5 * r;
                x = (Qzy - Qyz) * s;
                y = (Qxz - Qzx) * s;
                z = (Qyx - Qxy) * s;
            }
            else {
                T maxVal = std::max(Qxx, std::max(Qyy, Qzz));

                if (maxVal == Qxx) {
                    r = sqrt(1 + Qxx - Qyy - Qzz);
                    s = 0.5 / r;
                    w = (Qzy - Qyz) * s;
                    x = 0.5 * r;
                    y = (Qyx + Qxy) * s;
                    z = (Qxz + Qzx) * s;
                }
                else if (maxVal == Qyy) {
                    r = sqrt(1 + Qyy - Qxx - Qzz);
                    s = 0.5 / r;
                    w = (Qxz - Qzx) * s;
                    x = (Qyx + Qxy) * s;
                    y = 0.5 * r;
                    z = (Qzy + Qyz) * s;
                }
                else {
                    r = sqrt(1 + Qzz - Qxx - Qyy);
                    s = 0.5 / r;
                    w = (Qyx - Qxy) * s;
                    x = (Qxz + Qzx) * s;
                    y = (Qzy + Qyz) * s;
                    z = 0.5 * r;
                }
            }

            pose.push_back(tform(0, 3));
            pose.push_back(tform(1, 3));
            pose.push_back(tform(2, 3));
            pose.push_back(w);
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(z);
        }

        template <typename T>
        void quatpose2tform(const std::vector<T>& pose, cv::Matx<T,4,4>& tform) {

            tform = cv::Matx<T,4,4>::eye();

            tform(0, 3) = pose[0];
            tform(1, 3) = pose[1];
            tform(2, 3) = pose[2];

            T q0 = pose[3];
            T qx = pose[4];
            T qy = pose[5];
            T qz = pose[6];

            T q02(q0 * q0), qx2(qx * qx), qy2(qy * qy), qz2(qz * qz);
            T qxy(qx * qy), q0z(q0 * qz), qxz(qx * qz), q0y(q0 * qy);
            T qyz(qy * qz), q0x(q0 * qx);
            tform(0, 0) = q02 + qx2 - qy2 - qz2;
            tform(1, 0) = 2 * qxy + 2 * q0z;
            tform(2, 0) = 2 * qxz - 2 * q0y;
            tform(0, 1) = 2 * qxy - 2 * q0z;
            tform(1, 1) = q02 - qx2 + qy2 - qz2;
            tform(2, 1) = 2 * qyz + 2 * q0x;
            tform(0, 2) = 2 * qxz + 2 * q0y;
            tform(1, 2) = 2 * qyz - 2 * q0x;
            tform(2, 2) = q02 - qx2 - qy2 + qz2;
        }

        template <typename T>
        void tform2eigenquatpose(const cv::Matx<T,4,4>& tform, std::vector<T>& pose) {

            T Qxx = tform(0, 0);
            T Qyx = tform(1, 0);
            T Qzx = tform(2, 0);
            T Qxy = tform(0, 1);
            T Qyy = tform(1, 1);
            T Qzy = tform(2, 1);
            T Qxz = tform(0, 2);
            T Qyz = tform(1, 2);
            T Qzz = tform(2, 2);

            T t = Qxx + Qyy + Qzz;

            T r, s, w, x, y, z;
            if (t >= 0) {
                r = sqrt(1 + t);
                s = 0.5 / r;
                w = 0.5 * r;
                x = (Qzy - Qyz) * s;
                y = (Qxz - Qzx) * s;
                z = (Qyx - Qxy) * s;
            }
            else {
                T maxVal = std::max(Qxx, std::max(Qyy, Qzz));

                if (maxVal == Qxx) {
                    r = sqrt(1 + Qxx - Qyy - Qzz);
                    s = 0.5 / r;
                    w = (Qzy - Qyz) * s;
                    x = 0.5 * r;
                    y = (Qyx + Qxy) * s;
                    z = (Qxz + Qzx) * s;
                }
                else if (maxVal == Qyy) {
                    r = sqrt(1 + Qyy - Qxx - Qzz);
                    s = 0.5 / r;
                    w = (Qxz - Qzx) * s;
                    x = (Qyx + Qxy) * s;
                    y = 0.5 * r;
                    z = (Qzy + Qyz) * s;
                }
                else {
                    r = sqrt(1 + Qzz - Qxx - Qyy);
                    s = 0.5 / r;
                    w = (Qyx - Qxy) * s;
                    x = (Qxz + Qzx) * s;
                    y = (Qzy + Qyz) * s;
                    z = 0.5 * r;
                }
            }

            pose.push_back(tform(0, 3));
            pose.push_back(tform(1, 3));
            pose.push_back(tform(2, 3));
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(z);
            pose.push_back(w);
        }

        template <typename T>
        void eigenquatpose2tform(const std::vector<T>& pose, cv::Matx<T,4,4>& tform) {

            tform = cv::Matx<T,4,4>::eye();

            tform(0, 3) = pose[0];
            tform(1, 3) = pose[1];
            tform(2, 3) = pose[2];


            T qx = pose[3];
            T qy = pose[4];
            T qz = pose[5];
            T q0 = pose[6];

            T norm = std::sqrt(qx * qx + qy * qy + qz * qz + q0 * q0);

            qx = qx / norm;
            qy = qy / norm;
            qz = qz / norm;
            q0 = q0 / norm;

            T q02(q0 * q0), qx2(qx * qx), qy2(qy * qy), qz2(qz * qz);
            T qxy(qx * qy), q0z(q0 * qz), qxz(qx * qz), q0y(q0 * qy);
            T qyz(qy * qz), q0x(q0 * qx);
            tform(0, 0) = q02 + qx2 - qy2 - qz2;
            tform(1, 0) = 2 * qxy + 2 * q0z;
            tform(2, 0) = 2 * qxz - 2 * q0y;
            tform(0, 1) = 2 * qxy - 2 * q0z;
            tform(1, 1) = q02 - qx2 + qy2 - qz2;
            tform(2, 1) = 2 * qyz + 2 * q0x;
            tform(0, 2) = 2 * qxz + 2 * q0y;
            tform(1, 2) = 2 * qyz - 2 * q0x;
            tform(2, 2) = q02 - qx2 - qy2 + qz2;
        }
        
        /**
         * @brief Convert quaternion into rotation matrix
         *
         * @param[in]  q0    Constant value in quaternion
         * @param[in]  qx    Coeff. of X in quaternion
         * @param[in]  qy    Coeff. of Y in quaternion
         * @param[in]  qz    Coeff. of Z in quaternion
         * @param[out] rotM  Rotation matrix of size 3x3 in a mat
         */
        template <typename T>
        void quatrot2rotM(const T q0, const T qx, const T qy, const T qz, cv::Mat& rotM) {
            rotM = cv::Mat::eye(3, 3, CV_64F);

            T q02(q0 * q0), qx2(qx * qx), qy2(qy * qy), qz2(qz * qz);
            T qxy(qx * qy), q0z(q0 * qz), qxz(qx * qz), q0y(q0 * qy);
            T qyz(qy * qz), q0x(q0 * qx);

            rotM.at<T>(0, 0) = q02 + qx2 - qy2 - qz2;
            rotM.at<T>(1, 0) = 2 * qxy + 2 * q0z;
            rotM.at<T>(2, 0) = 2 * qxz - 2 * q0y;
            rotM.at<T>(0, 1) = 2 * qxy - 2 * q0z;
            rotM.at<T>(1, 1) = q02 - qx2 + qy2 - qz2;
            rotM.at<T>(2, 1) = 2 * qyz + 2 * q0x;
            rotM.at<T>(0, 2) = 2 * qxz + 2 * q0y;
            rotM.at<T>(1, 2) = 2 * qyz - 2 * q0x;
            rotM.at<T>(2, 2) = q02 - qx2 - qy2 + qz2;
        }

        /**
         * @brief Convert homogeneous points to cartesian with target datatype T
         *
         * @param[in/out]  points  [in]: M-by-N matrix, containing N M-dimension homogeneous coordinates.
         *                         [out]: (M-1)-by-N matrix, containing N (M-1)-dimension cartesian coordinates.
         */
        template <typename T>
        void hom2cart(cv::Mat& points) {
            points.convertTo(points, cv::traits::Type<T>::value);

            int numRowsOut = points.rows - 1;
            if (numRowsOut < 1)
                return;

            std::vector<T*> rowPtrs(numRowsOut);
            for (int i = 0; i < numRowsOut; ++i)
                rowPtrs[i] = points.template ptr<T>(i,0);
            const T* divPtr = points.template ptr<T>(numRowsOut,0);

            for( int i = 0; i < points.cols; i++ )
            {
                T scale = (std::abs(divPtr[i]) > std::numeric_limits<T>::epsilon()) ? 1./divPtr[i] : 1.;
                std::for_each(rowPtrs.begin(), rowPtrs.end(), [&](T* &p){p[i] *= scale;});
            }

            points = points.rowRange(0,numRowsOut);
        }

        /**
         * @brief Convert camera extrinsics to camera pose
         *
         * @param[in]   extrinsics_R  rotation matrix from world coordinate to camera coordinate, represented as a cv::Matx<T,3,3>.
         * @param[in]   extrinsics_t  translation vector from world coordinate to camera coordinate, represented as a cv::Vec<T,3>.
         * @param[out]  pose_R     relative rotation between the two views, represented as a cv::Matx<T,3,3>.
         * @param[out]  pose_t     relative translation between the two views, represented as a cv::Vec<T,3>.
         */
        template <typename T>
        void extr2pose(const cv::Matx<T,3,3>& extrinsics_R, const cv::Vec<T,3>& extrinsics_t, cv::Matx<T,3,3>& pose_R, cv::Vec<T,3>& pose_t) {
            pose_R = extrinsics_R.t();
            pose_t = -pose_R * extrinsics_t;
        }
        
        /**
         * @brief Constructs pose from rotation matrix and translation vector
         *
         * @param[in]   R        rotation matrix, represented as a cv::Matx<T,3,3>.
         * @param[in]   t        translation vector, represented as a cv::Vec<T,3>.
         * @return     pose, represented as a cv::Matx<T,4,4>.
         */
        template <typename T>
        cv::Matx<T,4,4> rt2pose(const cv::Matx<T,3,3>& R, const cv::Vec<T,3>& t) {
            return {R(0, 0), R(0, 1), R(0, 2), t[0],
                    R(1, 0), R(1, 1), R(1, 2), t[1],
                    R(2, 0), R(2, 1), R(2, 2), t[2],
                    0, 0, 0, 1};
        }
        
        /**
         * @brief Extracts rotation matrix and translation vector from pose
         *
         * @param[in]    pose     pose, represented as a cv::Matx<T,4,4>.
         * @param[out]   R        rotation matrix, represented as a cv::Matx<T,3,3>.
         * @param[out]   t        translation vector, represented as a cv::Vec<T,3>.
         */
        template <typename T>
        void pose2rt(const cv::Matx<T,4,4>& pose, cv::Matx<T,3,3>& R, cv::Vec<T,3>& t) {
            R(0, 0) = pose(0, 0);
            R(0, 1) = pose(0, 1);
            R(0, 2) = pose(0, 2);
            R(1, 0) = pose(1, 0);
            R(1, 1) = pose(1, 1);
            R(1, 2) = pose(1, 2);
            R(2, 0) = pose(2, 0);
            R(2, 1) = pose(2, 1);
            R(2, 2) = pose(2, 2);
            t[0] = pose(0, 3);
            t[1] = pose(1, 3);
            t[2] = pose(2, 3);
        }

        /**
         * @brief Returns string form of pose
         *
         * @param[in] pose     pose, represented as a cv::Matx<T,4,4>.
         * @returns   String representation of pose.
         */
        template <typename T>
        std::string pose2String(const cv::Matx<T,4,4>& pose) {
            std::string buffer;
            std::ostringstream oss;
            cv::Mat m(pose, true);
            oss << m;
            buffer = oss.str();
            return buffer;
        }

        template <typename T>
        std::string convertVectorToString(const std::vector<T>& vec) {
            std::string retval = "[ ";
            for(int i = 0 ; i < static_cast<int>(vec.size()) ; i++) {
                if(i != 0) {
                    retval = retval + ", ";
                }
                retval = retval + std::to_string(vec[i]);
            }
            retval = retval + " ]";
            return retval;
        }

        template <typename T>
        std::string convertVectorOfVectorToString(const std::vector<std::vector<T>>& vec) {
            std::string retval = "[ ";
            for(int i = 0 ; i < static_cast<int>(vec.size()) ; i++) {
                if(i != 0) {
                    retval = retval + ", ";
                }
                retval = retval + convertVectorToString<T>(vec[i]);
            }
            retval = retval + " ]";
            return retval;
        }

        inline
        void unoptimized2OptimizedCurrExtrinsicsUsingLastPose(QueueInstanceBase& qObj, const cv::Matx<decltype(QueueInstanceBase::lastPose_R)::value_type,4,4>& lastPose_opt) {
            // 1. Get inverse of last pose
            decltype(QueueInstanceBase::lastPose_R) lastPose_R_unopt_inv;
            decltype(QueueInstanceBase::lastPose_t) lastPose_t_unopt_inv;
            extr2pose(qObj.lastPose_R, qObj.lastPose_t, lastPose_R_unopt_inv, lastPose_t_unopt_inv);
            const auto lastPose_inv(rt2pose(lastPose_R_unopt_inv, lastPose_t_unopt_inv));

            // 2. Get transformation between last and current frames
            decltype(QueueInstanceBase::lastPose_R) currPose_R;
            decltype(QueueInstanceBase::lastPose_t) currPose_t;
            extr2pose(qObj.currExtrinsics_R, qObj.currExtrinsics_t, currPose_R, currPose_t);
            const auto currPose(rt2pose(currPose_R, currPose_t));
            const auto transH = lastPose_inv * currPose;

            // 3. Apply transformation to get optimized pose for current frame
            const auto currPose_opt = lastPose_opt * transH;

            // 4. Get optimized extrinsics for current frame
            decltype(QueueInstanceBase::lastPose_R) currPose_opt_R;
            decltype(QueueInstanceBase::lastPose_t) currPose_opt_t;
            pose2rt(currPose_opt, currPose_opt_R, currPose_opt_t);
            extr2pose(currPose_opt_R, currPose_opt_t, qObj.currExtrinsics_R, qObj.currExtrinsics_t);
        }
    }// namespace vslam
}// namespace vision

#endif // CONVERTER_HPP