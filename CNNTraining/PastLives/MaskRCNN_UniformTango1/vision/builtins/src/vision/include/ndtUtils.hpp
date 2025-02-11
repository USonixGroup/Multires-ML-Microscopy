//
//  ndtUtils.hpp
//  libmwvision
//
//  This file contains utilities used by visionNDTComputeScoreDerivatives.
//
//  Note: Some of the utilities (e.g. eulerAngleToRotationMatrix) can be pulled out later
//        if needed.
//
//  Copyright 2019 The MathWorks, Inc.


#include <cmath> // std::abs
#include <algorithm>
#include <functional>

#ifndef COMPILE_FOR_VISION_BUILTINS // Used only during Codegen
#include "matrixUtils.hpp"
#else
#include "calibration/matrixUtils.hpp" // vision::matrixMultiplyColumn
#endif

namespace vision {
    namespace ndt {
        
        ///
        /// @brief Convert Euler angles to rotation matrix
        /// Converts a vector of Euler angles [phix, phiy, phiz] to a 3D rotation matrix R.
        /// Rotation angles are in radians. The rotation sequence is 'XYZ', where the order
        /// of rotation angles is X axis rotation, Y axis rotation, and Z axis rotation,
        /// i.e., R = Rx * Ry * Rz.
        ///
        /// @param[in] eul Euler angle, specified as a single or double 3-element array.
        /// @param[out] R Rotation matrix, specified as a single or double 9-element array.
        template <typename T>
                inline void eulerAngleToRotationMatrix(const T* eul, T* R) {
            
            //     The rotation matrix R can be constructed as follows by
            //     ct = [cx cy cz] and st = [sx sy sz]
            //
            //     R = [            cy*cz,           -cy*sz,     sy
            //           cx*sz + cz*sx*sy, cx*cz - sx*sy*sz, -cy*sx
            //           sx*sz - cx*cz*sy, cz*sx + cx*sy*sz,  cx*cy]
            //
            //       = Rx(tx) * Ry(ty) * Rz(tz)
            
            const T cx = cos(eul[0]);
            const T sx = sin(eul[0]);
            const T cy = cos(eul[1]);
            const T sy = sin(eul[1]);
            const T cz = cos(eul[2]);
            const T sz = sin(eul[2]);
            
            R[0] = cy * cz;
            R[1] = -cy * sz;
            R[2] = sy;
            R[3] = (cx * sz) + (cz * sx * sy);
            R[4] = (cx * cz) - (sx * sy * sz);
            R[5] = -cy * sx;
            R[6] = (sx * sz) - (cx * cz * sy);
            R[7] = (cz * sx) + (cx * sy * sz);
            R[8] = cx * cy;
        }
        
        /// @brief Compute partial gradient and Hessian of transformation with respect to a point.
        /// The input pose, is represented by [tx, ty, tz, phix, phiy, phiz].
        /// Only components of these angle derivatives are returned, which will be used
        /// later for the evaluation of full Jacobian (Eq-6.18) and Hessian (Eq-6.20) matrix
        /// of the transformation matrix (Eq-6.17). As the values only depend on the pose
        /// parameters, it is only updated every time the pose changes. The function follows
        /// the method by Magnusson.
        ///
        /// @param[in] pose, specified as a 6-element single or double array
        /// @param[out] Ja, 27-element array of single or double, describing a 3-by-9 matrix
        /// @param[out] Ha, 81-element array of single or double, describing a 9-by-9 matirx
        ///
        /// [1] Martin Magnusson, The Three-Dimensional Normal-Distributions
        /// Transform - an Efficient Representation for Registration, Surface
        /// Analysis, and Loop Detection, Thesis, 2013
        template <typename T>
                inline void computeAngleDerivatives(const T* pose, T* Ja, T* Ha) {
            
            T cx(1.0), sx(0.0), cy(1.0), sy(0.0), cz(1.0), sz(0.0);
            
            //const T tenDegrees(0.1745);
            
            if (std::abs(pose[3]) >= T(0.1745)) {
                cx = cos(pose[3]);
                sx = sin(pose[3]);
            }
            
            if (std::abs(pose[4]) >= T(0.1745)) {
                cy = cos(pose[4]);
                sy = sin(pose[4]);
            }
            
            if (std::abs(pose[5]) >= T(0.1745)) {
                cz = cos(pose[5]);
                sz = sin(pose[5]);
            }
            
            // Compute the angular gradient component of p which will be used in the
            // evaluation of element of Jacobian (Eq-6.18 and 6.19) eventually.
            Ja[0]  = 0.0;
            Ja[1]  = -sx * sz + cx * sy * cz;
            Ja[2]  = cx * sz + sx * sy * cz;
            Ja[3]  = 0.0;
            Ja[4]  = -sx * cz - cx * sy * sz;
            Ja[5]  = cx * cz - sx * sy * sz;
            Ja[6]  = 0.0;
            Ja[7]  = -cx * cy;
            Ja[8]  = -sx * cy;
            Ja[9]  = -sy * cz;
            Ja[10] = sx * cy * cz;
            Ja[11] = -cx * cy * cz;
            Ja[12] = sy * sz;
            Ja[13] = -sx * cy * sz;
            Ja[14] = cx * cy * sz;
            Ja[15] = cy;
            Ja[16] = sx * sy;
            Ja[17] = -cx * sy;
            Ja[18] = -cy * sz;
            Ja[19] = cx * cz - sx * sy * sz;
            Ja[20] = sx * cz + cx * sy * sz;
            Ja[21] = -cy * cz;
            Ja[22] = -cx * sz - sx * sy * cz;
            Ja[23] = cx * sy * cz - sx * sz;
            Ja[24] = 0.0;
            Ja[25] = 0.0;
            Ja[26] = 0.0;
            
            // Compute the angular hessian components of p (Eq-6.20 and 6.21)
            T a[9], b[9], c[9], d[9], e[9], f[9];
            
            a[0] = 0.0;
            a[1] = (-cx * sz - sx * sy * cz);
            a[2] = (-sx * sz + cx * sy * cz);
            a[3] = 0.0;
            a[4] = (-cx * cz + sx * sy * sz);
            a[5] = (-cx * sy * sz - sx * cz);
            a[6] = 0.0;
            a[7] = sx * cy;
            a[8] = -cx * cy;
            
            b[0] = 0.0;
            b[1] = cx * cy * cz;
            b[2] = sx * cy * cz;
            b[3] = 0.0;
            b[4] = -cx * cy * sz;
            b[5] = -sx * cy * sz;
            b[6] = 0.0;
            b[7] = cx * sy;
            b[8] = sx * sy;
            
            c[0] = 0.0;
            c[1] = -sx * cz - cx * sy * sz;
            c[2] = cx * cz - sx * sy * sz;
            c[3] = 0.0;
            c[4] = sx * sz - cx * sy * cz;
            c[5] = -sx * sy * cz - cx * sz;
            c[6] = 0.0;
            c[7] = 0.0;
            c[8] = 0.0;
            
            d[0] = -cy * cz;
            d[1] = -sx * sy * cz;
            d[2] = cx * sy * cz;
            d[3] = cy * sz;
            d[4] = sx * sy * sz;
            d[5] = -cx * sy * sz;
            d[6] = -sy;
            d[7] = sx * cy;
            d[8] = -cx * cy;
            
            e[0] = sy * sz;
            e[1] = -sx * cy * sz;
            e[2] = cx * cy * sz;
            e[3] = sy * cz;
            e[4] = -sx * cy * cz;
            e[5] = cx * cy * cz;
            e[6] = 0.0;
            e[7] = 0.0;
            e[8] = 0.0;
            
            f[0] = -cy * cz;
            f[1] = -cx * sz - sx * sy * cz;
            f[2] = -sx * sz + cx * sy * cz;
            f[3] = cy * sz;
            f[4] = -cx * cz + sx * sy * sz;
            f[5] = -cx * sy * sz - sx * cz;
            f[6] = 0.0;
            f[7] = 0.0;
            f[8] = 0.0;
            
            
            Ha[0]  = a[0];
            Ha[1]  = a[1];
            Ha[2]  = a[2];
            Ha[3]  = b[0];
            Ha[4]  = b[1];
            Ha[5]  = b[2];
            Ha[6]  = c[0];
            Ha[7]  = c[1];
            Ha[8]  = c[2];
            
            Ha[9]  = a[3];
            Ha[10] = a[4];
            Ha[11] = a[5];
            Ha[12] = b[3];
            Ha[13] = b[4];
            Ha[14] = b[5];
            Ha[15] = c[3];
            Ha[16] = c[4];
            Ha[17] = c[5];
            
            Ha[18] = a[6];
            Ha[19] = a[7];
            Ha[20] = a[8];
            Ha[21] = b[6];
            Ha[22] = b[7];
            Ha[23] = b[8];
            Ha[24] = c[6];
            Ha[25] = c[7];
            Ha[26] = c[8];
            
            Ha[27] = b[0];
            Ha[28] = b[1];
            Ha[29] = b[2];
            Ha[30] = d[0];
            Ha[31] = d[1];
            Ha[32] = d[2];
            Ha[33] = e[0];
            Ha[34] = e[1];
            Ha[35] = e[2];
            
            Ha[36] = b[3];
            Ha[37] = b[4];
            Ha[38] = b[5];
            Ha[39] = d[3];
            Ha[40] = d[4];
            Ha[41] = d[5];
            Ha[42] = e[3];
            Ha[43] = e[4];
            Ha[44] = e[5];
            
            Ha[45] = b[6];
            Ha[46] = b[7];
            Ha[47] = b[8];
            Ha[48] = d[6];
            Ha[49] = d[7];
            Ha[50] = d[8];
            Ha[51] = e[6];
            Ha[52] = e[7];
            Ha[53] = e[8];
            
            Ha[54] = c[0];
            Ha[55] = c[1];
            Ha[56] = c[2];
            Ha[57] = e[0];
            Ha[58] = e[1];
            Ha[59] = e[2];
            Ha[60] = f[0];
            Ha[61] = f[1];
            Ha[62] = f[2];
            
            Ha[63] = c[3];
            Ha[64] = c[4];
            Ha[65] = c[5];
            Ha[66] = e[3];
            Ha[67] = e[4];
            Ha[68] = e[5];
            Ha[69] = f[3];
            Ha[70] = f[4];
            Ha[71] = f[5];
            
            Ha[72] = c[6];
            Ha[73] = c[7];
            Ha[74] = c[8];
            Ha[75] = e[6];
            Ha[76] = e[7];
            Ha[77] = e[8];
            Ha[78] = f[6];
            Ha[79] = f[7];
            Ha[80] = f[8];
        }
        
        ///
        /// @brief Evaluate point derivative
        /// Calculate the 3-by-6 Jacobian and 18-by-6 Hessian matrix of the transformation
        /// function (Eqn-6.18, 6.19 & 6.21).
        ///
        /// @param[in] Ja, specified as a 3-by-9 matrix of single or double
        /// @param[in] Ha, specified as a 9-by-9 matrix of single or double
        /// @param[in] p, point, specified as a 3-element single or double array
        /// @param[in] needHessian, logical specifying whether hessian output is needed
        /// @param[out] Jp, 3-by-6 single or double Jacobian matrix
        /// @param[out] Hp, 18-by-6 single or double Hessian matrix
        ///
        template <typename T>
                inline void jacobianHessian(T* Ja, T* Ha, const T* p, const bool needHessian, T* Jp, T* Hp) {
            
            T P[27] = {};
            
            for (size_t i = 0; i < 3; i++) {
                P[i]    = p[i];
                P[i+12] = p[i];
                P[i+24] = p[i];
            }
            
            Jp[0] = Jp[4] = Jp[8] = 1;
            vision::matrixMultiplyColumn(Ja, 3, 9, P, 3, Jp+9);
            
            if (needHessian) {
                vision::matrixMultiplyColumn(Ha, 9, 9, P, 3, Hp);
            }
        }
        
        ///
        /// @brief Evaluate gradient and Hessian at point
        template <typename T>
               inline void gradientHessian(const T* p, const T* m, const T* iCov, const T d1, const T d2, const T* Jp, const T* Hp, const bool isRetHessian, T* grad, T* hessian) {
            
            T q[3], qC[3], qCq, qCJ[6], c, a[36], JJ[36], JpT[18], tmp[18];
            
            for (size_t i = 0; i < 3; i++) {
                q[i] = p[i] - m[i];
            }
            
            vision::matrixMultiplyColumn(q, 1, 3, iCov, 3, qC);
            
            qCq = 0;
            for (size_t i = 0; i < 3; i++) {
                qCq += qC[i] * q[i];
            }
            
            vision::matrixMultiplyColumn(qC, 1, 3, Jp, 6, qCJ);
            
            c = d1 * d2 * exp(-d2 / 2 * qCq);
            
            for (size_t i = 0; i < 6; i++) {
                grad[i] = c * qCJ[i];
            }
            
            if (isRetHessian) {
                T hess[36];
                std::fill(hess, hess+36, T(0.0));
                
                for (size_t i = 0; i < 3; i++) {
                    for (size_t j = 0; j < 6; j++) {
                        JpT[i * 6 + j] = Jp[j * 3 + i];
                    }
                }
                
                vision::matrixMultiplyColumn(JpT, 6, 3, iCov, 3, tmp);
                vision::matrixMultiplyColumn(tmp, 6, 3, Jp, 6, JJ);
                
                vision::matrixMultiplyColumn(qCJ, 6, 1, qCJ, 6, a);
                
                T h11 = 0, h12 = 0, h13 = 0;
                T h21 = 0, h22 = 0, h23 = 0;
                T h31 = 0, h32 = 0, h33 = 0;
                for (size_t i = 0; i < 3; i++) {
                    h11 += qC[i] * Hp[i];
                    h12 += qC[i] * Hp[i + 9];
                    h13 += qC[i] * Hp[i + 18];
                    
                    h21 += qC[i] * Hp[i + 3];
                    h22 += qC[i] * Hp[i + 12];
                    h23 += qC[i] * Hp[i + 21];
                    
                    h31 += qC[i] * Hp[i + 6];
                    h32 += qC[i] * Hp[i + 15];
                    h33 += qC[i] * Hp[i + 24];
                }
                
                hess[21] = h11;
                hess[22] = h21;
                hess[23] = h31;
                
                hess[27] = h12;
                hess[28] = h22;
                hess[29] = h32;
                
                hess[33] = h13;
                hess[34] = h23;
                hess[35] = h33;
                
                for (size_t i = 0; i < 36; i++) {
                    hessian[i] = c * (-d2*a[i] + hess[i] + JJ[i]);
                }
                
            }
            
        }
        
        template<typename DataType>
                void computeJaHa(const DataType* mPose, DataType* mR, DataType* Ja, DataType* Ha) {
            
            vision::ndt::eulerAngleToRotationMatrix(mPose+3, mR);
            
            // Evaluate the angular gradient and hessian matrices from the parameter
            // pose for future use. These values remain the same until the pose is
            // updated. The return values Ja and Ha are structures containing only the
            // non-zero elements of Jacobian and Hassian matrix. The full gradient and
            // hessian matrix will be evaluated in the later steps.
            vision::ndt::computeAngleDerivatives(mPose, Ja, Ha);
            
        }
        
        template<typename DataType>
                void computeScoreGradient(const uint32_T* mIndices, DataType* p , const bool needsHessian, DataType* Ja,
                DataType* Ha, DataType* Jp, DataType* Hp, const DataType* mvals, const size_t numMvals, const DataType* iCov,
                DataType* hessian, const DataType  d1, const DataType  d2, DataType* score, DataType* g, DataType* h,
                DataType* q, DataType* grad, const size_t numIndices) {
            
            // Evaluate the point derivative. For each point calculate the 3x6 Jacobian
            // and 18x6 Hessian matrix (Eq-6.18, 6.19, 6.20 & 6.21) of the
            // transformation function.
            vision::ndt::jacobianHessian(Ja, Ha, p, needsHessian, Jp, Hp);
            
            DataType m[3], pp[3], prod(0.0);
            
            for (size_t j = 0; j < numIndices; j++) {
                uint32_T idx = mIndices[j] - 1; // Convert from MATLAB 1-based to C++ 0-based index
                m[0] = mvals[idx];
                m[1] = mvals[idx +   numMvals];
                m[2] = mvals[idx + 2*numMvals];
                
                const DataType* c = iCov + idx*9;
                
                // Evaluate the score - Eq-6.9
                pp[0] = q[0] - m[0];
                pp[1] = q[1] - m[1];
                pp[2] = q[2] - m[2];
                
                prod = pp[0] * (pp[0]*c[0] + pp[1]*c[1] + pp[2]*c[2]) +
                        pp[1] * (pp[0]*c[3] + pp[1]*c[4] + pp[2]*c[5]) +
                        pp[2] * (pp[0]*c[6] + pp[1]*c[7] + pp[2]*c[8]);
                score[0] += -d1 * std::exp( - d2 * prod / DataType(2.0) );
                // Evaluate the Jacobian and Hessian at that point.
                vision::ndt::gradientHessian(q, m, c, d1, d2, Jp, Hp, needsHessian, g, h);
                
                std::transform(grad,grad+6, g, grad, std::plus<DataType> ());
                
                if (needsHessian) {
                    std::transform(hessian, hessian+36, h, hessian, std::plus<DataType>());
                }
            }
        }
        
    } // namespace ndt
} // namespace vision
