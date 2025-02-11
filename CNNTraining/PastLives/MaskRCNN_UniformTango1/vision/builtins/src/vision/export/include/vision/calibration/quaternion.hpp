////////////////////////////////////////////////////////////////////////////////
//
// Utilities of quaternion operation
// https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_.E2.86.94_Quaternion
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TMW_QUATERNION_HPP
#define TMW_QUATERNION_HPP

#include <matrix.h>
#include <mex.h>
#include <cmath>
#include <algorithm> // for std::max
#include <limits>

namespace vision
{
	// Convert rodrigues vector to quaternion 
	template <typename T>
	void rodriguesVectorToQuaternion(T* rvec, T* q)
	{
		T theta = sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
		if (theta < 10 * std::numeric_limits<T>::epsilon()) {
			q[0] = 1;
			q[1] = 0;
			q[2] = 0;
			q[3] = 0;
		}
		else {
			T a = cos(theta / 2);
			T b = sin(theta / 2);
			q[0] = a;
			q[1] = rvec[0] / theta*b;
			q[2] = rvec[1] / theta*b;
			q[3] = rvec[2] / theta*b;
		}
	}

	// Quaternion product: q2 = q0 * q1
	template <typename T>
	void quaternionMultiplication(T* q0, T* q1, T* q2)
	{
		q2[0] = q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2] - q0[3] * q1[3];
		q2[1] = q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3] - q0[3] * q1[2];
		q2[2] = q0[0] * q1[2] - q0[1] * q1[3] + q0[2] * q1[0] + q0[3] * q1[1];
		q2[3] = q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1] + q0[3] * q1[0];
	}

	// Convert normalized quaternion to rodrigues vector
	template <typename T>
	void quaternionToRodriguesVector(T* q, T* rvec)
	{
		T qnorm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		if (qnorm < 1000 * std::numeric_limits<T>::epsilon()) {
			rvec[0] = rvec[1] = rvec[2] = 0;
			return;
		}

		T qn[4];
		qnorm = 1 / qnorm;
		qn[0] = q[0] * qnorm;
		qn[1] = q[1] * qnorm;
		qn[2] = q[2] * qnorm;
		qn[3] = q[3] * qnorm;
		T theta = 2 * acos(qn[0]); // [0, 2 * pi]

		const T PI = 3.1415926535897;
		const T twoPI = PI * 2;
		T thetaWrap = std::fmod(theta + PI, twoPI);
		if (thetaWrap < 1000 * std::numeric_limits<T>::epsilon()) {
			thetaWrap =  twoPI;
		}

		theta = thetaWrap - PI; // [-pi, pi]

		if (std::fabs(theta) < 1000 * std::numeric_limits<T>::epsilon()) {
			rvec[0] = rvec[1] = rvec[2] = 0;
		}
		else {
			T tmp = theta / sqrt(qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]);
			rvec[0] = tmp * qn[1];
			rvec[1] = tmp * qn[2];
			rvec[2] = tmp * qn[3];
		}
	}
    
    // Convert transformation matrix to pose containing translation and normalized quaternion
    // tform is a pre-multiply transformation matrix.
    template <typename T>
    void tform2quatpose(const T* tform, T* pose) {
        
        T Qxx = tform[0];
        T Qyx = tform[1];
        T Qzx = tform[2];
        T Qxy = tform[4];
        T Qyy = tform[5];
        T Qzy = tform[6];
        T Qxz = tform[8];
        T Qyz = tform[9];
        T Qzz = tform[10];
        
        T t = Qxx + Qyy + Qzz;
        
        T r, s, w, x, y, z;
        if (t >= 0) {
            r = sqrt(1+t);
            s = 0.5/r;
            w = 0.5*r;
            x = (Qzy-Qyz)*s;
            y = (Qxz-Qzx)*s;
            z = (Qyx-Qxy)*s;
        } else {
            T maxVal = std::max(Qxx, std::max(Qyy, Qzz));
            
            if (maxVal == Qxx) {
                r = sqrt(1+Qxx-Qyy-Qzz);
                s = 0.5/r;
                w = (Qzy-Qyz)*s;
                x = 0.5*r;
                y = (Qyx+Qxy)*s;
                z = (Qxz+Qzx)*s;
            } else if (maxVal == Qyy) {
                r = sqrt(1+Qyy-Qxx-Qzz);
                s = 0.5/r;
                w = (Qxz-Qzx)*s;
                x = (Qyx+Qxy)*s;
                y = 0.5*r;
                z = (Qzy+Qyz)*s;
            } else {
                r = sqrt(1+Qzz-Qxx-Qyy);
                s = 0.5/r;
                w = (Qyx-Qxy)*s;
                x = (Qxz+Qzx)*s;
                y = (Qzy+Qyz)*s;
                z = 0.5*r;
            }
        }
        pose[0] = tform[12];
        pose[1] = tform[13];
        pose[2] = tform[14];
        pose[3] = w;
        pose[4] = x;
        pose[5] = y;
        pose[6] = z;
    }
    
    template <typename T>
    void quatpose2tform(const T* pose, T* tform) {
        
        tform[3]  = T(0.0);
        tform[7]  = T(0.0);
        tform[11] = T(0.0);
        tform[12] = pose[0];
        tform[13] = pose[1];
        tform[14] = pose[2];
        tform[15] = T(1.0);
        
        T q0 = pose[3];
        T qx = pose[4];
        T qy = pose[5];
        T qz = pose[6];
        
        T q02(q0*q0), qx2(qx*qx), qy2(qy*qy), qz2(qz*qz);
        T qxy(qx*qy), q0z(q0*qz), qxz(qx*qz), q0y(q0*qy);
        T qyz(qy*qz), q0x(q0*qx);
        tform[0]  = q02 + qx2 - qy2 - qz2;
        tform[1]  = 2*qxy + 2*q0z;
        tform[2]  = 2*qxz - 2*q0y;
        tform[4]  = 2*qxy - 2*q0z;
        tform[5]  = q02 - qx2 + qy2 - qz2;
        tform[6]  = 2*qyz + 2*q0x;
        tform[8]  = 2*qxz + 2*q0y;
        tform[9]  = 2*qyz - 2*q0x;
        tform[10] = q02 - qx2 - qy2 + qz2;
    }
}

#endif // TMW_QUATERNION_HPP
