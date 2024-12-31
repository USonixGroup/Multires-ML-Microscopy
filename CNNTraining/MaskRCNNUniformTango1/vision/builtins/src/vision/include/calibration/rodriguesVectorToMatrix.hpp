////////////////////////////////////////////////////////////////////////////////
//
// Rodrigues vector to matrix form
//
// References:
// [1] R.Hartley, A.Zisserman, "Multiple View Geometry in Computer
//     Vision, " Cambridge University Press, 2003.
//
// [2] E.Trucco, A.Verri. "Introductory Techniques for 3-D Computer
//     Vision, " Prentice Hall, 1998.
////////////////////////////////////////////////////////////////////////////////

#ifndef TMW_RODRIGUESVECTORTOMATRIX_HPP
#define TMW_RODRIGUESVECTORTOMATRIX_HPP

#include <matrix.h>
#include <cmath>
#ifndef COMPILE_FOR_VISION_BUILTINS
#include "matrixUtils.hpp"
#else
#include <calibration/matrixUtils.hpp>
#include <mex.h>
#endif

namespace vision
{
	//========================================================================================
	// The function converts a 3-element rodrigues rotation vector to a 3-by-3 rotation matrix.
	// Additionally, it computes the Jacobian of the output w.r.t the input.
	// Note that the output assumes row-major memory layout.
	//
	// Parameters:
	//   rotationVector	: 3-element input vector
	//   rotationMatrix : buffer for output 3-by-3 matrix (allocated by caller)
	//   jac			: buffer for 9-by-3 jacobian matrix (allocated by caller)
	//========================================================================================
	template <typename T>
	void rodriguesVectorToMatrix(T* rotationVector, T* rotationMatrix, T* jac = NULL)
	{
		T theta = 0;
		for (int i = 0; i < 3; i++) {
			theta += rotationVector[i] * rotationVector[i];
		}

		theta = sqrt(theta);
		if (theta < 1e-5) {
			for (int i = 0; i < 9; i++) {
				rotationMatrix[i] = 0;
			}

			rotationMatrix[0] = 1;
			rotationMatrix[4] = 1;
			rotationMatrix[8] = 1;

			if (jac != NULL) {
				for (int i = 0; i < 27; i++) {
					jac[i] = 0;
				}

				jac[5] = 1;
				jac[7] = -1;
				jac[11] = -1;
				jac[15] = 1;
				jac[19] = 1;
				jac[21] = -1;
			}

			return;
		}

		T u[3];
		for (int i = 0; i < 3; i++) {
			u[i] = rotationVector[i] / theta;
		}

		T w1 = u[0], w2 = u[1], w3 = u[2];
		T A[9], B[9];
		A[0] = 0;
		A[1] = -w3;
		A[2] = w2;
		A[3] = w3;
		A[4] = 0;
		A[5] = -w1;
		A[6] = -w2;
		A[7] = w1;
		A[8] = 0;

		B[0] = w1 * w1;
		B[1] = w1 * w2;
		B[2] = w1 * w3;
		B[3] = w2 * w1;
		B[4] = w2 * w2;
		B[5] = w2 * w3;
		B[6] = w3 * w1;
		B[7] = w3 * w2;
		B[8] = w3 * w3;

		T alpha = cos(theta);
		T beta = sin(theta);
		T gamma = 1 - alpha;

		for (int i = 0; i < 9; i++) {
			rotationMatrix[i] = beta*A[i] + gamma*B[i];
		}

		rotationMatrix[0] += alpha;
		rotationMatrix[4] += alpha;
		rotationMatrix[8] += alpha;

		if (jac != NULL) {
			T dm3dr[12]; // 4x3
			dm3dr[0] = 1;
			dm3dr[4] = 1;
			dm3dr[8] = 1;
			dm3dr[1] = 0;
			dm3dr[2] = 0;
			dm3dr[3] = 0;
			dm3dr[5] = 0;
			dm3dr[6] = 0;
			dm3dr[7] = 0;
			dm3dr[9] = w1;
			dm3dr[10] = w2;
			dm3dr[11] = w3;

			T dm2dm3[16]; // 4x4
			dm2dm3[0] = 1 / theta;
			dm2dm3[5] = 1 / theta;
			dm2dm3[10] = 1 / theta;
			dm2dm3[1] = 0;
			dm2dm3[2] = 0;
			dm2dm3[3] = -rotationVector[0] / (theta*theta);
			dm2dm3[4] = 0;
			dm2dm3[6] = 0;
			dm2dm3[7] = -rotationVector[1] / (theta*theta);
			dm2dm3[8] = 0;
			dm2dm3[9] = 0;
			dm2dm3[11] = -rotationVector[2] / (theta*theta);
			dm2dm3[12] = 0;
			dm2dm3[13] = 0;
			dm2dm3[14] = 0;
			dm2dm3[15] = 1;

			T dm1dm2[84] = { 0, 0, 0, -beta,
				0, 0, 0, alpha,
				0, 0, 0, beta,
				0, 0, 0, 0,
				0, 0, 1, 0,
				0, -1, 0, 0,
				0, 0, -1, 0,
				0, 0, 0, 0,
				1, 0, 0, 0,
				0, 1, 0, 0,
				-1, 0, 0, 0,
				0, 0, 0, 0,
				2 * w1, 0, 0, 0,
				w2, w1, 0, 0,
				w3, 0, w1, 0,
				w2, w1, 0, 0,
				0, 2 * w2, 0, 0,
				0, w3, w2, 0,
				w3, 0, w1, 0,
				0, w3, w2, 0,
				0, 0, 2 * w3, 0 }; // 21x4

			T dRdm1[189]; // 9x21
			for (int i = 0; i < 189; i++) {
				dRdm1[i] = 0;
			}

			dRdm1[0] = 1;
			dRdm1[84] = 1;
			dRdm1[168] = 1;
			dRdm1[3] = beta;
			dRdm1[25] = beta;
			dRdm1[47] = beta;
			dRdm1[69] = beta;
			dRdm1[91] = beta;
			dRdm1[113] = beta;
			dRdm1[135] = beta;
			dRdm1[157] = beta;
			dRdm1[179] = beta;
			dRdm1[12] = gamma;
			dRdm1[34] = gamma;
			dRdm1[56] = gamma;
			dRdm1[78] = gamma;
			dRdm1[100] = gamma;
			dRdm1[122] = gamma;
			dRdm1[144] = gamma;
			dRdm1[166] = gamma;
			dRdm1[188] = gamma;

			T At[9], Bt[9];
			matrixTranspose(A, 3, 3, At);
			matrixTranspose(B, 3, 3, Bt);

			for (int i = 0; i < 9; i++) {
				dRdm1[i * 21 + 1] = At[i];
			}

			for (int i = 0; i < 9; i++) {
				dRdm1[i * 21 + 2] = Bt[i];
			}

			T tmp1[36], tmp2[36];
			matrixMultiply(dRdm1, 9, 21, dm1dm2, 4, tmp1);
			matrixMultiply(tmp1, 9, 4, dm2dm3, 4, tmp2);
			matrixMultiply(tmp2, 9, 4, dm3dr, 3, jac);
		}
	}

}

#endif // TMW_RODRIGUESVECTORTOMATRIX_HPP