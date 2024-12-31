////////////////////////////////////////////////////////////////////////////////
//
// Reprojection of a 3-D point into a camera and its jacobian matrix
//
//
// References:
// [1] R.Hartley, A.Zisserman, "Multiple View Geometry in Computer
//     Vision, " Cambridge University Press, 2003.
////////////////////////////////////////////////////////////////////////////////

#ifndef TMW_REPROJECTPOINTTOCAMERA_HPP
#define TMW_REPROJECTPOINTTOCAMERA_HPP

#include <matrix.h>
#include <cmath>
#ifndef COMPILE_FOR_VISION_BUILTINS
#include "rodriguesVectorToMatrix.hpp"
#else
#include <calibration/rodriguesVectorToMatrix.hpp>
#include <mex.h>
#endif

namespace vision
{
	//========================================================================================
	// The function projects a 3-D point into a camera image plane.
	// Additionally, it computes the Jacobian of the output w.r.t the input.
	// Note that the function assumes row-major memory layout.
	//
	// Parameters:
	//   X				: 3-D point (in)
	//   Rmatrix		: 3-by-3 rotation matrix (in)
	//   dRdr			: 9-by-3 jacobian of rotation matrix to vector
	//   tvector		: 3-by-1 translation vector (in)
	//   focal			: 2-by-1 focal length (in)
	//   center			: 2-by-1 principal point (in)
	//   kr             : 2-by-1 or 3-by-1 radial distortion coefficients (in)
	//   numRadialCoefs : specify 2 or 3 coefficients (in)
	//   kt				: 2-by-1 tangential distortion coefficients (in)
	//   skew			: skew coefficient (in)
	//   Xp				: 2-by-1 2-D projection (out)
	//   dXpdX			: 2-by-3 derivative of projection w.r.t X (out)
	//   dXpdr			: 2-by-3 derivative of projection w.r.t r (rotation vector) (out)
	//   dXpdt			: 2-by-3 derivative of projection w.r.t t (translation vector) (out)
	//   dXpdf			: 2-by-2 derivative of projection w.r.t f (focal length) (out)
	//   dXpdc			: 2-by-2 derivative of projection w.r.t c (center vector) (out)
	//   dXpdkr			: 2-by-2 or 2-by-3 derivative of projection w.r.t kr (radial distortion coefficients) (out)
	//   dXpdkt			: 2-by-2 derivative of projection w.r.t kt (tangential distortion coefficients) (out)
	//   dXpds			: 2-by-1 derivative of projection w.r.t skew  (out)
	//
	// Let P be a point in 3D of coordinates X in the world reference frame.
	// The coordinate vector of P in the camera reference frame is : Xc = R*X + T
	// where R is the rotation matrix corresponding to the rotation vector rvec.
	// Let x = Xc(1); y = Xc(2); z = Xc(3);
	// The pinehole projection coordinates of P is[a; b] where a = x / z and b = y / z.
	// Define r ^ 2 = a ^ 2 + b^2.
	// The distorted point coordinates are : xd = [xx; yy] where :
	//
	// xx = a*(1 + kr(1)*r ^ 2 + kr(2)*r ^ 4 + kr(3)*r ^ 6) + 2 * kt(1)*a*b + kt(2)*(r ^ 2 + 2 * a ^ 2);
	// yy = b*(1 + kr(1)*r ^ 2 + kr(2)*r ^ 4 + kr(3)*r ^ 6) + kt(1)*(r ^ 2 + 2 * b ^ 2) + 2 * kt(2)*a*b;
	//
	// The left terms correspond to radial distortion up to 6th degree, the
	// right terms correspond to tangential distortion
	//
	// The final pixel coordinates xp = [xxp; yyp] where:
	//
	// xxp = f(1)*(xx + skew*yy) + c(1)
	// yyp = f(2)*yy + c(2)
	//
	// Note, the function is used in three scenarios:
	// 1. only need to evaluate reprojection
	// reprojectPointToCamera(X, Rmatrix, NULL, tvector, focal, center, kr, numRadialCoefs, kt, skew, Xp, NULL, NULL, NULL,	NULL, NULL, NULL, NULL, NULL);
	//
	// 2. evaluate Xp, dXpdX, dXpdr, dXpdt (fix camera intrinsics)
	// reprojectPointToCamera(X, Rmatrix, dRdr, tvector, focal, center, kr, numRadialCoefs, kt, skew, Xp, dXpdX, dXpdr, dXpdt, NULL, NULL, NULL, NULL, NULL);
	//
	// 3. evaluate Xp, dXpdr, dXpdt, dXpdf, dXpdc, dXpdkr, dXpdkt, dXpds (camera calibration)
	// reprojectPointToCamera(X, Rmatrix, dRdr, tvector, focal, center, kr, numRadialCoefs, kt, skew, Xp, NULL, dXpdr, dXpdt, dXpdf, dXpdc, dXpdkr, dXpdkt, dXpds);
	//
	// If kr is NULL, distortion is ignored.
	//========================================================================================
	template <typename T>
	bool reprojectPointToCamera(T* X, T* Rmatrix, T* dRdr, T* tvector, 
		T* focal, T* center, T* kr, size_t numRadialCoefs, T* kt, T skew,
		T* Xp, T* dXpdX = NULL, T* dXpdr = NULL, T* dXpdt = NULL, 
		T* dXpdf = NULL, T* dXpdc = NULL, T* dXpdkr = NULL, T* dXpdkt = NULL, T* dXpds = NULL)
	{
		T Y[3];
		// apply pinhole model
		// Y = R * X + t
		matrixMultiply(Rmatrix, 3, 3, X, 1, Y);

		for (size_t m = 0; m < 3; ++m) {
			Y[m] += tvector[m];
		}

		if (Y[2] == 0) {
			// this should not happen with a reasonable camera model
			Xp[0] = 0;
			Xp[1] = 0;
			return false;
		}

		T z = 1 / Y[2];
		// normalize by z component
		T x = Y[0] * z;
		T y = Y[1] * z;

		T r2 = 0, r4 = 0, cdist = 0, r6 = 0;
		T a1 = 0, a2 = 0, a3 = 0, delta_x = 0, delta_y = 0;
		T xd2 = 0, yd2 = 0, xd3 = 0, yd3 = 0;

		bool noDistortion = (kr == NULL);

		if (noDistortion) {
			xd3 = x + skew * y;
			yd3 = y;
		}
		else {
			// add distortion
			r2 = x * x + y * y;
			r4 = r2 * r2;
			cdist = 1 + kr[0] * r2 + kr[1] * r4;

			if (numRadialCoefs == 3) {
				r6 = r2 * r4;
				cdist += kr[2] * r6;
			}

			T xd1 = x * cdist;
			T yd1 = y * cdist;

			a1 = 2 * x * y;
			a2 = r2 + 2 * x * x;
			a3 = r2 + 2 * y * y;

			delta_x = kt[0] * a1 + kt[1] * a2;
			delta_y = kt[0] * a3 + kt[1] * a1;
			xd2 = xd1 + delta_x;
			yd2 = yd1 + delta_y;
			xd3 = xd2 + skew * yd2;
			yd3 = yd2;
		}

		Xp[0] = focal[0] * xd3 + center[0];
		Xp[1] = focal[1] * yd3 + center[1];

		if (dRdr != NULL) {
			T dYdR[27]; // 3 x 9
			T dYdT[9]; // 3 x 3
			T dYdr[9]; // 3 x 3
			T dYdX[9]; // 9 x 1

			for (size_t m = 0; m < 27; ++m) {
				dYdR[m] = 0;
			}

			for (size_t m = 0; m < 3; ++m) {
				dYdR[m * 9 + m] = X[0];
				dYdR[m * 9 + m + 3] = X[1];
				dYdR[m * 9 + m + 6] = X[2];
			}

			for (size_t m = 0; m < 9; m++) {
				dYdT[m] = 0;
			}

			dYdT[0] = 1;
			dYdT[4] = 1;
			dYdT[8] = 1;

			// dYdr = dYdR * dRdr
			matrixMultiply(dYdR, 3, 9, dRdr, 3, dYdr);

			if (dXpdX != NULL) {
				for (size_t m = 0; m < 3; ++m) {
					T * p = dYdX + 3 * m;
					p[0] = Rmatrix[m * 3];
					p[1] = Rmatrix[m * 3 + 1];
					p[2] = Rmatrix[m * 3 + 2];
				}
			}

			T bb[3], cc[3];
			bb[0] = bb[1] = bb[2] = -x*z;
			cc[0] = cc[1] = cc[2] = -y*z;

			T dxdr[6], dxdT[6];
			dxdr[0] = z * dYdr[0] + bb[0] * dYdr[6];
			dxdr[1] = z * dYdr[1] + bb[1] * dYdr[7];
			dxdr[2] = z * dYdr[2] + bb[2] * dYdr[8];
			dxdr[3] = z * dYdr[3] + cc[0] * dYdr[6];
			dxdr[4] = z * dYdr[4] + cc[1] * dYdr[7];
			dxdr[5] = z * dYdr[5] + cc[2] * dYdr[8];

			dxdT[0] = z * dYdT[0] + bb[0] * dYdT[6];
			dxdT[1] = z * dYdT[1] + bb[1] * dYdT[7];
			dxdT[2] = z * dYdT[2] + bb[2] * dYdT[8];
			dxdT[3] = z * dYdT[3] + cc[0] * dYdT[6];
			dxdT[4] = z * dYdT[4] + cc[1] * dYdT[7];
			dxdT[5] = z * dYdT[5] + cc[2] * dYdT[8];

			T temp[9], dxdX[6];
			if (dXpdX != NULL) {
				for (size_t m = 0; m < 9; m++) {
					temp[m] = z * dYdX[m];
				}

				dxdX[0] = dxdX[1] = dxdX[2] = x;
				dxdX[3] = dxdX[4] = dxdX[5] = y;

				for (size_t m = 0; m < 3; ++m) {
					dxdX[m] = temp[m] - dxdX[m] * temp[m + 6];
					dxdX[m + 3] = temp[m + 3] - dxdX[m + 3] * temp[m + 6];
				}
			}

			if (noDistortion) {
				for (size_t m = 0; m < 3; ++m) {
					dxdr[m] = dxdr[m] + skew * dxdr[m + 3];
					dxdT[m] = dxdT[m] + skew * dxdT[m + 3];
				}

				for (size_t m = 0; m < 3; ++m) {
					dXpdr[m] = focal[0] * dxdr[m];
					dXpdr[m + 3] = focal[1] * dxdr[m + 3];
					dXpdt[m] = focal[0] * dxdT[m];
					dXpdt[m + 3] = focal[1] * dxdT[m + 3];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 3; ++m) {
						dxdX[m] = dxdX[m] + skew * dxdX[m + 3];
					}

					for (size_t m = 0; m < 3; ++m) {
						dXpdX[m] = focal[0] * dxdX[m];
						dXpdX[m + 3] = focal[1] * dxdX[m + 3];
					}
				}
			}
			else {
				T dr2dr[3], dr2dT[3], dr2dX[3], dr4dr[3], dr4dT[3], dr4dX[3];

				for (size_t m = 0; m < 3; ++m) {
					dr2dr[m] = 2 * x * dxdr[m] + 2 * y * dxdr[m + 3];
					dr2dT[m] = 2 * x * dxdT[m] + 2 * y * dxdT[m + 3];
					dr4dr[m] = 2 * r2 * dr2dr[m];
					dr4dT[m] = 2 * r2 * dr2dT[m];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 3; ++m) {
						dr2dX[m] = 2 * (x * dxdX[m] + y * dxdX[m + 3]);
						dr4dX[m] = 2 * r2 * dr2dX[m];
					}
				}

				T dcdistdr[3], dcdistdT[3], dcdistdX[3];
				for (size_t m = 0; m < 3; ++m) {
					dcdistdr[m] = kr[0] * dr2dr[m] + kr[1] * dr4dr[m];
					dcdistdT[m] = kr[0] * dr2dT[m] + kr[1] * dr4dT[m];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 3; ++m) {
						dcdistdX[m] = kr[0] * dr2dX[m] + kr[1] * dr4dX[m];
					}
				}

				if (numRadialCoefs == 3) {
					T dr6dr[3], dr6dT[3], dr6dX[3];
					for (size_t m = 0; m < 3; ++m) {
						dr6dr[m] = 3 * r4 * dr2dr[m];
						dr6dT[m] = 3 * r4 * dr2dT[m];
					}

					for (size_t m = 0; m < 3; ++m) {
						dcdistdr[m] += kr[2] * dr6dr[m];
						dcdistdT[m] += kr[2] * dr6dT[m];
					}

					if (dXpdX != NULL) {
						for (size_t m = 0; m < 3; ++m) {
							dr6dX[m] = 2 * r4 * dr2dX[m];
							dcdistdX[m] += kr[2] * dr6dX[m];
						}
					}
				}

				T dxd1dr[6], dxd1dT[6], dxd1dX[6];
				for (size_t m = 0; m < 3; ++m) {
					dxd1dr[m] = x * dcdistdr[m] + cdist * dxdr[m];
					dxd1dr[m + 3] = y * dcdistdr[m] + cdist * dxdr[m + 3];
					dxd1dT[m] = x * dcdistdT[m] + cdist * dxdT[m];
					dxd1dT[m + 3] = y * dcdistdT[m] + cdist * dxdT[m + 3];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 3; ++m) {
						dxd1dX[m] = x * dcdistdX[m] + cdist * dxdX[m];
						dxd1dX[m + 3] = y * dcdistdX[m] + cdist * dxdX[m + 3];
					}
				}

				T ddelta_xdr[6], ddelta_xdT[6], ddelta_xdX[6];
				T a, b, c;
				a = 2 * kt[0] * y + 6 * kt[1] * x;
				b = 2 * kt[0] * x + 2 * kt[1] * y;
				c = 6 * kt[0] * y + 2 * kt[1] * x;
				for (size_t m = 0; m < 3; ++m) {
					ddelta_xdr[m] = a * dxdr[m] + b * dxdr[m + 3];
					ddelta_xdr[m + 3] = b * dxdr[m] + c * dxdr[m + 3];
					ddelta_xdT[m] = a * dxdT[m] + b * dxdT[m + 3];
					ddelta_xdT[m + 3] = b * dxdT[m] + c * dxdT[m + 3];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 3; ++m) {
						ddelta_xdX[m] = 2 * kt[0] * (dxdX[m] * y + dxdX[m + 3] * x)
							+ kt[1] * (6 * dxdX[m] * x + 2 * dxdX[m + 3] * y);
						ddelta_xdX[m + 3] = kt[0] * (6 * dxdX[m + 3] * y + 2 * dxdX[m] * x)
							+ 2 * kt[1] * (dxdX[m] * y + dxdX[m + 3] * x);
					}
				}

				T dxd2dr[6], dxd2dT[6], dxd2dX[6];
				for (size_t m = 0; m < 6; ++m) {
					dxd2dr[m] = dxd1dr[m] + ddelta_xdr[m];
					dxd2dT[m] = dxd1dT[m] + ddelta_xdT[m];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 6; ++m) {
						dxd2dX[m] = dxd1dX[m] + ddelta_xdX[m];
					}
				}

				T dxd3dr[6], dxd3dT[6], dxd3dX[6];
				for (size_t m = 0; m < 3; ++m) {
					dxd3dr[m] = dxd2dr[m] + skew * dxd2dr[m + 3];
					dxd3dr[m + 3] = dxd2dr[m + 3];
					dxd3dT[m] = dxd2dT[m] + skew * dxd2dT[m + 3];
					dxd3dT[m + 3] = dxd2dT[m + 3];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 3; ++m) {
						dxd3dX[m] = dxd2dX[m] + skew * dxd2dX[m + 3];
						dxd3dX[m + 3] = dxd2dX[m + 3];
					}
				}

				for (size_t m = 0; m < 3; ++m) {
					dXpdr[m] = focal[0] * dxd3dr[m];
					dXpdr[m + 3] = focal[1] * dxd3dr[m + 3];
					dXpdt[m] = focal[0] * dxd3dT[m];
					dXpdt[m + 3] = focal[1] * dxd3dT[m + 3];
				}

				if (dXpdX != NULL) {
					for (size_t m = 0; m < 3; ++m) {
						dXpdX[m] = focal[0] * dxd3dX[m];
						dXpdX[m + 3] = focal[1] * dxd3dX[m + 3];
					}
				}

				// derivatives w.r.t intrinsics
				if (dXpdkr != NULL) {
					T dcdistdk[5], dxd1dk[10], ddelta_xdk[10], dxd2dk[10], dxd3dk[10];

					dcdistdk[0] = r2;
					dcdistdk[1] = r4;
					dcdistdk[2] = 0;
					dcdistdk[3] = 0;
					dcdistdk[4] = (numRadialCoefs == 3) ? r6 : 0;

					for (size_t m = 0; m < 5; ++m) {
						dxd1dk[m] = x * dcdistdk[m];
						dxd1dk[m + 5] = y * dcdistdk[m];
						ddelta_xdk[m] = 0;
						ddelta_xdk[m + 5] = 0;
					}

					ddelta_xdk[2] = a1;
					ddelta_xdk[3] = a2;
					ddelta_xdk[7] = a3;
					ddelta_xdk[8] = a1;

					for (size_t m = 0; m < 10; ++m) {
						dxd2dk[m] = dxd1dk[m] + ddelta_xdk[m];
						dxd3dk[m] = dxd2dk[m];
					}

					for (size_t m = 0; m < 5; ++m) {
						dxd3dk[m] += skew * dxd2dk[m + 5];
					}

					T dxd3ds[2];
					dxd3ds[0] = yd2;
					dxd3ds[1] = 0;

					if (numRadialCoefs == 3) {
						dXpdkr[0] = focal[0] * dxd3dk[0];
						dXpdkr[1] = focal[0] * dxd3dk[1];
						dXpdkr[2] = focal[0] * dxd3dk[4];
						dXpdkr[3] = focal[1] * dxd3dk[5];
						dXpdkr[4] = focal[1] * dxd3dk[6];
						dXpdkr[5] = focal[1] * dxd3dk[9];
					}
					else {
						dXpdkr[0] = focal[0] * dxd3dk[0];
						dXpdkr[1] = focal[0] * dxd3dk[1];
						dXpdkr[2] = focal[1] * dxd3dk[5];
						dXpdkr[3] = focal[1] * dxd3dk[6];
					}

					dXpdkt[0] = focal[0] * dxd3dk[2];
					dXpdkt[1] = focal[0] * dxd3dk[3];
					dXpdkt[2] = focal[1] * dxd3dk[7];
					dXpdkt[3] = focal[1] * dxd3dk[8];

					dXpds[0] = focal[0] * dxd3ds[0];
					dXpds[1] = focal[1] * dxd3ds[1];

					dXpdf[0] = xd3;
					dXpdf[1] = 0;
					dXpdf[2] = 0;
					dXpdf[3] = yd3;

					dXpdc[0] = 1;
					dXpdc[1] = 0;
					dXpdc[2] = 0;
					dXpdc[3] = 1;
				} // end if (dXpdkr != NULL)
			} // end if (noDistortion)
		} // end if (dRdr != NULL) 

		return true;
	} 
} 

#endif // TMW_REPROJECTPOINTTOCAMERA_HPP