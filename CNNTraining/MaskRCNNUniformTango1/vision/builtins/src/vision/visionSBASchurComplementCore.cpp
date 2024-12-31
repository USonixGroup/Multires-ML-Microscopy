/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains the built-in function to construct schur complements to solve SBA problem.
// The step is described in the following reference:
//
// Manolis Lourakis and Antonis Argyros, The Design and Implementation of a Generic Sparse Bundle Adjustment
// Software Package Based on the Levenberg-Marquardt Algorithm, Technical Report, 2004.
//
// The MATLAB API is:
// [S, e, Vii] = visionSBASchurComplement(Uj, Vi, Wij, eaj, ebi, visibility);
//
// - Uj: 6 x (6V) matrix, double
// - Vi: 3 x (3M) matrix, double
// - Wij: 6 x (6N) matrix, double
// - eaj: 6 x V matrix, double
// - ebi: 3 x M matrix, double
// - visibility: M (points) x V (views) sparse matrix, double
//
// - S: (6V) x (6V) matrix, double
// - e: 6 x V matrix, double
//      The camera parameters are solved by S \ e(:)
// - Vii: inverse of each 3 x 3 block in Vi
//
// Copyright 2021 The MathWorks, Inc.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS // Used only during Codegen
#include "visionSBASchurComplementCore_api.hpp"
#include "matrixUtils.hpp"
#include "realSymmetricEigenSolver.hpp"

void visionSBASchurComplement(void* mUj, void* mVi, void* mWij, void* meaj,
                             void* mebi, const int32_t* irs, const int32_t* jcs,
                             const size_t numPoints, const size_t numViews,
                             void* mS, void* me, void* mVii) {

    double* Uj = static_cast<double*>(mUj);
	double* Vi = static_cast<double*>(mVi);
	double* Wij = static_cast<double*>(mWij);
	double* eaj = static_cast<double*>(meaj);
	double* ebi = static_cast<double*>(mebi);

    // Outputs casting
    double* S = static_cast<double*>(mS);
    double* e = static_cast<double*>(me);
    double* Vii = static_cast<double*>(mVii);


    for (size_t j = 0; j < numViews * 6; ++j) {
		e[j] = eaj[j];
	}
    

    for (size_t i = 0; i < numPoints; ++i) {
		vision::syminv3<double>(Vi + i * 9, Vii + i * 9);
	}

    int32_t maxIs = 0; // maximum number of points in a view
	for (size_t j = 0; j < numViews; ++j) {
		int32_t n = jcs[j + 1] - jcs[j];
		maxIs = maxIs < n ? n : maxIs;
	}

    // only store j-column of Yij
	double * Ys = (double *)malloc(maxIs * 6 * 3 * sizeof(double));
	int* validIs = (int *)malloc(numPoints * sizeof(int));

    for (size_t j = 0; j < numViews; ++j) {
		int32_t k1 = jcs[j]; // # nonzeros before j-th column
		int32_t k2 = jcs[j + 1];
        
		for (size_t k = 0; k < numPoints; ++k) {
			validIs[k] = -1;
		}

		// compute all Y_ij = W_ij * (V*_i)^-1 for a fixed j
		for (int32_t k = k1; k < k2; k++) {
			int32_t i = irs[k];
			validIs[i] = static_cast<int>(k - k1);

			vision::matrixMultiplyColumn(Wij + 18 * k, 6, 3, Vii + 9 * i, 3, Ys + 18 * (k - k1));

			double tmp[6];
			vision::matrixMultiplyColumn(Ys + 18 * (k - k1), 6, 3, ebi + 3 * i, 1, tmp);

			for (size_t m = 0; m < 6; ++m) {
				e[j * 6 + m] -= tmp[m];
			}
		}


		for (size_t k = j; k < numViews; ++k) {
			double yjk[36];
			for (int m = 0; m < 36; m++) {
				yjk[m] = 0;
			}

			int32_t m1 = jcs[k]; // # nonzeros before j-th column
			int32_t m2 = jcs[k + 1];
			for (int32_t m = m1; m < m2; m++) {
				if (validIs[irs[m]] >= 0) {
					double temp[36], Wik[18];	
					double * pw = Wij + 18 * m;
					for (int mm = 0; mm < 6; mm++) {
						for (int nn = 0; nn < 3; nn++) {
							Wik[mm * 3 + nn] = pw[nn * 6 + mm];
						}
					}

					vision::matrixMultiplyColumn(Ys + 18 * (validIs[irs[m]]), 6, 3, Wik, 6, temp);

					for (int mm = 0; mm < 36; mm++) {
						yjk[mm] += temp[mm];
					}
				}
			}

			if (j == k) {
				for (int mm = 0; mm < 6; mm++) {
					for (int nn = 0; nn < 6; nn++) {
						S[(k * 6 + nn) * 6 * numViews + j * 6 + mm] = Uj[(6 * j + nn) * 6 + mm] - yjk[nn * 6 + mm];
					}
				}
			}
			else {
				for (int mm = 0; mm < 6; mm++) {
					for (int nn = 0; nn < 6; nn++) {
						S[(k * 6 + nn) * 6 * numViews + j * 6 + mm] = -yjk[nn * 6 + mm];
					}
				}
			}
		}
	}

	for (size_t j = 0; j < numViews; ++j) {
		for (size_t k = 0; k < j; ++k) {
			for (int mm = 0; mm < 6; mm++) {
				for (int nn = 0; nn < 6; nn++) {
					S[(k * 6 + nn) * 6 * numViews + j * 6 + mm] = S[(j * 6 + mm) * 6 * numViews + k * 6 + nn];
				}
			}
		}
	}
}
#endif