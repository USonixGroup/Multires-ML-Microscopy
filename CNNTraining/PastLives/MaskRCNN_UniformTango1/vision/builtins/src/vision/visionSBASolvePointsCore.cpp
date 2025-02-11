/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains the built-in function to compute 3-D points in an SBA problem.
// Copyright 2021 The MathWorks, Inc.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS // Used only during Codegen
#include "visionSBASolvePointsCore_api.hpp"
#include "matrixUtils.hpp"

void visionSBASolvePoints(void* mWij, void* mXa, void* mVii, void* mebi,
                         const int* irs, const int* jcs, const size_t numPoints,
                         const size_t numViews, void* mXb, void* mValidJs) {
    
    const double* Wij = static_cast<const double*>(mWij);
    const double* Xa = static_cast<const double*>(mXa);
    const double* Vii = static_cast<const double*>(mVii);
    const double* ebi = static_cast<const double*>(mebi);

    double* Xb = static_cast<double*>(mXb);
    int* validJs = static_cast<int*>(mValidJs);

	for (size_t j = 0; j < numViews; j++) {
		int k1 = jcs[j]; // # nonzeros before j-th column
		int k2 = jcs[j + 1];
		for (int k = k1; k < k2; k++) {
			int i = irs[k];
			validJs[i * numViews + j] = k + 1;
		}
	}

	for (size_t i = 0; i < numPoints; ++i) {
		double W[18], tmp[3];
		double d[3] = {0, 0, 0};
		for (size_t j = 0; j < numViews; ++j) {
			if (validJs[i * numViews + j] > 0) {
				size_t k = validJs[i * numViews + j] - 1;
				const double * pw = Wij + 18 * k;
				for (int mm = 0; mm < 6; mm++) {
					for (int nn = 0; nn < 3; nn++) {
						W[mm * 3 + nn] = pw[nn * 6 + mm];
					}
				}

				vision::matrixMultiplyColumn(W, 3, 6, Xa + j * 6, 1, tmp);
				d[0] += tmp[0];
				d[1] += tmp[1];
				d[2] += tmp[2];
			}
		}

		tmp[0] = ebi[3 * i] - d[0];
		tmp[1] = ebi[3 * i + 1] - d[1];
		tmp[2] = ebi[3 * i + 2] - d[2];

		vision::matrixMultiplyColumn(Vii + 9 * i, 3, 3, tmp, 1, Xb + 3 * i);
	}
}
#endif