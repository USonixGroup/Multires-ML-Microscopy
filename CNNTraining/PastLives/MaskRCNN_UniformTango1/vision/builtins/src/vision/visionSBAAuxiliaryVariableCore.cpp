/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains the built-in function to construct auxiliary variables to solve SBA problem.
// Copyright 2021 The MathWorks, Inc.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS // Used only during Codegen
#include "visionSBAAuxiliaryVariableCore_api.hpp"
#include "rodriguesVectorToMatrix.hpp"
#include "reprojectPointToCamera.hpp"
#include "quaternion.hpp"
#include <string>
#include <cstring>

void visionSBAAuxiliaryVariable(void* mxyzPoints, void* measurementsIn,
                               const size_t numViews, double* cameraPoses, double* quaternionBases,
                               const bool bSingleCamera, void* mType, const bool needJacobian,
                               const int* irs, const int* jcs, void* mFixedCameraIndex,
                               const size_t numFixedCameraIndex, void* mFocal, void* mCenter,
                               const bool flagRadial, const bool flagTang, void* mradialDistort,
                               void* mtangDistort, const size_t numRad, void* mSkew, void* mErr,
                               void* mUj, void* mVi, void* mWij, void* meaj, void* mebi) {

    const std::string optimType = static_cast<const char*>(mType);
    double* fixedCameraIndex = nullptr;
    const double* measurements = static_cast<const double*>(measurementsIn);
    double* xyzPoints = static_cast<double*>(mxyzPoints);
    size_t numCameraIndices = 0;
    double* focalD = static_cast<double*>(mFocal);
    double* centerD = static_cast<double*>(mCenter);
    double* skewD = static_cast<double*>(mSkew);
    
    if(strcmp("motion", optimType.c_str())) {
        fixedCameraIndex = static_cast<double*>(mFixedCameraIndex);
        numCameraIndices = numFixedCameraIndex;
    }

    double* err = static_cast<double*>(mErr);
    double* Uj = static_cast<double*>(mUj);
    double* Vi = static_cast<double*>(mVi);
    double* Wij = static_cast<double*>(mWij);
    double* eaj = static_cast<double*>(meaj);
    double* ebi = static_cast<double*>(mebi);

    for (size_t j = 0; j < numViews; j++) {
        size_t index = bSingleCamera ? 0 : j;
        double* focal = focalD + index*2;
        double* center = centerD + index*2;
        double* kr = nullptr;
        double* kt = nullptr;
        size_t numRadialCoefs = 0;

        if (flagRadial) {
            kr = static_cast<double*>(mradialDistort);
            numRadialCoefs = numRad;
        }

        if(flagTang)
            kt = static_cast<double*>(mtangDistort);

        double skew = skewD[index];
        double * rj_old = cameraPoses + j * 6;
        double * tj = rj_old + 3;
        double * qj = quaternionBases + j * 4;
        double rj[3], q1[4], q2[4];
        vision::rodriguesVectorToQuaternion(rj_old, q1);
        vision::quaternionMultiplication(qj, q1, q2);
        vision::quaternionToRodriguesVector(q2, rj);
        
        double R[9], dRdr[27];
        if (needJacobian) {
            vision::rodriguesVectorToMatrix(rj, R, dRdr);
        }
        else {
            vision::rodriguesVectorToMatrix(rj, R);
        }
        
        size_t k1 = jcs[j]; // # nonzeros before j-th column
        size_t k2 = jcs[j + 1];
        
        for (size_t k = k1; k < k2; k++) {
            size_t i = irs[k];
            double * Xi = xyzPoints + 3 * i;
            double Xp[2];
            double dXpdr[6], dXpdT[6]; // 2 x 3
            double dXpdX[6]; // 2 x 3


            if (needJacobian) {
                vision::reprojectPointToCamera<double>(Xi, R, dRdr, tj, focal, center, kr, numRadialCoefs, kt, skew, Xp, dXpdX, dXpdr, dXpdT);
            }
            else {
                vision::reprojectPointToCamera<double>(Xi, R, nullptr, tj, focal, center, kr, numRadialCoefs, kt, skew, Xp);
            }

            // set output
            double ex = measurements[k * 2] - Xp[0];
            double ey = measurements[k * 2 + 1] - Xp[1];
            err[k * 2] = ex;
            err[k * 2 + 1] = ey;
                        
            if (needJacobian) {
                // A_ij: 2x6 [dXpdr, dXpdT]
                double A[12], At[12];
                double C[36], Ct[36];
                
                bool isFixedCamera = false;
                for (size_t kk = 0; kk < numCameraIndices; ++kk) {
                    if (fixedCameraIndex[kk] == j + 1) {
                        isFixedCamera = true;
                        break;
                    }
                }
                if (!isFixedCamera) {
                    A[0] = dXpdr[0];
                    A[1] = dXpdr[1];
                    A[2] = dXpdr[2];
                    A[3] = dXpdT[0];
                    A[4] = dXpdT[1];
                    A[5] = dXpdT[2];
                    A[6] = dXpdr[3];
                    A[7] = dXpdr[4];
                    A[8] = dXpdr[5];
                    A[9] = dXpdT[3];
                    A[10] = dXpdT[4];
                    A[11] = dXpdT[5];
                    
                    vision::matrixTranspose(A, 2, 6, At);
                    
                    // A' * A
                    if (strcmp("struct", optimType.c_str())) {
                        vision::matrixMultiplyAtA(A, 2, 6, C);
                        vision::matrixTranspose(C, 6, 6, Ct);
                        
                        // Uj: see equation (12)
                        for (size_t m = 0; m < 36; ++m) {
                            Uj[j * 36 + m] += Ct[m];
                        }
                        
                        // eaj: see equation (15)
                        eaj[6 * j] += (dXpdr[0] * ex + dXpdr[3] * ey);
                        eaj[6 * j + 1] += (dXpdr[1] * ex + dXpdr[4] * ey);
                        eaj[6 * j + 2] += (dXpdr[2] * ex + dXpdr[5] * ey);
                        eaj[6 * j + 3] += (dXpdT[0] * ex + dXpdT[3] * ey);
                        eaj[6 * j + 4] += (dXpdT[1] * ex + dXpdT[4] * ey);
                        eaj[6 * j + 5] += (dXpdT[2] * ex + dXpdT[5] * ey);
                    }
                    
                    // W_ij: 6x3
                    // A' * B
                    if (!strcmp("full", optimType.c_str())) {
                        vision::matrixMultiply(At, 6, 2, dXpdX, 3, C);
                        
                        // W_ij: 6x3, see equation (12)
                        vision::matrixTranspose(C, 6, 3, Wij + k * 18);
                    }
                }
                
                // B_ij: 2x3 dXpdX
                // B' * B
                if (strcmp("motion", optimType.c_str())) {                  
                    vision::matrixMultiplyAtA(dXpdX, 2, 3, C);
                    vision::matrixTranspose(C, 3, 3, Ct);
                    for (size_t m = 0; m < 9; ++m) {
                        Vi[i * 9 + m] += Ct[m];
                    }
                    
                    // ebi: see equation (15)
                    ebi[3 * i] += (dXpdX[0] * ex + dXpdX[3] * ey);
                    ebi[3 * i + 1] += (dXpdX[1] * ex + dXpdX[4] * ey);
                    ebi[3 * i + 2] += (dXpdX[2] * ex + dXpdX[5] * ey);
                 }
             }
         } // end i point
    } // end j view
}
#endif