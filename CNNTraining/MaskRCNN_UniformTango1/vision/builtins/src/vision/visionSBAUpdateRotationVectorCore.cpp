/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file contains the built-in function to update rotation with quaternion in an SBA problem.
//
// Copyright 2021 The MathWorks, Inc.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS // Used only during Codegen
#include "visionSBAUpdateRotationVectorCore_api.hpp"
#include "quaternion.hpp"


void visionSBAUpdateRotationVector(void* mq, const size_t numViews, void* mr,
                             void* mout) {

    double* q = static_cast<double*>(mq);
    double* r = static_cast<double*>(mr);
    
    double* rout = static_cast<double*>(mout);

    for (size_t j = 0; j < numViews; j++) {
		double * qj = q + j * 4;
		double * rj = r + j * 3;
		double * routj = rout + j * 3;
		
		double q1[4], q2[4];
		vision::rodriguesVectorToQuaternion(rj, q1);
		vision::quaternionMultiplication(qj, q1, q2);
		vision::quaternionToRodriguesVector(q2, routj);
	}
}
#endif