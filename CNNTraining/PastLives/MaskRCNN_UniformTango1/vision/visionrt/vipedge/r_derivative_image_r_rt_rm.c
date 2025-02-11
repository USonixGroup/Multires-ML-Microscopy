/*
 *  R_DERIVATIVE_IMAGE_R_RT_RM Helper function for Edge block (Canny method).
 *
 *  Copyright 2019 The MathWorks, Inc.
 */
#include "vipedge_rt.h"

LIBMWVISIONRT_API void MWVIP_R_Derivative_Image_R_RM(const real32_T *input,
                                                const real32_T *dgauss1D,
                                                real32_T *filteredDataR,
                                                int_T inpRows,
                                                int_T inpCols,
                                                int_T halfFiltLen)
{   /* Compute the first derivative of the image in row direction */
    /* Seperable Convolution */
    int_T r,c,k, R1, R2;
    real32_T sum;
    
    for (r=0; r<inpRows; r++)
    {
        for (c=0; c<inpCols; c++)
        {
            sum = 0;
            for (k=1; k<halfFiltLen; k++)
            {
                R1 = (r+k)%inpRows; R2 = (r-k+inpRows)%inpRows;
                sum += dgauss1D[k]*(-input[R1*inpCols+c] + input[R2*inpCols+c]);
            }
            filteredDataR[r*inpCols+c] = sum;
        }
    }
}

/* [EOF] r_derivative_image_r_rt_rm.c */

