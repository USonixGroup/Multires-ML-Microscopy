/*
 *  RC_GAUSSIAN_SMOOTHING_R_RT Helper function for Edge block (Canny method).
 *
 *  Copyright 1995-2019 The MathWorks, Inc.
 */
#include "vipedge_rt.h"

LIBMWVISIONRT_API void MWVIP_RC_Gaussian_Smoothing_R_RM(const real32_T *input,
                                                    const real32_T *gauss1D,
                                                    real32_T *filteredDataC,
                                                    real32_T *filteredDataR,
                                                    int_T inpRows,
                                                    int_T inpCols,
                                                    int_T halfFiltLen)
{
    /* (1D Separable convolution) */
    int_T r,c,k, R1, R2, C1, C2;
    real32_T sumC, sumR;
    int32_T linIdx_rc; /* linear index of [r][c]: c*inpRows+r */
    
    for (r=0; r<inpRows; r++)
    {
        for (c=0; c<inpCols; c++)
        {
            linIdx_rc = r*inpCols+c;
            sumC = gauss1D[0] * input[linIdx_rc];
            sumR = sumC;
            for (k=1; k<halfFiltLen; k++)
            {
                /* Blur in the column direction  */
                R1 = (r+k)%inpRows; R2 = (r-k+inpRows)%inpRows;
                sumC += gauss1D[k]*(input[R1*inpCols+c] + input[R2*inpCols+c]);
                
                /* Blur in the row direction */
                C1 = (c+k)%inpCols; C2 = (c-k+inpCols)%inpCols;
                sumR += gauss1D[k]*(input[r*inpCols+C1] + input[r*inpCols+C2]);
            }
            filteredDataC[linIdx_rc] = sumC;
            filteredDataR[linIdx_rc] = sumR;
        }
    }
}

/* [EOF] rc_gaussian_smoothing_r_rt_rm.c */