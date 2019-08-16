/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: trvec2tform.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/trvec2tform.h"

/* Function Definitions */

/*
 * Arguments    : const double t[3]
 *                double H[16]
 * Return Type  : void
 */
void trvec2tform(const double t[3], double H[16])
{
  int jcol;
  signed char b_I[16];
  int iacol_tmp;
  for (jcol = 0; jcol < 16; jcol++) {
    b_I[jcol] = 0;
  }

  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  for (jcol = 0; jcol < 4; jcol++) {
    iacol_tmp = jcol << 2;
    H[iacol_tmp] = b_I[iacol_tmp];
    H[iacol_tmp + 1] = b_I[iacol_tmp + 1];
    H[iacol_tmp + 2] = b_I[iacol_tmp + 2];
    H[iacol_tmp + 3] = b_I[iacol_tmp + 3];
  }

  H[12] = t[0];
  H[13] = t[1];
  H[14] = t[2];
}

/*
 * File trailer for trvec2tform.c
 *
 * [EOF]
 */
