/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eye.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/eye.h"
#include "IK_solver/IK_solver_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : double varargin_1
 *                emxArray_real_T *b_I
 * Return Type  : void
 */
void eye(double varargin_1, emxArray_real_T *b_I)
{
  double t;
  int m;
  int k;
  int loop_ub;
  if (varargin_1 < 0.0) {
    t = 0.0;
  } else {
    t = varargin_1;
  }

  m = (int)t;
  k = b_I->size[0] * b_I->size[1];
  b_I->size[0] = (int)t;
  b_I->size[1] = (int)t;
  emxEnsureCapacity_real_T(b_I, k);
  loop_ub = (int)t * (int)t;
  for (k = 0; k < loop_ub; k++) {
    b_I->data[k] = 0.0;
  }

  if ((int)t > 0) {
    for (k = 0; k < m; k++) {
      b_I->data[k + b_I->size[0] * k] = 1.0;
    }
  }
}

/*
 * File trailer for eye.c
 *
 * [EOF]
 */
