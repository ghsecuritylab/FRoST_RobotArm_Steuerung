/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rdivide_helper.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/rdivide_helper.h"
#include "IK_solver/IK_solver_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                const emxArray_real_T *y
 *                emxArray_real_T *z
 * Return Type  : void
 */
void rdivide_helper(const emxArray_real_T *x, const emxArray_real_T *y,
                    emxArray_real_T *z)
{
  int i43;
  int loop_ub;
  i43 = z->size[0];
  z->size[0] = x->size[0];
  emxEnsureCapacity_real_T(z, i43);
  loop_ub = x->size[0];
  for (i43 = 0; i43 < loop_ub; i43++) {
    z->data[i43] = x->data[i43] / y->data[i43];
  }
}

/*
 * File trailer for rdivide_helper.c
 *
 * [EOF]
 */
