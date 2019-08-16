/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: isfinite.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/isfinite.h"
#include "IK_solver/IK_solver_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_boolean_T *b
 * Return Type  : void
 */
void b_isfinite(const emxArray_real_T *x, emxArray_boolean_T *b)
{
  int i8;
  int loop_ub;
  emxArray_boolean_T *r3;
  i8 = b->size[0];
  b->size[0] = x->size[0];
  emxEnsureCapacity_boolean_T(b, i8);
  loop_ub = x->size[0];
  for (i8 = 0; i8 < loop_ub; i8++) {
    b->data[i8] = rtIsInf(x->data[i8]);
  }

  emxInit_boolean_T(&r3, 1);
  i8 = r3->size[0];
  r3->size[0] = x->size[0];
  emxEnsureCapacity_boolean_T(r3, i8);
  loop_ub = x->size[0];
  for (i8 = 0; i8 < loop_ub; i8++) {
    r3->data[i8] = rtIsNaN(x->data[i8]);
  }

  i8 = b->size[0];
  emxEnsureCapacity_boolean_T(b, i8);
  loop_ub = b->size[0];
  for (i8 = 0; i8 < loop_ub; i8++) {
    b->data[i8] = ((!b->data[i8]) && (!r3->data[i8]));
  }

  emxFree_boolean_T(&r3);
}

/*
 * File trailer for isfinite.c
 *
 * [EOF]
 */
