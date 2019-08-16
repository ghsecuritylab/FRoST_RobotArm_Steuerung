/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: inv.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/inv.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/xtrsm.h"
#include "IK_solver/colon.h"
#include "IK_solver/xzgetrf.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *x
 *                emxArray_real_T *y
 * Return Type  : void
 */
void inv(const emxArray_real_T *x, emxArray_real_T *y)
{
  int n;
  int i42;
  int pipk;
  emxArray_real_T *b_x;
  emxArray_int32_T *p;
  emxArray_int32_T *ipiv;
  int k;
  int j;
  int i;
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    i42 = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(y, i42);
    pipk = x->size[0] * x->size[1];
    for (i42 = 0; i42 < pipk; i42++) {
      y->data[i42] = x->data[i42];
    }
  } else {
    n = x->size[0];
    i42 = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(y, i42);
    pipk = x->size[0] * x->size[1];
    for (i42 = 0; i42 < pipk; i42++) {
      y->data[i42] = 0.0;
    }

    emxInit_real_T(&b_x, 2);
    i42 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = x->size[0];
    b_x->size[1] = x->size[1];
    emxEnsureCapacity_real_T(b_x, i42);
    pipk = x->size[0] * x->size[1];
    for (i42 = 0; i42 < pipk; i42++) {
      b_x->data[i42] = x->data[i42];
    }

    emxInit_int32_T(&p, 2);
    emxInit_int32_T(&ipiv, 2);
    xzgetrf(x->size[0], x->size[0], b_x, x->size[0], ipiv, &pipk);
    eml_signed_integer_colon(x->size[0], p);
    i42 = ipiv->size[1];
    for (k = 0; k < i42; k++) {
      if (ipiv->data[k] > 1 + k) {
        pipk = p->data[ipiv->data[k] - 1];
        p->data[ipiv->data[k] - 1] = p->data[k];
        p->data[k] = pipk;
      }
    }

    emxFree_int32_T(&ipiv);
    for (k = 0; k < n; k++) {
      pipk = p->data[k] - 1;
      y->data[k + y->size[0] * (p->data[k] - 1)] = 1.0;
      for (j = k + 1; j <= n; j++) {
        if (y->data[(j + y->size[0] * pipk) - 1] != 0.0) {
          i42 = j + 1;
          for (i = i42; i <= n; i++) {
            y->data[(i + y->size[0] * pipk) - 1] -= y->data[(j + y->size[0] *
              pipk) - 1] * b_x->data[(i + b_x->size[0] * (j - 1)) - 1];
          }
        }
      }
    }

    emxFree_int32_T(&p);
    xtrsm(x->size[0], x->size[0], b_x, x->size[0], y, x->size[0]);
    emxFree_real_T(&b_x);
  }
}

/*
 * File trailer for inv.c
 *
 * [EOF]
 */
