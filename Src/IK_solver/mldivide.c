/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mldivide.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <math.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/mldivide.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/xtrsm.h"
#include "IK_solver/xzgetrf.h"
#include "IK_solver/xgeqp3.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *A
 *                const emxArray_real_T *B
 *                emxArray_real_T *Y
 * Return Type  : void
 */
void mldivide(const emxArray_real_T *A, const emxArray_real_T *B,
              emxArray_real_T *Y)
{
  emxArray_real_T *b_A;
  emxArray_real_T *tau;
  emxArray_int32_T *jpvt;
  emxArray_real_T *b_B;
  unsigned int unnamed_idx_0;
  int i40;
  int rankR;
  unsigned int unnamed_idx_1;
  int minmn;
  int nb;
  int maxmn;
  double tol;
  int j;
  int b_nb;
  int k;
  int i;
  int mn;
  emxInit_real_T(&b_A, 2);
  emxInit_real_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  emxInit_real_T(&b_B, 2);
  if ((A->size[0] == 0) || (A->size[1] == 0) || ((B->size[0] == 0) || (B->size[1]
        == 0))) {
    unnamed_idx_0 = (unsigned int)A->size[1];
    unnamed_idx_1 = (unsigned int)B->size[1];
    i40 = Y->size[0] * Y->size[1];
    Y->size[0] = (int)unnamed_idx_0;
    Y->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity_real_T(Y, i40);
    minmn = (int)unnamed_idx_0 * (int)unnamed_idx_1;
    for (i40 = 0; i40 < minmn; i40++) {
      Y->data[i40] = 0.0;
    }
  } else if (A->size[0] == A->size[1]) {
    rankR = A->size[1];
    i40 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, i40);
    minmn = A->size[0] * A->size[1];
    for (i40 = 0; i40 < minmn; i40++) {
      b_A->data[i40] = A->data[i40];
    }

    xzgetrf(A->size[1], A->size[1], b_A, A->size[1], jpvt, &minmn);
    nb = B->size[1] - 1;
    i40 = Y->size[0] * Y->size[1];
    Y->size[0] = B->size[0];
    Y->size[1] = B->size[1];
    emxEnsureCapacity_real_T(Y, i40);
    minmn = B->size[0] * B->size[1];
    for (i40 = 0; i40 < minmn; i40++) {
      Y->data[i40] = B->data[i40];
    }

    i40 = A->size[1];
    for (minmn = 0; minmn <= i40 - 2; minmn++) {
      if (jpvt->data[minmn] != minmn + 1) {
        maxmn = jpvt->data[minmn] - 1;
        for (b_nb = 0; b_nb <= nb; b_nb++) {
          tol = Y->data[minmn + Y->size[0] * b_nb];
          Y->data[minmn + Y->size[0] * b_nb] = Y->data[maxmn + Y->size[0] * b_nb];
          Y->data[maxmn + Y->size[0] * b_nb] = tol;
        }
      }
    }

    for (j = 0; j <= nb; j++) {
      minmn = rankR * j - 1;
      for (k = 0; k < rankR; k++) {
        maxmn = rankR * k;
        i40 = (k + minmn) + 1;
        if (Y->data[i40] != 0.0) {
          b_nb = k + 2;
          for (i = b_nb; i <= rankR; i++) {
            mn = i + minmn;
            Y->data[mn] -= Y->data[i40] * b_A->data[(i + maxmn) - 1];
          }
        }
      }
    }

    xtrsm(A->size[1], B->size[1], b_A, A->size[1], Y, A->size[1]);
  } else {
    i40 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T(b_A, i40);
    minmn = A->size[0] * A->size[1];
    for (i40 = 0; i40 < minmn; i40++) {
      b_A->data[i40] = A->data[i40];
    }

    xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    }

    if (minmn > 0) {
      tol = 2.2204460492503131E-15 * (double)maxmn;
      if (1.4901161193847656E-8 < tol) {
        tol = 1.4901161193847656E-8;
      }

      tol *= fabs(b_A->data[0]);
      while ((rankR < minmn) && (!(fabs(b_A->data[rankR + b_A->size[0] * rankR])
               <= tol))) {
        rankR++;
      }
    }

    i40 = b_B->size[0] * b_B->size[1];
    b_B->size[0] = B->size[0];
    b_B->size[1] = B->size[1];
    emxEnsureCapacity_real_T(b_B, i40);
    minmn = B->size[0] * B->size[1];
    for (i40 = 0; i40 < minmn; i40++) {
      b_B->data[i40] = B->data[i40];
    }

    nb = B->size[1];
    minmn = b_A->size[1];
    maxmn = B->size[1];
    i40 = Y->size[0] * Y->size[1];
    Y->size[0] = minmn;
    Y->size[1] = maxmn;
    emxEnsureCapacity_real_T(Y, i40);
    minmn *= maxmn;
    for (i40 = 0; i40 < minmn; i40++) {
      Y->data[i40] = 0.0;
    }

    maxmn = b_A->size[0];
    b_nb = B->size[1];
    minmn = b_A->size[0];
    mn = b_A->size[1];
    if (minmn < mn) {
      mn = minmn;
    }

    for (j = 0; j < mn; j++) {
      if (tau->data[j] != 0.0) {
        for (k = 0; k < b_nb; k++) {
          tol = b_B->data[j + b_B->size[0] * k];
          i40 = j + 2;
          for (i = i40; i <= maxmn; i++) {
            tol += b_A->data[(i + b_A->size[0] * j) - 1] * b_B->data[(i +
              b_B->size[0] * k) - 1];
          }

          tol *= tau->data[j];
          if (tol != 0.0) {
            b_B->data[j + b_B->size[0] * k] -= tol;
            i40 = j + 2;
            for (i = i40; i <= maxmn; i++) {
              b_B->data[(i + b_B->size[0] * k) - 1] -= b_A->data[(i + b_A->size
                [0] * j) - 1] * tol;
            }
          }
        }
      }
    }

    for (k = 0; k < nb; k++) {
      for (i = 0; i < rankR; i++) {
        Y->data[(jpvt->data[i] + Y->size[0] * k) - 1] = b_B->data[i + b_B->size
          [0] * k];
      }

      for (j = rankR; j >= 1; j--) {
        Y->data[(jpvt->data[j - 1] + Y->size[0] * k) - 1] /= b_A->data[(j +
          b_A->size[0] * (j - 1)) - 1];
        for (i = 0; i <= j - 2; i++) {
          Y->data[(jpvt->data[i] + Y->size[0] * k) - 1] -= Y->data[(jpvt->data[j
            - 1] + Y->size[0] * k) - 1] * b_A->data[i + b_A->size[0] * (j - 1)];
        }
      }
    }
  }

  emxFree_real_T(&b_B);
  emxFree_int32_T(&jpvt);
  emxFree_real_T(&tau);
  emxFree_real_T(&b_A);
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
