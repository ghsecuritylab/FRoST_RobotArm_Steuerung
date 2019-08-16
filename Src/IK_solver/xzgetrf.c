/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xzgetrf.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <math.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/xzgetrf.h"
#include "IK_solver/colon.h"

/* Function Definitions */

/*
 * Arguments    : int m
 *                int n
 *                emxArray_real_T *A
 *                int lda
 *                emxArray_int32_T *ipiv
 *                int *info
 * Return Type  : void
 */
void xzgetrf(int m, int n, emxArray_real_T *A, int lda, emxArray_int32_T *ipiv,
             int *info)
{
  int iy;
  int u0;
  int j;
  int mmj;
  int b_tmp;
  int jp1j;
  int ix;
  double smax;
  int jy;
  int i52;
  double s;
  int b_j;
  int i53;
  int ijA;
  if (m < n) {
    iy = m;
  } else {
    iy = n;
  }

  eml_signed_integer_colon(iy, ipiv);
  *info = 0;
  if ((m < 1) || (n < 1)) {
  } else {
    u0 = m - 1;
    if (u0 >= n) {
      u0 = n;
    }

    for (j = 0; j < u0; j++) {
      mmj = m - j;
      b_tmp = j * (lda + 1);
      jp1j = b_tmp + 2;
      if (mmj < 1) {
        iy = -1;
      } else {
        iy = 0;
        if (mmj > 1) {
          ix = b_tmp;
          smax = fabs(A->data[b_tmp]);
          for (jy = 2; jy <= mmj; jy++) {
            ix++;
            s = fabs(A->data[ix]);
            if (s > smax) {
              iy = jy - 1;
              smax = s;
            }
          }
        }
      }

      if (A->data[b_tmp + iy] != 0.0) {
        if (iy != 0) {
          iy += j;
          ipiv->data[j] = iy + 1;
          ix = j;
          for (jy = 0; jy < n; jy++) {
            smax = A->data[ix];
            A->data[ix] = A->data[iy];
            A->data[iy] = smax;
            ix += lda;
            iy += lda;
          }
        }

        i52 = b_tmp + mmj;
        for (iy = jp1j; iy <= i52; iy++) {
          A->data[iy - 1] /= A->data[b_tmp];
        }
      } else {
        *info = j + 1;
      }

      iy = n - j;
      jy = b_tmp + lda;
      jp1j = (b_tmp + lda) + 1;
      for (b_j = 0; b_j <= iy - 2; b_j++) {
        smax = A->data[jy];
        if (A->data[jy] != 0.0) {
          ix = b_tmp + 1;
          i52 = jp1j + 1;
          i53 = mmj + jp1j;
          for (ijA = i52; ijA < i53; ijA++) {
            A->data[ijA - 1] += A->data[ix] * -smax;
            ix++;
          }
        }

        jy += lda;
        jp1j += lda;
      }
    }

    if ((*info == 0) && (m <= n) && (!(A->data[(m + A->size[0] * (m - 1)) - 1]
          != 0.0))) {
      *info = m;
    }
  }
}

/*
 * File trailer for xzgetrf.c
 *
 * [EOF]
 */
