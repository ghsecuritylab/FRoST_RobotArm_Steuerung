/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: isPositiveDefinite.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <math.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/isPositiveDefinite.h"
#include "IK_solver/xscal.h"
#include "IK_solver/IK_solver_emxutil.h"

/* Function Definitions */

/*
 * Arguments    : const emxArray_real_T *B
 * Return Type  : boolean_T
 */
boolean_T isPositiveDefinite(const emxArray_real_T *B)
{
  int n;
  int info;
  emxArray_real_T *A;
  int i44;
  int nmj;
  int b_n;
  int j;
  boolean_T exitg1;
  int jj;
  double ajj;
  int ix;
  int iy;
  int ia0;
  int iac;
  double c;
  int i45;
  int ia;
  n = B->size[1];
  info = 0;
  if (B->size[1] != 0) {
    emxInit_real_T(&A, 2);
    i44 = A->size[0] * A->size[1];
    A->size[0] = B->size[0];
    A->size[1] = B->size[1];
    emxEnsureCapacity_real_T(A, i44);
    nmj = B->size[0] * B->size[1];
    for (i44 = 0; i44 < nmj; i44++) {
      A->data[i44] = B->data[i44];
    }

    b_n = B->size[0];
    if (B->size[0] != 0) {
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j <= b_n - 1)) {
        jj = j + j * n;
        ajj = 0.0;
        if (j >= 1) {
          ix = j;
          iy = j;
          for (nmj = 0; nmj < j; nmj++) {
            ajj += A->data[ix] * A->data[iy];
            ix += n;
            iy += n;
          }
        }

        ajj = A->data[jj] - ajj;
        if (ajj > 0.0) {
          ajj = sqrt(ajj);
          A->data[jj] = ajj;
          if (j + 1 < b_n) {
            nmj = (b_n - j) - 1;
            ia0 = j + 2;
            if ((nmj == 0) || (j == 0)) {
            } else {
              ix = j;
              i44 = (j + n * (j - 1)) + 2;
              for (iac = ia0; n < 0 ? iac >= i44 : iac <= i44; iac += n) {
                c = -A->data[ix];
                iy = jj + 1;
                i45 = (iac + nmj) - 1;
                for (ia = iac; ia <= i45; ia++) {
                  A->data[iy] += A->data[ia - 1] * c;
                  iy++;
                }

                ix += n;
              }
            }

            xscal(nmj, 1.0 / ajj, A, jj + 2);
          }

          j++;
        } else {
          info = j + 1;
          exitg1 = true;
        }
      }
    }

    emxFree_real_T(&A);
  }

  return info == 0;
}

/*
 * File trailer for isPositiveDefinite.c
 *
 * [EOF]
 */
