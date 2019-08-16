/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: xaxpy.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/xaxpy.h"

/* Function Definitions */

/*
 * Arguments    : int n
 *                double a
 *                const double x[9]
 *                int ix0
 *                double y[3]
 *                int iy0
 * Return Type  : void
 */
void b_xaxpy(int n, double a, const double x[9], int ix0, double y[3], int iy0)
{
  int ix;
  int iy;
  int i50;
  int k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i50 = n - 1;
    for (k = 0; k <= i50; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                double a
 *                const double x[3]
 *                int ix0
 *                double y[9]
 *                int iy0
 * Return Type  : void
 */
void c_xaxpy(int n, double a, const double x[3], int ix0, double y[9], int iy0)
{
  int ix;
  int iy;
  int i51;
  int k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i51 = n - 1;
    for (k = 0; k <= i51; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * Arguments    : int n
 *                double a
 *                int ix0
 *                double y[9]
 *                int iy0
 * Return Type  : void
 */
void xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
  int ix;
  int iy;
  int i49;
  int k;
  if (!(a == 0.0)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    i49 = n - 1;
    for (k = 0; k <= i49; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/*
 * File trailer for xaxpy.c
 *
 * [EOF]
 */
