/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rotm2axang.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <math.h>
#include "IK_solver/rt_defines.h"
#include "IK_solver/rt_nonfinite.h"
#include <string.h>
#include "IK_solver/IK_solver.h"
#include "IK_solver/rotm2axang.h"
#include "IK_solver/normalizeRows.h"
#include "IK_solver/svd1.h"
#include "IK_solver/complexTimes.h"
#include "IK_solver/sqrt1.h"

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : const double R[9]
 *                double axang[4]
 * Return Type  : void
 */
void rotm2axang(const double R[9], double axang[4])
{
  boolean_T b2;
  creal_T u;
  creal_T v;
  double ci;
  double b_v[3];
  double q;
  boolean_T rEQ0;
  boolean_T b3;
  int k;
  boolean_T exitg1;
  double vspecial_data[3];
  int i39;
  int i;
  double b_I[9];
  boolean_T p;
  double U[9];
  double V1[9];
  double dv3[9];
  b2 = false;
  u.re = 0.5 * (((R[0] + R[4]) + R[8]) - 1.0);
  if (!(fabs(u.re) > 1.0)) {
    u.re = acos(u.re);
  } else {
    v.re = 1.0 + u.re;
    v.im = 0.0;
    c_sqrt(&v);
    u.re = 1.0 - u.re;
    u.im = 0.0;
    c_sqrt(&u);
    if ((-v.im == 0.0) && (u.im == 0.0)) {
    } else {
      ci = v.re * u.im + -v.im * u.re;
      if ((rtIsInf(ci) || rtIsNaN(ci)) && (!rtIsNaN(v.re)) && (!rtIsNaN(-v.im)) &&
          (!rtIsNaN(u.re)) && (!rtIsNaN(u.im))) {
        ci = v.re;
        q = -v.im;
        rescale(&ci, &q);
        ci = u.re;
        q = u.im;
        rescale(&ci, &q);
      }
    }

    u.re = 2.0 * rt_atan2d_snf(u.re, v.re);
  }

  ci = 2.0 * sin(u.re);
  b_v[0] = R[5] - R[7];
  b_v[1] = R[6] - R[2];
  b_v[2] = R[1] - R[3];
  b_v[0] /= ci;
  b_v[1] /= ci;
  b_v[2] /= ci;
  if ((!rtIsInf(u.re)) && (!rtIsNaN(u.re))) {
    if (u.re == 0.0) {
      ci = 0.0;
    } else {
      ci = fmod(u.re, 3.1415926535897931);
      rEQ0 = (ci == 0.0);
      if (!rEQ0) {
        q = fabs(u.re / 3.1415926535897931);
        rEQ0 = (fabs(q - floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        ci = 0.0;
      } else {
        if (u.re < 0.0) {
          ci += 3.1415926535897931;
        }
      }
    }
  } else {
    ci = rtNaN;
  }

  rEQ0 = (ci == 0.0);
  b3 = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!(b_v[k] == 0.0)) {
      b3 = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (rEQ0 || b3) {
    k = 3 * (rEQ0 || b3);
    if (0 <= k - 1) {
      memset(&vspecial_data[0], 0, (unsigned int)(k * (int)sizeof(double)));
    }

    i39 = (rEQ0 || b3);
    for (i = 0; i < i39; i++) {
      memset(&b_I[0], 0, 9U * sizeof(double));
      b_I[0] = 1.0;
      b_I[4] = 1.0;
      b_I[8] = 1.0;
      p = true;
      for (k = 0; k < 9; k++) {
        ci = b_I[k] - R[k];
        if (p && ((!rtIsInf(ci)) && (!rtIsNaN(ci)))) {
          p = true;
        } else {
          p = false;
        }

        b_I[k] = ci;
      }

      if (p) {
        svd(b_I, U, vspecial_data, V1);
      } else {
        if (!b2) {
          memset(&dv3[0], 0, 9U * sizeof(double));
          b2 = true;
        }

        svd(dv3, b_I, vspecial_data, V1);
        for (k = 0; k < 9; k++) {
          V1[k] = rtNaN;
        }
      }

      vspecial_data[0] = V1[6];
      vspecial_data[1] = V1[7];
      vspecial_data[2] = V1[8];
    }

    k = 0;
    if (rEQ0 || b3) {
      k = 1;
    }

    for (i39 = 0; i39 < k; i39++) {
      b_v[0] = vspecial_data[0];
      b_v[1] = vspecial_data[1];
      b_v[2] = vspecial_data[2];
    }
  }

  vspecial_data[0] = b_v[0];
  vspecial_data[1] = b_v[1];
  vspecial_data[2] = b_v[2];
  normalizeRows(vspecial_data, b_v);
  axang[0] = b_v[0];
  axang[1] = b_v[1];
  axang[2] = b_v[2];
  axang[3] = u.re;
}

/*
 * File trailer for rotm2axang.c
 *
 * [EOF]
 */
