/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: DampedBFGSwGradientProjection.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include <math.h>
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/DampedBFGSwGradientProjection.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/IKHelpers.h"
#include "IK_solver/mldivide.h"
#include "IK_solver/eye.h"
#include "IK_solver/any.h"
#include "IK_solver/isPositiveDefinite.h"
#include "IK_solver/sqrt.h"
#include "IK_solver/rdivide_helper.h"
#include "IK_solver/norm.h"
#include "IK_solver/diag.h"
#include "IK_solver/inv.h"
#include "IK_solver/all1.h"
#include "IK_solver/SystemTimeProvider.h"

/* Function Declarations */
static boolean_T d_DampedBFGSwGradientProjection(const
  c_robotics_core_internal_Damped *obj, const emxArray_real_T *Hg, const
  emxArray_real_T *alpha);
static boolean_T e_DampedBFGSwGradientProjection(const
  c_robotics_core_internal_Damped *obj, const emxArray_real_T *xNew);

/* Function Definitions */

/*
 * Arguments    : const c_robotics_core_internal_Damped *obj
 *                const emxArray_real_T *Hg
 *                const emxArray_real_T *alpha
 * Return Type  : boolean_T
 */
static boolean_T d_DampedBFGSwGradientProjection(const
  c_robotics_core_internal_Damped *obj, const emxArray_real_T *Hg, const
  emxArray_real_T *alpha)
{
  boolean_T flag;
  emxArray_boolean_T *b_alpha;
  int i41;
  int loop_ub;
  emxInit_boolean_T(&b_alpha, 1);
  if (c_norm(Hg) < obj->GradientTolerance) {
    i41 = b_alpha->size[0];
    b_alpha->size[0] = alpha->size[0];
    emxEnsureCapacity_boolean_T(b_alpha, i41);
    loop_ub = alpha->size[0];
    for (i41 = 0; i41 < loop_ub; i41++) {
      b_alpha->data[i41] = (alpha->data[i41] <= 0.0);
    }

    if (all(b_alpha)) {
      flag = true;
    } else {
      flag = false;
    }
  } else {
    flag = false;
  }

  emxFree_boolean_T(&b_alpha);
  return flag;
}

/*
 * Arguments    : const c_robotics_core_internal_Damped *obj
 *                const emxArray_real_T *xNew
 * Return Type  : boolean_T
 */
static boolean_T e_DampedBFGSwGradientProjection(const
  c_robotics_core_internal_Damped *obj, const emxArray_real_T *xNew)
{
  boolean_T flag;
  emxArray_real_T *r21;
  emxArray_real_T *a;
  emxArray_boolean_T *r22;
  int i46;
  int loop_ub;
  int b_loop_ub;
  int m;
  int aoffset;
  int inner;
  emxInit_real_T(&r21, 1);
  emxInit_real_T(&a, 2);
  emxInit_boolean_T(&r22, 1);
  if (obj->ConstraintsOn) {
    i46 = a->size[0] * a->size[1];
    a->size[0] = obj->ConstraintMatrix->size[1];
    a->size[1] = obj->ConstraintMatrix->size[0];
    emxEnsureCapacity_real_T(a, i46);
    loop_ub = obj->ConstraintMatrix->size[0];
    for (i46 = 0; i46 < loop_ub; i46++) {
      b_loop_ub = obj->ConstraintMatrix->size[1];
      for (aoffset = 0; aoffset < b_loop_ub; aoffset++) {
        a->data[aoffset + a->size[0] * i46] = obj->ConstraintMatrix->data[i46 +
          obj->ConstraintMatrix->size[0] * aoffset];
      }
    }

    if ((a->size[1] == 1) || (xNew->size[0] == 1)) {
      i46 = r21->size[0];
      r21->size[0] = a->size[0];
      emxEnsureCapacity_real_T(r21, i46);
      loop_ub = a->size[0];
      for (i46 = 0; i46 < loop_ub; i46++) {
        r21->data[i46] = 0.0;
        b_loop_ub = a->size[1];
        for (aoffset = 0; aoffset < b_loop_ub; aoffset++) {
          r21->data[i46] += a->data[i46 + a->size[0] * aoffset] * xNew->
            data[aoffset];
        }
      }
    } else {
      m = a->size[0];
      inner = a->size[1];
      i46 = r21->size[0];
      r21->size[0] = a->size[0];
      emxEnsureCapacity_real_T(r21, i46);
      for (loop_ub = 0; loop_ub < m; loop_ub++) {
        r21->data[loop_ub] = 0.0;
      }

      for (b_loop_ub = 0; b_loop_ub < inner; b_loop_ub++) {
        aoffset = b_loop_ub * m;
        for (loop_ub = 0; loop_ub < m; loop_ub++) {
          r21->data[loop_ub] += xNew->data[b_loop_ub] * a->data[aoffset +
            loop_ub];
        }
      }
    }

    i46 = r22->size[0];
    r22->size[0] = r21->size[0];
    emxEnsureCapacity_boolean_T(r22, i46);
    loop_ub = r21->size[0];
    for (i46 = 0; i46 < loop_ub; i46++) {
      r22->data[i46] = (r21->data[i46] - obj->ConstraintBound->data[i46] >
                        1.4901161193847656E-8);
    }

    if (any(r22)) {
      flag = true;
    } else {
      flag = false;
    }
  } else {
    flag = false;
  }

  emxFree_boolean_T(&r22);
  emxFree_real_T(&a);
  emxFree_real_T(&r21);
  return flag;
}

/*
 * Arguments    : c_robotics_core_internal_Damped *obj
 *                emxArray_real_T *xSol
 *                c_robotics_core_internal_NLPSol *exitFlag
 *                double *err
 *                double *iter
 * Return Type  : void
 */
void c_DampedBFGSwGradientProjection(c_robotics_core_internal_Damped *obj,
  emxArray_real_T *xSol, c_robotics_core_internal_NLPSol *exitFlag, double *err,
  double *iter)
{
  emxArray_real_T *x;
  int i27;
  int coffset;
  emxArray_real_T *grad;
  emxArray_real_T *unusedU1;
  int n;
  double cost;
  double unusedU0[36];
  c_robotics_manip_internal_IKExt *r11;
  c_robotics_manip_internal_IKExt *args;
  emxArray_real_T *H;
  emxArray_boolean_T *activeSet;
  emxArray_real_T *A;
  emxArray_real_T *r12;
  emxArray_real_T *a;
  emxArray_int32_T *ii;
  int varargin_1_idx_0;
  int boffset;
  int m;
  int i;
  int inner;
  emxArray_real_T *b_a;
  emxArray_real_T *B;
  emxArray_real_T *r13;
  emxArray_real_T *y;
  int b_i;
  emxArray_real_T *c_a;
  emxArray_real_T *b_y;
  int k;
  int aoffset;
  int b_n;
  double d3;
  emxArray_real_T *Hg;
  emxArray_real_T *P;
  emxArray_uint32_T *inactiveConstraintIndices;
  emxArray_real_T *lambdas;
  emxArray_int32_T *r14;
  emxArray_int32_T *r15;
  emxArray_int32_T *r16;
  emxArray_int32_T *r17;
  emxArray_int32_T *r18;
  emxArray_boolean_T *b_x;
  emxArray_real_T *c_y;
  emxArray_real_T *b_tmp;
  emxArray_real_T *d_a;
  int nx;
  emxArray_boolean_T *b_activeSet;
  emxArray_real_T *b_A;
  boolean_T guard1 = false;
  int exitg2;
  double rho;
  boolean_T flag;
  double temp;
  boolean_T b_guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  int idxl;
  double lambda;
  double d_y;
  double b_gamma;
  double costNew;
  int idx;
  boolean_T exitg3;
  c_robotics_manip_internal_IKExt *r19;
  double b_m;
  int exitg1;
  c_robotics_manip_internal_IKExt *r20;
  emxInit_real_T(&x, 1);
  i27 = x->size[0];
  x->size[0] = obj->SeedInternal->size[0];
  emxEnsureCapacity_real_T(x, i27);
  coffset = obj->SeedInternal->size[0];
  for (i27 = 0; i27 < coffset; i27++) {
    x->data[i27] = obj->SeedInternal->data[i27];
  }

  emxInit_real_T(&grad, 1);
  emxInit_real_T(&unusedU1, 2);
  SystemTimeProvider_reset(&obj->TimeObjInternal);
  n = x->size[0];
  IKHelpers_computeCost(x, obj->ExtraArgs, &cost, unusedU0, unusedU1, &r11);
  args = r11;
  obj->ExtraArgs = args;
  args = obj->ExtraArgs;
  i27 = grad->size[0];
  grad->size[0] = args->GradTemp->size[0];
  emxEnsureCapacity_real_T(grad, i27);
  coffset = args->GradTemp->size[0];
  for (i27 = 0; i27 < coffset; i27++) {
    grad->data[i27] = args->GradTemp->data[i27];
  }

  emxInit_real_T(&H, 2);
  eye(x->size[0], H);
  emxInit_boolean_T(&activeSet, 1);
  emxInit_real_T(&A, 2);
  emxInit_real_T(&r12, 1);
  emxInit_real_T(&a, 2);
  emxInit_int32_T(&ii, 1);
  if (obj->ConstraintsOn) {
    i27 = a->size[0] * a->size[1];
    a->size[0] = obj->ConstraintMatrix->size[1];
    a->size[1] = obj->ConstraintMatrix->size[0];
    emxEnsureCapacity_real_T(a, i27);
    coffset = obj->ConstraintMatrix->size[0];
    for (i27 = 0; i27 < coffset; i27++) {
      boffset = obj->ConstraintMatrix->size[1];
      for (i = 0; i < boffset; i++) {
        a->data[i + a->size[0] * i27] = obj->ConstraintMatrix->data[i27 +
          obj->ConstraintMatrix->size[0] * i];
      }
    }

    if ((a->size[1] == 1) || (x->size[0] == 1)) {
      i27 = r12->size[0];
      r12->size[0] = a->size[0];
      emxEnsureCapacity_real_T(r12, i27);
      coffset = a->size[0];
      for (i27 = 0; i27 < coffset; i27++) {
        r12->data[i27] = 0.0;
        boffset = a->size[1];
        for (i = 0; i < boffset; i++) {
          r12->data[i27] += a->data[i27 + a->size[0] * i] * x->data[i];
        }
      }
    } else {
      m = a->size[0];
      inner = a->size[1];
      i27 = r12->size[0];
      r12->size[0] = a->size[0];
      emxEnsureCapacity_real_T(r12, i27);
      for (b_i = 0; b_i < m; b_i++) {
        r12->data[b_i] = 0.0;
      }

      for (k = 0; k < inner; k++) {
        aoffset = k * m;
        for (b_i = 0; b_i < m; b_i++) {
          r12->data[b_i] += x->data[k] * a->data[aoffset + b_i];
        }
      }
    }

    i27 = activeSet->size[0];
    activeSet->size[0] = r12->size[0];
    emxEnsureCapacity_boolean_T(activeSet, i27);
    coffset = r12->size[0];
    for (i27 = 0; i27 < coffset; i27++) {
      activeSet->data[i27] = (r12->data[i27] >= obj->ConstraintBound->data[i27]);
    }

    b_n = activeSet->size[0] - 1;
    varargin_1_idx_0 = 0;
    for (b_i = 0; b_i <= b_n; b_i++) {
      if (activeSet->data[b_i]) {
        varargin_1_idx_0++;
      }
    }

    i27 = ii->size[0];
    ii->size[0] = varargin_1_idx_0;
    emxEnsureCapacity_int32_T(ii, i27);
    varargin_1_idx_0 = 0;
    for (b_i = 0; b_i <= b_n; b_i++) {
      if (activeSet->data[b_i]) {
        ii->data[varargin_1_idx_0] = b_i + 1;
        varargin_1_idx_0++;
      }
    }

    coffset = obj->ConstraintMatrix->size[0];
    i27 = A->size[0] * A->size[1];
    A->size[0] = coffset;
    A->size[1] = ii->size[0];
    emxEnsureCapacity_real_T(A, i27);
    boffset = ii->size[0];
    for (i27 = 0; i27 < boffset; i27++) {
      for (i = 0; i < coffset; i++) {
        A->data[i + A->size[0] * i27] = obj->ConstraintMatrix->data[i +
          obj->ConstraintMatrix->size[0] * (ii->data[i27] - 1)];
      }
    }
  } else {
    varargin_1_idx_0 = obj->ConstraintBound->size[0];
    i27 = activeSet->size[0];
    activeSet->size[0] = varargin_1_idx_0;
    emxEnsureCapacity_boolean_T(activeSet, i27);
    for (i27 = 0; i27 < varargin_1_idx_0; i27++) {
      activeSet->data[i27] = false;
    }

    A->size[0] = x->size[0];
    A->size[1] = 0;
  }

  i27 = A->size[1];
  emxInit_real_T(&b_a, 1);
  emxInit_real_T(&B, 2);
  emxInit_real_T(&r13, 2);
  emxInit_real_T(&y, 2);
  emxInit_real_T(&c_a, 2);
  emxInit_real_T(&b_y, 2);
  for (k = 0; k < i27; k++) {
    coffset = A->size[0];
    i = b_a->size[0];
    b_a->size[0] = coffset;
    emxEnsureCapacity_real_T(b_a, i);
    for (i = 0; i < coffset; i++) {
      b_a->data[i] = A->data[i + A->size[0] * k];
    }

    i = c_a->size[0] * c_a->size[1];
    c_a->size[0] = 1;
    c_a->size[1] = b_a->size[0];
    emxEnsureCapacity_real_T(c_a, i);
    coffset = b_a->size[0];
    for (i = 0; i < coffset; i++) {
      c_a->data[i] = b_a->data[i];
    }

    if ((c_a->size[1] == 1) || (H->size[0] == 1)) {
      i = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = H->size[1];
      emxEnsureCapacity_real_T(y, i);
      coffset = H->size[1];
      for (i = 0; i < coffset; i++) {
        y->data[i] = 0.0;
        boffset = c_a->size[1];
        for (b_n = 0; b_n < boffset; b_n++) {
          y->data[i] += c_a->data[b_n] * H->data[b_n + H->size[0] * i];
        }
      }
    } else {
      inner = c_a->size[1];
      b_n = H->size[1];
      i = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = H->size[1];
      emxEnsureCapacity_real_T(y, i);
      for (nx = 0; nx < b_n; nx++) {
        boffset = nx * inner;
        y->data[nx] = 0.0;
        for (varargin_1_idx_0 = 0; varargin_1_idx_0 < inner; varargin_1_idx_0++)
        {
          y->data[nx] += H->data[boffset + varargin_1_idx_0] * c_a->
            data[varargin_1_idx_0];
        }
      }
    }

    guard1 = false;
    if (y->size[1] == 1) {
      guard1 = true;
    } else {
      i = A->size[0];
      if (i == 1) {
        guard1 = true;
      } else {
        rho = 0.0;
        coffset = y->size[1];
        for (i = 0; i < coffset; i++) {
          rho += y->data[i] * b_a->data[i];
        }
      }
    }

    if (guard1) {
      rho = 0.0;
      coffset = y->size[1];
      for (i = 0; i < coffset; i++) {
        rho += y->data[i] * b_a->data[i];
      }
    }

    temp = 1.0 / rho;
    i = b_y->size[0] * b_y->size[1];
    b_y->size[0] = H->size[0];
    b_y->size[1] = H->size[1];
    emxEnsureCapacity_real_T(b_y, i);
    coffset = H->size[0] * H->size[1];
    for (i = 0; i < coffset; i++) {
      b_y->data[i] = temp * H->data[i];
    }

    i = B->size[0] * B->size[1];
    B->size[0] = b_a->size[0];
    B->size[1] = b_a->size[0];
    emxEnsureCapacity_real_T(B, i);
    coffset = b_a->size[0];
    for (i = 0; i < coffset; i++) {
      boffset = b_a->size[0];
      for (b_n = 0; b_n < boffset; b_n++) {
        B->data[i + B->size[0] * b_n] = b_a->data[i] * b_a->data[b_n];
      }
    }

    if ((b_y->size[1] == 1) || (B->size[0] == 1)) {
      i = a->size[0] * a->size[1];
      a->size[0] = b_y->size[0];
      a->size[1] = B->size[1];
      emxEnsureCapacity_real_T(a, i);
      coffset = b_y->size[0];
      for (i = 0; i < coffset; i++) {
        boffset = B->size[1];
        for (b_n = 0; b_n < boffset; b_n++) {
          a->data[i + a->size[0] * b_n] = 0.0;
          nx = b_y->size[1];
          for (varargin_1_idx_0 = 0; varargin_1_idx_0 < nx; varargin_1_idx_0++)
          {
            a->data[i + a->size[0] * b_n] += b_y->data[i + b_y->size[0] *
              varargin_1_idx_0] * B->data[varargin_1_idx_0 + B->size[0] * b_n];
          }
        }
      }
    } else {
      m = b_y->size[0];
      inner = b_y->size[1];
      b_n = B->size[1];
      i = a->size[0] * a->size[1];
      a->size[0] = b_y->size[0];
      a->size[1] = B->size[1];
      emxEnsureCapacity_real_T(a, i);
      for (nx = 0; nx < b_n; nx++) {
        coffset = nx * m;
        boffset = nx * inner;
        for (b_i = 0; b_i < m; b_i++) {
          a->data[coffset + b_i] = 0.0;
        }

        for (varargin_1_idx_0 = 0; varargin_1_idx_0 < inner; varargin_1_idx_0++)
        {
          aoffset = varargin_1_idx_0 * m;
          temp = B->data[boffset + varargin_1_idx_0];
          for (b_i = 0; b_i < m; b_i++) {
            i = coffset + b_i;
            a->data[i] += temp * b_y->data[aoffset + b_i];
          }
        }
      }
    }

    if ((a->size[1] == 1) || (H->size[0] == 1)) {
      i = r13->size[0] * r13->size[1];
      r13->size[0] = a->size[0];
      r13->size[1] = H->size[1];
      emxEnsureCapacity_real_T(r13, i);
      coffset = a->size[0];
      for (i = 0; i < coffset; i++) {
        boffset = H->size[1];
        for (b_n = 0; b_n < boffset; b_n++) {
          r13->data[i + r13->size[0] * b_n] = 0.0;
          nx = a->size[1];
          for (varargin_1_idx_0 = 0; varargin_1_idx_0 < nx; varargin_1_idx_0++)
          {
            r13->data[i + r13->size[0] * b_n] += a->data[i + a->size[0] *
              varargin_1_idx_0] * H->data[varargin_1_idx_0 + H->size[0] * b_n];
          }
        }
      }
    } else {
      m = a->size[0];
      inner = a->size[1];
      b_n = H->size[1];
      i = r13->size[0] * r13->size[1];
      r13->size[0] = a->size[0];
      r13->size[1] = H->size[1];
      emxEnsureCapacity_real_T(r13, i);
      for (nx = 0; nx < b_n; nx++) {
        coffset = nx * m;
        boffset = nx * inner;
        for (b_i = 0; b_i < m; b_i++) {
          r13->data[coffset + b_i] = 0.0;
        }

        for (varargin_1_idx_0 = 0; varargin_1_idx_0 < inner; varargin_1_idx_0++)
        {
          aoffset = varargin_1_idx_0 * m;
          temp = H->data[boffset + varargin_1_idx_0];
          for (b_i = 0; b_i < m; b_i++) {
            i = coffset + b_i;
            r13->data[i] += temp * a->data[aoffset + b_i];
          }
        }
      }
    }

    i = H->size[0] * H->size[1];
    b_n = H->size[0] * H->size[1];
    emxEnsureCapacity_real_T(H, b_n);
    coffset = i - 1;
    for (i = 0; i <= coffset; i++) {
      H->data[i] -= r13->data[i];
    }
  }

  i27 = xSol->size[0];
  xSol->size[0] = x->size[0];
  emxEnsureCapacity_real_T(xSol, i27);
  coffset = x->size[0];
  for (i27 = 0; i27 < coffset; i27++) {
    xSol->data[i27] = x->data[i27];
  }

  d3 = obj->MaxNumIterationInternal;
  b_i = 0;
  emxInit_real_T(&Hg, 1);
  emxInit_real_T(&P, 2);
  emxInit_uint32_T(&inactiveConstraintIndices, 1);
  emxInit_real_T(&lambdas, 1);
  emxInit_int32_T(&r14, 1);
  emxInit_int32_T(&r15, 1);
  emxInit_int32_T(&r16, 1);
  emxInit_int32_T(&r17, 1);
  emxInit_int32_T(&r18, 1);
  emxInit_boolean_T(&b_x, 1);
  emxInit_real_T(&c_y, 1);
  emxInit_real_T(&b_tmp, 2);
  emxInit_real_T(&d_a, 1);
  emxInit_boolean_T(&b_activeSet, 1);
  emxInit_real_T(&b_A, 2);
  do {
    exitg2 = 0;
    if (b_i <= (int)d3 - 1) {
      rho = c_SystemTimeProvider_getElapsed(&obj->TimeObjInternal);
      flag = (rho > obj->MaxTimeInternal);
      if (flag) {
        *exitFlag = TimeLimitExceeded;
        *err = IKHelpers_evaluateSolution(obj->ExtraArgs);
        *iter = 1.0 + (double)b_i;
        exitg2 = 1;
      } else {
        if ((A->size[0] == 0) || (A->size[1] == 0)) {
          i27 = b_a->size[0];
          b_a->size[0] = 1;
          emxEnsureCapacity_real_T(b_a, i27);
          b_a->data[0] = 0.0;
        } else {
          i27 = a->size[0] * a->size[1];
          a->size[0] = A->size[1];
          a->size[1] = A->size[0];
          emxEnsureCapacity_real_T(a, i27);
          coffset = A->size[0];
          for (i27 = 0; i27 < coffset; i27++) {
            boffset = A->size[1];
            for (i = 0; i < boffset; i++) {
              a->data[i + a->size[0] * i27] = A->data[i27 + A->size[0] * i];
            }
          }

          if ((a->size[1] == 1) || (A->size[0] == 1)) {
            i27 = b_y->size[0] * b_y->size[1];
            b_y->size[0] = a->size[0];
            b_y->size[1] = A->size[1];
            emxEnsureCapacity_real_T(b_y, i27);
            coffset = a->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              boffset = A->size[1];
              for (i = 0; i < boffset; i++) {
                b_y->data[i27 + b_y->size[0] * i] = 0.0;
                nx = a->size[1];
                for (b_n = 0; b_n < nx; b_n++) {
                  b_y->data[i27 + b_y->size[0] * i] += a->data[i27 + a->size[0] *
                    b_n] * A->data[b_n + A->size[0] * i];
                }
              }
            }
          } else {
            m = a->size[0];
            inner = a->size[1];
            b_n = A->size[1];
            i27 = b_y->size[0] * b_y->size[1];
            b_y->size[0] = a->size[0];
            b_y->size[1] = A->size[1];
            emxEnsureCapacity_real_T(b_y, i27);
            for (nx = 0; nx < b_n; nx++) {
              coffset = nx * m;
              boffset = nx * inner;
              for (i = 0; i < m; i++) {
                b_y->data[coffset + i] = 0.0;
              }

              for (k = 0; k < inner; k++) {
                aoffset = k * m;
                temp = A->data[boffset + k];
                for (i = 0; i < m; i++) {
                  i27 = coffset + i;
                  b_y->data[i27] += temp * a->data[aoffset + i];
                }
              }
            }
          }

          i27 = b_A->size[0] * b_A->size[1];
          b_A->size[0] = A->size[1];
          b_A->size[1] = A->size[0];
          emxEnsureCapacity_real_T(b_A, i27);
          coffset = A->size[0];
          for (i27 = 0; i27 < coffset; i27++) {
            boffset = A->size[1];
            for (i = 0; i < boffset; i++) {
              b_A->data[i + b_A->size[0] * i27] = A->data[i27 + A->size[0] * i];
            }
          }

          mldivide(b_y, b_A, a);
          if ((a->size[1] == 1) || (grad->size[0] == 1)) {
            i27 = b_a->size[0];
            b_a->size[0] = a->size[0];
            emxEnsureCapacity_real_T(b_a, i27);
            coffset = a->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              b_a->data[i27] = 0.0;
              boffset = a->size[1];
              for (i = 0; i < boffset; i++) {
                b_a->data[i27] += a->data[i27 + a->size[0] * i] * grad->data[i];
              }
            }
          } else {
            m = a->size[0];
            inner = a->size[1];
            i27 = b_a->size[0];
            b_a->size[0] = a->size[0];
            emxEnsureCapacity_real_T(b_a, i27);
            for (i = 0; i < m; i++) {
              b_a->data[i] = 0.0;
            }

            for (k = 0; k < inner; k++) {
              aoffset = k * m;
              for (i = 0; i < m; i++) {
                b_a->data[i] += grad->data[k] * a->data[aoffset + i];
              }
            }
          }
        }

        if ((H->size[1] == 1) || (grad->size[0] == 1)) {
          i27 = Hg->size[0];
          Hg->size[0] = H->size[0];
          emxEnsureCapacity_real_T(Hg, i27);
          coffset = H->size[0];
          for (i27 = 0; i27 < coffset; i27++) {
            Hg->data[i27] = 0.0;
            boffset = H->size[1];
            for (i = 0; i < boffset; i++) {
              Hg->data[i27] += H->data[i27 + H->size[0] * i] * grad->data[i];
            }
          }
        } else {
          m = H->size[0];
          inner = H->size[1];
          i27 = Hg->size[0];
          Hg->size[0] = H->size[0];
          emxEnsureCapacity_real_T(Hg, i27);
          for (i = 0; i < m; i++) {
            Hg->data[i] = 0.0;
          }

          for (k = 0; k < inner; k++) {
            aoffset = k * m;
            for (i = 0; i < m; i++) {
              Hg->data[i] += grad->data[k] * H->data[aoffset + i];
            }
          }
        }

        if (d_DampedBFGSwGradientProjection(obj, Hg, b_a)) {
          *exitFlag = LocalMinimumFound;
          *err = IKHelpers_evaluateSolution(obj->ExtraArgs);
          *iter = 1.0 + (double)b_i;
          exitg2 = 1;
        } else {
          b_guard1 = false;
          guard2 = false;
          guard3 = false;
          if (obj->ConstraintsOn && ((A->size[0] != 0) && (A->size[1] != 0))) {
            i27 = a->size[0] * a->size[1];
            a->size[0] = A->size[1];
            a->size[1] = A->size[0];
            emxEnsureCapacity_real_T(a, i27);
            coffset = A->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              boffset = A->size[1];
              for (i = 0; i < boffset; i++) {
                a->data[i + a->size[0] * i27] = A->data[i27 + A->size[0] * i];
              }
            }

            if ((a->size[1] == 1) || (A->size[0] == 1)) {
              i27 = b_y->size[0] * b_y->size[1];
              b_y->size[0] = a->size[0];
              b_y->size[1] = A->size[1];
              emxEnsureCapacity_real_T(b_y, i27);
              coffset = a->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                boffset = A->size[1];
                for (i = 0; i < boffset; i++) {
                  b_y->data[i27 + b_y->size[0] * i] = 0.0;
                  nx = a->size[1];
                  for (b_n = 0; b_n < nx; b_n++) {
                    b_y->data[i27 + b_y->size[0] * i] += a->data[i27 + a->size[0]
                      * b_n] * A->data[b_n + A->size[0] * i];
                  }
                }
              }
            } else {
              m = a->size[0];
              inner = a->size[1];
              b_n = A->size[1];
              i27 = b_y->size[0] * b_y->size[1];
              b_y->size[0] = a->size[0];
              b_y->size[1] = A->size[1];
              emxEnsureCapacity_real_T(b_y, i27);
              for (nx = 0; nx < b_n; nx++) {
                coffset = nx * m;
                boffset = nx * inner;
                for (i = 0; i < m; i++) {
                  b_y->data[coffset + i] = 0.0;
                }

                for (k = 0; k < inner; k++) {
                  aoffset = k * m;
                  temp = A->data[boffset + k];
                  for (i = 0; i < m; i++) {
                    i27 = coffset + i;
                    b_y->data[i27] += temp * a->data[aoffset + i];
                  }
                }
              }
            }

            inv(b_y, B);
            diag(B, r12);
            d_sqrt(r12);
            rdivide_helper(b_a, r12, lambdas);
            b_n = lambdas->size[0];
            if (lambdas->size[0] <= 2) {
              if (lambdas->size[0] == 1) {
                rho = lambdas->data[0];
                idx = 1;
              } else if ((lambdas->data[0] < lambdas->data[1]) || (rtIsNaN
                          (lambdas->data[0]) && (!rtIsNaN(lambdas->data[1])))) {
                rho = lambdas->data[1];
                idx = 2;
              } else {
                rho = lambdas->data[0];
                idx = 1;
              }
            } else {
              if (!rtIsNaN(lambdas->data[0])) {
                idx = 1;
              } else {
                idx = 0;
                k = 2;
                exitg3 = false;
                while ((!exitg3) && (k <= lambdas->size[0])) {
                  if (!rtIsNaN(lambdas->data[k - 1])) {
                    idx = k;
                    exitg3 = true;
                  } else {
                    k++;
                  }
                }
              }

              if (idx == 0) {
                rho = lambdas->data[0];
                idx = 1;
              } else {
                rho = lambdas->data[idx - 1];
                i27 = idx + 1;
                for (k = i27; k <= b_n; k++) {
                  if (rho < lambdas->data[k - 1]) {
                    rho = lambdas->data[k - 1];
                    idx = k;
                  }
                }
              }
            }

            if (c_norm(Hg) < 0.5 * rho) {
              nx = activeSet->size[0];
              b_n = 0;
              i27 = ii->size[0];
              ii->size[0] = activeSet->size[0];
              emxEnsureCapacity_int32_T(ii, i27);
              varargin_1_idx_0 = 0;
              exitg3 = false;
              while ((!exitg3) && (varargin_1_idx_0 <= nx - 1)) {
                if (activeSet->data[varargin_1_idx_0]) {
                  b_n++;
                  ii->data[b_n - 1] = varargin_1_idx_0 + 1;
                  if (b_n >= nx) {
                    exitg3 = true;
                  } else {
                    varargin_1_idx_0++;
                  }
                } else {
                  varargin_1_idx_0++;
                }
              }

              if (activeSet->size[0] == 1) {
                if (b_n == 0) {
                  ii->size[0] = 0;
                }
              } else if (1 > b_n) {
                ii->size[0] = 0;
              } else {
                i27 = ii->size[0];
                ii->size[0] = b_n;
                emxEnsureCapacity_int32_T(ii, i27);
              }

              i27 = b_a->size[0];
              b_a->size[0] = ii->size[0];
              emxEnsureCapacity_real_T(b_a, i27);
              coffset = ii->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                b_a->data[i27] = ii->data[i27];
              }

              activeSet->data[(int)b_a->data[idx - 1] - 1] = false;
              b_n = activeSet->size[0] - 1;
              varargin_1_idx_0 = 0;
              for (i = 0; i <= b_n; i++) {
                if (activeSet->data[i]) {
                  varargin_1_idx_0++;
                }
              }

              i27 = r16->size[0];
              r16->size[0] = varargin_1_idx_0;
              emxEnsureCapacity_int32_T(r16, i27);
              varargin_1_idx_0 = 0;
              for (i = 0; i <= b_n; i++) {
                if (activeSet->data[i]) {
                  r16->data[varargin_1_idx_0] = i + 1;
                  varargin_1_idx_0++;
                }
              }

              coffset = obj->ConstraintMatrix->size[0];
              i27 = A->size[0] * A->size[1];
              A->size[0] = coffset;
              A->size[1] = r16->size[0];
              emxEnsureCapacity_real_T(A, i27);
              boffset = r16->size[0];
              for (i27 = 0; i27 < boffset; i27++) {
                for (i = 0; i < coffset; i++) {
                  A->data[i + A->size[0] * i27] = obj->ConstraintMatrix->data[i
                    + obj->ConstraintMatrix->size[0] * (r16->data[i27] - 1)];
                }
              }

              eye(n, P);
              i27 = a->size[0] * a->size[1];
              a->size[0] = A->size[1];
              a->size[1] = A->size[0];
              emxEnsureCapacity_real_T(a, i27);
              coffset = A->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                boffset = A->size[1];
                for (i = 0; i < boffset; i++) {
                  a->data[i + a->size[0] * i27] = A->data[i27 + A->size[0] * i];
                }
              }

              if ((a->size[1] == 1) || (A->size[0] == 1)) {
                i27 = b_y->size[0] * b_y->size[1];
                b_y->size[0] = a->size[0];
                b_y->size[1] = A->size[1];
                emxEnsureCapacity_real_T(b_y, i27);
                coffset = a->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  boffset = A->size[1];
                  for (i = 0; i < boffset; i++) {
                    b_y->data[i27 + b_y->size[0] * i] = 0.0;
                    nx = a->size[1];
                    for (b_n = 0; b_n < nx; b_n++) {
                      b_y->data[i27 + b_y->size[0] * i] += a->data[i27 + a->
                        size[0] * b_n] * A->data[b_n + A->size[0] * i];
                    }
                  }
                }
              } else {
                m = a->size[0];
                inner = a->size[1];
                b_n = A->size[1];
                i27 = b_y->size[0] * b_y->size[1];
                b_y->size[0] = a->size[0];
                b_y->size[1] = A->size[1];
                emxEnsureCapacity_real_T(b_y, i27);
                for (nx = 0; nx < b_n; nx++) {
                  coffset = nx * m;
                  boffset = nx * inner;
                  for (i = 0; i < m; i++) {
                    b_y->data[coffset + i] = 0.0;
                  }

                  for (k = 0; k < inner; k++) {
                    aoffset = k * m;
                    temp = A->data[boffset + k];
                    for (i = 0; i < m; i++) {
                      i27 = coffset + i;
                      b_y->data[i27] += temp * a->data[aoffset + i];
                    }
                  }
                }
              }

              i27 = b_A->size[0] * b_A->size[1];
              b_A->size[0] = A->size[1];
              b_A->size[1] = A->size[0];
              emxEnsureCapacity_real_T(b_A, i27);
              coffset = A->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                boffset = A->size[1];
                for (i = 0; i < boffset; i++) {
                  b_A->data[i + b_A->size[0] * i27] = A->data[i27 + A->size[0] *
                    i];
                }
              }

              mldivide(b_y, b_A, a);
              if ((A->size[1] == 1) || (a->size[0] == 1)) {
                i27 = r13->size[0] * r13->size[1];
                r13->size[0] = A->size[0];
                r13->size[1] = a->size[1];
                emxEnsureCapacity_real_T(r13, i27);
                coffset = A->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  boffset = a->size[1];
                  for (i = 0; i < boffset; i++) {
                    r13->data[i27 + r13->size[0] * i] = 0.0;
                    nx = A->size[1];
                    for (b_n = 0; b_n < nx; b_n++) {
                      r13->data[i27 + r13->size[0] * i] += A->data[i27 + A->
                        size[0] * b_n] * a->data[b_n + a->size[0] * i];
                    }
                  }
                }
              } else {
                m = A->size[0];
                inner = A->size[1];
                b_n = a->size[1];
                i27 = r13->size[0] * r13->size[1];
                r13->size[0] = A->size[0];
                r13->size[1] = a->size[1];
                emxEnsureCapacity_real_T(r13, i27);
                for (nx = 0; nx < b_n; nx++) {
                  coffset = nx * m;
                  boffset = nx * inner;
                  for (i = 0; i < m; i++) {
                    r13->data[coffset + i] = 0.0;
                  }

                  for (k = 0; k < inner; k++) {
                    aoffset = k * m;
                    temp = a->data[boffset + k];
                    for (i = 0; i < m; i++) {
                      i27 = coffset + i;
                      r13->data[i27] += temp * A->data[aoffset + i];
                    }
                  }
                }
              }

              i27 = P->size[0] * P->size[1];
              i = P->size[0] * P->size[1];
              emxEnsureCapacity_real_T(P, i);
              coffset = i27 - 1;
              for (i27 = 0; i27 <= coffset; i27++) {
                P->data[i27] -= r13->data[i27];
              }

              coffset = obj->ConstraintMatrix->size[0];
              varargin_1_idx_0 = (int)b_a->data[idx - 1];
              i27 = lambdas->size[0];
              lambdas->size[0] = coffset;
              emxEnsureCapacity_real_T(lambdas, i27);
              for (i27 = 0; i27 < coffset; i27++) {
                lambdas->data[i27] = obj->ConstraintMatrix->data[i27 +
                  obj->ConstraintMatrix->size[0] * (varargin_1_idx_0 - 1)];
              }

              i27 = c_a->size[0] * c_a->size[1];
              c_a->size[0] = 1;
              c_a->size[1] = lambdas->size[0];
              emxEnsureCapacity_real_T(c_a, i27);
              coffset = lambdas->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                c_a->data[i27] = lambdas->data[i27];
              }

              if ((c_a->size[1] == 1) || (P->size[0] == 1)) {
                i27 = y->size[0] * y->size[1];
                y->size[0] = 1;
                y->size[1] = P->size[1];
                emxEnsureCapacity_real_T(y, i27);
                coffset = P->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  y->data[i27] = 0.0;
                  boffset = c_a->size[1];
                  for (i = 0; i < boffset; i++) {
                    y->data[i27] += c_a->data[i] * P->data[i + P->size[0] * i27];
                  }
                }
              } else {
                inner = c_a->size[1];
                b_n = P->size[1];
                i27 = y->size[0] * y->size[1];
                y->size[0] = 1;
                y->size[1] = P->size[1];
                emxEnsureCapacity_real_T(y, i27);
                for (nx = 0; nx < b_n; nx++) {
                  boffset = nx * inner;
                  y->data[nx] = 0.0;
                  for (k = 0; k < inner; k++) {
                    y->data[nx] += P->data[boffset + k] * c_a->data[k];
                  }
                }
              }

              if ((y->size[1] == 1) || (lambdas->size[0] == 1)) {
                d_y = 0.0;
                coffset = y->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  d_y += y->data[i27] * lambdas->data[i27];
                }
              } else {
                d_y = 0.0;
                coffset = y->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  d_y += y->data[i27] * lambdas->data[i27];
                }
              }

              temp = 1.0 / d_y;
              i27 = b_y->size[0] * b_y->size[1];
              b_y->size[0] = P->size[0];
              b_y->size[1] = P->size[1];
              emxEnsureCapacity_real_T(b_y, i27);
              coffset = P->size[0] * P->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                b_y->data[i27] = temp * P->data[i27];
              }

              i27 = B->size[0] * B->size[1];
              B->size[0] = lambdas->size[0];
              B->size[1] = lambdas->size[0];
              emxEnsureCapacity_real_T(B, i27);
              coffset = lambdas->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                boffset = lambdas->size[0];
                for (i = 0; i < boffset; i++) {
                  B->data[i27 + B->size[0] * i] = lambdas->data[i27] *
                    lambdas->data[i];
                }
              }

              if ((b_y->size[1] == 1) || (B->size[0] == 1)) {
                i27 = a->size[0] * a->size[1];
                a->size[0] = b_y->size[0];
                a->size[1] = B->size[1];
                emxEnsureCapacity_real_T(a, i27);
                coffset = b_y->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  boffset = B->size[1];
                  for (i = 0; i < boffset; i++) {
                    a->data[i27 + a->size[0] * i] = 0.0;
                    nx = b_y->size[1];
                    for (b_n = 0; b_n < nx; b_n++) {
                      a->data[i27 + a->size[0] * i] += b_y->data[i27 + b_y->
                        size[0] * b_n] * B->data[b_n + B->size[0] * i];
                    }
                  }
                }
              } else {
                m = b_y->size[0];
                inner = b_y->size[1];
                b_n = B->size[1];
                i27 = a->size[0] * a->size[1];
                a->size[0] = b_y->size[0];
                a->size[1] = B->size[1];
                emxEnsureCapacity_real_T(a, i27);
                for (nx = 0; nx < b_n; nx++) {
                  coffset = nx * m;
                  boffset = nx * inner;
                  for (i = 0; i < m; i++) {
                    a->data[coffset + i] = 0.0;
                  }

                  for (k = 0; k < inner; k++) {
                    aoffset = k * m;
                    temp = B->data[boffset + k];
                    for (i = 0; i < m; i++) {
                      i27 = coffset + i;
                      a->data[i27] += temp * b_y->data[aoffset + i];
                    }
                  }
                }
              }

              if ((a->size[1] == 1) || (P->size[0] == 1)) {
                i27 = r13->size[0] * r13->size[1];
                r13->size[0] = a->size[0];
                r13->size[1] = P->size[1];
                emxEnsureCapacity_real_T(r13, i27);
                coffset = a->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  boffset = P->size[1];
                  for (i = 0; i < boffset; i++) {
                    r13->data[i27 + r13->size[0] * i] = 0.0;
                    nx = a->size[1];
                    for (b_n = 0; b_n < nx; b_n++) {
                      r13->data[i27 + r13->size[0] * i] += a->data[i27 + a->
                        size[0] * b_n] * P->data[b_n + P->size[0] * i];
                    }
                  }
                }
              } else {
                m = a->size[0];
                inner = a->size[1];
                b_n = P->size[1];
                i27 = r13->size[0] * r13->size[1];
                r13->size[0] = a->size[0];
                r13->size[1] = P->size[1];
                emxEnsureCapacity_real_T(r13, i27);
                for (nx = 0; nx < b_n; nx++) {
                  coffset = nx * m;
                  boffset = nx * inner;
                  for (i = 0; i < m; i++) {
                    r13->data[coffset + i] = 0.0;
                  }

                  for (k = 0; k < inner; k++) {
                    aoffset = k * m;
                    temp = P->data[boffset + k];
                    for (i = 0; i < m; i++) {
                      i27 = coffset + i;
                      r13->data[i27] += temp * a->data[aoffset + i];
                    }
                  }
                }
              }

              i27 = H->size[0] * H->size[1];
              i = H->size[0] * H->size[1];
              emxEnsureCapacity_real_T(H, i);
              coffset = i27 - 1;
              for (i27 = 0; i27 <= coffset; i27++) {
                H->data[i27] += r13->data[i27];
              }

              b_i++;
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }

          if (guard3) {
            i27 = Hg->size[0];
            emxEnsureCapacity_real_T(Hg, i27);
            coffset = Hg->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              Hg->data[i27] = -Hg->data[i27];
            }

            idxl = -2;
            if (obj->ConstraintsOn) {
              i27 = b_activeSet->size[0];
              b_activeSet->size[0] = activeSet->size[0];
              emxEnsureCapacity_boolean_T(b_activeSet, i27);
              coffset = activeSet->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                b_activeSet->data[i27] = !activeSet->data[i27];
              }

              if (any(b_activeSet)) {
                b_n = activeSet->size[0] - 1;
                varargin_1_idx_0 = 0;
                for (i = 0; i <= b_n; i++) {
                  if (!activeSet->data[i]) {
                    varargin_1_idx_0++;
                  }
                }

                i27 = r14->size[0];
                r14->size[0] = varargin_1_idx_0;
                emxEnsureCapacity_int32_T(r14, i27);
                varargin_1_idx_0 = 0;
                for (i = 0; i <= b_n; i++) {
                  if (!activeSet->data[i]) {
                    r14->data[varargin_1_idx_0] = i + 1;
                    varargin_1_idx_0++;
                  }
                }

                i27 = b_a->size[0];
                b_a->size[0] = r14->size[0];
                emxEnsureCapacity_real_T(b_a, i27);
                coffset = r14->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  b_a->data[i27] = obj->ConstraintBound->data[r14->data[i27] - 1];
                }

                b_n = activeSet->size[0] - 1;
                varargin_1_idx_0 = 0;
                for (i = 0; i <= b_n; i++) {
                  if (!activeSet->data[i]) {
                    varargin_1_idx_0++;
                  }
                }

                i27 = r15->size[0];
                r15->size[0] = varargin_1_idx_0;
                emxEnsureCapacity_int32_T(r15, i27);
                varargin_1_idx_0 = 0;
                for (i = 0; i <= b_n; i++) {
                  if (!activeSet->data[i]) {
                    r15->data[varargin_1_idx_0] = i + 1;
                    varargin_1_idx_0++;
                  }
                }

                coffset = obj->ConstraintMatrix->size[0];
                i27 = B->size[0] * B->size[1];
                B->size[0] = coffset;
                B->size[1] = r15->size[0];
                emxEnsureCapacity_real_T(B, i27);
                boffset = r15->size[0];
                for (i27 = 0; i27 < boffset; i27++) {
                  for (i = 0; i < coffset; i++) {
                    B->data[i + B->size[0] * i27] = obj->ConstraintMatrix->
                      data[i + obj->ConstraintMatrix->size[0] * (r15->data[i27]
                      - 1)];
                  }
                }

                i27 = b_x->size[0];
                b_x->size[0] = activeSet->size[0];
                emxEnsureCapacity_boolean_T(b_x, i27);
                coffset = activeSet->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  b_x->data[i27] = !activeSet->data[i27];
                }

                nx = b_x->size[0];
                idx = 0;
                i27 = ii->size[0];
                ii->size[0] = b_x->size[0];
                emxEnsureCapacity_int32_T(ii, i27);
                varargin_1_idx_0 = 0;
                exitg3 = false;
                while ((!exitg3) && (varargin_1_idx_0 <= nx - 1)) {
                  if (b_x->data[varargin_1_idx_0]) {
                    idx++;
                    ii->data[idx - 1] = varargin_1_idx_0 + 1;
                    if (idx >= nx) {
                      exitg3 = true;
                    } else {
                      varargin_1_idx_0++;
                    }
                  } else {
                    varargin_1_idx_0++;
                  }
                }

                if (b_x->size[0] == 1) {
                  if (idx == 0) {
                    ii->size[0] = 0;
                  }
                } else if (1 > idx) {
                  ii->size[0] = 0;
                } else {
                  i27 = ii->size[0];
                  ii->size[0] = idx;
                  emxEnsureCapacity_int32_T(ii, i27);
                }

                i27 = inactiveConstraintIndices->size[0];
                inactiveConstraintIndices->size[0] = ii->size[0];
                emxEnsureCapacity_uint32_T(inactiveConstraintIndices, i27);
                coffset = ii->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  inactiveConstraintIndices->data[i27] = (unsigned int)ii->
                    data[i27];
                }

                i27 = a->size[0] * a->size[1];
                a->size[0] = B->size[1];
                a->size[1] = B->size[0];
                emxEnsureCapacity_real_T(a, i27);
                coffset = B->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  boffset = B->size[1];
                  for (i = 0; i < boffset; i++) {
                    a->data[i + a->size[0] * i27] = B->data[i27 + B->size[0] * i];
                  }
                }

                if ((a->size[1] == 1) || (x->size[0] == 1)) {
                  i27 = r12->size[0];
                  r12->size[0] = a->size[0];
                  emxEnsureCapacity_real_T(r12, i27);
                  coffset = a->size[0];
                  for (i27 = 0; i27 < coffset; i27++) {
                    r12->data[i27] = 0.0;
                    boffset = a->size[1];
                    for (i = 0; i < boffset; i++) {
                      r12->data[i27] += a->data[i27 + a->size[0] * i] * x->
                        data[i];
                    }
                  }
                } else {
                  m = a->size[0];
                  inner = a->size[1];
                  i27 = r12->size[0];
                  r12->size[0] = a->size[0];
                  emxEnsureCapacity_real_T(r12, i27);
                  for (i = 0; i < m; i++) {
                    r12->data[i] = 0.0;
                  }

                  for (k = 0; k < inner; k++) {
                    aoffset = k * m;
                    for (i = 0; i < m; i++) {
                      r12->data[i] += x->data[k] * a->data[aoffset + i];
                    }
                  }
                }

                i27 = a->size[0] * a->size[1];
                a->size[0] = B->size[1];
                a->size[1] = B->size[0];
                emxEnsureCapacity_real_T(a, i27);
                coffset = B->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  boffset = B->size[1];
                  for (i = 0; i < boffset; i++) {
                    a->data[i + a->size[0] * i27] = B->data[i27 + B->size[0] * i];
                  }
                }

                if ((a->size[1] == 1) || (Hg->size[0] == 1)) {
                  i27 = c_y->size[0];
                  c_y->size[0] = a->size[0];
                  emxEnsureCapacity_real_T(c_y, i27);
                  coffset = a->size[0];
                  for (i27 = 0; i27 < coffset; i27++) {
                    c_y->data[i27] = 0.0;
                    boffset = a->size[1];
                    for (i = 0; i < boffset; i++) {
                      c_y->data[i27] += a->data[i27 + a->size[0] * i] * Hg->
                        data[i];
                    }
                  }
                } else {
                  m = a->size[0];
                  inner = a->size[1];
                  i27 = c_y->size[0];
                  c_y->size[0] = a->size[0];
                  emxEnsureCapacity_real_T(c_y, i27);
                  for (i = 0; i < m; i++) {
                    c_y->data[i] = 0.0;
                  }

                  for (k = 0; k < inner; k++) {
                    aoffset = k * m;
                    for (i = 0; i < m; i++) {
                      c_y->data[i] += Hg->data[k] * a->data[aoffset + i];
                    }
                  }
                }

                i27 = d_a->size[0];
                d_a->size[0] = b_a->size[0];
                emxEnsureCapacity_real_T(d_a, i27);
                coffset = b_a->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  d_a->data[i27] = b_a->data[i27] - r12->data[i27];
                }

                rdivide_helper(d_a, c_y, lambdas);
                i27 = b_x->size[0];
                b_x->size[0] = lambdas->size[0];
                emxEnsureCapacity_boolean_T(b_x, i27);
                coffset = lambdas->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  b_x->data[i27] = (lambdas->data[i27] > 0.0);
                }

                nx = b_x->size[0];
                idx = 0;
                i27 = ii->size[0];
                ii->size[0] = b_x->size[0];
                emxEnsureCapacity_int32_T(ii, i27);
                varargin_1_idx_0 = 0;
                exitg3 = false;
                while ((!exitg3) && (varargin_1_idx_0 <= nx - 1)) {
                  if (b_x->data[varargin_1_idx_0]) {
                    idx++;
                    ii->data[idx - 1] = varargin_1_idx_0 + 1;
                    if (idx >= nx) {
                      exitg3 = true;
                    } else {
                      varargin_1_idx_0++;
                    }
                  } else {
                    varargin_1_idx_0++;
                  }
                }

                if (b_x->size[0] == 1) {
                  if (idx == 0) {
                    ii->size[0] = 0;
                  }
                } else if (1 > idx) {
                  ii->size[0] = 0;
                } else {
                  i27 = ii->size[0];
                  ii->size[0] = idx;
                  emxEnsureCapacity_int32_T(ii, i27);
                }

                i27 = b_a->size[0];
                b_a->size[0] = ii->size[0];
                emxEnsureCapacity_real_T(b_a, i27);
                coffset = ii->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  b_a->data[i27] = ii->data[i27];
                }

                if (b_a->size[0] != 0) {
                  b_n = lambdas->size[0] - 1;
                  varargin_1_idx_0 = 0;
                  for (i = 0; i <= b_n; i++) {
                    if (lambdas->data[i] > 0.0) {
                      varargin_1_idx_0++;
                    }
                  }

                  i27 = r17->size[0];
                  r17->size[0] = varargin_1_idx_0;
                  emxEnsureCapacity_int32_T(r17, i27);
                  varargin_1_idx_0 = 0;
                  for (i = 0; i <= b_n; i++) {
                    if (lambdas->data[i] > 0.0) {
                      r17->data[varargin_1_idx_0] = i + 1;
                      varargin_1_idx_0++;
                    }
                  }

                  b_n = r17->size[0];
                  if (r17->size[0] <= 2) {
                    if (r17->size[0] == 1) {
                      lambda = lambdas->data[r17->data[0] - 1];
                      idx = 1;
                    } else if ((lambdas->data[r17->data[0] - 1] > lambdas->
                                data[r17->data[1] - 1]) || (rtIsNaN
                                (lambdas->data[r17->data[0] - 1]) && (!rtIsNaN
                                 (lambdas->data[r17->data[1] - 1])))) {
                      lambda = lambdas->data[r17->data[1] - 1];
                      idx = 2;
                    } else {
                      lambda = lambdas->data[r17->data[0] - 1];
                      idx = 1;
                    }
                  } else {
                    if (!rtIsNaN(lambdas->data[r17->data[0] - 1])) {
                      idx = 1;
                    } else {
                      idx = 0;
                      k = 2;
                      exitg3 = false;
                      while ((!exitg3) && (k <= r17->size[0])) {
                        if (!rtIsNaN(lambdas->data[r17->data[k - 1] - 1])) {
                          idx = k;
                          exitg3 = true;
                        } else {
                          k++;
                        }
                      }
                    }

                    if (idx == 0) {
                      lambda = lambdas->data[r17->data[0] - 1];
                      idx = 1;
                    } else {
                      lambda = lambdas->data[r17->data[idx - 1] - 1];
                      i27 = idx + 1;
                      for (k = i27; k <= b_n; k++) {
                        if (lambda > lambdas->data[r17->data[k - 1] - 1]) {
                          lambda = lambdas->data[r17->data[k - 1] - 1];
                          idx = k;
                        }
                      }
                    }
                  }

                  idxl = (int)inactiveConstraintIndices->data[(int)b_a->data[idx
                    - 1] - 1] - 1;
                } else {
                  lambda = 0.0;
                }
              } else {
                lambda = 0.0;
              }
            } else {
              lambda = 0.0;
            }

            if ((1.0 < lambda) || rtIsNaN(lambda)) {
              d_y = 1.0;
            } else {
              d_y = lambda;
            }

            if (lambda > 0.0) {
              b_gamma = d_y;
            } else {
              b_gamma = 1.0;
            }

            rho = obj->ArmijoRuleBeta;
            temp = obj->ArmijoRuleSigma;
            i27 = d_a->size[0];
            d_a->size[0] = x->size[0];
            emxEnsureCapacity_real_T(d_a, i27);
            coffset = x->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              d_a->data[i27] = x->data[i27] + b_gamma * Hg->data[i27];
            }

            IKHelpers_computeCost(d_a, obj->ExtraArgs, &costNew, unusedU0,
                                  unusedU1, &r19);
            args = r19;
            obj->ExtraArgs = args;
            b_m = 0.0;
            do {
              exitg1 = 0;
              i27 = y->size[0] * y->size[1];
              y->size[0] = 1;
              y->size[1] = grad->size[0];
              emxEnsureCapacity_real_T(y, i27);
              coffset = grad->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                y->data[i27] = -temp * grad->data[i27];
              }

              i27 = c_y->size[0];
              c_y->size[0] = Hg->size[0];
              emxEnsureCapacity_real_T(c_y, i27);
              coffset = Hg->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                c_y->data[i27] = b_gamma * Hg->data[i27];
              }

              if ((y->size[1] == 1) || (c_y->size[0] == 1)) {
                d_y = 0.0;
                coffset = y->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  d_y += y->data[i27] * c_y->data[i27];
                }
              } else {
                d_y = 0.0;
                coffset = y->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  d_y += y->data[i27] * c_y->data[i27];
                }
              }

              if (cost - costNew < d_y) {
                flag = (b_gamma < obj->StepTolerance);
                if (flag) {
                  *exitFlag = StepSizeBelowMinimum;
                  *err = IKHelpers_evaluateSolution(obj->ExtraArgs);
                  *iter = 1.0 + (double)b_i;
                  exitg1 = 1;
                } else {
                  b_gamma *= rho;
                  b_m++;
                  i27 = d_a->size[0];
                  d_a->size[0] = x->size[0];
                  emxEnsureCapacity_real_T(d_a, i27);
                  coffset = x->size[0];
                  for (i27 = 0; i27 < coffset; i27++) {
                    d_a->data[i27] = x->data[i27] + b_gamma * Hg->data[i27];
                  }

                  IKHelpers_computeCost(d_a, obj->ExtraArgs, &costNew, unusedU0,
                                        unusedU1, &r20);
                  args = r20;
                  obj->ExtraArgs = args;
                }
              } else {
                i27 = xSol->size[0];
                xSol->size[0] = x->size[0];
                emxEnsureCapacity_real_T(xSol, i27);
                coffset = x->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  xSol->data[i27] = x->data[i27] + b_gamma * Hg->data[i27];
                }

                args = obj->ExtraArgs;
                i27 = lambdas->size[0];
                lambdas->size[0] = args->GradTemp->size[0];
                emxEnsureCapacity_real_T(lambdas, i27);
                coffset = args->GradTemp->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  lambdas->data[i27] = args->GradTemp->data[i27];
                }

                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = 1;
            } else if (b_m == 0.0) {
              temp = 2.2204460492503131E-16;
              b_sqrt(&temp);
              if (fabs(b_gamma - lambda) < 1.4901161193847656E-8) {
                coffset = obj->ConstraintMatrix->size[0];
                i27 = b_a->size[0];
                b_a->size[0] = coffset;
                emxEnsureCapacity_real_T(b_a, i27);
                for (i27 = 0; i27 < coffset; i27++) {
                  b_a->data[i27] = obj->ConstraintMatrix->data[i27 +
                    obj->ConstraintMatrix->size[0] * idxl];
                }

                activeSet->data[idxl] = true;
                b_n = activeSet->size[0] - 1;
                varargin_1_idx_0 = 0;
                for (i = 0; i <= b_n; i++) {
                  if (activeSet->data[i]) {
                    varargin_1_idx_0++;
                  }
                }

                i27 = r18->size[0];
                r18->size[0] = varargin_1_idx_0;
                emxEnsureCapacity_int32_T(r18, i27);
                varargin_1_idx_0 = 0;
                for (i = 0; i <= b_n; i++) {
                  if (activeSet->data[i]) {
                    r18->data[varargin_1_idx_0] = i + 1;
                    varargin_1_idx_0++;
                  }
                }

                coffset = obj->ConstraintMatrix->size[0];
                i27 = A->size[0] * A->size[1];
                A->size[0] = coffset;
                A->size[1] = r18->size[0];
                emxEnsureCapacity_real_T(A, i27);
                boffset = r18->size[0];
                for (i27 = 0; i27 < boffset; i27++) {
                  for (i = 0; i < coffset; i++) {
                    A->data[i + A->size[0] * i27] = obj->ConstraintMatrix->
                      data[i + obj->ConstraintMatrix->size[0] * (r18->data[i27]
                      - 1)];
                  }
                }

                i27 = c_a->size[0] * c_a->size[1];
                c_a->size[0] = 1;
                c_a->size[1] = b_a->size[0];
                emxEnsureCapacity_real_T(c_a, i27);
                coffset = b_a->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  c_a->data[i27] = b_a->data[i27];
                }

                if ((c_a->size[1] == 1) || (H->size[0] == 1)) {
                  i27 = y->size[0] * y->size[1];
                  y->size[0] = 1;
                  y->size[1] = H->size[1];
                  emxEnsureCapacity_real_T(y, i27);
                  coffset = H->size[1];
                  for (i27 = 0; i27 < coffset; i27++) {
                    y->data[i27] = 0.0;
                    boffset = c_a->size[1];
                    for (i = 0; i < boffset; i++) {
                      y->data[i27] += c_a->data[i] * H->data[i + H->size[0] *
                        i27];
                    }
                  }
                } else {
                  inner = c_a->size[1];
                  b_n = H->size[1];
                  i27 = y->size[0] * y->size[1];
                  y->size[0] = 1;
                  y->size[1] = H->size[1];
                  emxEnsureCapacity_real_T(y, i27);
                  for (nx = 0; nx < b_n; nx++) {
                    boffset = nx * inner;
                    y->data[nx] = 0.0;
                    for (k = 0; k < inner; k++) {
                      y->data[nx] += H->data[boffset + k] * c_a->data[k];
                    }
                  }
                }

                if ((y->size[1] == 1) || (b_a->size[0] == 1)) {
                  d_y = 0.0;
                  coffset = y->size[1];
                  for (i27 = 0; i27 < coffset; i27++) {
                    d_y += y->data[i27] * b_a->data[i27];
                  }
                } else {
                  d_y = 0.0;
                  coffset = y->size[1];
                  for (i27 = 0; i27 < coffset; i27++) {
                    d_y += y->data[i27] * b_a->data[i27];
                  }
                }

                i27 = b_y->size[0] * b_y->size[1];
                b_y->size[0] = b_a->size[0];
                b_y->size[1] = b_a->size[0];
                emxEnsureCapacity_real_T(b_y, i27);
                coffset = b_a->size[0];
                for (i27 = 0; i27 < coffset; i27++) {
                  boffset = b_a->size[0];
                  for (i = 0; i < boffset; i++) {
                    b_y->data[i27 + b_y->size[0] * i] = b_a->data[i27] *
                      b_a->data[i];
                  }
                }

                if ((b_y->size[1] == 1) || (H->size[0] == 1)) {
                  i27 = B->size[0] * B->size[1];
                  B->size[0] = b_y->size[0];
                  B->size[1] = H->size[1];
                  emxEnsureCapacity_real_T(B, i27);
                  coffset = b_y->size[0];
                  for (i27 = 0; i27 < coffset; i27++) {
                    boffset = H->size[1];
                    for (i = 0; i < boffset; i++) {
                      B->data[i27 + B->size[0] * i] = 0.0;
                      nx = b_y->size[1];
                      for (b_n = 0; b_n < nx; b_n++) {
                        B->data[i27 + B->size[0] * i] += b_y->data[i27 +
                          b_y->size[0] * b_n] * H->data[b_n + H->size[0] * i];
                      }
                    }
                  }
                } else {
                  m = b_y->size[0];
                  inner = b_y->size[1];
                  b_n = H->size[1];
                  i27 = B->size[0] * B->size[1];
                  B->size[0] = b_y->size[0];
                  B->size[1] = H->size[1];
                  emxEnsureCapacity_real_T(B, i27);
                  for (nx = 0; nx < b_n; nx++) {
                    coffset = nx * m;
                    boffset = nx * inner;
                    for (i = 0; i < m; i++) {
                      B->data[coffset + i] = 0.0;
                    }

                    for (k = 0; k < inner; k++) {
                      aoffset = k * m;
                      temp = H->data[boffset + k];
                      for (i = 0; i < m; i++) {
                        i27 = coffset + i;
                        B->data[i27] += temp * b_y->data[aoffset + i];
                      }
                    }
                  }
                }

                if ((H->size[1] == 1) || (B->size[0] == 1)) {
                  i27 = b_y->size[0] * b_y->size[1];
                  b_y->size[0] = H->size[0];
                  b_y->size[1] = B->size[1];
                  emxEnsureCapacity_real_T(b_y, i27);
                  coffset = H->size[0];
                  for (i27 = 0; i27 < coffset; i27++) {
                    boffset = B->size[1];
                    for (i = 0; i < boffset; i++) {
                      b_y->data[i27 + b_y->size[0] * i] = 0.0;
                      nx = H->size[1];
                      for (b_n = 0; b_n < nx; b_n++) {
                        b_y->data[i27 + b_y->size[0] * i] += H->data[i27 +
                          H->size[0] * b_n] * B->data[b_n + B->size[0] * i];
                      }
                    }
                  }
                } else {
                  m = H->size[0];
                  inner = H->size[1];
                  b_n = B->size[1];
                  i27 = b_y->size[0] * b_y->size[1];
                  b_y->size[0] = H->size[0];
                  b_y->size[1] = B->size[1];
                  emxEnsureCapacity_real_T(b_y, i27);
                  for (nx = 0; nx < b_n; nx++) {
                    coffset = nx * m;
                    boffset = nx * inner;
                    for (i = 0; i < m; i++) {
                      b_y->data[coffset + i] = 0.0;
                    }

                    for (k = 0; k < inner; k++) {
                      aoffset = k * m;
                      temp = B->data[boffset + k];
                      for (i = 0; i < m; i++) {
                        i27 = coffset + i;
                        b_y->data[i27] += temp * H->data[aoffset + i];
                      }
                    }
                  }
                }

                temp = 1.0 / d_y;
                i27 = H->size[0] * H->size[1];
                i = H->size[0] * H->size[1];
                emxEnsureCapacity_real_T(H, i);
                coffset = i27 - 1;
                for (i27 = 0; i27 <= coffset; i27++) {
                  H->data[i27] -= temp * b_y->data[i27];
                }

                b_guard1 = true;
              } else {
                guard2 = true;
              }
            } else {
              guard2 = true;
            }
          }

          if (guard2) {
            i27 = grad->size[0];
            grad->size[0] = lambdas->size[0];
            emxEnsureCapacity_real_T(grad, i27);
            coffset = lambdas->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              grad->data[i27] = lambdas->data[i27] - grad->data[i27];
            }

            i27 = c_a->size[0] * c_a->size[1];
            c_a->size[0] = 1;
            c_a->size[1] = Hg->size[0];
            emxEnsureCapacity_real_T(c_a, i27);
            coffset = Hg->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              c_a->data[i27] = Hg->data[i27];
            }

            if ((c_a->size[1] == 1) || (grad->size[0] == 1)) {
              temp = 0.0;
              coffset = c_a->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                temp += c_a->data[i27] * grad->data[i27];
              }
            } else {
              temp = 0.0;
              coffset = c_a->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                temp += c_a->data[i27] * grad->data[i27];
              }
            }

            i27 = b_tmp->size[0] * b_tmp->size[1];
            b_tmp->size[0] = 1;
            b_tmp->size[1] = grad->size[0];
            emxEnsureCapacity_real_T(b_tmp, i27);
            coffset = grad->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              b_tmp->data[i27] = grad->data[i27];
            }

            i27 = y->size[0] * y->size[1];
            y->size[0] = 1;
            y->size[1] = b_tmp->size[1];
            emxEnsureCapacity_real_T(y, i27);
            coffset = b_tmp->size[0] * b_tmp->size[1];
            for (i27 = 0; i27 < coffset; i27++) {
              y->data[i27] = 0.2 * b_tmp->data[i27];
            }

            if ((y->size[1] == 1) || (H->size[0] == 1)) {
              i27 = c_a->size[0] * c_a->size[1];
              c_a->size[0] = 1;
              c_a->size[1] = H->size[1];
              emxEnsureCapacity_real_T(c_a, i27);
              coffset = H->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                c_a->data[i27] = 0.0;
                boffset = y->size[1];
                for (i = 0; i < boffset; i++) {
                  c_a->data[i27] += y->data[i] * H->data[i + H->size[0] * i27];
                }
              }
            } else {
              inner = y->size[1];
              b_n = H->size[1];
              i27 = c_a->size[0] * c_a->size[1];
              c_a->size[0] = 1;
              c_a->size[1] = H->size[1];
              emxEnsureCapacity_real_T(c_a, i27);
              for (nx = 0; nx < b_n; nx++) {
                boffset = nx * inner;
                c_a->data[nx] = 0.0;
                for (k = 0; k < inner; k++) {
                  c_a->data[nx] += H->data[boffset + k] * y->data[k];
                }
              }
            }

            if ((c_a->size[1] == 1) || (grad->size[0] == 1)) {
              rho = 0.0;
              coffset = c_a->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                rho += c_a->data[i27] * grad->data[i27];
              }
            } else {
              rho = 0.0;
              coffset = c_a->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                rho += c_a->data[i27] * grad->data[i27];
              }
            }

            if (temp < rho) {
              i27 = y->size[0] * y->size[1];
              y->size[0] = 1;
              y->size[1] = b_tmp->size[1];
              emxEnsureCapacity_real_T(y, i27);
              coffset = b_tmp->size[0] * b_tmp->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                y->data[i27] = 0.8 * b_tmp->data[i27];
              }

              if ((y->size[1] == 1) || (H->size[0] == 1)) {
                i27 = c_a->size[0] * c_a->size[1];
                c_a->size[0] = 1;
                c_a->size[1] = H->size[1];
                emxEnsureCapacity_real_T(c_a, i27);
                coffset = H->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  c_a->data[i27] = 0.0;
                  boffset = y->size[1];
                  for (i = 0; i < boffset; i++) {
                    c_a->data[i27] += y->data[i] * H->data[i + H->size[0] * i27];
                  }
                }
              } else {
                inner = y->size[1];
                b_n = H->size[1];
                i27 = c_a->size[0] * c_a->size[1];
                c_a->size[0] = 1;
                c_a->size[1] = H->size[1];
                emxEnsureCapacity_real_T(c_a, i27);
                for (nx = 0; nx < b_n; nx++) {
                  boffset = nx * inner;
                  c_a->data[nx] = 0.0;
                  for (k = 0; k < inner; k++) {
                    c_a->data[nx] += H->data[boffset + k] * y->data[k];
                  }
                }
              }

              if ((c_a->size[1] == 1) || (grad->size[0] == 1)) {
                temp = 0.0;
                coffset = c_a->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  temp += c_a->data[i27] * grad->data[i27];
                }
              } else {
                temp = 0.0;
                coffset = c_a->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  temp += c_a->data[i27] * grad->data[i27];
                }
              }

              if ((b_tmp->size[1] == 1) || (H->size[0] == 1)) {
                i27 = y->size[0] * y->size[1];
                y->size[0] = 1;
                y->size[1] = H->size[1];
                emxEnsureCapacity_real_T(y, i27);
                coffset = H->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  y->data[i27] = 0.0;
                  boffset = b_tmp->size[1];
                  for (i = 0; i < boffset; i++) {
                    y->data[i27] += b_tmp->data[i] * H->data[i + H->size[0] *
                      i27];
                  }
                }
              } else {
                inner = b_tmp->size[1];
                b_n = H->size[1];
                i27 = y->size[0] * y->size[1];
                y->size[0] = 1;
                y->size[1] = H->size[1];
                emxEnsureCapacity_real_T(y, i27);
                for (nx = 0; nx < b_n; nx++) {
                  boffset = nx * inner;
                  y->data[nx] = 0.0;
                  for (k = 0; k < inner; k++) {
                    y->data[nx] += H->data[boffset + k] * b_tmp->data[k];
                  }
                }
              }

              if ((y->size[1] == 1) || (grad->size[0] == 1)) {
                d_y = 0.0;
                coffset = y->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  d_y += y->data[i27] * grad->data[i27];
                }
              } else {
                d_y = 0.0;
                coffset = y->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  d_y += y->data[i27] * grad->data[i27];
                }
              }

              i27 = c_a->size[0] * c_a->size[1];
              c_a->size[0] = 1;
              c_a->size[1] = Hg->size[0];
              emxEnsureCapacity_real_T(c_a, i27);
              coffset = Hg->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                c_a->data[i27] = Hg->data[i27];
              }

              if ((c_a->size[1] == 1) || (grad->size[0] == 1)) {
                rho = 0.0;
                coffset = c_a->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  rho += c_a->data[i27] * grad->data[i27];
                }
              } else {
                rho = 0.0;
                coffset = c_a->size[1];
                for (i27 = 0; i27 < coffset; i27++) {
                  rho += c_a->data[i27] * grad->data[i27];
                }
              }

              rho = temp / (d_y - rho);
            } else {
              rho = 1.0;
            }

            i27 = b_y->size[0] * b_y->size[1];
            b_y->size[0] = H->size[0];
            b_y->size[1] = H->size[1];
            emxEnsureCapacity_real_T(b_y, i27);
            coffset = H->size[0] * H->size[1];
            for (i27 = 0; i27 < coffset; i27++) {
              b_y->data[i27] = (1.0 - rho) * H->data[i27];
            }

            if ((b_y->size[1] == 1) || (grad->size[0] == 1)) {
              i27 = r12->size[0];
              r12->size[0] = b_y->size[0];
              emxEnsureCapacity_real_T(r12, i27);
              coffset = b_y->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                r12->data[i27] = 0.0;
                boffset = b_y->size[1];
                for (i = 0; i < boffset; i++) {
                  r12->data[i27] += b_y->data[i27 + b_y->size[0] * i] *
                    grad->data[i];
                }
              }
            } else {
              m = b_y->size[0];
              inner = b_y->size[1];
              i27 = r12->size[0];
              r12->size[0] = b_y->size[0];
              emxEnsureCapacity_real_T(r12, i27);
              for (i = 0; i < m; i++) {
                r12->data[i] = 0.0;
              }

              for (k = 0; k < inner; k++) {
                aoffset = k * m;
                for (i = 0; i < m; i++) {
                  r12->data[i] += grad->data[k] * b_y->data[aoffset + i];
                }
              }
            }

            i27 = Hg->size[0];
            emxEnsureCapacity_real_T(Hg, i27);
            coffset = Hg->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              Hg->data[i27] = rho * Hg->data[i27] + r12->data[i27];
            }

            i27 = c_a->size[0] * c_a->size[1];
            c_a->size[0] = 1;
            c_a->size[1] = Hg->size[0];
            emxEnsureCapacity_real_T(c_a, i27);
            coffset = Hg->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              c_a->data[i27] = Hg->data[i27];
            }

            if ((c_a->size[1] == 1) || (grad->size[0] == 1)) {
              rho = 0.0;
              coffset = c_a->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                rho += c_a->data[i27] * grad->data[i27];
              }
            } else {
              rho = 0.0;
              coffset = c_a->size[1];
              for (i27 = 0; i27 < coffset; i27++) {
                rho += c_a->data[i27] * grad->data[i27];
              }
            }

            eye(n, r13);
            i27 = B->size[0] * B->size[1];
            B->size[0] = Hg->size[0];
            B->size[1] = b_tmp->size[1];
            emxEnsureCapacity_real_T(B, i27);
            coffset = Hg->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              boffset = b_tmp->size[1];
              for (i = 0; i < boffset; i++) {
                temp = Hg->data[i27] * b_tmp->data[i];
                B->data[i27 + B->size[0] * i] = r13->data[i27 + r13->size[0] * i]
                  - temp / rho;
              }
            }

            if ((B->size[1] == 1) || (H->size[0] == 1)) {
              i27 = b_y->size[0] * b_y->size[1];
              b_y->size[0] = B->size[0];
              b_y->size[1] = H->size[1];
              emxEnsureCapacity_real_T(b_y, i27);
              coffset = B->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                boffset = H->size[1];
                for (i = 0; i < boffset; i++) {
                  b_y->data[i27 + b_y->size[0] * i] = 0.0;
                  nx = B->size[1];
                  for (b_n = 0; b_n < nx; b_n++) {
                    b_y->data[i27 + b_y->size[0] * i] += B->data[i27 + B->size[0]
                      * b_n] * H->data[b_n + H->size[0] * i];
                  }
                }
              }
            } else {
              m = B->size[0];
              inner = B->size[1];
              b_n = H->size[1];
              i27 = b_y->size[0] * b_y->size[1];
              b_y->size[0] = B->size[0];
              b_y->size[1] = H->size[1];
              emxEnsureCapacity_real_T(b_y, i27);
              for (nx = 0; nx < b_n; nx++) {
                coffset = nx * m;
                boffset = nx * inner;
                for (i = 0; i < m; i++) {
                  b_y->data[coffset + i] = 0.0;
                }

                for (k = 0; k < inner; k++) {
                  aoffset = k * m;
                  temp = H->data[boffset + k];
                  for (i = 0; i < m; i++) {
                    i27 = coffset + i;
                    b_y->data[i27] += temp * B->data[aoffset + i];
                  }
                }
              }
            }

            i27 = a->size[0] * a->size[1];
            a->size[0] = B->size[1];
            a->size[1] = B->size[0];
            emxEnsureCapacity_real_T(a, i27);
            coffset = B->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              boffset = B->size[1];
              for (i = 0; i < boffset; i++) {
                a->data[i + a->size[0] * i27] = B->data[i27 + B->size[0] * i];
              }
            }

            if ((b_y->size[1] == 1) || (a->size[0] == 1)) {
              i27 = r13->size[0] * r13->size[1];
              r13->size[0] = b_y->size[0];
              r13->size[1] = a->size[1];
              emxEnsureCapacity_real_T(r13, i27);
              coffset = b_y->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                boffset = a->size[1];
                for (i = 0; i < boffset; i++) {
                  r13->data[i27 + r13->size[0] * i] = 0.0;
                  nx = b_y->size[1];
                  for (b_n = 0; b_n < nx; b_n++) {
                    r13->data[i27 + r13->size[0] * i] += b_y->data[i27 +
                      b_y->size[0] * b_n] * a->data[b_n + a->size[0] * i];
                  }
                }
              }
            } else {
              m = b_y->size[0];
              inner = b_y->size[1];
              b_n = a->size[1];
              i27 = r13->size[0] * r13->size[1];
              r13->size[0] = b_y->size[0];
              r13->size[1] = a->size[1];
              emxEnsureCapacity_real_T(r13, i27);
              for (nx = 0; nx < b_n; nx++) {
                coffset = nx * m;
                boffset = nx * inner;
                for (i = 0; i < m; i++) {
                  r13->data[coffset + i] = 0.0;
                }

                for (k = 0; k < inner; k++) {
                  aoffset = k * m;
                  temp = a->data[boffset + k];
                  for (i = 0; i < m; i++) {
                    i27 = coffset + i;
                    r13->data[i27] += temp * b_y->data[aoffset + i];
                  }
                }
              }
            }

            i27 = H->size[0] * H->size[1];
            H->size[0] = Hg->size[0];
            H->size[1] = Hg->size[0];
            emxEnsureCapacity_real_T(H, i27);
            coffset = Hg->size[0];
            for (i27 = 0; i27 < coffset; i27++) {
              boffset = Hg->size[0];
              for (i = 0; i < boffset; i++) {
                temp = Hg->data[i27] * Hg->data[i];
                H->data[i27 + H->size[0] * i] = r13->data[i27 + r13->size[0] * i]
                  + temp / rho;
              }
            }

            varargin_1_idx_0 = H->size[0];
            b_n = H->size[1];
            if (varargin_1_idx_0 < b_n) {
              nx = varargin_1_idx_0;
            } else {
              nx = b_n;
            }

            i27 = B->size[0] * B->size[1];
            B->size[0] = varargin_1_idx_0;
            B->size[1] = b_n;
            emxEnsureCapacity_real_T(B, i27);
            coffset = varargin_1_idx_0 * b_n;
            for (i27 = 0; i27 < coffset; i27++) {
              B->data[i27] = 0.0;
            }

            if (nx > 0) {
              for (k = 0; k < nx; k++) {
                B->data[k + B->size[0] * k] = 1.0;
              }
            }

            i27 = b_A->size[0] * b_A->size[1];
            b_A->size[0] = H->size[0];
            b_A->size[1] = H->size[1];
            emxEnsureCapacity_real_T(b_A, i27);
            coffset = H->size[0] * H->size[1];
            for (i27 = 0; i27 < coffset; i27++) {
              b_A->data[i27] = H->data[i27] + 1.4901161193847656E-8 * B->
                data[i27];
            }

            if (!isPositiveDefinite(b_A)) {
              *exitFlag = HessianNotPositiveSemidefinite;
              *err = IKHelpers_evaluateSolution(obj->ExtraArgs);
              *iter = 1.0 + (double)b_i;
              exitg2 = 1;
            } else {
              b_guard1 = true;
            }
          }

          if (b_guard1) {
            if (e_DampedBFGSwGradientProjection(obj, xSol)) {
              i27 = xSol->size[0];
              xSol->size[0] = x->size[0];
              emxEnsureCapacity_real_T(xSol, i27);
              coffset = x->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                xSol->data[i27] = x->data[i27];
              }

              *exitFlag = SearchDirectionInvalid;
              *err = IKHelpers_evaluateSolution(obj->ExtraArgs);
              *iter = 1.0 + (double)b_i;
              exitg2 = 1;
            } else {
              i27 = x->size[0];
              x->size[0] = xSol->size[0];
              emxEnsureCapacity_real_T(x, i27);
              coffset = xSol->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                x->data[i27] = xSol->data[i27];
              }

              i27 = grad->size[0];
              grad->size[0] = lambdas->size[0];
              emxEnsureCapacity_real_T(grad, i27);
              coffset = lambdas->size[0];
              for (i27 = 0; i27 < coffset; i27++) {
                grad->data[i27] = lambdas->data[i27];
              }

              cost = costNew;
              b_i++;
            }
          }
        }
      }
    } else {
      *exitFlag = IterationLimitExceeded;
      *err = IKHelpers_evaluateSolution(obj->ExtraArgs);
      *iter = obj->MaxNumIterationInternal;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  emxFree_real_T(&b_A);
  emxFree_boolean_T(&b_activeSet);
  emxFree_real_T(&d_a);
  emxFree_real_T(&b_tmp);
  emxFree_real_T(&c_y);
  emxFree_int32_T(&ii);
  emxFree_boolean_T(&b_x);
  emxFree_real_T(&b_y);
  emxFree_real_T(&c_a);
  emxFree_real_T(&y);
  emxFree_real_T(&a);
  emxFree_int32_T(&r18);
  emxFree_int32_T(&r17);
  emxFree_int32_T(&r16);
  emxFree_int32_T(&r15);
  emxFree_int32_T(&r14);
  emxFree_real_T(&r13);
  emxFree_real_T(&r12);
  emxFree_real_T(&unusedU1);
  emxFree_real_T(&lambdas);
  emxFree_uint32_T(&inactiveConstraintIndices);
  emxFree_real_T(&P);
  emxFree_real_T(&B);
  emxFree_real_T(&Hg);
  emxFree_real_T(&b_a);
  emxFree_real_T(&A);
  emxFree_boolean_T(&activeSet);
  emxFree_real_T(&H);
  emxFree_real_T(&grad);
  emxFree_real_T(&x);
}

/*
 * File trailer for DampedBFGSwGradientProjection.c
 *
 * [EOF]
 */
