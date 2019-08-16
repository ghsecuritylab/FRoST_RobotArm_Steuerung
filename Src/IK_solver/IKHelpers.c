/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IKHelpers.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

/* Include Files */
#include "IK_solver/rt_nonfinite.h"
#include "IK_solver/IK_solver.h"
#include "IK_solver/IKHelpers.h"
#include "IK_solver/norm.h"
#include "IK_solver/IK_solver_emxutil.h"
#include "IK_solver/rotm2axang.h"
#include "IK_solver/RigidBodyTree1.h"

/* Function Declarations */
static void IKHelpers_poseError(const double Td[16], const double T_data[],
  const int T_size[2], double errorvec[6]);

/* Function Definitions */

/*
 * Arguments    : const double Td[16]
 *                const double T_data[]
 *                const int T_size[2]
 *                double errorvec[6]
 * Return Type  : void
 */
static void IKHelpers_poseError(const double Td[16], const double T_data[],
  const int T_size[2], double errorvec[6])
{
  int i37;
  double T[9];
  double b_Td[9];
  double v[4];
  int i38;
  int Td_tmp;
  for (i37 = 0; i37 < 3; i37++) {
    T[3 * i37] = T_data[i37];
    T[1 + 3 * i37] = T_data[i37 + T_size[0]];
    T[2 + 3 * i37] = T_data[i37 + (T_size[0] << 1)];
  }

  for (i37 = 0; i37 < 3; i37++) {
    for (i38 = 0; i38 < 3; i38++) {
      Td_tmp = i37 + 3 * i38;
      b_Td[Td_tmp] = 0.0;
      b_Td[Td_tmp] = (Td[i37] * T[3 * i38] + Td[i37 + 4] * T[1 + 3 * i38]) +
        Td[i37 + 8] * T[2 + 3 * i38];
    }
  }

  rotm2axang(b_Td, v);
  errorvec[0] = v[3] * v[0];
  errorvec[3] = Td[12] - T_data[T_size[0] * 3];
  errorvec[1] = v[3] * v[1];
  errorvec[4] = Td[13] - T_data[1 + T_size[0] * 3];
  errorvec[2] = v[3] * v[2];
  errorvec[5] = Td[14] - T_data[2 + T_size[0] * 3];
}

/*
 * Arguments    : const emxArray_real_T *x
 *                c_robotics_manip_internal_IKExt *args
 *                double *cost
 *                double W[36]
 *                emxArray_real_T *Jac
 *                c_robotics_manip_internal_IKExt **b_args
 * Return Type  : void
 */
void IKHelpers_computeCost(const emxArray_real_T *x,
  c_robotics_manip_internal_IKExt *args, double *cost, double W[36],
  emxArray_real_T *Jac, c_robotics_manip_internal_IKExt **b_args)
{
  emxArray_char_T *bodyName;
  d_robotics_manip_internal_Rigid *treeInternal;
  int boffset;
  int loop_ub;
  double Td[16];
  double T_data[16];
  int T_size[2];
  double e[6];
  double y;
  double b_y[6];
  double d4;
  emxArray_real_T *c_y;
  int n;
  int k;
  emxInit_char_T(&bodyName, 2);
  *b_args = args;
  treeInternal = (*b_args)->Robot;
  boffset = bodyName->size[0] * bodyName->size[1];
  bodyName->size[0] = 1;
  bodyName->size[1] = (*b_args)->BodyName->size[1];
  emxEnsureCapacity_char_T(bodyName, boffset);
  loop_ub = (*b_args)->BodyName->size[0] * (*b_args)->BodyName->size[1];
  for (boffset = 0; boffset < loop_ub; boffset++) {
    bodyName->data[boffset] = (*b_args)->BodyName->data[boffset];
  }

  for (boffset = 0; boffset < 16; boffset++) {
    Td[boffset] = (*b_args)->Tform[boffset];
  }

  for (boffset = 0; boffset < 36; boffset++) {
    W[boffset] = (*b_args)->WeightMatrix[boffset];
  }

  c_RigidBodyTree_efficientFKAndJ(treeInternal, x, bodyName, T_data, T_size, Jac);
  boffset = Jac->size[0] * Jac->size[1];
  loop_ub = Jac->size[0] * Jac->size[1];
  Jac->size[0] = 6;
  emxEnsureCapacity_real_T(Jac, loop_ub);
  loop_ub = boffset - 1;
  emxFree_char_T(&bodyName);
  for (boffset = 0; boffset <= loop_ub; boffset++) {
    Jac->data[boffset] = -Jac->data[boffset];
  }

  IKHelpers_poseError(Td, T_data, T_size, e);
  boffset = (*b_args)->ErrTemp->size[0];
  (*b_args)->ErrTemp->size[0] = 6;
  emxEnsureCapacity_real_T((*b_args)->ErrTemp, boffset);
  for (boffset = 0; boffset < 6; boffset++) {
    (*b_args)->ErrTemp->data[boffset] = e[boffset];
  }

  y = 0.0;
  for (boffset = 0; boffset < 6; boffset++) {
    b_y[boffset] = 0.0;
    d4 = 0.0;
    for (loop_ub = 0; loop_ub < 6; loop_ub++) {
      d4 += 0.5 * e[loop_ub] * W[loop_ub + 6 * boffset];
    }

    b_y[boffset] = d4;
    y += d4 * e[boffset];
  }

  (*b_args)->CostTemp = y;
  for (boffset = 0; boffset < 6; boffset++) {
    b_y[boffset] = 0.0;
    d4 = 0.0;
    for (loop_ub = 0; loop_ub < 6; loop_ub++) {
      d4 += e[loop_ub] * W[loop_ub + 6 * boffset];
    }

    b_y[boffset] = d4;
  }

  emxInit_real_T(&c_y, 2);
  n = Jac->size[1];
  boffset = c_y->size[0] * c_y->size[1];
  c_y->size[0] = 1;
  c_y->size[1] = Jac->size[1];
  emxEnsureCapacity_real_T(c_y, boffset);
  for (loop_ub = 0; loop_ub < n; loop_ub++) {
    boffset = loop_ub * 6;
    c_y->data[loop_ub] = 0.0;
    for (k = 0; k < 6; k++) {
      c_y->data[loop_ub] += Jac->data[boffset + k] * b_y[k];
    }
  }

  boffset = (*b_args)->GradTemp->size[0];
  (*b_args)->GradTemp->size[0] = c_y->size[1];
  emxEnsureCapacity_real_T((*b_args)->GradTemp, boffset);
  loop_ub = c_y->size[1];
  for (boffset = 0; boffset < loop_ub; boffset++) {
    (*b_args)->GradTemp->data[boffset] = c_y->data[boffset];
  }

  emxFree_real_T(&c_y);
  *cost = (*b_args)->CostTemp;
}

/*
 * Arguments    : const c_robotics_manip_internal_IKExt *args
 * Return Type  : double
 */
double IKHelpers_evaluateSolution(const c_robotics_manip_internal_IKExt *args)
{
  int k;
  emxArray_real_T *b;
  double a[36];
  int loop_ub;
  double y[6];
  double d5;
  int aoffset;
  for (k = 0; k < 36; k++) {
    a[k] = args->WeightMatrix[k];
  }

  emxInit_real_T(&b, 1);
  k = b->size[0];
  b->size[0] = args->ErrTemp->size[0];
  emxEnsureCapacity_real_T(b, k);
  loop_ub = args->ErrTemp->size[0];
  for (k = 0; k < loop_ub; k++) {
    b->data[k] = args->ErrTemp->data[k];
  }

  if (b->size[0] == 1) {
    for (k = 0; k < 6; k++) {
      y[k] = 0.0;
      d5 = 0.0;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        d5 += a[k + 6 * loop_ub] * b->data[loop_ub];
      }

      y[k] = d5;
    }
  } else {
    for (loop_ub = 0; loop_ub < 6; loop_ub++) {
      y[loop_ub] = 0.0;
    }

    for (k = 0; k < 6; k++) {
      aoffset = k * 6;
      for (loop_ub = 0; loop_ub < 6; loop_ub++) {
        y[loop_ub] += b->data[k] * a[aoffset + loop_ub];
      }
    }
  }

  emxFree_real_T(&b);
  return b_norm(y);
}

/*
 * File trailer for IKHelpers.c
 *
 * [EOF]
 */
