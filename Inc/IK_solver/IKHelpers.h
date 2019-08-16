/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IKHelpers.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef IKHELPERS_H
#define IKHELPERS_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void IKHelpers_computeCost(const emxArray_real_T *x,
  c_robotics_manip_internal_IKExt *args, double *cost, double W[36],
  emxArray_real_T *Jac, c_robotics_manip_internal_IKExt **b_args);
extern double IKHelpers_evaluateSolution(const c_robotics_manip_internal_IKExt
  *args);

#endif

/*
 * File trailer for IKHelpers.h
 *
 * [EOF]
 */
