/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: DampedBFGSwGradientProjection.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef DAMPEDBFGSWGRADIENTPROJECTION_H
#define DAMPEDBFGSWGRADIENTPROJECTION_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void c_DampedBFGSwGradientProjection(c_robotics_core_internal_Damped *obj,
  emxArray_real_T *xSol, c_robotics_core_internal_NLPSol *exitFlag, double *err,
  double *iter);

#endif

/*
 * File trailer for DampedBFGSwGradientProjection.h
 *
 * [EOF]
 */
