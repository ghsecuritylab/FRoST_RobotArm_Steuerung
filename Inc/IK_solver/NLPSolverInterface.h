/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: NLPSolverInterface.h
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 14-Aug-2019 12:52:54
 */

#ifndef NLPSOLVERINTERFACE_H
#define NLPSOLVERINTERFACE_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "IK_solver/rtwtypes.h"
#include "IK_solver/IK_solver_types.h"

/* Function Declarations */
extern void NLPSolverInterface_solve(c_robotics_core_internal_Damped *obj, const
  emxArray_real_T *seed, emxArray_real_T *xSol, double *solutionInfo_Iterations,
  double *solutionInfo_RRAttempts, double *solutionInfo_Error, double
  *solutionInfo_ExitFlag, char solutionInfo_Status_data[], int
  solutionInfo_Status_size[2]);

#endif

/*
 * File trailer for NLPSolverInterface.h
 *
 * [EOF]
 */
